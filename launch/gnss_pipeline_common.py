#!/usr/bin/env python3
"""Shared wiring for the pipeline launch files.

This is a plain module, not a launch file. `pipeline.launch.py` imports it after
putting this directory on `sys.path`, which works both from the source tree and
from the installed `share/` tree.

Two things live here so the launch file stays readable. First, everything
receiver-specific: which executable a make/mode pair maps to, which parameters
that executable actually declares, and the rover/base topic map the estimators
already expect. Second, the settings resolver.

Settings come from three places, in this order:

    CLI argument  >  site YAML  >  node/estimator YAML  >  node code default

with **empty meaning "not specified"** at every level. That rule is load-bearing,
not a nicety: the selected YAML parameters are passed to `Node` as `[yaml_dict, overrides]`
and the overrides win, so any value this module puts in that dict silently overrides the
YAML. Emitting a launch argument's *default* would therefore replace a real
configured value - a zero lever arm in place of a measured one, for instance. So
a value that nobody explicitly set is never emitted at all, and the YAML's own
value survives untouched.
"""

import collections
import os
import math
import re
import socket
import sys
from datetime import datetime

import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

PACKAGE = 'gnss_ros_standardization'

# The solution topic every pipeline normalizes its estimator onto, so that the
# EKF's GNSS input and `rosbag_to_pos` work the same whichever estimator ran.
SOLUTION_TOPIC = '/gnss/solution'
EKF_SOLUTION_TOPIC = '/gnss/fusion/ekf_solution'

# Executable names are the ones registered in CMakeLists.txt.
#
# `calibrated_imu` is about the *parameter surface*: sbf_decoder_node and
# sbf_driver_node declare no `topics.imu`, because Septentrio has no calibrated
# IMU path. It is not a recommendation - see `imu_messages` below.
#
# `imu_messages` is what a *driver* must be told to enable for raw IMU output.
# Only ESF-RAW / ExtSensorMeas / RAWIMUSX appear: the calibrated topics
# (ESF-INS, CORRIMUDATA) are the receiver's own GNSS-corrected fusion output,
# so feeding them to an estimator that also consumes those GNSS measurements
# double-counts them. u-blox additionally gets NAV-CLOCK, whose tAcc gates the
# GNSS epochs used to anchor the ESF-RAW timeline.
RECEIVERS = {
    'ubx': {
        'decoder': 'ubx_decoder_node',
        'driver': 'ubx_driver_node',
        'driver_yaml': 'ubx_driver.yaml',
        'relay_port': 5556,
        'calibrated_imu': True,
        'imu_messages': {'messages.esf_raw': True, 'messages.nav_clock': True},
    },
    'sbf': {
        'decoder': 'sbf_decoder_node',
        'driver': 'sbf_driver_node',
        'driver_yaml': 'sbf_driver.yaml',
        'relay_port': 5557,
        'calibrated_imu': False,
        'imu_messages': {'messages.ext_sensor_meas': True},
    },
    'novatel': {
        'decoder': 'novatel_decoder_node',
        'driver': 'novatel_driver_node',
        'driver_yaml': 'novatel_driver.yaml',
        'relay_port': 5558,
        'calibrated_imu': True,
        'imu_messages': {'messages.rawimusx': True},
    },
    'rtcm': {
        # RTCM3 is stream-only: there is nothing to configure, so no driver.
        'decoder': 'rtcm_decoder_node',
        'driver': None,
        'driver_yaml': None,
        'relay_port': None,
        'calibrated_imu': False,
        'imu_messages': {},
    },
}

# `base_keys` is the (postype, pos) parameter pair each estimator spells
# differently. The EKF has none: it consumes a solution, not observations.
ESTIMATORS = {
    'rtk': {
        'executable': 'real_time_kinematic',
        'yaml': 'real_time_kinematic.yaml',
        'base_keys': ('ant2.postype', 'ant2.pos'),
        'needs_gtsam': False,
        'needs_imu': False,
    },
    'gnss_fgo': {
        'executable': 'gnss_fgo',
        'yaml': 'gnss_fgo.yaml',
        'base_keys': ('base_position.postype', 'base_position.pos'),
        'needs_gtsam': True,
        'needs_imu': False,
    },
    'gnss_imu_fgo': {
        'executable': 'gnss_imu_fgo',
        'yaml': 'gnss_imu_fgo.yaml',
        'base_keys': ('base_position.postype', 'base_position.pos'),
        'needs_gtsam': True,
        'needs_imu': True,
    },
    'ekf': {
        'executable': 'gnss_imu_kalman_filter',
        'yaml': 'gnss_imu_kalman_filter.yaml',
        'base_keys': None,
        'needs_gtsam': False,
        'needs_imu': True,
    },
}


# What `run.pipeline` (or `pipeline:=`) may name. `fallbacks` are the defaults
# that differ by pipeline - the logging pipeline records, the GNSS/IMU ones turn
# the IMU and GPS timestamps on. They apply only when neither the command line
# nor the site file says anything, so the site file stays authoritative.
PIPELINES = {
    'logging': {
        'fallbacks': {'record': True},
        'summary': 'record rover / base / IMU to a rosbag',
    },
    'rtk': {
        'fallbacks': {},
        'summary': 'RTK on the host (real_time_kinematic)',
    },
    'gnss_fgo': {
        'fallbacks': {},
        'summary': 'tightly-coupled GNSS factor graph (needs GTSAM)',
    },
    'gnss_imu_fgo': {
        'fallbacks': {'use_imu': True, 'use_gps_timestamp': True},
        'summary': 'tightly-coupled GNSS/IMU factor graph (needs GTSAM)',
    },
    'ekf': {
        'fallbacks': {'use_imu': True, 'use_gps_timestamp': True},
        'summary': 'loosely-coupled GNSS/IMU error-state EKF',
    },
    'onboard_rtk': {
        'fallbacks': {},
        'summary': 'RTK on the receiver, corrections relayed from the base stream',
    },
}


class PipelineError(RuntimeError):
    """Raised for a configuration that cannot work.

    Launch surfaces the message, which is the point: these are all cases where
    the alternative is a node that starts and then silently does nothing useful,
    or worse, something subtly wrong.
    """


def require(condition, message):
    if not condition:
        raise PipelineError(message)


# --------------------------------------------------------------------------
# paths
# --------------------------------------------------------------------------

def share(*parts):
    return os.path.join(get_package_share_directory(PACKAGE), *parts)


def config(name):
    return share('config', name)


def executable_available(name):
    """True when `name` was actually built into this install.

    `gnss_visualizer` builds only with OpenCV/cv_bridge/image_transport, and the
    FGO nodes only with -DBUILD_FGO_NODES=ON, so a pipeline has to be
    able to ask before it adds the node.
    """
    return os.path.isfile(os.path.join(get_package_prefix(PACKAGE), 'lib', PACKAGE, name))


# --------------------------------------------------------------------------
# the site file
# --------------------------------------------------------------------------

# Every key the site file may contain. A section maps to its allowed keys; a
# `None` value marks a scalar at the top level. Unknown keys are an error rather
# than silence, because a typo'd site key would otherwise look exactly like a
# setting that had no effect.
SITE_SCHEMA = {
    'run': ('pipeline', 'ekf_solution_source', 'replay', 'visualize'),
    'rover': ('receiver', 'mode', 'source', 'params_file'),
    # The one list-valued section: several receivers logged at once, each in
    # its own topic namespace. Replaces `rover`; the two cannot be combined.
    'rovers': ('name', 'receiver', 'mode', 'source', 'params_file', 'imu'),
    'base': ('receiver', 'source', 'merge_ephemeris', 'reference_week', 'reference_tow'),
    'imu': ('enabled', 'use_gps_timestamp'),
    'estimator': ('params_file', 'base_position', 'base_position_type', 'lever_arm'),
    'relay': ('port', 'novatel_output'),
    'record': ('enabled', 'output', 'storage', 'compress'),
}

_ENV_RE = re.compile(r'\$\{([A-Za-z_][A-Za-z0-9_]*)\}|\$([A-Za-z_][A-Za-z0-9_]*)')


def expand_env(text, where):
    """Expand ${VAR} / $VAR, failing loudly on anything unset.

    An unset variable must not pass through: a URI still containing a literal
    `${NTRIP_PASSWORD}` fails much later, as an opaque stream error. An empty
    variable counts as unset for the same reason.
    """
    missing = []

    def substitute(match):
        name = match.group(1) or match.group(2)
        value = os.environ.get(name)
        if not value:
            missing.append(name)
            return match.group(0)
        return value

    expanded = _ENV_RE.sub(substitute, text)
    require(not missing,
            '{} references environment variable(s) that are unset or empty: {}. '
            'Export them, or write the value directly into the site file.'
            .format(where, ', '.join(sorted(set(missing)))))
    return expanded


def site_candidates(spec):
    """Paths to try for a `site:=` value, most specific first.

    A `.local.yaml` beside the shipped template wins, so a private setup stays
    out of git while the zero-argument command still means "my setup".
    """
    spec = (spec or '').strip()
    if spec.endswith('.yaml') or '/' in spec:
        return [os.path.expanduser(spec)]
    stem = 'site' if not spec else 'site_{}'.format(spec)
    return [config('{}.local.yaml'.format(stem)), config('{}.yaml'.format(stem))]


def load_site(spec):
    """Read and validate the site file. Returns (settings dict, path or None)."""
    candidates = site_candidates(spec)
    path = next((p for p in candidates if os.path.isfile(p)), None)
    if path is None:
        require(not (spec or '').strip(),
                'no site file for site:="{}" - looked for:\n  {}'
                .format(spec, '\n  '.join(candidates)))
        # A bare run with no site file at all is allowed; everything then has to
        # come from the command line, which is what this file exists to avoid.
        return {}, None

    with open(path, 'r') as handle:
        loaded = yaml.safe_load(handle)
    if loaded is None:
        return {}, path
    require(isinstance(loaded, dict),
            '{} must be a YAML mapping, got {}'.format(path, type(loaded).__name__))

    for section, value in loaded.items():
        require(section in SITE_SCHEMA,
                '{}: unknown section "{}". Known sections: {}'
                .format(path, section, ', '.join(sorted(SITE_SCHEMA))))
        allowed = SITE_SCHEMA[section]
        if allowed is None:
            continue
        if section == 'rovers':
            require(isinstance(value, list) and value,
                    '{}: "rovers" must be a non-empty list, one entry per receiver'
                    .format(path))
            for index, item in enumerate(value):
                require(isinstance(item, dict),
                        '{}: rovers[{}] must be a mapping'.format(path, index))
                for key in item:
                    require(key in allowed,
                            '{}: unknown key "rovers[{}].{}". Known keys: {}'
                            .format(path, index, key, ', '.join(allowed)))
            continue
        require(isinstance(value, dict) or value is None,
                '{}: section "{}" must be a mapping'.format(path, section))
        for key in (value or {}):
            require(key in allowed,
                    '{}: unknown key "{}.{}". Known keys: {}'
                    .format(path, section, key, ', '.join(allowed)))

    require(not (loaded.get('rover') and loaded.get('rovers')),
            '{}: has both "rover" and "rovers". Use "rover" for one receiver or '
            '"rovers" for several - not both.'.format(path))

    return _expand_site(loaded, path), path


def _expand_site(node, path, trail=''):
    if isinstance(node, dict):
        return {k: _expand_site(v, path, '{}.{}'.format(trail, k).lstrip('.'))
                for k, v in node.items()}
    if isinstance(node, list):
        return [_expand_site(v, path, trail) for v in node]
    if isinstance(node, str):
        return expand_env(node, '{} key "{}"'.format(path, trail))
    return node


# --------------------------------------------------------------------------
# settings resolution
# --------------------------------------------------------------------------

# Launch argument name -> (site section, site key).
SITE_KEYS = {
    'pipeline': ('run', 'pipeline'),
    'ekf_solution_source': ('run', 'ekf_solution_source'),
    'replay': ('run', 'replay'),
    'visualize': ('run', 'visualize'),
    'rover_receiver': ('rover', 'receiver'),
    'rover_mode': ('rover', 'mode'),
    'rover_source': ('rover', 'source'),
    'rover_params_file': ('rover', 'params_file'),
    'base_receiver': ('base', 'receiver'),
    'base_source': ('base', 'source'),
    'merge_base_ephemeris': ('base', 'merge_ephemeris'),
    'base_reference_week': ('base', 'reference_week'),
    'base_reference_tow': ('base', 'reference_tow'),
    'use_imu': ('imu', 'enabled'),
    'use_gps_timestamp': ('imu', 'use_gps_timestamp'),
    'record': ('record', 'enabled'),
    'bag_out': ('record', 'output'),
    'bag_storage': ('record', 'storage'),
    'bag_compress': ('record', 'compress'),
    'params_file': ('estimator', 'params_file'),
    'base_position': ('estimator', 'base_position'),
    'base_position_type': ('estimator', 'base_position_type'),
    'lever_arm': ('estimator', 'lever_arm'),
    'relay_port': ('relay', 'port'),
    'novatel_relay_output': ('relay', 'novatel_output'),
}

BOOL_KEYS = frozenset((
    'replay', 'visualize', 'merge_base_ephemeris', 'use_imu',
    'use_gps_timestamp', 'record', 'bag_compress',
))
LIST_KEYS = frozenset(('base_position', 'lever_arm'))

# Used only when neither the CLI nor the site file says anything. Anything absent
# here resolves to None, which means "emit no parameter".
FALLBACKS = {
    # logging needs nothing but a rover, so it is the one pipeline that can run
    # on a bare install without the site file being filled in first
    'pipeline': 'logging',
    'ekf_solution_source': 'rtk',
    'replay': False,
    'rover_receiver': 'ubx',
    'rover_mode': 'decoder',
    'base_receiver': 'rtcm',
    'bag_storage': 'mcap',
    'relay_port': 0,
    'merge_base_ephemeris': False,
    'use_imu': False,
    'use_gps_timestamp': False,
    'record': False,
    'bag_compress': False,
    'visualize': False,
}


def as_bool(value):
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    require(text in ('1', 'true', 'yes', 'on', '0', 'false', 'no', 'off'),
            'invalid boolean value: {!r}'.format(value))
    return text in ('1', 'true', 'yes', 'on')


def parse_triplet(value, what):
    """Accept "a,b,c", "a b c" or a YAML list; return three floats."""
    if isinstance(value, (list, tuple)):
        parts = [str(v) for v in value]
    else:
        parts = [p for p in str(value).replace(',', ' ').split() if p]
    require(len(parts) == 3,
            '{} needs three numbers, e.g. {}:="35.66633426,139.79220181,59.82" '
            '(got "{}")'.format(what, what, value))
    try:
        result = [float(p) for p in parts]
        require(all(math.isfinite(v) for v in result),
                "{} must contain finite numbers".format(what))
        return result
    except ValueError:
        raise PipelineError('{} is not numeric: "{}"'.format(what, value))


def _site_lookup(site, name):
    section, key = SITE_KEYS[name]
    if key is None:
        return site.get(section)
    return (site.get(section) or {}).get(key)


def _is_unset(value):
    return value is None or value == '' or value == []


class Settings(object):
    """Resolved settings, each remembering where it came from.

    `rovers` is None for the usual single receiver. With a `rovers:` list in the
    site file it holds one resolved dict per receiver instead, and the single
    `rover_*` values are unused.
    """

    def __init__(self, values, sources, site_path, rovers=None):
        self._values = values
        self._sources = sources
        self.site_path = site_path
        self.rovers = rovers

    def __getitem__(self, name):
        return self._values[name]

    def get(self, name, default=None):
        value = self._values.get(name)
        return default if value is None else value

    def source(self, name):
        return self._sources.get(name, 'unset')

    def specified(self, name):
        """True when the CLI or the site file actually set this."""
        return self._sources.get(name) in ('command line', 'site file')


def _resolve_one(context, site, name, fallbacks):
    """(value, origin) for one argument: command line > site file > fallback."""
    from launch.substitutions import LaunchConfiguration

    raw = LaunchConfiguration(name).perform(context)
    origin = 'command line'
    if _is_unset(raw):
        raw = _site_lookup(site, name) if name in SITE_KEYS else None
        origin = 'site file'
    if _is_unset(raw):
        raw = fallbacks.get(name)
        origin = 'default' if raw is not None else 'unset'

    if raw is None:
        return None, origin
    if name in BOOL_KEYS:
        return as_bool(raw), origin
    if name in LIST_KEYS:
        return parse_triplet(raw, name), origin
    if name in ('base_reference_week', 'base_reference_tow'):
        text = str(raw).strip()
        try:
            value = int(text) if name == 'base_reference_week' else float(text)
        except ValueError:
            raise PipelineError('{}:="{}" is not a number'.format(name, raw))
        if name == 'base_reference_week':
            require(value >= 0, 'base reference_week must be >= 0 (0 = none)')
        else:
            require(0.0 <= value < 604800.0, 'base reference_tow must be in [0, 604800)')
        return value, origin
    if name == 'relay_port':
        try:
            text = str(raw).strip()
            require(re.fullmatch(r'[0-9]+', text) is not None,
                    'relay_port must be an integer in [0, 65535]')
            port = int(text)
            require(0 <= port <= 65535, 'relay_port must be in [0, 65535]')
            return port, origin
        except (TypeError, ValueError):
            raise PipelineError('relay_port:="{}" is not a number'.format(raw))
    return str(raw), origin


def resolve_settings(context):
    """Resolve every launch argument against the site file.

    The pipeline is resolved first, because it decides the fallbacks for the
    rest: the logging pipeline records by default, the GNSS/IMU ones turn the IMU
    on. Those fallbacks still lose to the site file, which is what keeps the site
    file authoritative.
    """
    from launch.substitutions import LaunchConfiguration

    site, site_path = load_site(LaunchConfiguration('site').perform(context))

    pipeline, pipeline_origin = _resolve_one(context, site, 'pipeline', FALLBACKS)
    require(pipeline in PIPELINES,
            'pipeline "{}" ({}) is not one of: {}'.format(
                pipeline, pipeline_origin, ', '.join(PIPELINES)))

    fallbacks = dict(FALLBACKS)
    fallbacks.update(PIPELINES[pipeline]['fallbacks'])

    values = {'pipeline': pipeline}
    sources = {'pipeline': pipeline_origin}
    for name in ARGUMENT_NAMES:
        if name in ('site', 'pipeline'):
            continue
        values[name], sources[name] = _resolve_one(context, site, name, fallbacks)

    rovers = None
    # Under replay no receiver node starts, so a rovers list is simply unused -
    # which is what lets a multi-receiver site file also drive the replay of
    # one of its receivers into an estimator.
    if site.get('rovers') and not values['replay']:
        require(pipeline == 'logging',
                '{} lists several rovers, which only the logging pipeline '
                'supports so far. For {}, use a site file with a single "rover" '
                'section (site:=<name>), or replay one receiver out of a logged '
                'bag - see "Several receivers at once" in launch/README.md.'
                .format(site_path, pipeline))
        overridden = [n for n in ('rover_receiver', 'rover_mode', 'rover_source',
                                  'rover_params_file')
                      if sources[n] == 'command line']
        require(not overridden,
                '{} is ambiguous with a "rovers" list in {} - it cannot say which '
                'receiver it means. Edit that receiver in the site file instead.'
                .format(', '.join(n + ':=' for n in overridden), site_path))
        require(not values['merge_base_ephemeris'],
                'base.merge_ephemeris has no meaning with several rovers: each '
                'publishes its own /<name>/gnss/ephemeris, and nothing reads '
                '/gnss/ephemeris.')
        rovers = _resolve_rovers(site['rovers'], site_path, values['use_imu'])

    return Settings(values, sources, site_path, rovers)


# A rover name becomes a topic namespace and a node namespace, so it has to be a
# plain ROS name token. `base` is taken by the base station's topics.
_ROVER_NAME_RE = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')


def _resolve_rovers(items, site_path, use_imu):
    """One resolved dict per `rovers:` entry.

    `receiver` and `source` are required per entry: with several receivers on
    one machine, a silently defaulted device path is how two entries end up
    reading the same port.
    """
    resolved = []
    seen = set()
    for index, item in enumerate(items):
        name = str(item.get('name') or '')
        require(_ROVER_NAME_RE.fullmatch(name) is not None,
                '{}: rovers[{}] needs a "name" made of letters, digits and _, '
                'starting with a letter (got "{}"). It becomes the namespace of '
                'that receiver\'s topics, /<name>/gnss/...'.format(site_path, index, name))
        require(name != 'base',
                '{}: rovers[{}] cannot be named "base" - /base/gnss/... is the base '
                'station\'s namespace.'.format(site_path, index))
        require(name not in seen,
                '{}: two rovers are named "{}"'.format(site_path, name))
        seen.add(name)

        where = 'rover "{}"'.format(name)
        receiver = str(item.get('receiver') or '')
        source = str(item.get('source') or '')
        require(receiver, '{}: {} needs a "receiver"'.format(site_path, where))
        require(source, '{}: {} needs a "source"'.format(site_path, where))
        mode = str(item.get('mode') or 'decoder')
        validate_receiver(receiver, mode, 'rover', where=where)

        # Unset follows the global imu setting, except for an RTCM stream, which
        # cannot carry IMU - so one IMU-less receiver in the list does not force
        # the IMU off for the rest.
        imu = item.get('imu')
        if imu is None or imu == '':
            imu = bool(use_imu) and receiver != 'rtcm'
        else:
            imu = as_bool(imu)
            require(not (imu and receiver == 'rtcm'),
                    '{}: {} is rtcm, which carries no IMU - remove "imu: true"'
                    .format(site_path, where))

        resolved.append({
            'name': name,
            'receiver': receiver,
            'mode': mode,
            'source': source,
            'params_file': str(item.get('params_file') or ''),
            'imu': imu,
        })

    sources = [r['source'] for r in resolved]
    duplicated = sorted({src for src in sources if sources.count(src) > 1})
    require(not duplicated,
            '{}: several rovers read the same source {} - one port can only be '
            'opened once.'.format(site_path, ', '.join(_mask_uri(d) for d in duplicated)))
    return resolved


# --------------------------------------------------------------------------
# reading a value back out of an estimator YAML
# --------------------------------------------------------------------------

def read_node_parameters(params_file, node_name=None, namespace=''):
    """Select applicable ROS parameter blocks; never pick an unrelated first node.

    Namespace-specific blocks override wildcards and the unqualified node block.
    The latter remains a convenience fallback for a renamed/namespaced receiver.
    Return flattened names so validation and the values passed to Node agree.
    """
    if not params_file:
        return {}
    require(os.path.isfile(params_file), 'params file not found: {}'.format(params_file))
    with open(params_file, 'r') as handle:
        loaded = yaml.safe_load(handle) or {}
    require(isinstance(loaded, dict), '{} must be a YAML mapping'.format(params_file))
    blocks = []

    def collect(mapping, prefix=''):
        for key, value in mapping.items():
            require(isinstance(key, str), 'parameter YAML keys must be strings')
            if key == 'ros__parameters':
                require(isinstance(value, dict), 'ros__parameters must be a mapping')
                blocks.append((prefix or '/', value))
            elif isinstance(value, dict):
                collect(value, prefix.rstrip('/') + '/' + key.strip('/'))
    collect(loaded)
    if node_name is None:
        require(len(blocks) <= 1, 'node_name is required for a multi-node parameter file')
        selected = [(0, block) for _, block in blocks]
    else:
        target = '/' + '/'.join(p for p in (namespace.strip('/'), node_name.strip('/')) if p)
        selected = []
        for selector, block in blocks:
            tokens = selector.strip('/').split('/')
            pattern = ''.join('(?:/[^/]+)*' if t == '**' else
                              '/[^/]+' if t == '*' else '/' + re.escape(t) for t in tokens)
            if selector == target:
                priority = 1000
            elif selector == '/' + node_name.strip('/'):
                priority = 900
            elif re.fullmatch(pattern, target):
                priority = sum(t not in ('*', '**') for t in tokens)
            else:
                continue
            selected.append((priority, block))
        require(selected, '{} has no parameter block for {}'.format(params_file, target))
    result = {}

    def flatten(mapping, prefix='', out=None):
        out = {} if out is None else out
        for key, value in mapping.items():
            require(isinstance(key, str), 'parameter names must be strings')
            full = prefix + key
            if isinstance(value, dict):
                flatten(value, full + '.', out)
            else:
                require(full not in out, 'duplicate parameter name: {}'.format(full))
                out[full] = value
        return out
    for _, block in sorted(selected, key=lambda item: item[0]):
        result.update(flatten(block))
    return result


def _dotted(params, dotted_key):
    if not isinstance(params, dict):
        return None
    if dotted_key in params:
        return params[dotted_key]
    first, separator, rest = dotted_key.partition('.')
    if separator:
        return _dotted(params.get(first), rest)
    return params.get(first)


def validate_base_position(pos, postype, origin, allow_zero_llh=False):
    """Reject the two ways a base position silently ruins a whole trajectory."""
    require(postype != 'rtcm',
            'base position type is "rtcm" ({}), which cannot work in this '
            'architecture: RTCM 1005/1006 station information is parsed by '
            'rtcm_decoder_node but never republished to topics, and '
            'GnssObservations carries no station position, so the base position '
            'would stay at zero and translate the whole trajectory. Set an '
            'explicit llh or ecef position instead.'.format(origin))
    require(postype in ('llh', 'ecef'),
            'base position type "{}" ({}) must be "llh" or "ecef"'.format(postype, origin))
    require(pos is not None, "base position is missing ({})".format(origin))
    values = parse_triplet(pos, "base position ({})".format(origin))
    if postype == 'llh':
        require(-90 <= values[0] <= 90 and -180 <= values[1] <= 180,
                "base LLH latitude/longitude is out of range")
    require(any(v != 0.0 for v in values) or (postype == 'llh' and allow_zero_llh),
            'no usable base station position ({}). RTK and FGO need the real '
            'base antenna position - an error here translates the whole '
            'trajectory. Set it in the site file under estimator.base_position, '
            'in the estimator YAML, or pass base_position:="lat,lon,height".'
            .format(origin))


def resolve_estimator_deployment(kind, settings, own_params_file=True):
    """Work out the effective params file, base position and lever arm.

    A value the launch was not told about is returned as None, meaning "do not
    emit a parameter for this" - the estimator YAML's own value then applies
    untouched. The YAML is still read, so the base position can be validated and
    so the startup summary can say where each value came from.

    `own_params_file=False` is for a supporting estimator that is not the one the
    pipeline is named after - the RTK stage feeding the EKF. `params_file` then
    belongs to the EKF, and handing it to the RTK node would load a file keyed
    `gnss_imu_kalman_filter:` that the RTK node ignores, leaving it on code
    defaults. It gets its packaged config instead; a base position set through
    the site file or the command line still reaches it.
    """
    spec = ESTIMATORS[kind]
    use_settings_file = own_params_file and settings.specified('params_file')
    params_file = settings['params_file'] if use_settings_file else config(spec['yaml'])
    yaml_params = read_node_parameters(params_file, spec['executable'])
    yaml_label = os.path.basename(params_file)

    out = {
        'params_file': params_file,
        'params_file_source': (settings.source('params_file')
                               if use_settings_file else 'packaged default'),
        'base_position': None,
        'base_position_type': None,
        'base_position_source': None,
        'base_position_value': None,
        'lever_arm': None,
        'lever_arm_source': None,
        'lever_arm_value': None,
    }

    if spec['base_keys'] is not None:
        postype_key, pos_key = spec['base_keys']
        if settings.specified('base_position'):
            pos = settings['base_position']
            postype = settings.get('base_position_type') or 'llh'
            origin = settings.source('base_position')
            out['base_position'] = pos
            out['base_position_type'] = postype
        else:
            pos = _dotted(yaml_params, pos_key)
            postype = _dotted(yaml_params, postype_key) or 'llh'
            origin = yaml_label
            # Left as None on purpose: the YAML already carries these, and
            # emitting them would only re-state what the node will read anyway.
            if settings.specified('base_position_type'):
                out['base_position_type'] = settings['base_position_type']
                postype = settings['base_position_type']
                origin = '{} + {}'.format(yaml_label, settings.source('base_position_type'))
        validate_base_position(pos, postype, origin,
                               allow_zero_llh=settings.specified('base_position'))
        out['base_position_value'] = pos
        out['base_position_source'] = origin

    if spec['needs_imu']:
        if settings.specified('lever_arm'):
            out['lever_arm'] = settings['lever_arm']
            out['lever_arm_value'] = settings['lever_arm']
            out['lever_arm_source'] = settings.source('lever_arm')
        else:
            value = _dotted(yaml_params, 'lever_arm')
            out['lever_arm_value'] = parse_triplet(value, 'lever_arm') if value is not None else None
            out['lever_arm_source'] = yaml_label

    return out


# --------------------------------------------------------------------------
# topic map
# --------------------------------------------------------------------------

def topic_map(role, merge_base_ephemeris=False, name=None):
    """Topic names for a rover or base source node.

    Every driver/decoder defaults its topics to absolute `/gnss/...` names read
    from string parameters, so `-r __ns:=/base` does not move them. Rover/base
    separation has to be done by setting the topic parameters, which is what
    these launch files do for you.

    `name` is a rover from a `rovers:` list: its topics move under /<name>/ and
    its frame_id carries the name, so the bag alone says which receiver each
    message came from. Without it the rover keeps the plain /gnss/... names the
    estimators subscribe to.
    """
    require(role in ('rover', 'base'), 'unknown role "{}"'.format(role))
    if role == 'rover':
        prefix = '/{}'.format(name) if name else ''
        return {
            'observation': prefix + '/gnss/observation',
            'ephemeris': prefix + '/gnss/ephemeris',
            'solution': prefix + '/gnss/nmea_solution',
            'imu_raw': prefix + '/gnss/imu/data_raw',
            'imu': prefix + '/gnss/imu/data',
            'frame_id': '{}_gnss_link'.format(name) if name else 'gnss_link',
        }
    return {
        'observation': '/base/gnss/observation',
        # Parked by default so a stale base nav message can never displace the
        # rover's. merge_base_ephemeris:=true is for a rover stream that carries
        # no navigation data of its own.
        'ephemeris': '/gnss/ephemeris' if merge_base_ephemeris else '/base/gnss/ephemeris',
        'solution': '/base/gnss/nmea_solution',
        'imu_raw': '/base/gnss/imu/data_raw',
        'imu': '/base/gnss/imu/data',
        'frame_id': 'base_gnss_link',
    }


# file:// playback
#
# A decoder reading file:// starts the moment it opens the file, while a
# recorder (or any consumer) can subscribe only after it has discovered the
# decoder's topics, so the start of every file was lost. A file:// decoder is
# therefore started listening on loopback, and gnss_file_playback.py feeds it
# the file once every topic this pipeline consumes has all its subscribers.

_PLAYBACKS = []


def playback_source(source, mode, topics):
    """The stream_path for a receiver node; a file:// decoder is played back.

    RTKLIB file options (`::T`, `::x<speed>`, ...) need RTKLIB's own file
    stream, so such a path is opened directly as before.
    """
    if mode != 'decoder' or not source.startswith('file://') or '::' in source:
        return source
    with socket.socket() as probe:  # a free loopback port for this receiver
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    path = os.path.abspath(os.path.expanduser(source[len('file://'):]))
    _PLAYBACKS.append({'file': path, 'port': port, 'topics': topics})
    return 'tcpsvr://127.0.0.1:{}'.format(port)


def take_playbacks():
    """The file:// playbacks registered since the last call."""
    out = list(_PLAYBACKS)
    _PLAYBACKS.clear()
    return out


def stream_consumers(settings, has_base, estimator_topic=''):
    """How many nodes of this pipeline subscribe each receiver topic.

    Mirrors what the builders and tail_actions start; a consumer counted here
    that never subscribes would hold the playback forever.
    """
    kind = settings['pipeline']
    consumers = collections.Counter()
    rover = topic_map('rover')
    base = topic_map('base', settings.get('merge_base_ephemeris'))
    on_receiver_pvt = kind == 'ekf' and settings.get('ekf_solution_source') == 'receiver'
    if kind in ('rtk', 'gnss_fgo', 'gnss_imu_fgo') or (kind == 'ekf' and not on_receiver_pvt):
        consumers[rover['observation']] += 1
        consumers[rover['ephemeris']] += 1
        consumers[base['observation']] += 1
    if kind in ('gnss_imu_fgo', 'ekf'):
        consumers[rover['imu_raw']] += 1
    if on_receiver_pvt:
        consumers[rover['solution']] += 1
    if settings.get('visualize') and executable_available('gnss_visualizer'):
        if settings.rovers:
            named = topic_map('rover', name=settings.rovers[0]['name'])
            shown = [named['observation'], named['ephemeris'], named['solution']]
        else:
            shown = [rover['observation'], rover['ephemeris'],
                     estimator_topic or rover['solution']]
        for topic in shown:
            consumers[topic] += 1
    if settings.get('record'):
        for topic in record_topics(with_base=has_base, with_imu=settings.get('use_imu'),
                                   estimator_topic=estimator_topic,
                                   merge_base_ephemeris=settings.get('merge_base_ephemeris'),
                                   rovers=settings.rovers, estimator_kind=kind):
            consumers[topic] += 1
    return consumers


def playback_actions(playbacks, consumers):
    """One gnss_file_playback.py process per file:// decoder."""
    script = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gnss_file_playback.py')
    actions = []
    for playback in playbacks:
        waits = ['{}={}'.format(t, consumers[t]) for t in sorted(set(playback['topics'].values()))
                 if t.startswith('/') and consumers[t] > 0]
        actions.append(ExecuteProcess(
            cmd=[sys.executable, script, playback['file'], str(playback['port'])] + waits,
            output='screen', name='file_playback_{}'.format(playback['port'])))
    return actions


def validate_receiver(receiver, mode, role, where=None):
    """`where` names an entry of a rovers list in the messages instead of the
    single-receiver launch argument."""
    receiver_label = '{} receiver'.format(where) if where else '{}_receiver:='.format(role)
    mode_label = '{} mode'.format(where) if where else '{}_mode:='.format(role)
    require(receiver in RECEIVERS,
            '{}"{}" is not one of {}'.format(
                receiver_label, receiver, '/'.join(sorted(RECEIVERS))))
    require(mode in ('decoder', 'driver'),
            '{}"{}" must be "decoder" or "driver"'.format(mode_label, mode))
    require(not (mode == 'driver' and RECEIVERS[receiver]['driver'] is None),
            'receiver "{}" has no driver: RTCM3 is a stream format with nothing '
            'to configure. Use mode decoder.'.format(receiver))


# --------------------------------------------------------------------------
# nodes
# --------------------------------------------------------------------------

def load_params_dict(params_file, node_name=None, namespace=''):
    """A params file's `ros__parameters`, for passing to a node as values.

    Needed for a node in a namespace: a params *file* keyed `ubx_driver_node:`
    does not apply to /<ns>/ubx_driver_node - verified, the namespaced driver
    silently ran on its code defaults (rate 5 Hz, NAV-PVT off) instead of the
    YAML's. Unlike `read_node_parameters`, a missing or empty file is an error
    here, because silence would mean a receiver configured with nothing.
    """
    require(os.path.isfile(params_file), 'params file not found: {}'.format(params_file))
    params = read_node_parameters(params_file, node_name, namespace)
    require(bool(params),
            '{} has no "<node>: ros__parameters:" block'.format(params_file))
    return params


def receiver_node(role, receiver, mode, source, params_file='', relay_listen='',
                  relay_forward='', use_imu=False, use_gps_timestamp=None,
                  merge_base_ephemeris=False, novatel_relay_output='', name=None,
                  reference_time=None):
    """Build the driver or decoder node for one role.

    Only parameters the chosen executable actually declares are emitted, because
    the four decoders do not share a parameter surface: `rtcm_decoder_node`
    declares six parameters and has no frame_id, solution or IMU topics at all.
    """
    validate_receiver(receiver, mode, role)
    require(bool(source),
            '{} stream URI is not set. Put it in the site file under {}.source, '
            'or pass {}_source:=...'.format(role, role, role))
    spec = RECEIVERS[receiver]
    executable = spec['driver'] if mode == 'driver' else spec['decoder']
    topics = topic_map(role, merge_base_ephemeris, name=name)

    params = {
        'stream_path': playback_source(source, mode, topics),
        'topics.observation': topics['observation'],
        'topics.ephemeris': topics['ephemeris'],
    }

    if receiver != 'rtcm':
        params['frame_id'] = topics['frame_id']
        params['topics.solution'] = topics['solution']
        params['topics.imu_raw'] = topics['imu_raw']
        if spec['calibrated_imu']:
            params['topics.imu'] = topics['imu']
        # Left out entirely when nobody asked, so a driver YAML's own setting
        # stands rather than being overwritten with this module's idea of false.
    if use_gps_timestamp is not None:
        params['use_gps_timestamp'] = use_gps_timestamp
    # RTCM MSM carries time of week only; a historical log needs its week.
    if reference_time is not None:
        require(receiver == 'rtcm',
                'base reference_week/reference_tow apply to an rtcm base only; '
                '"{}" carries the GPS week itself'.format(receiver))
        params['reference_week'], params['reference_tow'] = reference_time

    if mode == 'driver' and use_imu:
        params.update(spec['imu_messages'])

    # Receiver correction input and source-byte output have distinct names.
    if relay_listen:
        require(receiver != 'rtcm',
                'rtcm_decoder_node cannot host the correction server - its '
                'rtcm_output.destination parameter is the output URI. Point relay_forward '
                'at the rover node instead.')
        if mode == 'driver':
            params['rtcm_input.enable'] = True
            params['rtcm_input.listen'] = relay_listen
            if receiver == 'novatel':
                params['rtcm_input.receiver_stream'] = novatel_relay_output
                params['rtcm_input.receiver_port'] = 'THISPORT'
        else:
            params['rtcm_input.listen'] = relay_listen
            if receiver == 'novatel':
                params['rtcm_input.receiver_stream'] = novatel_relay_output
                params['rtcm_input.receiver_port'] = 'THISPORT'
    elif relay_forward:
        require(receiver == 'rtcm',
                'relay_forward is an rtcm_decoder_node feature (it forwards the '
                'raw source bytes on); got receiver "{}"'.format(receiver))
        params['rtcm_output.destination'] = relay_forward

    if name:
        # One of several rovers: its own namespace, so two receivers of the same
        # make do not collide as /ubx_decoder_node twice. The node name stays
        # the executable's, and the YAML goes in as values (see load_params_dict).
        parameters = ([load_params_dict(params_file, executable, name)] if params_file else []) + [params]
        return Node(package=PACKAGE, executable=executable, name=executable,
                    namespace=name, output='screen', parameters=parameters)

    parameters = []
    if params_file:
        parameters.append(load_params_dict(params_file, executable))
    parameters.append(params)

    # The rover keeps the node name the executable already has, so a
    # --params-file keyed on `ubx_driver_node:` still applies. The base is
    # renamed to avoid a collision, and therefore takes no params file.
    node_name = executable if role == 'rover' else 'base_{}'.format(executable)

    return Node(package=PACKAGE, executable=executable, name=node_name,
                output='screen', parameters=parameters)


def estimator_node(kind, deployment, gnss_solution_topic=None):
    """Build one estimator node with its topics normalized onto the pipeline map.

    `deployment` comes from `resolve_estimator_deployment`; its None entries are
    the values the estimator YAML already carries and must keep.
    """
    require(kind in ESTIMATORS,
            'unknown estimator "{}"; known: {}'.format(kind, '/'.join(sorted(ESTIMATORS))))
    spec = ESTIMATORS[kind]
    executable = spec['executable']

    if spec['needs_gtsam']:
        require(executable_available(executable),
                '"{}" is not in this install. The FGO nodes '
                'are opt-in because they need the pinned GTSAM revision: rebuild '
                'with -DBUILD_FGO_NODES=ON -DGTSAM_DIR=... See the '
                'factor graph section of the '
                'repository README.'.format(executable))
    else:
        require(executable_available(executable),
                '"{}" is not in this install - it is built by default, so the '
                'workspace is probably not sourced or not built.'.format(executable))

    rover = topic_map('rover')
    base = topic_map('base')

    if kind == 'ekf':
        params = {
            'topics.gnss_solution': gnss_solution_topic or SOLUTION_TOPIC,
            'topics.imu_raw': rover['imu_raw'],
            'topics.solution': EKF_SOLUTION_TOPIC,
        }
    else:
        params = {
            'topics.rover_observation': rover['observation'],
            'topics.base_observation': base['observation'],
            'topics.ephemeris': rover['ephemeris'],
            'topics.solution': SOLUTION_TOPIC,
        }
        if spec['needs_imu']:
            params['topics.imu'] = rover['imu_raw']
        postype_key, pos_key = spec['base_keys']
        if deployment['base_position'] is not None:
            params[pos_key] = list(deployment['base_position'])
        if deployment['base_position_type'] is not None:
            params[postype_key] = deployment['base_position_type']

    if spec['needs_imu'] and deployment['lever_arm'] is not None:
        params['lever_arm'] = list(deployment['lever_arm'])

    parameters = [load_params_dict(deployment['params_file'], executable), params]
    return Node(package=PACKAGE, executable=executable, name=executable,
                output='screen', parameters=parameters)


def visualizer_node(sol_topic=SOLUTION_TOPIC, obs_topic='/gnss/observation',
                    nav_topic='/gnss/ephemeris'):
    """The dashboard node, or None when it was not built."""
    if not executable_available('gnss_visualizer'):
        return None
    return Node(
        package=PACKAGE, executable='gnss_visualizer', name='gnss_visualizer',
        output='screen',
        parameters=[{
            'topics.observation': obs_topic,
            'topics.ephemeris': nav_topic,
            'topics.solution': sol_topic,
        }])


# --------------------------------------------------------------------------
# recording
# --------------------------------------------------------------------------

def record_topics(with_base=False, with_imu=False, estimator_topic='',
                  merge_base_ephemeris=False, rovers=None, estimator_kind=""):
    """The topic list a pipeline records, built from what is actually running."""
    if rovers:
        topics = []
        for rover in rovers:
            named = topic_map('rover', name=rover['name'])
            topics += [named['observation'], named['ephemeris'], named['solution']]
            if rover['imu']:
                topics.append(named['imu_raw'])
    else:
        rover = topic_map('rover')
        topics = [rover['observation'], rover['ephemeris'], rover['solution']]
        if with_imu:
            topics.append(rover['imu_raw'])
    if with_base:
        base = topic_map('base', merge_base_ephemeris)
        topics += [base['observation']]
        if not merge_base_ephemeris:
            topics.append(base['ephemeris'])
    if estimator_topic:
        topics.append(estimator_topic)
        if estimator_kind in ("gnss_imu_fgo", "ekf"):
            topics.append(estimator_topic + "_odom")
    # The on-chip RTK pipeline's "estimator" output IS the receiver PVT topic,
    # which is already in the list; `ros2 bag record` must not be given it twice.
    seen = set()
    return [t for t in topics if not (t in seen or seen.add(t))]


def record_action(topics, bag_out, storage='mcap', compress=False):
    """`ros2 bag record` as a launch process.

    The generous sigterm timeout matters: rosbag2 finalizes `metadata.yaml` on
    SIGINT, and a bag killed before that is unreadable. gnss_record.py makes a
    terminal Ctrl-C reach the recorder once (process group + launch forwarding
    would otherwise deliver two SIGINTs and a non-zero exit).
    """
    # Topics are passed positionally, not as --topics: Jazzy deprecates the
    # positional form but still accepts it, while Humble only has it.
    wrapper = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gnss_record.py')
    cmd = [sys.executable, wrapper, 'ros2', 'bag', 'record', '-o', bag_out]
    if storage:
        cmd += ['-s', storage]
    if compress:
        cmd += ['--compression-mode', 'file', '--compression-format', 'zstd']
    cmd += list(topics)
    return ExecuteProcess(cmd=cmd, output='screen', name='bag_record',
                          sigterm_timeout='20', sigkill_timeout='30')


def default_bag_name(pipeline):
    return 'gnss_{}_{}'.format(pipeline, datetime.now().strftime('%Y%m%d_%H%M%S'))


# --------------------------------------------------------------------------
# launch arguments
# --------------------------------------------------------------------------

# Every default is empty on purpose - see the module docstring. The real
# defaults live in the site file, and `FALLBACKS` covers a bare run with none.
# Each one is a one-off override of the site key named in its description.
ARGUMENTS = [
    ('site', '',
     'Which site file to read. Empty uses config/site.local.yaml if present, '
     'else config/site.yaml. A bare name <n> uses config/site_<n>.local.yaml '
     'then config/site_<n>.yaml. A path is used as given.'),

    # what runs
    ('pipeline', '',
     'What to run: logging / rtk / gnss_fgo / gnss_imu_fgo / ekf / onboard_rtk. '
     '[site: run.pipeline]'),
    ('ekf_solution_source', '',
     'pipeline ekf only. rtk = also run real_time_kinematic and feed the EKF its '
     'solution. receiver = feed the EKF the receiver PVT on /gnss/nmea_solution. '
     '[site: run.ekf_solution_source]'),
    ('replay', '',
     'Skip the receiver nodes and let `ros2 bag play` feed the estimator. '
     '[site: run.replay]'),
    ('visualize', '',
     'Run gnss_visualizer. Skipped with a log line when it was not built. '
     '[site: run.visualize]'),

    # the receivers
    ('rover_receiver', '',
     'Rover receiver family: ubx / sbf / novatel / rtcm. [site: rover.receiver]'),
    ('rover_mode', '',
     'decoder = passive, take the receiver as it is configured. '
     'driver = configure the receiver output messages on startup. [site: rover.mode]'),
    ('rover_source', '',
     'Rover stream URI: serial:// tcpcli:// tcpsvr:// ntrip:// file://. '
     'file:// replays a captured raw log through the real decoder. '
     '[site: rover.source]'),
    ('rover_params_file', '',
     'Driver YAML for the rover. Empty uses the packaged '
     'config/<receiver>_driver.yaml. Only read when rover_mode:=driver. '
     '[site: rover.params_file]'),
    ('base_receiver', '',
     'Base receiver family: rtcm / ubx / sbf / novatel. [site: base.receiver]'),
    ('base_source', '',
     'Base stream URI, e.g. ntrip://user:pass@caster:2101/MOUNT. '
     'Empty runs no base node. [site: base.source]'),
    ('merge_base_ephemeris', '',
     'Publish base ephemeris onto /gnss/ephemeris instead of '
     '/base/gnss/ephemeris. For a rover stream that carries no navigation data. '
     '[site: base.merge_ephemeris]'),
    ('base_reference_week', '',
     'base receiver rtcm only. GPS week near the first observation of a '
     'historical RTCM log (RTCM MSM carries no week); 0 or empty uses the host '
     'date. [site: base.reference_week]'),
    ('base_reference_tow', '',
     'With base_reference_week: time of week [s] near the first observation. '
     '[site: base.reference_tow]'),
    ('use_imu', '',
     'Enable the raw IMU topic (/gnss/imu/data_raw). With rover_mode:=driver '
     'the driver is told to turn the messages on; with rover_mode:=decoder they '
     'must already be enabled on the receiver. [site: imu.enabled]'),
    ('use_gps_timestamp', '',
     'Stamp message headers with UTC(GPST) instead of the host clock. '
     'Recommended whenever the IMU is used - it puts the IMU and the GNSS '
     'topics of one node on the same time base. [site: imu.use_gps_timestamp]'),

    # the estimator
    ('params_file', '',
     'Estimator YAML. Empty uses the packaged config for the pipeline\'s '
     'estimator. [site: estimator.params_file]'),
    ('base_position', '',
     'Base antenna position "lat,lon,height". Empty keeps whatever the '
     'estimator YAML carries. [site: estimator.base_position]'),
    ('base_position_type', '',
     'llh (degrees, degrees, metres) or ecef (metres). '
     '[site: estimator.base_position_type]'),
    ('lever_arm', '',
     'IMU body to GNSS antenna "x,y,z", metres. Empty keeps the estimator YAML. '
     'Use IMU body axes after imu_orientation. EKF applies this only with '
     'output_reference_frame=imu. [site: estimator.lever_arm]'),

    # on-chip RTK
    ('relay_port', '',
     'pipeline onboard_rtk only. TCP port the rover node listens on for '
     'corrections; empty or 0 uses 5556 (ubx) / 5557 (sbf) / 5558 (novatel). '
     '[site: relay.port]'),
    ('novatel_relay_output', '',
     'pipeline onboard_rtk with NovAtel only, required there: a SECOND device '
     'for the dedicated RTCMV3 port, e.g. "serial:///dev/ttyUSB2:230400". '
     '[site: relay.novatel_output]'),

    # recording
    ('record', '',
     'Record the pipeline to a rosbag. [site: record.enabled]'),
    ('bag_out', '',
     'Bag output path. Empty auto-names gnss_<pipeline>_<YYYYmmdd_HHMMSS>. '
     '[site: record.output]'),
    ('bag_storage', '',
     'rosbag2 storage backend: mcap / sqlite3. mcap works on both Humble and '
     'Jazzy here (rosbag2_storage_mcap is an exec dependency). '
     '[site: record.storage]'),
    ('bag_compress', '',
     'Compress the bag with zstd (--compression-mode file). [site: record.compress]'),
]

ARGUMENT_NAMES = [name for name, _, _ in ARGUMENTS]


def launch_arguments():
    """DeclareLaunchArgument for every argument; all default to empty."""
    from launch.actions import DeclareLaunchArgument
    return [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in ARGUMENTS
    ]


# --------------------------------------------------------------------------
# composition helpers used by the pipelines
# --------------------------------------------------------------------------

def _mask_uri(uri):
    """Hide NTRIP userinfo so a log or screenshot never leaks a password."""
    value = str(uri or '')
    scheme = value.find('://')
    start = scheme + 3 if scheme >= 0 else 0
    at = value.rfind('@')
    return value[:start] + '***' + value[at:] if at >= start else value


def summary_actions(settings, extra=None):
    """One startup block naming every effective value and where it came from.

    Settings now arrive from three places, so without this the indirection is
    only debuggable by reading three files. It also replaces the safety that
    making `base_position` mandatory used to give.
    """
    from launch.actions import LogInfo

    def line(label, value, source):
        return '  {:<16}{:<46} ({})'.format(label, value, source)

    def merged_source(*names):
        # A row can combine fields that came from different places - receiver
        # from the command line, source from the site file. Naming only one of
        # them would point at the wrong file when something looks wrong.
        seen = []
        for name in names:
            origin = settings.source(name)
            if origin not in seen:
                seen.append(origin)
        return ', '.join(seen)

    pipeline = settings['pipeline']
    rows = [
        'site: {}'.format(settings.site_path or 'none - command line and defaults only'),
        line('pipeline:', '{} - {}'.format(pipeline, PIPELINES[pipeline]['summary']),
             settings.source('pipeline')),
    ]
    if settings.get('replay'):
        # The receiver settings are resolved but unused, and printing them would
        # suggest a device is being opened when it is not.
        rows.append(line('input:', 'ros2 bag play (replay - no receiver nodes)',
                         settings.source('replay')))
    elif settings.rovers:
        for rover in settings.rovers:
            rows.append(line('{}:'.format(rover['name']), '{} / {} / {}{}'.format(
                rover['receiver'], rover['mode'], _mask_uri(rover['source']),
                ' + imu' if rover['imu'] else ''), 'site file, rovers'))
        if settings.get('base_source'):
            rows.append(line('base:', '{} / {}'.format(
                settings.get('base_receiver'), _mask_uri(settings.get('base_source'))),
                merged_source('base_receiver', 'base_source')))
    else:
        rows.append(line('rover:', '{} / {} / {}'.format(
            settings.get('rover_receiver'), settings.get('rover_mode'),
            _mask_uri(settings.get('rover_source')) or '(unset)'),
            merged_source('rover_receiver', 'rover_mode', 'rover_source')))
        if settings.get('base_source'):
            rows.append(line('base:', '{} / {}'.format(
                settings.get('base_receiver'), _mask_uri(settings.get('base_source'))),
                merged_source('base_receiver', 'base_source')))
    if settings.rovers:
        # Per-rover IMU is shown on each rover's row; the time base is shared.
        rows.append(line('gps timestamps:', 'on' if settings.get('use_gps_timestamp') else 'off',
                         settings.source('use_gps_timestamp')))
    elif settings.get('use_imu'):
        rows.append(line('imu:', 'on, gps timestamps {}'.format(
            'on' if settings.get('use_gps_timestamp') else 'off'),
            merged_source('use_imu', 'use_gps_timestamp')))
    for label, value, source in (extra or []):
        rows.append(line(label, value, source))

    return [LogInfo(msg='\n'.join(rows))]


def estimator_summary_rows(deployment):
    """The `extra` rows for `summary_actions` describing an estimator."""
    # 12 significant digits: this line is what the operator checks a base
    # position against, and 8 would round 35.66633426 to 35.666334 - a third of
    # a metre, silently, in the one place meant to catch exactly that.
    def triplet(values):
        return ' '.join('{:.12g}'.format(float(v)) for v in values)

    rows = [('config:', os.path.basename(deployment['params_file']),
             deployment['params_file_source'])]
    if deployment['base_position_source'] is not None:
        rows.append(('base position:', triplet(deployment['base_position_value'] or []),
                     deployment['base_position_source']))
    if deployment['lever_arm_source'] is not None:
        rows.append(('lever arm:', triplet(deployment['lever_arm_value'] or [0.0, 0.0, 0.0]),
                     deployment['lever_arm_source']))
    return rows


def base_reference_time(settings):
    """(week, tow) for a historical RTCM base, or None when no week is set."""
    week = settings.get('base_reference_week')
    if not week:
        require(not settings.get('base_reference_tow'),
                'base reference_tow is set without reference_week')
        return None
    return int(week), float(settings.get('base_reference_tow') or 0.0)


def source_nodes(settings, relay_listen='', relay_forward='',
                 novatel_relay_output=''):
    """The rover node, and the base node when base_source is set.

    Returns (nodes, has_base).
    """
    if settings.rovers:
        return _multi_rover_nodes(settings)

    receiver = settings.get('rover_receiver')
    mode = settings.get('rover_mode')
    validate_receiver(receiver, mode, 'rover')

    require(not (settings.get('use_imu') and receiver == 'rtcm'),
            'IMU is enabled but the rover receiver is rtcm - RTCM3 carries '
            'observations and ephemerides only. Use the receiver\'s own binary '
            'protocol for the {} pipeline.'.format(settings['pipeline']))

    params_file = settings.get('rover_params_file')
    if mode == 'driver' and not params_file:
        params_file = config(RECEIVERS[receiver]['driver_yaml'])

    nodes = [receiver_node(
        'rover', receiver, mode, settings.get('rover_source'),
        params_file=params_file,
        relay_listen=relay_listen,
        use_imu=settings.get('use_imu'),
        use_gps_timestamp=(settings['use_gps_timestamp']
                           if settings.specified('use_gps_timestamp') or settings.get('use_gps_timestamp') else None),
        novatel_relay_output=novatel_relay_output)]

    has_base = bool(settings.get('base_source'))
    if has_base:
        nodes.append(receiver_node(
            'base', settings.get('base_receiver'), 'decoder',
            settings.get('base_source'),
            relay_forward=relay_forward, reference_time=base_reference_time(settings),
            use_gps_timestamp=(settings['use_gps_timestamp']
                               if settings.specified('use_gps_timestamp') or settings.get('use_gps_timestamp') else None),
            merge_base_ephemeris=settings.get('merge_base_ephemeris')))
    return nodes, has_base


def _multi_rover_nodes(settings):
    """One node per entry of a rovers list, plus the shared base."""
    gps_ts = (settings['use_gps_timestamp']
              if settings.specified('use_gps_timestamp') or settings.get('use_gps_timestamp') else None)
    nodes = []
    for rover in settings.rovers:
        params_file = rover['params_file']
        if rover['mode'] == 'driver' and not params_file:
            params_file = config(RECEIVERS[rover['receiver']]['driver_yaml'])
        nodes.append(receiver_node(
            'rover', rover['receiver'], rover['mode'], rover['source'],
            params_file=params_file, use_imu=rover['imu'],
            use_gps_timestamp=gps_ts, name=rover['name']))

    # One base for everyone: it is the same correction stream, and a second
    # login to the same NTRIP mountpoint is often refused by the caster anyway.
    has_base = bool(settings.get('base_source'))
    if has_base:
        nodes.append(receiver_node(
            'base', settings.get('base_receiver'), 'decoder',
            settings.get('base_source'), use_gps_timestamp=gps_ts,
            reference_time=base_reference_time(settings)))
    return nodes, has_base


def tail_actions(settings, has_base, estimator_topic=''):
    """Recording and visualization, shared by every pipeline."""
    from launch.actions import LogInfo
    actions = []

    if settings.get('visualize'):
        if settings.rovers:
            # The dashboard shows one receiver; show the first and say so.
            first = settings.rovers[0]['name']
            named = topic_map('rover', name=first)
            node = visualizer_node(sol_topic=named['solution'],
                                   obs_topic=named['observation'],
                                   nav_topic=named['ephemeris'])
            if node is not None:
                actions.append(LogInfo(msg='gnss_visualizer shows rover "{}" - the '
                                           'first in the rovers list.'.format(first)))
        else:
            node = visualizer_node(sol_topic=estimator_topic or topic_map('rover')['solution'])
        if node is None:
            actions.append(LogInfo(msg='gnss_visualizer is not in this install '
                                       '(it needs OpenCV, cv_bridge and '
                                       'image_transport) - skipping it.'))
        else:
            actions.append(node)

    if settings.get('record'):
        bag_out = settings.get('bag_out') or default_bag_name(settings['pipeline'])
        topics = record_topics(with_base=has_base,
                               with_imu=settings.get('use_imu'),
                               estimator_topic=estimator_topic,
                               merge_base_ephemeris=settings.get('merge_base_ephemeris'),
                               rovers=settings.rovers,
                               estimator_kind=settings["pipeline"])
        actions.append(LogInfo(msg='Recording {} topic(s) to {}'.format(
            len(topics), bag_out)))
        actions.append(record_action(topics, bag_out,
                                     storage=settings.get('bag_storage'),
                                     compress=settings.get('bag_compress')))
    return actions
