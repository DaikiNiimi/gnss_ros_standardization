#!/usr/bin/env python3
"""Every pipeline, chosen by the site file.

    ros2 launch gnss_ros_standardization pipeline.launch.py

`run.pipeline` in config/site.yaml decides what runs - logging, rtk, gnss_fgo,
gnss_imu_fgo, ekf or onboard_rtk - and the rest of the site file says which
receiver, which base stream, and so on. `pipeline:=<name>` (or any other
argument) overrides the site file for one run. See launch/README.md.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, TimerAction

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gnss_pipeline_common import (  # noqa: E402
    EKF_SOLUTION_TOPIC, RECEIVERS, SOLUTION_TOPIC, PipelineError, config,
    base_reference_time, estimator_node, estimator_summary_rows, launch_arguments, receiver_node,
    playback_actions, require, resolve_estimator_deployment, resolve_settings, source_nodes,
    stream_consumers, summary_actions, tail_actions, take_playbacks, topic_map,
)

# The receiver needs the relay server listening before the correction source
# dials in.
RELAY_START_DELAY_S = 3.0


# Each builder returns (actions, summary rows, has_base, topic to record as the
# pipeline's output). Recording, visualization and the startup summary are added
# around them by `_setup`, identically for every pipeline.


def _live_sources(settings, needs_base):
    """Receiver nodes, or none at all when `ros2 bag play` is the input."""
    if settings.get('replay'):
        return [], needs_base
    if needs_base:
        require(bool(settings.get('base_source')),
                'no base stream: the {} pipeline needs base observations. Set '
                'base.source in the site file, pass base_source:=..., or set '
                'run.replay: true to be fed by `ros2 bag play`.'
                .format(settings['pipeline']))
    return source_nodes(settings)


def build_logging(settings):
    require(not settings.get('replay'),
            'the logging pipeline has nothing to log under replay - the input '
            'already is a bag. Set run.replay: false, or pick an estimator '
            'pipeline to replay into.')
    nodes, has_base = source_nodes(settings)
    return nodes, [], has_base, ''


def build_gnss(settings):
    """rtk and gnss_fgo: rover + base observations into one estimator."""
    kind = settings['pipeline']
    nodes, has_base = _live_sources(settings, needs_base=True)
    deployment = resolve_estimator_deployment(kind, settings)
    return (nodes + [estimator_node(kind, deployment)],
            estimator_summary_rows(deployment), has_base, SOLUTION_TOPIC)


def build_gnss_imu(settings):
    """gnss_imu_fgo (tight) and ekf (loose)."""
    kind = settings['pipeline']
    source = settings.get('ekf_solution_source')

    require(settings.get('use_imu'),
            'the IMU is disabled on the {} pipeline. Set imu.enabled: true in '
            'the site file (or leave it unset - this pipeline turns it on by '
            'itself), pass use_imu:=true, or pick rtk for a GNSS-only run.'
            .format(kind))
    if kind == 'gnss_imu_fgo':
        require(settings.get('use_gps_timestamp'),
                'gnss_imu_fgo requires sensor timestamps. Set '
                'imu.use_gps_timestamp: true in the site file; host receipt '
                'time is not an IMU epoch.')
    if kind == 'ekf':
        require(source in ('rtk', 'receiver'),
                'run.ekf_solution_source "{}" must be "rtk" or "receiver"'.format(source))

    # The EKF can run off the receiver's own PVT, in which case nothing in this
    # repository needs base observations or a base position.
    on_receiver_pvt = kind == 'ekf' and source == 'receiver'
    actions, has_base = _live_sources(settings, needs_base=not on_receiver_pvt)
    actions = list(actions)

    if not settings.get('use_gps_timestamp'):
        actions.append(LogInfo(
            msg='GPS timestamps are off - the IMU and GNSS topics stay on the '
                'host clock. That is a consistent time base and works, but '
                'imu.use_gps_timestamp: true is what puts them on UTC(GPST); '
                'see "Message time base" in src/decoders/README.md.'))

    rows = []
    if kind == 'ekf' and not on_receiver_pvt:
        # Three stages: source nodes -> RTK -> EKF. The RTK output is normalized
        # onto /gnss/solution, which is exactly what the EKF is pointed at.
        # params_file belongs to the EKF here, so the RTK stage keeps its own.
        rtk = resolve_estimator_deployment('rtk', settings, own_params_file=False)
        actions.append(estimator_node('rtk', rtk))
        rows += estimator_summary_rows(rtk)

    deployment = resolve_estimator_deployment(kind, settings)
    gnss_solution_topic = (topic_map('rover')['solution'] if on_receiver_pvt
                           else SOLUTION_TOPIC)
    actions.append(estimator_node(kind, deployment,
                                  gnss_solution_topic=gnss_solution_topic))
    rows += estimator_summary_rows(deployment)

    output = EKF_SOLUTION_TOPIC if kind == 'ekf' else SOLUTION_TOPIC
    return actions, rows, has_base, output


def build_onboard_rtk(settings):
    """The receiver computes RTK; this only relays the base stream to it."""
    receiver = settings.get('rover_receiver')
    mode = settings.get('rover_mode')
    novatel_relay_output = settings.get('novatel_relay_output') or ''

    require(not settings.get('replay'),
            'onboard_rtk cannot replay: the receiver itself computes the '
            'solution, so it has to be connected. Replay the bag into the rtk '
            'pipeline to compare against the on-chip solution instead.')
    require(receiver != 'rtcm',
            'the rover receiver is rtcm, which cannot do on-chip RTK: '
            'rtcm_decoder_node is the correction *source* here, not the rover. '
            'Set rover.receiver to the make of the receiver that should compute '
            'RTK.')
    require(bool(settings.get('base_source')),
            'no base stream: it is the correction stream that gets relayed. Set '
            'base.source in the site file, e.g. '
            '"ntrip://${NTRIP_USER}:${NTRIP_PASSWORD}@caster:2101/MOUNT".')
    require(settings.get('base_receiver') == 'rtcm',
            'base.receiver must be "rtcm" for onboard_rtk - the relay forwards '
            'RTCM bytes verbatim.')

    port = settings.get('relay_port') or 0
    port_source = settings.source('relay_port')
    if port == 0:
        # 0 means "whatever this receiver's documented port is", so say that
        # rather than crediting the file that merely left it at 0.
        port = RECEIVERS[receiver]['relay_port']
        port_source = '{} default'.format(receiver)

    if receiver == 'novatel':
        require(bool(novatel_relay_output),
                'NovAtel needs a dedicated correction port: RTCMV3 receive '
                'cannot share the NOVATEL log port. Set relay.novatel_output to '
                'a second device, e.g. "serial:///dev/ttyUSB2:230400".')
        require(novatel_relay_output != settings.get('rover_source'),
                'relay.novatel_output must differ from rover.source - they are '
                'two separate ports on the receiver.')

    params_file = settings.get('rover_params_file')
    if mode == 'driver' and not params_file:
        params_file = config(RECEIVERS[receiver]['driver_yaml'])

    gps_ts = settings['use_gps_timestamp'] if settings.specified('use_gps_timestamp') or settings.get('use_gps_timestamp') else None
    rover = receiver_node(
        'rover', receiver, mode, settings.get('rover_source'),
        params_file=params_file,
        relay_listen='tcpsvr://:{}'.format(port),
        use_imu=settings.get('use_imu'),
        use_gps_timestamp=gps_ts,
        novatel_relay_output=novatel_relay_output)

    # Decoded to /base/gnss/observation as well as forwarded, so a bag recorded
    # here replays through the rtk pipeline - the receiver's on-chip solution
    # against this package's own RTK, on identical input.
    correction_source = receiver_node(
        'base', 'rtcm', 'decoder', settings.get('base_source'),
        relay_forward='tcpcli://127.0.0.1:{}'.format(port),
        use_gps_timestamp=gps_ts,
        merge_base_ephemeris=settings.get('merge_base_ephemeris'),
        reference_time=base_reference_time(settings))

    actions = [
        rover,
        LogInfo(msg='RTCM relay on port {}; starting the correction source in '
                    '{:.0f}s so the server is listening first.'.format(
                        port, RELAY_START_DELAY_S)),
        TimerAction(period=RELAY_START_DELAY_S, actions=[correction_source]),
    ]
    # The solution here is the receiver's own, on the PVT topic.
    return (actions, [('relay port:', str(port), port_source)], True,
            topic_map('rover')['solution'])


BUILDERS = {
    'logging': build_logging,
    'rtk': build_gnss,
    'gnss_fgo': build_gnss,
    'gnss_imu_fgo': build_gnss_imu,
    'ekf': build_gnss_imu,
    'onboard_rtk': build_onboard_rtk,
}


def _setup(context, *args, **kwargs):
    settings = resolve_settings(context)
    take_playbacks()
    actions, rows, has_base, output = BUILDERS[settings['pipeline']](settings)
    playbacks = playback_actions(take_playbacks(),
                                 stream_consumers(settings, has_base, estimator_topic=output))
    return (summary_actions(settings, extra=rows)
            + actions
            + tail_actions(settings, has_base, estimator_topic=output)
            + playbacks)


def _guarded(context, *args, **kwargs):
    try:
        return _setup(context, *args, **kwargs)
    except PipelineError as exc:
        raise RuntimeError('pipeline.launch.py: {}'.format(exc))


def generate_launch_description():
    return LaunchDescription(launch_arguments() + [OpaqueFunction(function=_guarded)])
