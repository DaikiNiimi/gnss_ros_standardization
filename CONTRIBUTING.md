# Contributing to gnss_ros_standardization

Thank you for your interest in this package. This document explains how to get
help, report problems and propose changes. For installation and usage, start with
the [README](README.md).

## Getting help

Ask questions about installation, configuration or results by opening an issue in
[GitHub Issues](https://github.com/DaikiNiimi/gnss_ros_standardization/issues).
Start the title with `Question:` and include the information listed under
[Reporting issues](#reporting-issues) that applies to your question. Searching the
existing issues first often gives the answer faster.

For matters that should not be public, write to the maintainer at the address in
the [README](README.md#license).

## Reporting issues

Open an issue at
<https://github.com/DaikiNiimi/gnss_ros_standardization/issues> and include:

- ROS 2 distribution and Ubuntu version, and the commit or release you built
- Receiver model and firmware version, and the connection (serial, TCP, NTRIP, file)
- The exact command, launch file or parameter file you used
- What you expected and what happened, with the console output attached as a file
- Whether you can share a short binary log, RINEX file or bag that reproduces the
  problem (attach it compressed, or give a link if it is large)

Before you post, remove credentials: an NTRIP user name and password are part of
the stream address (`ntrip://user:password@caster:2101/MOUNT`) and therefore appear
in commands, parameter files and console output. Logs and bags also contain the
position of your receiver; share them only if that is acceptable to you.

For positioning results (SPP, RTK, EKF, FGO), also give the fix rate and the
error you observed, the reference you compared against, and the location and
duration of the data. If the problem also appears on a run of
[PPC-Dataset-ROS2](https://github.com/DaikiNiimi/PPC-Dataset-ROS2), name the run;
it lets others reproduce the problem without your data.

Report a security problem by e-mail to the maintainer instead of a public issue.

## Contributing changes

Changes are accepted as pull requests from a fork; the repository takes no direct
pushes.

1. For changes beyond small fixes (new receiver, message field, parameter or node),
   open an issue first to agree on the approach. Message definitions are shared by
   all receivers and by recorded bags, so changes to them need discussion.
2. Fork the repository on GitHub, clone your fork with its RTKLIB submodule and
   create a branch from `main`:

   ```bash
   git clone --recursive https://github.com/<your-account>/gnss_ros_standardization.git
   cd gnss_ros_standardization
   git checkout -b my-change main
   ```

3. Install the dependencies as in the [README](README.md#installation), then build
   and run the tests as described in [test/README.md](test/README.md):

   ```bash
   colcon build --cmake-args -DBUILD_TESTING=ON
   source install/setup.bash
   colcon test --packages-select gnss_ros_standardization
   colcon test-result --verbose
   ```

   When you change the FGO nodes, build GTSAM first and add `-DBUILD_FGO_NODES=ON`
   and `-DGTSAM_DIR=...` as in
   [FGO nodes](README.md#fgo-nodes-optional-need-gtsam); without them the FGO code
   and its tests are not built.
4. Add or update tests for the behavior you change. Update the README of the
   affected component, [docs/PARAMETER_NAMES.md](docs/PARAMETER_NAMES.md) when a
   parameter is added or renamed, and
   [docs/TOPIC_DATA_CONTRACT.md](docs/TOPIC_DATA_CONTRACT.md) when a topic or the
   content of a message changes.
5. Commit, push the branch to your fork and open a pull request against `main`:

   ```bash
   git push origin my-change
   ```

   Describe the change and how you tested it (receiver or data used). Keep one
   topic per pull request, and do not commit bags or receiver logs; link them
   instead.
6. Continuous integration builds and tests the pull request, including the FGO
   nodes, on ROS 2 Humble and Jazzy. GitHub may hold the run of a first-time
   contributor until the maintainer approves it.

A new receiver or file format is supported by adding an input node that publishes
the existing messages; positioning nodes should not need changes. Receiver-specific
code stays in the decoders, drivers and converters.

By contributing, you agree that your contribution is licensed under the
[MIT License](LICENSE) of this repository. Code derived from third parties keeps
its own license: the RTKLIB sources (BSD-2-Clause) and the parts of
`src/positioning/fgo` derived from inuex35/tightly-coupled-gnss-imu-fgo
(BSD-3-Clause), as stated in [LICENSE](LICENSE).
