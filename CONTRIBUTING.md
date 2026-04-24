# Contributing to gnss_ros_standardization

Thank you for your interest in contributing. Please read the guidelines below before opening an issue or pull request.

## Development Environment

```bash
# Clone with submodules
git clone --recursive https://github.com/DaikiNiimi/gnss_ros_standardization.git
cd gnss_ros_standardization

# Install dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build (debug mode recommended during development)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install/setup.bash
```

## Branch Conventions

| Branch | Purpose |
|---|---|
| `main` | Stable releases only |
| `develop` | Integration branch for new features |
| `feature/<name>` | Individual feature branches |
| `fix/<name>` | Bug fix branches |

Base new work off `develop`. Open PRs targeting `develop`.

## Commit Message Format

```
<type>: <short summary in imperative mood>

<optional body: explain why, not what>
```

Types: `feat`, `fix`, `docs`, `refactor`, `test`, `ci`, `chore`

Examples:
```
feat: add Septentrio NMEA GnssSolution output
fix: remove debug cerr from ObsWriter destructor
docs: document GnssSolution coordinate frame conventions
```

## Code Style

- C++17, follow the style already present in the codebase.
- Compiler warnings are enabled (`-Wall -Wextra -Wpedantic`). PRs must build without new warnings.
- All source files, comments, log messages, and `.msg` field descriptions must be in **English**.
- No debug `std::cerr` or `printf` in production paths; use `RCLCPP_INFO/WARN/ERROR`.

## Adding a New Receiver

1. Add protocol constants to `include/gnss_ros_standardization/<vendor>_protocol.hpp`.
2. Implement a decoder node under `src/decoders/`.
3. Implement a driver node under `src/drivers/`.
4. Add a sample YAML config under `config/`.
5. Register executables in `CMakeLists.txt`.
6. Update the **Supported Receivers** table in `README.md`.

## Message Definitions

When adding or modifying `.msg` files:
- Include the unit and coordinate frame in a comment for every numeric field.
- Use SI units (meters, radians, seconds, m/s).

## Reporting Issues

Use the [GitHub issue tracker](https://github.com/DaikiNiimi/gnss_ros_standardization/issues).  
Choose the appropriate template (Bug report or Feature request).
