#!/usr/bin/env bash
# Build the official GTSAM (borglab/gtsam develop) revision the FGO examples are
# tested with. It provides the explicit Logmap IMU error mode they use.
set -euo pipefail
task_root=${1:?Usage: build_fgo_dependency.sh BUILD_ROOT [JOBS]}
task_jobs=${2:-2}
mkdir -p "$task_root"
task_root=$(cd "$task_root" && pwd)
task_commit=6f31b6df3b56b60ec82d3218b47a2163bb41dd34
task_archive="$task_root/gtsam-$task_commit.tar.gz"
if [[ ! -f "$task_archive" ]]; then
  curl --fail --location --retry 3 "https://codeload.github.com/borglab/gtsam/tar.gz/$task_commit" --output "$task_archive"
fi
printf '%s  %s\n' 597d0ce39128c1423475ab11f4eb759bc4f6e6899db97f5afe355c11b3eed825 "$task_archive" | sha256sum --check
if [[ ! -d "$task_root/gtsam-$task_commit" ]]; then
  tar -xzf "$task_archive" -C "$task_root"
fi
cmake -S "$task_root/gtsam-$task_commit" -B "$task_root/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$task_root/install" \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TIMING_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_BUILD_WITH_CCACHE=OFF
CCACHE_DISABLE=1 cmake --build "$task_root/build" -j "$task_jobs"
cmake --install "$task_root/build"
printf 'GTSAM_DIR=%s/install/lib/cmake/GTSAM\n' "$task_root"
