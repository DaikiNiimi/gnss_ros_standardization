#!/usr/bin/env bash
# Run in a clean shell after sourcing the ROS underlay and the intended install.
# Usage: bash test/check_install.sh <package-prefix> [new-output-directory]
set -euo pipefail
package_prefix="$(realpath "${1:?package install prefix is required}")"
fixture_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/install_consumer" && pwd)"
consumer_work="${2:-$(mktemp -d /tmp/gnss-install-consumer.XXXXXX)}"
mkdir -p "$consumer_work"
consumer_work="$(realpath "$consumer_work")"
if [[ -e "$consumer_work/source" || -e "$consumer_work/build" ]]; then
  echo "Use a fresh output directory: $consumer_work" >&2
  exit 2
fi
cp -r "$fixture_dir" "$consumer_work/source"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="$consumer_work/ros_logs"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-231}"
# Optional FGO executables must resolve their pinned GTSAM after installation,
# including when that dependency was built inside the package source tree.
for fgo_name in gnss_fgo gnss_imu_fgo gnss_imu_fgo_native; do
  fgo_binary="$package_prefix/lib/gnss_ros_standardization/$fgo_name"
  if [[ -f "$fgo_binary" ]]; then
    ldd "$fgo_binary" > "$consumer_work/${fgo_name}_libraries.txt"
    python3 - "$consumer_work/${fgo_name}_libraries.txt" <<'PY'
from pathlib import Path
import sys
text = Path(sys.argv[1]).read_text()
assert "not found" not in text, text
PY
  fi
done
# Pin package discovery as well as checking the runtime module/library paths.
cmake -S "$consumer_work/source" -B "$consumer_work/build" \
  -Dgnss_ros_standardization_DIR="$package_prefix/share/gnss_ros_standardization/cmake" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build "$consumer_work/build" --parallel 2
ldd "$consumer_work/build/install_consumer" > "$consumer_work/linked_libraries.txt"
python3 - "$package_prefix" "$consumer_work/linked_libraries.txt" \
  "$consumer_work/build/installed_core.txt" <<'PY'
from pathlib import Path
import re
import sys
prefix = Path(sys.argv[1])
text = Path(sys.argv[2]).read_text()
assert "not found" not in text, text
libraries = re.findall(r"(libgnss_\S+) => (\S+)", text)
kind, core = Path(sys.argv[3]).read_text().splitlines()
assert Path(core).resolve().is_relative_to(prefix) and Path(core).is_file(), core
if kind == "SHARED_LIBRARY":
    assert any(name == "libgnss_utils.so" for name, _ in libraries), text
assert all(Path(path).resolve().is_relative_to(prefix) for _, path in libraries), libraries
PY
python3 "$consumer_work/source/check.py" \
  --prefix "$package_prefix" --executable "$consumer_work/build/install_consumer"
echo "Installed consumer evidence: $consumer_work"
