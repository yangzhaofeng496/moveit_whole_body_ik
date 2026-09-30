#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "ROS Noetic was not found at /opt/ros/noetic/setup.bash" >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash
set -u
if [[ ! -e "${workspace_root}/src/CMakeLists.txt" ]]; then
  (cd "${workspace_root}" && catkin_init_workspace src)
fi

cd "${workspace_root}"
catkin_make "$@"
