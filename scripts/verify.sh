#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_port="${ROS_PORT:-11321}"
export ROS_MASTER_URI="http://127.0.0.1:${ros_port}"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "ROS Noetic was not found at /opt/ros/noetic/setup.bash" >&2
  exit 1
fi

cleanup() {
  [[ -n "${launch_pid:-}" ]] && kill "${launch_pid}" 2>/dev/null || true
  [[ -n "${roscore_pid:-}" ]] && kill "${roscore_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

source /opt/ros/noetic/setup.bash
set -u
"${workspace_root}/scripts/build.sh" --pkg moveit_whole_body_ik -j4
source "${workspace_root}/devel/setup.bash"

roscore -p "${ros_port}" >"${workspace_root}/.verify_roscore.log" 2>&1 &
roscore_pid=$!
sleep 2

roslaunch moveit_whole_body_ik whole_body_ik.launch use_rviz:=false \
  pcd_file:="${workspace_root}/maps/smoke_test.pcd" \
  >"${workspace_root}/.verify_launch.log" 2>&1 &
launch_pid=$!

for _ in $(seq 1 45); do
  if rosservice list | grep -qx /whole_body_moveit_ik/solve && \
     grep -q 'Loaded .* cropped PCD points into OctoMap' "${workspace_root}/.verify_launch.log"; then
    break
  fi
  sleep 1
done

rosservice list | grep -qx /whole_body_moveit_ik/solve
rosservice list | grep -qx /moveit_ik/solve
grep -q 'Loaded .* cropped PCD points into OctoMap' "${workspace_root}/.verify_launch.log"

rosservice call /whole_body_moveit_ik/solve "{target_pose:
 {position: {x: 1.5, y: 0.8, z: 1.5},
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
 reference_frame: 'world', timeout_sec: 2.0,
 base_samples: 12, base_radius: 1.0, base_yaw_step: 0.5236}" \
  >"${workspace_root}/.verify_ik_response.yaml"

timeout 10 rostopic echo -n 1 /whole_body_ik/pcd_cloud \
  >"${workspace_root}/.verify_pcd_cloud.yaml"
grep -q 'width: [1-9]' "${workspace_root}/.verify_pcd_cloud.yaml"

echo "Verification passed: IK services started and the PCD OctoMap/cloud loaded."
