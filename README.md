# IR100 MoveIt Whole-Body IK

This repository is a self-contained ROS Noetic catkin workspace for the IR100
mobile manipulator. It owns the robot descriptions, MoveIt configuration,
PCD map, and MoveIt whole-body IK services.

## Bundled runtime assets

- `src/ir100_moveit`: IR100 MoveIt configuration and launch files.
- `src/ir100_description`: IR100 URDF/Xacro and base/gripper meshes.
- `src/dobot_description`: Dobot CR10 resources referenced by the IR100 Xacro.
- `maps/scans_voxel_5cm_xyz.pcd`: static collision map.

All bundled source assets are kept in this repository so the final system does
not require the original REMAIN-Planner checkout. Generated catkin products
are intentionally ignored.

## Demo

![RViz whole-body IK demonstration](demo.gif)

## Build, launch, and verify

The host needs ROS Noetic with MoveIt, RViz, PCL, and OctoMap installed.

```bash
./scripts/build.sh
source devel/setup.bash
roslaunch moveit_whole_body_ik whole_body_ik.launch
```

The launch starts MoveIt, collision-map loading, `/moveit_ik/solve`,
`/whole_body_moveit_ik/solve`, robot-state publishing, and RViz. The RViz
panel reads its bundled sample CSV by default.

Run the isolated smoke test with:

```bash
./scripts/verify.sh
```

It starts a separate ROS master on port `11321` by default, so it does not
interfere with an existing ROS session. The smoke test uses the bundled small
PCD; the normal launch uses the full collision map. Set `ROS_PORT` to use
another free port.
