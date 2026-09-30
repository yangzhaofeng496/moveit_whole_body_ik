# IR100 MoveIt Whole-Body IK

This repository is a self-contained ROS Noetic catkin workspace for the IR100
mobile manipulator. It owns the robot descriptions, MoveIt configuration,
PCD map, and the MoveIt whole-body IK package that will be added under `src/`.

## Bundled runtime assets

- `src/ir100_moveit`: IR100 MoveIt configuration and launch files.
- `src/ir100_description`: IR100 URDF/Xacro and base/gripper meshes.
- `src/dobot_description`: Dobot CR10 resources referenced by the IR100 Xacro.
- `maps/scans_voxel_5cm_xyz.pcd`: static collision map.

All bundled source assets are kept in this repository so the final system does
not require the original REMAIN-Planner checkout. Generated catkin products
are intentionally ignored.
