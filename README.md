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

<img src="demo.gif" alt="RViz whole-body IK demonstration" width="100%">

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

## TODO

- [ ] Replace the CSV waypoints. / 更换 CSV 点位。

  The RViz `CSV Whole-Body IK` panel loads
  `src/moveit_whole_body_ik/data/template_current_HXD1D_ee_pose_moveit_fk.csv`
  by default. To use a different set of points:

  1. **Edit the CSV** (or add a new `.csv` file). The header row must contain the
     columns `ee_x, ee_y, ee_z, ee_qx, ee_qy, ee_qz, ee_qw` in any order;
     missing columns make loading fail. Other columns (e.g. `项点编号`, `ee_reference`)
     are ignored by the panel.
  2. **Each data row is one IK target.** `ee_x/ee_y/ee_z` are the end-effector
     position and `ee_qx..ee_qw` its quaternion in the map frame. Rows whose
     `项点编号` is `0` (types such as `TEMP` / `TEMP_NO_MOVE_BASE`) are still
     listed as selectable poses, so remove them if they are not needed.
  3. **Point the panel at the file** in one of three ways:
     - Click **Browse** in the RViz panel and select the new CSV, then **Load**;
     - Set `csv_path` under the `CSV Whole-Body IK` panel in
       `src/moveit_whole_body_ik/launch/moveit_whole_body_ik.rviz`
       (an empty string means "use the bundled default");
     - Or edit the default file listed above directly and just re-run the launch.
  4. After editing, press **Load** in the panel to re-read the file (no rebuild
     needed for CSV-only changes).
