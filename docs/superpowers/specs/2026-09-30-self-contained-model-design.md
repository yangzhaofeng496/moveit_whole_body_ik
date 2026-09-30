# Self-contained IR100 model package design

## Goal

Make `moveit_whole_body_ik` independently own every model and MoveIt asset
needed by the IR100 whole-body IK launch.  Starting the package must not
require `ir100_description`, `dobot_description`, `ir100_moveit`, or a
pre-generated `/tmp/ir100.urdf` file.

## Scope

Copy only assets used by IR100:

- IR100 top-level Xacro files and IR100 base, wheel, and gripper meshes.
- The CR10 Xacro macro and its six-link mesh set.
- IR100 MoveIt configuration and the launch files reached from
  `whole_body_ik.launch`.

Other Dobot models and unrelated display, Gazebo, benchmark, and warehouse
assets remain in their existing packages and are not copied.

## Layout

`moveit_whole_body_ik` gains `urdf/`, `meshes/`, `config/`, and
`launch/moveit/`.  Xacro includes and `package://` mesh URIs point to
`moveit_whole_body_ik` exclusively.  Runtime MoveIt launches use this package
for all configuration and include paths.

## Runtime behavior

`planning_context.launch` sets `robot_description` using the Xacro command
directly.  It no longer reads `/tmp/ir100.urdf`, so the launch command remains:

```bash
source devel/setup.bash
roslaunch moveit_whole_body_ik whole_body_ik.launch
```

## Packaging

The package declares its direct ROS runtime dependencies, including `xacro`
and `robot_state_publisher`, and removes the `ir100_moveit` runtime
dependency.  CMake installs the copied models, meshes, configuration, and
launch directories.

## Validation

- Expand the packaged top-level Xacro with only this package in the ROS path.
- Assert the generated URDF has no old package URIs.
- Run the existing launch smoke test and verify both IK services.
- Keep old packages in the workspace but verify the whole-body package no
  longer references them.
