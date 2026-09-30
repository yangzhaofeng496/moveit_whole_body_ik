# Virtual camera preview design

## Goal

Let an operator inspect the RViz scene from the currently selected CSV target
pose before committing the target. The preview is a virtual camera, not a ROS
image topic or physical device.

## Scope

The existing `CSV Whole-Body IK` RViz panel gains an embedded live preview.
It uses the selected point's position and editable RPY orientation as the
camera pose. Changes to position or orientation update the preview without
editing or saving the CSV.

## Architecture

The panel owns an RViz render widget backed by a dedicated OGRE camera and
viewport. It renders the same RViz scene manager already used by the main
display, so robot geometry, point-cloud displays, markers, planning-scene
objects, and fixed-frame transforms match the operator's main RViz view.
No ROS subscriber, image transport dependency, or duplicate renderer is
introduced.

The preview camera transform is set from the selected target pose in the
panel's configured reference frame. A small configurable local offset keeps
the camera from clipping into geometry at the target origin. The camera faces
the target's positive local X direction, consistent with the panel's existing
arrow marker.

## Panel controls

- **Enable preview:** creates or hides the preview viewport without changing
  CSV data.
- **Preview size:** fixed panel-friendly resolution, initially 640×360.
- **Vertical FOV:** editable degree value, default 60°.
- **Near / far clip:** editable metre values, defaults 0.02 m and 100 m.
- **Camera offset:** editable XYZ metres in the target-local frame, default
  zero. This makes it possible to look from a position slightly behind the
  end-effector rather than inside it.

The controls are saved in RViz panel configuration. The selected CSV pose and
preview settings are independent of CSV persistence; only **Save CSV** writes
the seven `ee_*` pose fields.

## Rendering lifecycle and errors

The panel initializes rendering only after RViz's display context and scene
manager are available. It destroys the camera, viewport, and render widget
when the panel closes or preview is disabled. Invalid clip planes, absent
scene manager, or rendering initialization failures disable the preview and
show a concise status error; they never affect IK or CSV editing.

## Validation

- Unit-test target-pose plus local-offset transform math, including identity
  and 90° yaw cases.
- Build the package against ROS Noetic/RViz.
- Start `whole_body_ik.launch`; enable the preview and verify it tracks a
  selected point while X/Y/Z and RPY controls change.
- Verify saving a CSV does not depend on preview state.
