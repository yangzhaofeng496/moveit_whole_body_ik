# Virtual Camera Preview Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Render the RViz scene live from the selected editable CSV target pose inside the CSV Whole-Body IK panel.

**Architecture:** The panel will create an `rviz::RenderPanel` using the owning `VisualizationManager` scene manager. A dedicated `rviz::FPSViewController` will place its OGRE camera from the target pose plus local camera offset. Controls update preview parameters and persist in RViz config; they never modify CSV fields.

**Tech Stack:** ROS Noetic, RViz 1.14, OGRE, Qt5, catkin/gtest.

**Spec:** `docs/superpowers/specs/2026-09-30-virtual-camera-preview-design.md`

## Global Constraints

- Reuse RViz's existing scene manager; do not subscribe to a camera topic or duplicate robot/point-cloud rendering.
- Preview defaults: 640×360, vertical FOV 60°, near clip 0.02 m, far clip 100 m, zero target-local offset.
- Preview uses the target's positive local X direction and is independent of CSV persistence.
- Invalid clip planes or unavailable RViz rendering context disable only the preview and report panel status.

## Review Focus

- Switching CSV points while preview is enabled moves the preview camera without creating duplicate OGRE resources.
- A 90° yaw rotates a local X offset onto world Y before applying it to the camera position.
- Near clip greater than or equal to far clip leaves CSV editing and IK usable while disabling preview.
- Disabling preview releases its render panel and camera before the RViz scene manager closes.
- Saved RViz panel configuration restores preview parameters but does not alter the CSV.

### Task 1: Preview pose transform

**Files:**
- Create: `src/moveit_whole_body_ik/include/moveit_whole_body_ik/virtual_camera_preview.hpp`
- Test: `src/moveit_whole_body_ik/test/virtual_camera_preview_test.cpp`
- Modify: `src/moveit_whole_body_ik/CMakeLists.txt`

**Interfaces:**
- Produces: `VirtualCameraPose composeVirtualCameraPose(const geometry_msgs::Pose&, const geometry_msgs::Vector3&)` with world position and normalized orientation.

- [ ] **Step 1: Write failing transform tests** for zero offset and a +90° target yaw with a +X local offset mapping to +Y world.
- [ ] **Step 2: Run test to verify it fails**

Run: `catkin_make run_tests_moveit_whole_body_ik_gtest_virtual_camera_preview_test`

Expected: FAIL because the preview transform interface is absent.

- [ ] **Step 3: Implement `composeVirtualCameraPose`** with quaternion-vector rotation and normalization.
- [ ] **Step 4: Run transform test to verify it passes**
- [ ] **Step 5: Commit**

### Task 2: RViz preview widget and controls

**Files:**
- Modify: `src/moveit_whole_body_ik/include/moveit_whole_body_ik/csv_whole_body_ik_panel.hpp`
- Modify: `src/moveit_whole_body_ik/src/csv_whole_body_ik_panel.cpp`
- Modify: `src/moveit_whole_body_ik/CMakeLists.txt`

**Interfaces:**
- Consumes: `composeVirtualCameraPose` from Task 1.
- Produces: `initializePreview()`, `updatePreviewCamera()`, and `destroyPreview()` lifecycle methods on `CsvWholeBodyIkPanel`.

- [ ] **Step 1: Write failing UI/lifecycle test** covering preview controls and invalid near/far clip rejection.
- [ ] **Step 2: Run test to verify it fails**
- [ ] **Step 3: Add controls and a disabled-by-default `rviz::RenderPanel`**, initialize it with `vis_manager_->getSceneManager()` and the visualization manager display context, and attach an FPS view controller.
- [ ] **Step 4: Update the preview camera** after point changes and RPY/XYZ edits; apply FOV, clips, and local offset without modifying CSV rows.
- [ ] **Step 5: Release preview OGRE resources** on disable and panel destruction; report unavailable render context or invalid clip planes through the existing status label.
- [ ] **Step 6: Run the package build and tests**

Run: `source /opt/ros/noetic/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES=moveit_whole_body_ik && catkin_make run_tests`

Expected: build succeeds and all available package tests pass.

- [ ] **Step 7: Commit**

### Task 3: RViz configuration persistence and manual verification

**Files:**
- Modify: `src/moveit_whole_body_ik/src/csv_whole_body_ik_panel.cpp`
- Modify: `README.md`

**Interfaces:**
- Consumes: preview control values from Task 2.
- Produces: RViz config keys `preview_enabled`, `preview_fov_degrees`, `preview_near_clip`, `preview_far_clip`, and `preview_offset_xyz`.

- [ ] **Step 1: Write a failing configuration round-trip test** for preview settings independent of CSV writes.
- [ ] **Step 2: Run test to verify it fails**
- [ ] **Step 3: Save and restore preview controls** in `CsvWholeBodyIkPanel::save/load`, including validation of clip planes.
- [ ] **Step 4: Document operator controls** and clarify the preview is virtual and not a physical camera feed.
- [ ] **Step 5: Run manual RViz verification**: enable preview, edit position and RPY, select another point, disable preview, and save CSV.
- [ ] **Step 6: Run full verification and commit**
