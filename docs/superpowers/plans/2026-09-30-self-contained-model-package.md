# Self-contained IR100 Model Package Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `moveit_whole_body_ik` launch IR100 whole-body IK with its own model, mesh, and MoveIt assets.

**Architecture:** Copy the IR100 and CR10-only model assets and the MoveIt runtime configuration into `moveit_whole_body_ik`.  Rewrite package-resource references and have MoveIt generate `robot_description` directly from the packaged top-level Xacro.

**Tech Stack:** ROS Noetic, catkin, Xacro, MoveIt, RViz, Python unittest.

**Spec:** `docs/superpowers/specs/2026-09-30-self-contained-model-design.md`

## Global Constraints

- Copy only IR100 and CR10 model assets; do not copy other Dobot models.
- Do not retain runtime references to `ir100_description`, `dobot_description`, or `ir100_moveit`.
- Do not require `/tmp/ir100.urdf` before launch.
- Keep old source packages in the workspace for compatibility with unrelated projects.

## Review Focus

- Generated URDF must have no `package://ir100_description` or `package://dobot_description` URI.
- The MoveIt launch include chain must resolve only paths within `moveit_whole_body_ik`.
- `robot_description` must be generated at launch rather than loaded from `/tmp/ir100.urdf`.
- Both direct-arm and sampled whole-body IK services must remain available.
- Installed package resources must include Xacro, meshes, configuration, and runtime launch files.

---

### Task 1: Package IR100 model and MoveIt runtime assets

**Files:**
- Create: `src/moveit_whole_body_ik/urdf/ir100_robot.xacro`
- Create: `src/moveit_whole_body_ik/urdf/ir100.xacro`
- Create: `src/moveit_whole_body_ik/urdf/cr10_robot_urdf.xacro`
- Create: `src/moveit_whole_body_ik/meshes/ir100/*`
- Create: `src/moveit_whole_body_ik/meshes/cr10/*`
- Create: `src/moveit_whole_body_ik/config/*`
- Create: `src/moveit_whole_body_ik/launch/moveit/*`
- Test: `src/moveit_whole_body_ik/test/test_package_layout.py`

**Interfaces:**
- Consumes: existing IR100, CR10, and MoveIt source assets.
- Produces: package-local `urdf`, `meshes`, `config`, and `launch/moveit` paths.

- [ ] **Step 1: Write failing static layout tests**

Assert the package-local top-level Xacro, CR10 Xacro, IR100 and CR10 mesh
directories, and package-local MoveIt runtime launch exist.

- [ ] **Step 2: Run the static test to verify it fails**

Run: `python3 -m unittest src/moveit_whole_body_ik/test/test_package_layout.py -v`

Expected: FAIL because package-local asset directories do not exist.

- [ ] **Step 3: Copy only required assets and rewrite model resource references**

Copy IR100 Xacros and the CR10 Xacro/masses; copy IR100 meshes and CR10
meshes.  Change all included Xacro and mesh package references to
`moveit_whole_body_ik`.

- [ ] **Step 4: Run the static test to verify it passes**

Run: `python3 -m unittest src/moveit_whole_body_ik/test/test_package_layout.py -v`

Expected: PASS.

### Task 2: Make packaged MoveIt launch self-contained

**Files:**
- Modify: `src/moveit_whole_body_ik/launch/whole_body_ik.launch`
- Modify: `src/moveit_whole_body_ik/launch/moveit/*.launch*`
- Modify: `src/moveit_whole_body_ik/package.xml`
- Modify: `src/moveit_whole_body_ik/CMakeLists.txt`
- Test: `src/moveit_whole_body_ik/test/test_package_layout.py`

**Interfaces:**
- Consumes: package-local runtime files from Task 1.
- Produces: a `whole_body_ik.launch` that resolves all models/configuration
  within `moveit_whole_body_ik`.

- [ ] **Step 1: Extend static tests for references and launch behavior**

Assert no package-local runtime Xacro/launch files refer to old model or
MoveIt packages; assert the planning context uses the Xacro command and does
not contain `/tmp/ir100.urdf`.

- [ ] **Step 2: Run the static test to verify it fails**

Run: `python3 -m unittest src/moveit_whole_body_ik/test/test_package_layout.py -v`

Expected: FAIL because the main launch includes `ir100_moveit`.

- [ ] **Step 3: Rewrite launch/config references and package installation rules**

Point includes and configuration loads to this package, replace the
`textfile` robot-description load with Xacro `command`, declare direct
runtime dependencies, and install all package-local runtime assets.

- [ ] **Step 4: Run static and Xacro validation**

Run: `source devel/setup.bash && xacro "$(rospack find moveit_whole_body_ik)/urdf/ir100_robot.xacro" >/tmp/ir100-self-contained.urdf`

Expected: exit 0 and generated URDF has no old package URIs.

### Task 3: Verify build and isolated runtime

**Files:**
- Modify: `README.md`
- Modify: `scripts/verify.sh` if its setup still assumes `/tmp/ir100.urdf`
- Test: existing package tests and `scripts/verify.sh`

**Interfaces:**
- Consumes: self-contained launch from Task 2.
- Produces: documented direct host/container startup without a temporary URDF.

- [ ] **Step 1: Update tests/docs to require direct launch**

Document `source devel/setup.bash && roslaunch moveit_whole_body_ik
whole_body_ik.launch`; add a test that the main launch has no old package
reference.

- [ ] **Step 2: Build and run package tests**

Run: `./scripts/build.sh --pkg moveit_whole_body_ik -j4` and the Python
package tests.

Expected: build and tests pass.

- [ ] **Step 3: Run the isolated smoke test**

Run: `./scripts/verify.sh`

Expected: both IK services start and the PCD OctoMap/cloud loads.
