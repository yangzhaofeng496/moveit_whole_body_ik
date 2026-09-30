#!/usr/bin/env python3
"""Static contracts for the extracted MoveIt whole-body IK package."""

import pathlib
import json
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class CorePackageLayoutTest(unittest.TestCase):
    def setUp(self):
        self.package_xml = (ROOT / "package.xml").read_text()
        self.pcd_source = (ROOT / "src/pcd_octomap_planning_scene_node.cpp").read_text()

    def test_core_package_has_no_remani_planner_dependency(self):
        self.assertNotIn("remani_planner", self.package_xml)

    def test_pcd_node_keeps_shared_display_and_collision_crop(self):
        self.assertIn("workspace_max_z_", self.pcd_source)
        start = self.pcd_source.index("const double z = source_z + translation_z_;")
        end = self.pcd_source.index("transformed.emplace_back", start)
        transform = self.pcd_source[start:end]
        self.assertLess(transform.index("if (crop_to_workspace_"),
                        transform.index("addDisplayPoint(x, y, z);"))

    def test_launch_uses_only_bundled_packages_and_map(self):
        launch = (ROOT / "launch/whole_body_ik.launch").read_text()
        self.assertIn("$(find moveit_whole_body_ik)/launch/moveit/move_group.launch", launch)
        self.assertNotIn("ir100_moveit", launch)
        self.assertIn(
            "$(find moveit_whole_body_ik)/../../maps/scans_voxel_5cm_xyz.pcd",
            launch,
        )
        self.assertNotIn("remani_planner", launch)

    def test_package_owns_ir100_model_assets(self):
        for relative_path in (
            "urdf/ir100_robot.xacro",
            "urdf/ir100.xacro",
            "urdf/cr10_robot_urdf.xacro",
            "meshes/ir100/base_link.STL",
            "meshes/cr10/Link6.STL",
            "config/ir100.srdf",
            "launch/moveit/planning_context.launch",
        ):
            self.assertTrue((ROOT / relative_path).is_file(), relative_path)

    def test_legacy_ir100_xacro_uses_a_relative_cr10_include(self):
        legacy_xacro = (ROOT.parent / "ir100_description/urdf/ir100.xacro").read_text()
        self.assertIn(
            'filename="../../dobot_description/urdf/cr10_robot_urdf.xacro"',
            legacy_xacro,
        )
        self.assertNotIn("$(find dobot_description)", legacy_xacro)

    def test_legacy_ir100_visualizer_entrypoint_uses_relative_include(self):
        entrypoint = (ROOT.parent / "ir100_description/urdf/ir100_robot.xacro").read_text()
        self.assertIn('filename="ir100.xacro"', entrypoint)
        self.assertIn("<xacro:ir100_robot />", entrypoint)
        self.assertNotIn("$(find ir100_description)", entrypoint)

    def test_vscode_urdf_visualizer_maps_legacy_mesh_packages(self):
        settings_path = ROOT.parents[1] / ".vscode/settings.json"
        settings = json.loads(settings_path.read_text())
        packages = settings["urdf-visualizer.packages"]
        self.assertEqual(packages["ir100_description"], "${workspaceFolder}/src/ir100_description")
        self.assertEqual(packages["dobot_description"], "${workspaceFolder}/src/dobot_description")

    def test_packaged_model_and_moveit_launch_have_no_old_package_references(self):
        model = (ROOT / "urdf/ir100.xacro").read_text()
        arm = (ROOT / "urdf/cr10_robot_urdf.xacro").read_text()
        context = (ROOT / "launch/moveit/planning_context.launch").read_text()
        self.assertNotIn("ir100_description", model)
        self.assertNotIn("dobot_description", model + arm)
        self.assertNotIn("ir100_moveit", context)
        self.assertNotIn("/tmp/ir100.urdf", context)
        self.assertIn("command=", context)
        for launch_path in (ROOT / "launch/moveit").glob("*.launch*"):
            text = launch_path.read_text()
            self.assertNotIn("ir100_moveit", text, launch_path.name)
            self.assertNotIn("/tmp/ir100.urdf", text, launch_path.name)

    def test_rviz_uses_the_new_panel_plugin_class(self):
        rviz = (ROOT / "launch/moveit_whole_body_ik.rviz").read_text()
        self.assertIn("moveit_whole_body_ik/CsvWholeBodyIkPanel", rviz)


if __name__ == "__main__":
    unittest.main()
