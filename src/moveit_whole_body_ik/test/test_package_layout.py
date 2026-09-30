#!/usr/bin/env python3
"""Static contracts for the extracted MoveIt whole-body IK package."""

import pathlib
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


if __name__ == "__main__":
    unittest.main()
