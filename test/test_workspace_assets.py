#!/usr/bin/env python3
"""Checks that the standalone workspace owns every required runtime asset."""

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class WorkspaceAssetTest(unittest.TestCase):
    def test_workspace_contains_all_runtime_packages_and_map(self):
        self.assertTrue((ROOT / "src/ir100_moveit/package.xml").is_file())
        self.assertTrue((ROOT / "src/ir100_description/urdf/ir100_robot.xacro").is_file())
        self.assertTrue((ROOT / "src/dobot_description/urdf/cr10_robot_urdf.xacro").is_file())
        self.assertTrue((ROOT / "maps/scans_voxel_5cm_xyz.pcd").is_file())
        self.assertTrue((ROOT / "maps/smoke_test.pcd").is_file())

    def test_workspace_has_portable_build_and_verification_scripts(self):
        self.assertTrue((ROOT / "scripts/build.sh").is_file())
        self.assertTrue((ROOT / "scripts/verify.sh").is_file())


if __name__ == "__main__":
    unittest.main()
