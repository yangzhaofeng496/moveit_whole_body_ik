#include <gtest/gtest.h>

#include <moveit_whole_body_ik/csv_pose_utils.hpp>

#include <cmath>

namespace moveit_whole_body_ik {
namespace {

TEST(CsvPoseUtils, ConvertsEditedRpyDegreesToNormalizedQuaternion) {
  const auto quaternion = rpyDegreesToQuaternion(0.0, 0.0, 90.0);

  EXPECT_NEAR(0.0, quaternion.x, 1e-9);
  EXPECT_NEAR(0.0, quaternion.y, 1e-9);
  EXPECT_NEAR(std::sqrt(0.5), quaternion.z, 1e-9);
  EXPECT_NEAR(std::sqrt(0.5), quaternion.w, 1e-9);
}

TEST(CsvPoseUtils, ConvertsCsvQuaternionBackToEditableRpyDegrees) {
  const auto quaternion = rpyDegreesToQuaternion(0.0, 0.0, 90.0);
  const auto rpy = quaternionToRpyDegrees(quaternion);

  EXPECT_NEAR(0.0, rpy.roll, 1e-9);
  EXPECT_NEAR(0.0, rpy.pitch, 1e-9);
  EXPECT_NEAR(90.0, rpy.yaw, 1e-9);
}

}  // namespace
}  // namespace moveit_whole_body_ik
