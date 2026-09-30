#ifndef MOVEIT_WHOLE_BODY_IK_CSV_POSE_UTILS_HPP
#define MOVEIT_WHOLE_BODY_IK_CSV_POSE_UTILS_HPP

#include <geometry_msgs/Quaternion.h>

#include <algorithm>
#include <cmath>

namespace moveit_whole_body_ik {

struct RpyDegrees {
  double roll;
  double pitch;
  double yaw;
};

inline geometry_msgs::Quaternion rpyDegreesToQuaternion(double roll, double pitch,
                                                         double yaw) {
  constexpr double kDegreesToRadians = M_PI / 180.0;
  const double cr = std::cos(roll * kDegreesToRadians / 2.0);
  const double sr = std::sin(roll * kDegreesToRadians / 2.0);
  const double cp = std::cos(pitch * kDegreesToRadians / 2.0);
  const double sp = std::sin(pitch * kDegreesToRadians / 2.0);
  const double cy = std::cos(yaw * kDegreesToRadians / 2.0);
  const double sy = std::sin(yaw * kDegreesToRadians / 2.0);
  geometry_msgs::Quaternion result;
  result.w = cr * cp * cy + sr * sp * sy;
  result.x = sr * cp * cy - cr * sp * sy;
  result.y = cr * sp * cy + sr * cp * sy;
  result.z = cr * cp * sy - sr * sp * cy;
  return result;
}

inline RpyDegrees quaternionToRpyDegrees(const geometry_msgs::Quaternion& value) {
  constexpr double kRadiansToDegrees = 180.0 / M_PI;
  const double norm = std::sqrt(value.x * value.x + value.y * value.y +
                                value.z * value.z + value.w * value.w);
  const double x = norm > 1e-12 ? value.x / norm : 0.0;
  const double y = norm > 1e-12 ? value.y / norm : 0.0;
  const double z = norm > 1e-12 ? value.z / norm : 0.0;
  const double w = norm > 1e-12 ? value.w / norm : 1.0;
  const double sin_roll = 2.0 * (w * x + y * z);
  const double cos_roll = 1.0 - 2.0 * (x * x + y * y);
  const double sin_pitch = std::max(-1.0, std::min(1.0, 2.0 * (w * y - z * x)));
  const double sin_yaw = 2.0 * (w * z + x * y);
  const double cos_yaw = 1.0 - 2.0 * (y * y + z * z);
  return {std::atan2(sin_roll, cos_roll) * kRadiansToDegrees,
          std::asin(sin_pitch) * kRadiansToDegrees,
          std::atan2(sin_yaw, cos_yaw) * kRadiansToDegrees};
}

}  // namespace moveit_whole_body_ik

#endif
