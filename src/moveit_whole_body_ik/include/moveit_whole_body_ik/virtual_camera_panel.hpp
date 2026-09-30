#ifndef MOVEIT_WHOLE_BODY_IK_VIRTUAL_CAMERA_PANEL_HPP
#define MOVEIT_WHOLE_BODY_IK_VIRTUAL_CAMERA_PANEL_HPP
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <rviz/panel.h>
class QLabel;
namespace rviz { class RenderPanel; }
namespace moveit_whole_body_ik {
class VirtualCameraPanel : public rviz::Panel {
  Q_OBJECT
public:
  explicit VirtualCameraPanel(QWidget* parent = nullptr);
  void save(rviz::Config config) const override;
  void load(const rviz::Config& config) override;
private Q_SLOTS:
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
private:
  void updateCamera();
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  geometry_msgs::Pose pose_;
  bool have_pose_ = false;
  rviz::RenderPanel* render_panel_ = nullptr;
  QLabel* status_ = nullptr;
};
}
#endif
