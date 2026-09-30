#include "moveit_whole_body_ik/virtual_camera_panel.hpp"
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <OgreCamera.h>
#include <OgreRenderWindow.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>
#include <OgreViewport.h>
#include <QLabel>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>
namespace moveit_whole_body_ik {
VirtualCameraPanel::VirtualCameraPanel(QWidget* parent) : rviz::Panel(parent) {
  status_ = new QLabel("Waiting for CSV target pose", this);
  auto* layout = new QVBoxLayout(this);
  layout->addWidget(status_);
  pose_sub_ = nh_.subscribe("/whole_body_ik/virtual_camera_pose", 1,
                            &VirtualCameraPanel::poseCallback, this);
}
void VirtualCameraPanel::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  pose_ = msg->pose; have_pose_ = true;
  if (!render_panel_ && vis_manager_) {
    render_panel_ = new rviz::RenderPanel(this);
    render_panel_->setMinimumSize(640, 360);
    render_panel_->initialize(vis_manager_->getSceneManager(), vis_manager_);
    layout()->addWidget(render_panel_);
  }
  updateCamera();
  status_->setText("Virtual camera: current CSV pose");
}
void VirtualCameraPanel::updateCamera() {
  if (!render_panel_ || !have_pose_) return;
  auto* w = render_panel_->getRenderWindow();
  if (!w || !w->getNumViewports()) return;
  auto* c = w->getViewport(0)->getCamera();
  c->setPosition(pose_.position.x, pose_.position.y, pose_.position.z);
  // CSV/EEF pose points along local +X, while an OGRE camera looks along -Z.
  // Rotate the camera frame -90 degrees around its local Y axis so both
  // representations observe the same direction.
  const Ogre::Quaternion target_orientation(
      pose_.orientation.w, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z);
  const Ogre::Quaternion camera_frame_offset(Ogre::Degree(-90.0), Ogre::Vector3::UNIT_Y);
  c->setOrientation(target_orientation * camera_frame_offset);
  c->setFOVy(Ogre::Degree(60)); c->setNearClipDistance(0.02); c->setFarClipDistance(100);
  w->update();
}
void VirtualCameraPanel::save(rviz::Config config) const { rviz::Panel::save(config); }
void VirtualCameraPanel::load(const rviz::Config& config) { rviz::Panel::load(config); }
}
PLUGINLIB_EXPORT_CLASS(moveit_whole_body_ik::VirtualCameraPanel, rviz::Panel)
