#ifndef MOVEIT_WHOLE_BODY_IK_CSV_WHOLE_BODY_IK_PANEL_HPP
#define MOVEIT_WHOLE_BODY_IK_CSV_WHOLE_BODY_IK_PANEL_HPP

#include <moveit_whole_body_ik/SolveSampledWholeBodyIK.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <QVector>

#include <string>
#include <unordered_map>
#include <vector>

class QComboBox;
class QCheckBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QSlider;
class QScrollArea;
class QTableWidget;
class QTimer;
namespace rviz { class RenderPanel; }

namespace moveit_whole_body_ik {

class CsvWholeBodyIkPanel : public rviz::Panel {
  Q_OBJECT

 public:
  explicit CsvWholeBodyIkPanel(QWidget* parent = nullptr);
  void save(rviz::Config config) const override;
  void load(const rviz::Config& config) override;

 private Q_SLOTS:
  void browseCsv();
  void loadCsv();
  void saveCsv();
  void saveCsvAs();
  void applyPoseEdits();
  void updateAngleStep(double degrees);
  void toggleVirtualCamera(bool enabled);
  void previousPoint();
  void nextPoint();
  void solveCurrent();
  void togglePlayback();
  void playbackTick();

 private:
  struct CsvPose {
    geometry_msgs::Pose pose;
    QString label;
    std::vector<std::string> row;
    double roll_degrees = 0.0;
    double pitch_degrees = 0.0;
    double yaw_degrees = 0.0;
  };

  bool parseCsv(const QString& path);
  void updatePointUi();
  void updateMetricsTable();
  void updatePoseEditors();
  bool writeCsv(const QString& path);
  void setStatus(const QString& text, bool error = false);
  void publishCsvMarkers();
  void updateVirtualCamera();

  QLineEdit* path_edit_ = nullptr;
  QPushButton* browse_button_ = nullptr;
  QPushButton* load_button_ = nullptr;
  QPushButton* save_button_ = nullptr;
  QPushButton* save_as_button_ = nullptr;
  QComboBox* point_box_ = nullptr;
  QSpinBox* index_spin_ = nullptr;
  QPushButton* previous_button_ = nullptr;
  QPushButton* next_button_ = nullptr;
  QPushButton* solve_button_ = nullptr;
  QPushButton* play_button_ = nullptr;
  QDoubleSpinBox* x_edit_ = nullptr;
  QDoubleSpinBox* y_edit_ = nullptr;
  QDoubleSpinBox* z_edit_ = nullptr;
  QDoubleSpinBox* angle_step_edit_ = nullptr;
  QSlider* roll_slider_ = nullptr;
  QSlider* pitch_slider_ = nullptr;
  QSlider* yaw_slider_ = nullptr;
  QLabel* roll_value_label_ = nullptr;
  QLabel* pitch_value_label_ = nullptr;
  QLabel* yaw_value_label_ = nullptr;
  QLabel* status_label_ = nullptr;
  QTableWidget* metrics_table_ = nullptr;
  QTimer* playback_timer_ = nullptr;
  QCheckBox* virtual_camera_enabled_ = nullptr;
  QScrollArea* scroll_area_ = nullptr;
  rviz::RenderPanel* virtual_camera_panel_ = nullptr;

  QVector<CsvPose> poses_;
  int current_index_ = -1;
  ros::NodeHandle nh_;
  ros::ServiceClient solve_client_;
  ros::Publisher marker_pub_;
  ros::Publisher camera_pose_pub_;
  QString reference_frame_ = "world";
  QVector<int> point_status_;  // -1 pending, 0 failed, 1 succeeded
  QVector<double> position_error_;
  QVector<double> orientation_error_;
  QVector<double> solve_time_ms_;
  QVector<QString> result_messages_;
  std::vector<std::string> header_;
  std::unordered_map<std::string, size_t> columns_;
};

}  // namespace moveit_whole_body_ik

#endif
