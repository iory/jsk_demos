#ifndef K_ARM_TELEOP_TELEOP_PANEL_H
#define K_ARM_TELEOP_TELEOP_PANEL_H

#ifndef Q_MOC_RUN
#include <k_arm_teleop/LeaderServoStatus.h>
#include <k_arm_teleop/TeleopStatus.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include <QString>
#include <map>
#include <string>

class QCheckBox;
class QComboBox;
class QGridLayout;
class QHBoxLayout;
class QLabel;
class QLineEdit;
class QPushButton;
class QTableWidget;
class QTimer;

namespace k_arm_teleop
{

// Front end of teleop_manager.py: target selection, start/stop, zero pose,
// leader calibration and rosbag recording. All logic lives in the manager;
// the panel only calls its services and shows ~status.
class TeleopPanel : public rviz::Panel
{
  Q_OBJECT
public:
  explicit TeleopPanel(QWidget* parent = nullptr);

  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

private Q_SLOTS:
  void onManagerEdited();
  void onTargetChanged(int index);
  void onStart();
  void onStop();
  void onGoZero();
  void onCalibrate();
  void onBrowse();
  void onApplyDirectory();
  void onRecordToggle();
  void refresh();

private:
  void connectManager(const std::string& ns);
  void statusCallback(const k_arm_teleop::TeleopStatus::ConstPtr& msg);
  void servoCallback(const k_arm_teleop::LeaderServoStatus::ConstPtr& msg);
  void refreshServos();
  void flipDirection(int row);
  void calibrateTrigger(const std::string& joint, const std::string& point);
  bool callTrigger(const std::string& name);
  bool callSetString(const std::string& name, const std::string& data);
  void callSetArm(const std::string& arm, bool enabled);
  void report(bool ok, const std::string& message);
  void rebuildArms();

  ros::NodeHandle nh_;
  ros::Subscriber status_sub_;
  ros::Subscriber servo_sub_;
  k_arm_teleop::LeaderServoStatus servo_status_;
  ros::WallTime last_servo_time_;
  bool have_servo_status_;
  std::string manager_ns_;
  std::string pending_directory_;
  k_arm_teleop::TeleopStatus status_;
  ros::WallTime last_status_time_;
  bool have_status_;
  bool dir_edited_;

  QLineEdit* manager_edit_;
  QLabel* phase_label_;
  QLabel* message_label_;
  QLabel* sim_warning_label_;
  QComboBox* target_combo_;
  QHBoxLayout* arms_layout_;
  std::map<std::string, QCheckBox*> arm_boxes_;
  QPushButton* start_button_;
  QPushButton* stop_button_;
  QPushButton* zero_button_;
  QPushButton* calibrate_button_;
  QLabel* leader_label_;
  QLabel* tracking_label_;
  QLabel* servo_summary_label_;
  QTableWidget* servo_table_;
  QGridLayout* trigger_layout_;
  std::map<std::string, QLabel*> trigger_labels_;
  QLineEdit* dir_edit_;
  QPushButton* browse_button_;
  QPushButton* apply_dir_button_;
  QPushButton* record_button_;
  QLabel* bag_stats_label_;
  QLabel* current_bag_label_;
  QLabel* result_label_;
  QTimer* timer_;
};

}  // namespace k_arm_teleop

#endif  // K_ARM_TELEOP_TELEOP_PANEL_H
