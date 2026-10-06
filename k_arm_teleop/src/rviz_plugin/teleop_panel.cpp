#include "teleop_panel.h"

#include <k_arm_teleop/SetArmEnabled.h>
#include <k_arm_teleop/SetServoDirection.h>
#include <k_arm_teleop/SetString.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>
#include <std_srvs/Trigger.h>

#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QFileDialog>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QHeaderView>
#include <QPushButton>
#include <QScrollArea>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>

namespace k_arm_teleop
{
namespace
{
const char* kDefaultManager = "/k_arm_teleop_manager";
const char* kServoStatusTopic = "/teleop_leader/servo_status";
const char* kSetDirectionService = "/teleop_leader/set_direction";
const int kDirectionColumn = 2;
const char* kCalibrateTriggerService = "/teleop_leader/calibrate_trigger";
const double kWarnTemperature = 55.0;  // [C] STS3215 default limit is 70
const double kLowVoltage = 6.0;        // [V]
const double kStatusTimeout = 2.0;  // [s] without ~status -> manager shown as missing
const int kRefreshMs = 200;

QString humanSize(uint64_t bytes)
{
  const char* units[] = { "B", "KB", "MB", "GB", "TB" };
  double value = static_cast<double>(bytes);
  int unit = 0;
  while (value >= 1024.0 && unit < 4)
  {
    value /= 1024.0;
    ++unit;
  }
  return QString::number(value, 'f', unit == 0 ? 0 : 1) + " " + units[unit];
}

QString humanDuration(double seconds)
{
  int total = static_cast<int>(seconds);
  return QString("%1:%2").arg(total / 60).arg(total % 60, 2, 10, QChar('0'));
}

QString phaseColor(const std::string& phase)
{
  if (phase == "following")
    return "#2e7d32";
  if (phase == "approaching" || phase == "moving_to_zero")
    return "#ef6c00";
  if (phase == "error" || phase == "leader_lost")
    return "#c62828";
  return "#555555";
}
}  // namespace

TeleopPanel::TeleopPanel(QWidget* parent)
  : rviz::Panel(parent), have_servo_status_(false), have_status_(false), dir_edited_(false)
{
  QVBoxLayout* layout = new QVBoxLayout;

  QHBoxLayout* manager_row = new QHBoxLayout;
  manager_row->addWidget(new QLabel("Manager:"));
  manager_edit_ = new QLineEdit(kDefaultManager);
  manager_row->addWidget(manager_edit_);
  layout->addLayout(manager_row);

  phase_label_ = new QLabel;
  phase_label_->setStyleSheet("font-size: 16px; font-weight: bold;");
  message_label_ = new QLabel;
  message_label_->setWordWrap(true);
  sim_warning_label_ = new QLabel(
      "\"Real\" is the SIMULATED stand-in (teleop.launch real:=false), not hardware.");
  sim_warning_label_->setWordWrap(true);
  sim_warning_label_->setStyleSheet("color: #ef6c00;");
  layout->addWidget(phase_label_);
  layout->addWidget(message_label_);
  layout->addWidget(sim_warning_label_);

  // -- command target and arms
  QGroupBox* control_box = new QGroupBox("Teleoperation");
  QGridLayout* control = new QGridLayout;
  control->addWidget(new QLabel("Send to:"), 0, 0);
  target_combo_ = new QComboBox;
  target_combo_->addItem("Virtual (ghost robot)", "virtual");
  target_combo_->addItem("Real robot", "real");
  control->addWidget(target_combo_, 0, 1, 1, 2);
  control->addWidget(new QLabel("Arms:"), 1, 0);
  arms_layout_ = new QHBoxLayout;
  control->addLayout(arms_layout_, 1, 1, 1, 2);
  start_button_ = new QPushButton("Start (approach, then follow)");
  stop_button_ = new QPushButton("STOP");
  stop_button_->setStyleSheet("background-color: #c62828; color: white; font-weight: bold;");
  zero_button_ = new QPushButton("Robot -> zero pose");
  control->addWidget(start_button_, 2, 0, 1, 2);
  control->addWidget(stop_button_, 2, 2);
  control->addWidget(zero_button_, 3, 0, 1, 3);
  control_box->setLayout(control);
  layout->addWidget(control_box);

  // -- leader
  QGroupBox* leader_box = new QGroupBox("Leader arm");
  QVBoxLayout* leader = new QVBoxLayout;
  leader_label_ = new QLabel;
  tracking_label_ = new QLabel;
  calibrate_button_ = new QPushButton("Calibrate leader zero (current pose = robot zero pose)");
  servo_summary_label_ = new QLabel;
  servo_table_ = new QTableWidget(0, 9);
  servo_table_->setHorizontalHeaderLabels(
      { "ID", "Joint", "Direction", "Angle [deg]", "State", "Raw", "V", "C", "Errors" });
  servo_table_->horizontalHeaderItem(kDirectionColumn)->setToolTip(
      "Move one joint and watch the leader model in rviz. If it turns the other way, press flip.");
  servo_table_->verticalHeader()->setVisible(false);
  servo_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  servo_table_->horizontalHeader()->setStretchLastSection(true);
  servo_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  servo_table_->setSelectionMode(QAbstractItemView::NoSelection);
  servo_table_->verticalHeader()->setDefaultSectionSize(servo_table_->fontMetrics().height() + 6);
  servo_table_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  servo_table_->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  leader->addWidget(leader_label_);
  leader->addWidget(servo_summary_label_);
  leader->addWidget(servo_table_);
  trigger_layout_ = new QGridLayout;
  leader->addLayout(trigger_layout_);
  leader->addWidget(tracking_label_);
  leader->addWidget(calibrate_button_);
  leader_box->setLayout(leader);
  layout->addWidget(leader_box);

  // -- recording
  QGroupBox* record_box = new QGroupBox("Recording (rosbag)");
  QGridLayout* record = new QGridLayout;
  dir_edit_ = new QLineEdit;
  browse_button_ = new QPushButton("Browse...");
  apply_dir_button_ = new QPushButton("Set");
  record->addWidget(new QLabel("Directory:"), 0, 0);
  record->addWidget(dir_edit_, 0, 1);
  record->addWidget(browse_button_, 0, 2);
  record->addWidget(apply_dir_button_, 0, 3);
  record_button_ = new QPushButton("Start recording");
  record_button_->setMinimumHeight(36);
  record->addWidget(record_button_, 1, 0, 1, 4);
  bag_stats_label_ = new QLabel;
  current_bag_label_ = new QLabel;
  current_bag_label_->setWordWrap(true);
  record->addWidget(bag_stats_label_, 2, 0, 1, 4);
  record->addWidget(current_bag_label_, 3, 0, 1, 4);
  record_box->setLayout(record);
  layout->addWidget(record_box);

  result_label_ = new QLabel;
  result_label_->setWordWrap(true);
  layout->addWidget(result_label_);
  layout->addStretch();
  // Scroll rather than squeeze the widgets when the dock is shorter than the panel.
  QWidget* content = new QWidget;
  content->setLayout(layout);
  QScrollArea* scroll = new QScrollArea;
  scroll->setWidget(content);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  QVBoxLayout* outer = new QVBoxLayout;
  outer->setContentsMargins(0, 0, 0, 0);
  outer->addWidget(scroll);
  setLayout(outer);

  connect(manager_edit_, SIGNAL(editingFinished()), this, SLOT(onManagerEdited()));
  connect(target_combo_, SIGNAL(activated(int)), this, SLOT(onTargetChanged(int)));
  connect(start_button_, SIGNAL(clicked()), this, SLOT(onStart()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(onStop()));
  connect(zero_button_, SIGNAL(clicked()), this, SLOT(onGoZero()));
  connect(calibrate_button_, SIGNAL(clicked()), this, SLOT(onCalibrate()));
  connect(browse_button_, SIGNAL(clicked()), this, SLOT(onBrowse()));
  connect(apply_dir_button_, SIGNAL(clicked()), this, SLOT(onApplyDirectory()));
  connect(dir_edit_, &QLineEdit::textEdited, this, [this]() { dir_edited_ = true; });
  connect(dir_edit_, SIGNAL(returnPressed()), this, SLOT(onApplyDirectory()));
  connect(record_button_, SIGNAL(clicked()), this, SLOT(onRecordToggle()));

  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(refresh()));
  timer_->start(kRefreshMs);

  connectManager(kDefaultManager);
  servo_sub_ = nh_.subscribe(kServoStatusTopic, 1, &TeleopPanel::servoCallback, this);
  refresh();
}

void TeleopPanel::servoCallback(const k_arm_teleop::LeaderServoStatus::ConstPtr& msg)
{
  servo_status_ = *msg;
  have_servo_status_ = true;
  last_servo_time_ = ros::WallTime::now();
}

void TeleopPanel::refreshServos()
{
  bool alive = have_servo_status_ && (ros::WallTime::now() - last_servo_time_).toSec() < kStatusTimeout;
  if (!alive)
  {
    servo_summary_label_->setText(QString("<b style='color:#c62828'>no %1</b> (feetech_leader_driver not running)")
                                      .arg(kServoStatusTopic));
    servo_table_->setRowCount(0);
    return;
  }
  const LeaderServoStatus& st = servo_status_;
  int connected = 0;
  for (const ServoState& servo : st.servos)
    connected += servo.connected ? 1 : 0;
  QString summary = QString("%1: <b style='color:%2'>%3 / %4 servos</b>")
                        .arg(QString::fromStdString(st.port), connected == static_cast<int>(st.servos.size()) ? "#2e7d32" : "#c62828")
                        .arg(connected)
                        .arg(st.servos.size());
  if (!st.bus_error.empty())
    summary += QString("  <b style='color:#c62828'>%1</b>").arg(QString::fromStdString(st.bus_error).toHtmlEscaped());
  else if (!st.all_connected)
    summary += "  <span style='color:#c62828'>(joint_states paused until all answer)</span>";
  servo_summary_label_->setText(summary);

  servo_table_->setRowCount(static_cast<int>(st.servos.size()));
  for (int row = 0; row < static_cast<int>(st.servos.size()); ++row)
  {
    const ServoState& servo = st.servos[row];
    QStringList cells;
    cells << QString::number(servo.id) << QString::fromStdString(servo.joint) << QString()
          << QString::number(servo.angle * 180.0 / M_PI, 'f', 1) << (servo.connected ? "OK" : "NO ANSWER")
          << QString::number(servo.raw_position) << QString::number(servo.voltage, 'f', 1)
          << QString::number(servo.temperature) << QString::number(servo.read_errors);
    for (int col = 0; col < cells.size(); ++col)
    {
      if (col == kDirectionColumn)
        continue;
      QTableWidgetItem* item = servo_table_->item(row, col);
      if (!item)
      {
        item = new QTableWidgetItem;
        servo_table_->setItem(row, col, item);
      }
      item->setText(cells[col]);
      item->setToolTip(QString::fromStdString(servo.last_error));
      QColor color = Qt::black;
      if (!servo.connected)
        color = QColor("#c62828");
      else if ((col == 6 && servo.voltage < kLowVoltage) || (col == 7 && servo.temperature >= kWarnTemperature) ||
               (col == 3 && servo.out_of_range))
        color = QColor("#ef6c00");
      item->setForeground(color);
      if (col == 3)
        item->setToolTip(servo.out_of_range ? "Outside the joint's range (leader_bus.yaml joint_ranges_deg): the direction or the calibration "
                                              "of this joint is probably wrong." :
                                              QString::fromStdString(servo.last_error));
    }
  }
  // Gripper triggers: two-point calibration (released / pulled), one row per trigger.
  for (const ServoState& servo : st.servos)
  {
    if (servo.joint.find("GRIPPER") == std::string::npos)
      continue;
    if (!trigger_labels_.count(servo.joint))
    {
      int row = static_cast<int>(trigger_labels_.size());
      std::string joint = servo.joint;
      QLabel* label = new QLabel;
      QPushButton* released = new QPushButton("Trigger released");
      QPushButton* pulled = new QPushButton("Trigger pulled");
      released->setToolTip("Let go of the trigger, then press: records the open end of the gripper.");
      pulled->setToolTip("Pull the trigger all the way, then press: records the closed end of the gripper.");
      connect(released, &QPushButton::clicked, this, [this, joint]() { calibrateTrigger(joint, "released"); });
      connect(pulled, &QPushButton::clicked, this, [this, joint]() { calibrateTrigger(joint, "pulled"); });
      trigger_layout_->addWidget(label, row, 0);
      trigger_layout_->addWidget(released, row, 1);
      trigger_layout_->addWidget(pulled, row, 2);
      trigger_labels_[joint] = label;
    }
    trigger_labels_[servo.joint]->setText(
        QString("%1: %2").arg(QString::fromStdString(servo.joint),
                              servo.trigger_calibrated ? "<span style='color:#2e7d32'>calibrated</span>" :
                                                         "<b style='color:#c62828'>set released + pulled</b>"));
  }

  // Direction column: a flip button per servo. Disabled while the robot follows,
  // since flipping a joint makes its target jump.
  bool active = have_status_ && (status_.phase == "approaching" || status_.phase == "following" ||
                                 status_.phase == "moving_to_zero");
  for (int row = 0; row < static_cast<int>(st.servos.size()); ++row)
  {
    QPushButton* button = qobject_cast<QPushButton*>(servo_table_->cellWidget(row, kDirectionColumn));
    if (!button)
    {
      button = new QPushButton;
      connect(button, &QPushButton::clicked, this, [this, row]() { flipDirection(row); });
      servo_table_->setCellWidget(row, kDirectionColumn, button);
    }
    button->setText(QString("%1  flip").arg(st.servos[row].direction > 0 ? "+1" : "-1"));
    button->setEnabled(!active);
  }
  // Show every row without scrolling.
  int height = std::max(servo_table_->horizontalHeader()->height(), servo_table_->horizontalHeader()->sizeHint().height()) +
               2 * servo_table_->frameWidth() + 2;
  for (int row = 0; row < servo_table_->rowCount(); ++row)
    height += servo_table_->rowHeight(row);
  servo_table_->setFixedHeight(height);
}

void TeleopPanel::flipDirection(int row)
{
  if (row >= static_cast<int>(servo_status_.servos.size()))
    return;
  const ServoState& servo = servo_status_.servos[row];
  k_arm_teleop::SetServoDirection srv;
  srv.request.joint = servo.joint;
  srv.request.direction = servo.direction > 0 ? -1 : 1;
  if (!ros::service::waitForService(kSetDirectionService, ros::Duration(0.5)) ||
      !ros::service::call(kSetDirectionService, srv))
    report(false, std::string(kSetDirectionService) + " call failed");
  else
    report(srv.response.success, srv.response.message);
}

void TeleopPanel::calibrateTrigger(const std::string& joint, const std::string& point)
{
  k_arm_teleop::SetString srv;
  srv.request.data = joint + " " + point;
  if (!ros::service::waitForService(kCalibrateTriggerService, ros::Duration(0.5)) ||
      !ros::service::call(kCalibrateTriggerService, srv))
    report(false, std::string(kCalibrateTriggerService) + " call failed");
  else
    report(srv.response.success, srv.response.message);
}

void TeleopPanel::connectManager(const std::string& ns)
{
  manager_ns_ = ns;
  have_status_ = false;
  status_sub_ = nh_.subscribe(manager_ns_ + "/status", 1, &TeleopPanel::statusCallback, this);
}

void TeleopPanel::statusCallback(const k_arm_teleop::TeleopStatus::ConstPtr& msg)
{
  // rviz spins ROS callbacks from its Qt update loop, so this runs in the GUI thread.
  bool arms_changed = !have_status_ || msg->arms != status_.arms;
  bool first = !have_status_;
  status_ = *msg;
  have_status_ = true;
  last_status_time_ = ros::WallTime::now();
  if (arms_changed)
    rebuildArms();
  // Re-apply the directory saved in the rviz config to a freshly started manager.
  if (first && !pending_directory_.empty() && pending_directory_ != msg->record_directory && !msg->recording)
    callSetString("set_record_directory", pending_directory_);
}

void TeleopPanel::rebuildArms()
{
  for (auto& entry : arm_boxes_)
    delete entry.second;
  arm_boxes_.clear();
  for (const std::string& arm : status_.arms)
  {
    QCheckBox* box = new QCheckBox(QString::fromStdString(arm));
    arms_layout_->addWidget(box);
    arm_boxes_[arm] = box;
    connect(box, &QCheckBox::clicked, this, [this, arm](bool checked) { callSetArm(arm, checked); });
  }
}

void TeleopPanel::refresh()
{
  refreshServos();
  bool alive = have_status_ && (ros::WallTime::now() - last_status_time_).toSec() < kStatusTimeout;
  const TeleopStatus& s = status_;
  bool active = alive && (s.phase == "approaching" || s.phase == "following" ||
                          s.phase == "moving_to_zero" || s.phase == "leader_lost");

  if (!alive)
  {
    phase_label_->setText("manager not running");
    phase_label_->setStyleSheet("font-size: 16px; font-weight: bold; color: #c62828;");
    message_label_->setText(QString("no %1/status").arg(QString::fromStdString(manager_ns_)));
  }
  else
  {
    QString target = s.target == "real" ? "REAL ROBOT" : "virtual";
    phase_label_->setText(QString("%1  ->  %2").arg(QString::fromStdString(s.phase), target));
    phase_label_->setStyleSheet(QString("font-size: 16px; font-weight: bold; color: %1;")
                                    .arg(phaseColor(s.phase)));
    message_label_->setText(QString::fromStdString(s.message));
  }
  sim_warning_label_->setVisible(alive && s.real_is_simulated);

  int index = target_combo_->findData(QString::fromStdString(s.target));
  if (alive && index >= 0 && !target_combo_->view()->isVisible())
    target_combo_->setCurrentIndex(index);
  for (size_t i = 0; i < s.arms.size() && i < s.arms_enabled.size(); ++i)
  {
    QCheckBox* box = arm_boxes_[s.arms[i]];
    box->setChecked(s.arms_enabled[i]);
    box->setEnabled(alive && !active);
  }

  target_combo_->setEnabled(alive && !active);
  start_button_->setEnabled(alive && !active);
  zero_button_->setEnabled(alive && !active);
  stop_button_->setEnabled(alive);
  calibrate_button_->setEnabled(alive && !active && s.leader_source == "feetech");

  if (alive)
  {
    QString leader = QString("source: %1   ").arg(QString::fromStdString(s.leader_source));
    leader += s.leader_alive ? QString("<b style='color:#2e7d32'>alive</b> %1 Hz").arg(s.leader_rate, 0, 'f', 0) :
                               QString("<b style='color:#c62828'>NO DATA</b>");
    leader += s.leader_calibrated ? "   calibrated" : "   <b style='color:#c62828'>NOT CALIBRATED</b>";
    leader_label_->setText(leader);
    tracking_label_->setText(QString("max |command - robot|: %1 rad").arg(s.max_tracking_error, 0, 'f', 3));
  }

  // Show the manager's directory unless the user is typing a new one.
  if (alive && !dir_edited_)
    dir_edit_->setText(QString::fromStdString(s.record_directory));
  dir_edit_->setEnabled(alive && !s.recording);
  browse_button_->setEnabled(alive && !s.recording);
  apply_dir_button_->setEnabled(alive && !s.recording);
  record_button_->setEnabled(alive);
  if (alive && s.recording)
  {
    record_button_->setText("Stop recording");
    record_button_->setStyleSheet("background-color: #c62828; color: white; font-weight: bold;");
    current_bag_label_->setText(QString("<b style='color:#c62828'>REC</b> %1  %2  %3")
                                    .arg(humanDuration(s.recording_duration), humanSize(s.current_bag_size),
                                         QString::fromStdString(s.current_bag)));
  }
  else
  {
    record_button_->setText("Start recording");
    record_button_->setStyleSheet("");
    current_bag_label_->setText("not recording");
  }
  if (alive)
  {
    bag_stats_label_->setText(QString("<b>%1</b> bags in %2   total %3   disk free %4")
                                  .arg(s.bag_count)
                                  .arg(QString::fromStdString(s.record_directory), humanSize(s.total_bag_size),
                                       humanSize(s.disk_free)));
  }
}

bool TeleopPanel::callTrigger(const std::string& name)
{
  std::string service = manager_ns_ + "/" + name;
  if (!ros::service::waitForService(service, ros::Duration(0.5)))
  {
    report(false, service + " is not available");
    return false;
  }
  std_srvs::Trigger srv;
  if (!ros::service::call(service, srv))
  {
    report(false, service + " call failed");
    return false;
  }
  report(srv.response.success, srv.response.message);
  return srv.response.success;
}

bool TeleopPanel::callSetString(const std::string& name, const std::string& data)
{
  std::string service = manager_ns_ + "/" + name;
  if (!ros::service::waitForService(service, ros::Duration(0.5)))
  {
    report(false, service + " is not available");
    return false;
  }
  k_arm_teleop::SetString srv;
  srv.request.data = data;
  if (!ros::service::call(service, srv))
  {
    report(false, service + " call failed");
    return false;
  }
  report(srv.response.success, srv.response.message);
  return srv.response.success;
}

void TeleopPanel::callSetArm(const std::string& arm, bool enabled)
{
  std::string service = manager_ns_ + "/set_arm_enabled";
  k_arm_teleop::SetArmEnabled srv;
  srv.request.arm = arm;
  srv.request.enabled = enabled;
  if (!ros::service::waitForService(service, ros::Duration(0.5)) || !ros::service::call(service, srv))
    report(false, service + " call failed");
  else
    report(srv.response.success, srv.response.message);
}

void TeleopPanel::report(bool ok, const std::string& message)
{
  result_label_->setText(QString("<span style='color:%1'>%2</span>")
                             .arg(ok ? "#2e7d32" : "#c62828", QString::fromStdString(message).toHtmlEscaped()));
}

void TeleopPanel::onManagerEdited()
{
  std::string ns = manager_edit_->text().trimmed().toStdString();
  if (!ns.empty() && ns != manager_ns_)
  {
    connectManager(ns);
    Q_EMIT configChanged();
  }
}

void TeleopPanel::onTargetChanged(int index)
{
  std::string target = target_combo_->itemData(index).toString().toStdString();
  if (target == status_.target)
    return;
  if (target == "real" && !status_.real_is_simulated)
  {
    QMessageBox::StandardButton answer = QMessageBox::warning(
        this, "Send to the real robot",
        "Commands will move the REAL robot.\nStart approaches the leader pose slowly, then follows it.\n\n"
        "Continue?",
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer != QMessageBox::Yes)
    {
      refresh();
      return;
    }
  }
  callSetString("set_target", target);
}

void TeleopPanel::onStart()
{
  callTrigger("start");
}

void TeleopPanel::onStop()
{
  callTrigger("stop");
}

void TeleopPanel::onGoZero()
{
  QString where = status_.target == "real" ? "the REAL robot" : "the virtual robot";
  if (QMessageBox::question(this, "Zero pose", QString("Move %1 slowly to the zero pose?").arg(where),
                            QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes)
    callTrigger("go_zero");
}

void TeleopPanel::onCalibrate()
{
  if (QMessageBox::question(this, "Calibrate leader",
                            "Hold the leader arm in the pose that matches the robot's zero pose "
                            "(shoulder pointing down, arm straight down, elbow straight, trigger released).\n\n"
                            "Store this pose as zero?",
                            QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes)
    callTrigger("calibrate_leader_zero");
}

void TeleopPanel::onBrowse()
{
  QString dir = QFileDialog::getExistingDirectory(this, "Directory for rosbags", dir_edit_->text());
  if (!dir.isEmpty())
  {
    dir_edit_->setText(dir);
    onApplyDirectory();
  }
}

void TeleopPanel::onApplyDirectory()
{
  if (callSetString("set_record_directory", dir_edit_->text().trimmed().toStdString()))
  {
    dir_edited_ = false;
    Q_EMIT configChanged();
  }
}

void TeleopPanel::onRecordToggle()
{
  callTrigger(status_.recording ? "stop_recording" : "start_recording");
}

void TeleopPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  QString ns;
  if (config.mapGetString("manager", &ns) && !ns.isEmpty())
  {
    manager_edit_->setText(ns);
    connectManager(ns.toStdString());
  }
  QString dir;
  if (config.mapGetString("record_directory", &dir) && !dir.isEmpty())
  {
    pending_directory_ = dir.toStdString();
  }
}

void TeleopPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("manager", QString::fromStdString(manager_ns_));
  config.mapSetValue("record_directory", have_status_ ? QString::fromStdString(status_.record_directory) :
                                                       QString::fromStdString(pending_directory_));
}

}  // namespace k_arm_teleop

PLUGINLIB_EXPORT_CLASS(k_arm_teleop::TeleopPanel, rviz::Panel)
