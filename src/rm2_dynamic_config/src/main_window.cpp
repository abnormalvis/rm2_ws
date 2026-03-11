#include "rm2_dynamic_config/main_window.hpp"

#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QDateTime>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QComboBox>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <chrono>
#include <fstream>
#include <unordered_map>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/qos.hpp>
#include <yaml-cpp/yaml.h>

namespace rm2_dynamic_config {

namespace {

std::string normalizeNodeName(const std::string &name) {
  if (name.empty() || name.front() != '/') {
    return name;
  }
  return name.substr(1);
}

bool parseScalarDouble(const YAML::Node &node, double *value) {
  if (!node.IsScalar()) {
    return false;
  }
  try {
    *value = node.as<double>();
    return true;
  } catch (const YAML::Exception &) {
    return false;
  }
}

void flattenRosParameters(const YAML::Node &node, const std::string &prefix,
                          std::unordered_map<std::string, double> *out) {
  if (!node || !node.IsMap()) {
    return;
  }

  for (const auto &entry : node) {
    if (!entry.first.IsScalar()) {
      continue;
    }

    const std::string key = entry.first.as<std::string>();
    const std::string full_key = prefix.empty() ? key : prefix + "." + key;
    const YAML::Node value_node = entry.second;
    if (value_node.IsMap()) {
      flattenRosParameters(value_node, full_key, out);
      continue;
    }

    double value = 0.0;
    if (parseScalarDouble(value_node, &value)) {
      (*out)[full_key] = value;
    }
  }
}

}  // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      param_specs_({
          {"publish_rate", 1.0, 500.0, 1.0, 0},
          {"yaw_k_v", -20.0, 20.0, 0.01, 3},
          {"pitch_k_v", -20.0, 20.0, 0.01, 3},
          {"chassis_compensation.a", -20.0, 20.0, 0.01, 3},
          {"chassis_compensation.b", -50.0, 50.0, 0.01, 3},
          {"chassis_compensation.c", -20.0, 20.0, 0.01, 3},
          {"chassis_compensation.d", -20.0, 20.0, 0.01, 3},
          {"chassis_vel.cutoff_frequency", 0.1, 200.0, 0.1, 2},
          {"yaw.k_chassis_vel", -20.0, 20.0, 0.01, 3},
          {"yaw.resistance_compensation.resistance", -20.0, 20.0, 0.01, 3},
          {"yaw.resistance_compensation.error_tolerance", 0.0, 10.0, 0.01, 3},
      }) {
  node_ = std::make_shared<rclcpp::Node>("rm2_dynamic_config_gui");
  buildUi();

  spin_timer_ = new QTimer(this);
  spin_timer_->setInterval(20);
  connect(spin_timer_, &QTimer::timeout, this, &MainWindow::spinRosOnce);
  spin_timer_->start();

  immediate_apply_timer_ = new QTimer(this);
  immediate_apply_timer_->setSingleShot(true);
  immediate_apply_timer_->setInterval(180);
  connect(immediate_apply_timer_, &QTimer::timeout, this,
          &MainWindow::onImmediateApplyTimeout);
}

void MainWindow::buildUi() {
  auto *central = new QWidget(this);
  auto *main_layout = new QVBoxLayout(central);

  auto *target_layout = new QHBoxLayout();
  target_layout->addWidget(new QLabel("Target Node", this));
  target_node_edit_ = new QLineEdit("/gimbal_controller", this);
  target_layout->addWidget(target_node_edit_);

  connect_btn_ = new QPushButton("Connect", this);
  refresh_btn_ = new QPushButton("Refresh", this);
  apply_btn_ = new QPushButton("Apply", this);
  export_yaml_btn_ = new QPushButton("Export YAML", this);
  import_yaml_btn_ = new QPushButton("Import YAML", this);
  submit_mode_combo_ = new QComboBox(this);
  submit_mode_combo_->addItem("Batch Apply", static_cast<int>(SubmitMode::Batch));
  submit_mode_combo_->addItem("Immediate", static_cast<int>(SubmitMode::Immediate));
  target_layout->addWidget(connect_btn_);
  target_layout->addWidget(refresh_btn_);
  target_layout->addWidget(apply_btn_);
  target_layout->addWidget(export_yaml_btn_);
  target_layout->addWidget(import_yaml_btn_);
  target_layout->addWidget(new QLabel("Mode", this));
  target_layout->addWidget(submit_mode_combo_);

  main_layout->addLayout(target_layout);

  auto *form_layout = new QFormLayout();
  for (const auto &spec : param_specs_) {
    auto *input = new QDoubleSpinBox(this);
    input->setRange(spec.min, spec.max);
    input->setSingleStep(spec.step);
    input->setDecimals(spec.decimals);
    input->setEnabled(false);

    const QString key = QString::fromStdString(spec.name);
    form_layout->addRow(key, input);
    param_inputs_.insert(key, input);
    connect(input, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this, key](double) { onParameterInputChanged(key); });
  }

  main_layout->addLayout(form_layout);

  status_label_ = new QLabel("Disconnected", this);
  main_layout->addWidget(status_label_);

  setCentralWidget(central);
  resize(760, 520);
  setWindowTitle("RM2 Dynamic Config");

  connect(connect_btn_, &QPushButton::clicked, this,
          &MainWindow::onConnectClicked);
  connect(refresh_btn_, &QPushButton::clicked, this,
          &MainWindow::onRefreshClicked);
  connect(apply_btn_, &QPushButton::clicked, this, &MainWindow::onApplyClicked);
    connect(export_yaml_btn_, &QPushButton::clicked, this,
      &MainWindow::onExportYamlClicked);
    connect(import_yaml_btn_, &QPushButton::clicked, this,
      &MainWindow::onImportYamlClicked);
  connect(submit_mode_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
      this, [this](int) { updateUiState(); });

  updateUiState();
}

void MainWindow::setStatus(const QString &text) {
  status_label_->setText(text);
}

void MainWindow::setControlsEnabled(bool enabled) {
  for (auto it = param_inputs_.begin(); it != param_inputs_.end(); ++it) {
    it.value()->setEnabled(enabled);
  }
  updateUiState();
}

void MainWindow::updateUiState() {
  const bool has_dirty = !dirty_params_.isEmpty();
  const bool connected = connected_;
  const auto mode = currentSubmitMode();

  refresh_btn_->setEnabled(connected);
  export_yaml_btn_->setEnabled(connected);
  import_yaml_btn_->setEnabled(connected);
  submit_mode_combo_->setEnabled(connected);
  apply_btn_->setEnabled(connected && mode == SubmitMode::Batch && has_dirty);
}

void MainWindow::onConnectClicked() {
  const std::string target = target_node_edit_->text().trimmed().toStdString();
  if (target.empty()) {
    setStatus("Target node cannot be empty");
    return;
  }

  param_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>(node_, target);

  if (!param_client_->wait_for_service(std::chrono::seconds(2))) {
    connected_ = false;
    connected_target_node_.clear();
    parameter_event_sub_.reset();
    setControlsEnabled(false);
    setStatus(QString("Service unreachable for target node: %1")
                  .arg(QString::fromStdString(target)));
    return;
  }

  connected_ = true;
  connected_target_node_ = QString::fromStdString(target);
  dirty_params_.clear();
  immediate_pending_params_.clear();
  subscribeParameterEvents();
  setControlsEnabled(true);
  setStatus(QString("Connected: %1").arg(connected_target_node_));
  requestRefresh();
}

void MainWindow::onRefreshClicked() {
  requestRefresh();
}

void MainWindow::requestRefresh() {
  if (!connected_ || !param_client_) {
    setStatus("Not connected");
    return;
  }

  std::vector<std::string> names;
  names.reserve(param_specs_.size());
  for (const auto &spec : param_specs_) {
    names.push_back(spec.name);
  }

  auto future = param_client_->get_parameters(names);
  const auto refresh_status =
      rclcpp::spin_until_future_complete(node_, future, std::chrono::milliseconds(800));
  if (refresh_status != rclcpp::FutureReturnCode::SUCCESS) {
    setStatus("Refresh timeout: parameter service did not respond");
    return;
  }

  const auto params = future.get();
  if (params.size() != param_specs_.size()) {
    setStatus("Refresh failed: unexpected parameter count");
    return;
  }

  updating_widgets_ = true;
  for (const auto &param : params) {
    const QString key = QString::fromStdString(param.get_name());
    if (!param_inputs_.contains(key)) {
      continue;
    }

    QSignalBlocker blocker(param_inputs_[key]);
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      param_inputs_[key]->setValue(param.as_double());
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      param_inputs_[key]->setValue(static_cast<double>(param.as_int()));
    }
  }
  updating_widgets_ = false;

  dirty_params_.clear();
  immediate_pending_params_.clear();
  updateUiState();
  setStatus("Refresh ok");
}

void MainWindow::onApplyClicked() {
  if (currentSubmitMode() != SubmitMode::Batch) {
    setStatus("Immediate mode enabled: edits are applied automatically");
    return;
  }

  const auto params = buildParameterList(true);
  if (params.empty()) {
    setStatus("No pending parameter changes");
    return;
  }

  if (applyParameters(params, "Apply")) {
    for (const auto &param : params) {
      dirty_params_.remove(QString::fromStdString(param.get_name()));
    }
    updateUiState();
  }
}

void MainWindow::onExportYamlClicked() {
  if (!connected_) {
    setStatus("Export unavailable: connect to target node first");
    return;
  }

  const QString node_key =
      QString::fromStdString(normalizeNodeName(connected_target_node_.toStdString()));
  const QString default_name = QString("%1_params_%2.yaml")
                                   .arg(node_key.isEmpty() ? "gimbal_controller" : node_key)
                                   .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
  const QString suggested = last_yaml_path_.isEmpty()
                                ? default_name
                                : QFileInfo(last_yaml_path_).absolutePath() + "/" + default_name;
  const QString file_path = QFileDialog::getSaveFileName(
      this, "Export YAML", suggested, "YAML files (*.yaml *.yml)");
  if (file_path.isEmpty()) {
    return;
  }

  if (!exportSnapshot(file_path)) {
    return;
  }

  last_yaml_path_ = file_path;
  setStatus(QString("Export complete: %1 parameters saved")
                .arg(static_cast<int>(param_specs_.size())));
}

void MainWindow::onImportYamlClicked() {
  if (!connected_ || !param_client_) {
    setStatus("Import unavailable: connect to target node first");
    return;
  }

  const QString start_dir = last_yaml_path_.isEmpty()
                                ? QString()
                                : QFileInfo(last_yaml_path_).absolutePath();
  const QString file_path = QFileDialog::getOpenFileName(
      this, "Import YAML", start_dir, "YAML files (*.yaml *.yml)");
  if (file_path.isEmpty()) {
    return;
  }

  std::vector<rclcpp::Parameter> params;
  QString warning;
  if (!importSnapshot(file_path, &params, &warning)) {
    return;
  }

  if (!applyParameters(params, "Import apply")) {
    return;
  }

  updateWidgets(params);
  dirty_params_.clear();
  immediate_pending_params_.clear();
  updateUiState();

  last_yaml_path_ = file_path;
  if (warning.isEmpty()) {
    setStatus(QString("Import complete: %1 parameters applied")
                  .arg(static_cast<int>(params.size())));
  } else {
    setStatus(QString("Import complete: %1 (%2)")
                  .arg(static_cast<int>(params.size()))
                  .arg(warning));
  }
}

void MainWindow::onImmediateApplyTimeout() {
  if (currentSubmitMode() != SubmitMode::Immediate) {
    immediate_pending_params_.clear();
    return;
  }

  if (immediate_pending_params_.isEmpty()) {
    return;
  }

  std::vector<rclcpp::Parameter> params;
  params.reserve(immediate_pending_params_.size());
  for (const auto &key : immediate_pending_params_) {
    const auto *widget = param_inputs_.value(key, nullptr);
    if (!widget) {
      continue;
    }
    params.emplace_back(key.toStdString(), widget->value());
  }

  if (params.empty()) {
    return;
  }

  if (applyParameters(params, "Immediate apply")) {
    for (const auto &param : params) {
      const QString key = QString::fromStdString(param.get_name());
      dirty_params_.remove(key);
      immediate_pending_params_.remove(key);
    }
    updateUiState();
  }
}

void MainWindow::onParameterInputChanged(const QString &name) {
  if (!connected_ || updating_widgets_) {
    return;
  }

  dirty_params_.insert(name);
  if (currentSubmitMode() == SubmitMode::Immediate) {
    immediate_pending_params_.insert(name);
    immediate_apply_timer_->start();
    setStatus(QString("Pending immediate apply: %1").arg(name));
  } else {
    setStatus(QString("Pending changes: %1").arg(dirty_params_.size()));
  }
  updateUiState();
}

void MainWindow::updateWidgets(const std::vector<rclcpp::Parameter> &params) {
  updating_widgets_ = true;
  for (const auto &param : params) {
    const QString key = QString::fromStdString(param.get_name());
    auto *widget = param_inputs_.value(key, nullptr);
    if (!widget) {
      continue;
    }

    QSignalBlocker blocker(widget);
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      widget->setValue(param.as_double());
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      widget->setValue(static_cast<double>(param.as_int()));
    }
  }
  updating_widgets_ = false;
}

void MainWindow::subscribeParameterEvents() {
  parameter_event_sub_ = node_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events", rclcpp::ParameterEventsQoS(),
      std::bind(&MainWindow::handleParameterEvents, this, std::placeholders::_1));
}

void MainWindow::handleParameterEvents(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr msg) {
  if (!connected_ || connected_target_node_.isEmpty()) {
    return;
  }

  auto normalize_node = [](const std::string &name) {
    if (name.empty() || name.front() == '/') {
      return name;
    }
    return std::string("/") + name;
  };

  if (normalize_node(msg->node) !=
      normalize_node(connected_target_node_.toStdString())) {
    return;
  }

  bool updated_any = false;
  const auto handle_value_update = [this, &updated_any](
                                     const rcl_interfaces::msg::Parameter &parameter) {
    const QString key = QString::fromStdString(parameter.name);
    auto *widget = param_inputs_.value(key, nullptr);
    if (!widget || widget->hasFocus()) {
      return;
    }

    double value = 0.0;
    if (parameter.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      value = parameter.value.double_value;
    } else if (parameter.value.type ==
               rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      value = static_cast<double>(parameter.value.integer_value);
    } else {
      return;
    }

    updating_widgets_ = true;
    QSignalBlocker blocker(widget);
    widget->setValue(value);
    updating_widgets_ = false;
    dirty_params_.remove(key);
    immediate_pending_params_.remove(key);
    updated_any = true;
  };

  for (const auto &parameter : msg->new_parameters) {
    handle_value_update(parameter);
  }
  for (const auto &parameter : msg->changed_parameters) {
    handle_value_update(parameter);
  }

  if (!msg->deleted_parameters.empty()) {
    setStatus("Parameter event: some parameters were deleted on target node");
  } else if (updated_any) {
    setStatus("Auto-sync: parameters updated from /parameter_events");
  }

  updateUiState();
}

std::vector<rclcpp::Parameter> MainWindow::buildParameterList(bool only_dirty) const {
  std::vector<rclcpp::Parameter> params;
  params.reserve(param_specs_.size());

  for (const auto &spec : param_specs_) {
    const QString key = QString::fromStdString(spec.name);
    if (only_dirty && !dirty_params_.contains(key)) {
      continue;
    }

    const auto *widget = param_inputs_.value(key, nullptr);
    if (!widget) {
      continue;
    }
    params.emplace_back(spec.name, widget->value());
  }

  return params;
}

bool MainWindow::exportSnapshot(const QString &file_path) {
  YAML::Node root;
  YAML::Node snapshot;
  YAML::Node metadata;
  metadata["target_node"] = connected_target_node_.toStdString();
  metadata["export_time"] = QDateTime::currentDateTimeUtc().toString(Qt::ISODate).toStdString();
  snapshot["metadata"] = metadata;

  YAML::Node params_node;
  for (const auto &spec : param_specs_) {
    const QString key = QString::fromStdString(spec.name);
    const auto *widget = param_inputs_.value(key, nullptr);
    if (!widget) {
      continue;
    }
    params_node[spec.name] = widget->value();
  }
  snapshot["parameters"] = params_node;
  root["snapshot"] = snapshot;

  std::ofstream fout(file_path.toStdString());
  if (!fout.is_open()) {
    setStatus(QString("Export failed: cannot open file %1").arg(file_path));
    return false;
  }

  fout << root;
  if (!fout.good()) {
    setStatus(QString("Export failed: write error %1").arg(file_path));
    return false;
  }

  return true;
}

bool MainWindow::importSnapshot(const QString &file_path,
                                std::vector<rclcpp::Parameter> *params,
                                QString *warning) {
  if (!params || !warning) {
    return false;
  }

  params->clear();
  warning->clear();

  YAML::Node root;
  try {
    root = YAML::LoadFile(file_path.toStdString());
  } catch (const YAML::Exception &e) {
    setStatus(QString("Import failed: YAML parse error (%1)").arg(e.what()));
    return false;
  }

  std::unordered_map<std::string, double> imported;
  bool parsed = false;

  const YAML::Node snapshot_node = root["snapshot"];
  if (snapshot_node && snapshot_node["parameters"] && snapshot_node["parameters"].IsMap()) {
    const YAML::Node flat_params = snapshot_node["parameters"];
    for (const auto &entry : flat_params) {
      if (!entry.first.IsScalar()) {
        continue;
      }
      double value = 0.0;
      if (!parseScalarDouble(entry.second, &value)) {
        continue;
      }
      imported[entry.first.as<std::string>()] = value;
    }
    parsed = true;
  }

  if (!parsed && root["parameters"] && root["parameters"].IsMap()) {
    const YAML::Node flat_params = root["parameters"];
    for (const auto &entry : flat_params) {
      if (!entry.first.IsScalar()) {
        continue;
      }
      double value = 0.0;
      if (!parseScalarDouble(entry.second, &value)) {
        continue;
      }
      imported[entry.first.as<std::string>()] = value;
    }
    parsed = true;
  }

  if (!parsed) {
    const std::string node_key = normalizeNodeName(connected_target_node_.toStdString());
    const YAML::Node node_params = root[node_key]["ros__parameters"];
    if (!node_params || !node_params.IsMap()) {
      setStatus("Import failed: no supported parameter section found in YAML");
      return false;
    }
    flattenRosParameters(node_params, "", &imported);
  }

  int unknown_count = 0;
  for (const auto &item : imported) {
    const QString key = QString::fromStdString(item.first);
    if (!param_inputs_.contains(key)) {
      ++unknown_count;
      continue;
    }
    params->emplace_back(item.first, item.second);
  }

  if (params->empty()) {
    setStatus("Import failed: import file contains no supported parameters");
    return false;
  }

  if (unknown_count > 0) {
    *warning = QString("skipped %1 unknown parameters").arg(unknown_count);
  }

  return true;
}

bool MainWindow::applyParameters(const std::vector<rclcpp::Parameter> &params,
                                 const QString &source) {
  if (!connected_ || !param_client_) {
    setStatus("Not connected");
    return false;
  }

  if (params.empty()) {
    setStatus("No parameters to apply");
    return false;
  }

  auto future = param_client_->set_parameters(params);
  const auto apply_status =
      rclcpp::spin_until_future_complete(node_, future, std::chrono::milliseconds(800));
  if (apply_status != rclcpp::FutureReturnCode::SUCCESS) {
    setStatus(QString("%1 timeout: parameter service did not respond").arg(source));
    return false;
  }

  const auto results = future.get();
  for (const auto &result : results) {
    if (!result.successful) {
      setStatus(QString("%1 rejected: %2")
                    .arg(source, QString::fromStdString(result.reason)));
      return false;
    }
  }

  setStatus(QString("%1 ok").arg(source));
  return true;
}

MainWindow::SubmitMode MainWindow::currentSubmitMode() const {
  if (!submit_mode_combo_) {
    return SubmitMode::Batch;
  }

  return static_cast<SubmitMode>(submit_mode_combo_->currentData().toInt());
}

void MainWindow::spinRosOnce() { rclcpp::spin_some(node_); }

}  // namespace rm2_dynamic_config
