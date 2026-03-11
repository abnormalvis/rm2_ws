#include "rm2_dynamic_config/main_window.hpp"

#include <QComboBox>
#include <QCheckBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
  #include <QScrollArea>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <unordered_map>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/qos.hpp>
#include <yaml-cpp/yaml.h>

namespace rm2_dynamic_config {

namespace {

constexpr const char *kFallbackTargetNode = "/gimbal_controller";

#ifdef RM2_DYNAMIC_CONFIG_DEFAULT_SCHEMA
constexpr const char *kDefaultSchemaPath = RM2_DYNAMIC_CONFIG_DEFAULT_SCHEMA;
#else
constexpr const char *kDefaultSchemaPath = "";
#endif

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

bool parseScalarBool(const YAML::Node &node, bool *value) {
  if (!node.IsScalar()) {
    return false;
  }
  try {
    *value = node.as<bool>();
    return true;
  } catch (const YAML::Exception &) {
    return false;
  }
}

bool parseScalarInteger(const YAML::Node &node, int64_t *value) {
  if (!node.IsScalar()) {
    return false;
  }
  try {
    *value = node.as<int64_t>();
    return true;
  } catch (const YAML::Exception &) {
    return false;
  }
}

bool parseScalarString(const YAML::Node &node, std::string *value) {
  if (!node.IsScalar()) {
    return false;
  }
  try {
    *value = node.as<std::string>();
    return true;
  } catch (const YAML::Exception &) {
    return false;
  }
}

void flattenRosParameters(const YAML::Node &node, const std::string &prefix,
                          std::unordered_map<std::string, YAML::Node> *out) {
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

    (*out)[full_key] = value_node;
  }
}

ParamValueType parseParamType(const std::string &type_str) {
  if (type_str == "bool" || type_str == "boolean") {
    return ParamValueType::kBool;
  }
  if (type_str == "int" || type_str == "integer") {
    return ParamValueType::kInteger;
  }
  if (type_str == "string") {
    return ParamValueType::kString;
  }
  if (type_str == "string_array") {
    return ParamValueType::kStringArray;
  }
  if (type_str == "double_array") {
    return ParamValueType::kDoubleArray;
  }
  return ParamValueType::kDouble;
}

QString yamlNodeToEditorText(const YAML::Node &node) {
  YAML::Emitter emitter;
  emitter << node;
  return QString::fromStdString(emitter.c_str());
}

YAML::Node parameterToYamlNode(const rclcpp::Parameter &param) {
  YAML::Node node;
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      node = param.as_bool();
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      node = param.as_int();
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      node = param.as_double();
      break;
    case rclcpp::ParameterType::PARAMETER_STRING:
      node = param.as_string();
      break;
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: {
      for (const auto &value : param.as_string_array()) {
        node.push_back(value);
      }
      break;
    }
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
      for (double value : param.as_double_array()) {
        node.push_back(value);
      }
      break;
    }
    default:
      break;
  }
  return node;
}

void clearLayout(QLayout *layout) {
  if (!layout) {
    return;
  }

  QLayoutItem *item = nullptr;
  while ((item = layout->takeAt(0)) != nullptr) {
    if (item->layout()) {
      clearLayout(item->layout());
      delete item->layout();
    }
    if (item->widget()) {
      delete item->widget();
    }
    delete item;
  }
}

}  // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent) {
  node_ = std::make_shared<rclcpp::Node>("rm2_dynamic_config_gui");
  loadDefaultSchema();

  QString schema_load_result;
  if (std::strlen(kDefaultSchemaPath) > 0) {
    schema_path_ = QString::fromUtf8(kDefaultSchemaPath);
    QString error;
    if (!loadSchemaFile(schema_path_, &error)) {
      schema_load_result = QString("Schema fallback: %1").arg(error);
    }
  }

  if (const char *env_schema = std::getenv("RM2_DYNAMIC_CONFIG_SCHEMA");
      env_schema != nullptr && std::strlen(env_schema) > 0) {
    const QString env_path = QString::fromLocal8Bit(env_schema);
    QString error;
    if (loadSchemaFile(env_path, &error)) {
      schema_path_ = env_path;
      schema_load_result = QString("Schema loaded: %1").arg(schema_path_);
    } else {
      schema_load_result = QString("Schema fallback: %1").arg(error);
    }
  }

  buildUi();
  if (!schema_load_result.isEmpty()) {
    setStatus(schema_load_result);
  }

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
  target_node_edit_ = new QLineEdit(schema_target_node_default_, this);
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

  auto *param_editor_widget = new QWidget(this);
  param_groups_layout_ = new QVBoxLayout(param_editor_widget);
  param_groups_layout_->setContentsMargins(0, 0, 0, 0);
  param_groups_layout_->setSpacing(8);

  auto *param_scroll_area = new QScrollArea(this);
  param_scroll_area->setWidgetResizable(true);
  param_scroll_area->setFrameShape(QFrame::NoFrame);
  param_scroll_area->setWidget(param_editor_widget);
  main_layout->addWidget(param_scroll_area, 1);

  rebuildParameterEditors();

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
    if (it.value()) {
      it.value()->setEnabled(enabled);
    }
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
  if (immediate_apply_timer_) {
    immediate_apply_timer_->stop();
  }
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
  updateWidgets(params);

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
    rclcpp::Parameter param;
    if (readWidgetParameter(key, &param)) {
      params.push_back(param);
    }
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
    setWidgetParameterValue(QString::fromStdString(param.get_name()), param);
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

    rclcpp::Parameter converted;
    switch (parameter.value.type) {
      case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
        converted = rclcpp::Parameter(parameter.name, parameter.value.bool_value);
        break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
        converted = rclcpp::Parameter(parameter.name, parameter.value.integer_value);
        break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
        converted = rclcpp::Parameter(parameter.name, parameter.value.double_value);
        break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
        converted = rclcpp::Parameter(parameter.name, parameter.value.string_value);
        break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
        converted = rclcpp::Parameter(parameter.name, parameter.value.string_array_value);
        break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:
        converted = rclcpp::Parameter(parameter.name, parameter.value.double_array_value);
        break;
      default:
        return;
    }

    updating_widgets_ = true;
    if (!setWidgetParameterValue(key, converted)) {
      updating_widgets_ = false;
      return;
    }
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

    rclcpp::Parameter param;
    if (readWidgetParameter(key, &param)) {
      params.push_back(param);
    }
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
    rclcpp::Parameter param;
    if (!readWidgetParameter(key, &param)) {
      continue;
    }

    params_node[spec.name] = parameterToYamlNode(param);
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

  std::unordered_map<std::string, YAML::Node> imported;
  bool parsed = false;

  const YAML::Node snapshot_node = root["snapshot"];
  if (snapshot_node && snapshot_node["parameters"] && snapshot_node["parameters"].IsMap()) {
    const YAML::Node flat_params = snapshot_node["parameters"];
    for (const auto &entry : flat_params) {
      if (!entry.first.IsScalar()) {
        continue;
      }
      imported[entry.first.as<std::string>()] = entry.second;
    }
    parsed = true;
  }

  if (!parsed && root["parameters"] && root["parameters"].IsMap()) {
    const YAML::Node flat_params = root["parameters"];
    for (const auto &entry : flat_params) {
      if (!entry.first.IsScalar()) {
        continue;
      }
      imported[entry.first.as<std::string>()] = entry.second;
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
    if (!param_spec_map_.contains(key)) {
      ++unknown_count;
      continue;
    }

    rclcpp::Parameter param;
    if (!makeParameterFromYamlNode(key, item.second, &param)) {
      continue;
    }
    params->push_back(param);
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

void MainWindow::loadDefaultSchema() {
  schema_target_node_default_ = kFallbackTargetNode;
  param_specs_ = {
      {"publish_rate", "publish_rate", "Controller", "", ParamValueType::kInteger,
       1.0, 500.0, 1.0, 0},
      {"yaw_k_v", "yaw_k_v", "Velocity", "", ParamValueType::kDouble,
       -20.0, 20.0, 0.01, 3},
      {"pitch_k_v", "pitch_k_v", "Velocity", "", ParamValueType::kDouble,
       -20.0, 20.0, 0.01, 3},
      {"chassis_compensation.a", "chassis_compensation.a", "Compensation", "",
       ParamValueType::kDouble, -20.0, 20.0, 0.01, 3},
      {"chassis_compensation.b", "chassis_compensation.b", "Compensation", "",
       ParamValueType::kDouble, -50.0, 50.0, 0.01, 3},
      {"chassis_compensation.c", "chassis_compensation.c", "Compensation", "",
       ParamValueType::kDouble, -20.0, 20.0, 0.01, 3},
      {"chassis_compensation.d", "chassis_compensation.d", "Compensation", "",
       ParamValueType::kDouble, -20.0, 20.0, 0.01, 3},
      {"chassis_vel.cutoff_frequency", "chassis_vel.cutoff_frequency", "Compensation", "",
       ParamValueType::kDouble, 0.1, 200.0, 0.1, 2},
      {"yaw.k_chassis_vel", "yaw.k_chassis_vel", "Yaw", "", ParamValueType::kDouble,
       -20.0, 20.0, 0.01, 3},
      {"yaw.resistance_compensation.resistance", "yaw.resistance_compensation.resistance",
       "Yaw", "", ParamValueType::kDouble, -20.0, 20.0, 0.01, 3},
      {"yaw.resistance_compensation.error_tolerance",
       "yaw.resistance_compensation.error_tolerance", "Yaw", "", ParamValueType::kDouble,
       0.0, 10.0, 0.01, 3},
  };

  param_spec_map_.clear();
  for (const auto &spec : param_specs_) {
    param_spec_map_.insert(QString::fromStdString(spec.name), spec);
  }
}

bool MainWindow::loadSchemaFile(const QString &schema_path, QString *error) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(schema_path.toStdString());
  } catch (const YAML::Exception &e) {
    if (error) {
      *error = QString("cannot parse schema %1 (%2)").arg(schema_path, e.what());
    }
    return false;
  }

  const YAML::Node parameters = root["parameters"];
  if (!parameters || !parameters.IsSequence()) {
    if (error) {
      *error = QString("schema %1 missing 'parameters' list").arg(schema_path);
    }
    return false;
  }

  std::vector<ParamSpec> parsed_specs;
  parsed_specs.reserve(parameters.size());
  QMap<QString, ParamSpec> parsed_map;

  for (std::size_t i = 0; i < parameters.size(); ++i) {
    const YAML::Node entry = parameters[i];
    if (!entry.IsMap() || !entry["name"] || !entry["name"].IsScalar()) {
      continue;
    }

    ParamSpec spec;
    spec.name = entry["name"].as<std::string>();
    spec.label = entry["label"] && entry["label"].IsScalar()
                     ? entry["label"].as<std::string>()
                     : spec.name;
    spec.group = entry["group"] && entry["group"].IsScalar()
                     ? entry["group"].as<std::string>()
                     : "General";
    spec.description = entry["description"] && entry["description"].IsScalar()
                           ? entry["description"].as<std::string>()
                           : "";
    spec.type = entry["type"] && entry["type"].IsScalar()
                    ? parseParamType(entry["type"].as<std::string>())
                    : ParamValueType::kDouble;

    const bool is_integer = spec.type == ParamValueType::kInteger;
    const bool is_numeric =
        spec.type == ParamValueType::kInteger || spec.type == ParamValueType::kDouble;
    spec.min = entry["min"] && entry["min"].IsScalar()
                   ? entry["min"].as<double>()
                   : (is_integer ? -1000.0 : -100.0);
    spec.max = entry["max"] && entry["max"].IsScalar()
                   ? entry["max"].as<double>()
                   : (is_integer ? 1000.0 : 100.0);
    spec.step = entry["step"] && entry["step"].IsScalar()
                    ? entry["step"].as<double>()
                    : (is_integer ? 1.0 : 0.01);
    spec.decimals = entry["decimals"] && entry["decimals"].IsScalar()
                        ? entry["decimals"].as<int>()
                        : (is_integer ? 0 : 3);

    if (is_numeric && spec.max < spec.min) {
      std::swap(spec.min, spec.max);
    }
    if (is_numeric && spec.step <= 0.0) {
      spec.step = is_integer ? 1.0 : 0.01;
    }
    if (spec.decimals < 0) {
      spec.decimals = is_integer ? 0 : 3;
    }

    const QString key = QString::fromStdString(spec.name);
    if (parsed_map.contains(key)) {
      continue;
    }

    parsed_map.insert(key, spec);
    parsed_specs.push_back(spec);
  }

  if (parsed_specs.empty()) {
    if (error) {
      *error = QString("schema %1 has no valid parameter entries").arg(schema_path);
    }
    return false;
  }

  if (root["target_node_default"] && root["target_node_default"].IsScalar()) {
    schema_target_node_default_ =
        QString::fromStdString(root["target_node_default"].as<std::string>());
  }

  param_specs_ = parsed_specs;
  param_spec_map_ = parsed_map;
  return true;
}

void MainWindow::rebuildParameterEditors() {
  if (!param_groups_layout_) {
    return;
  }

  clearLayout(param_groups_layout_);
  param_inputs_.clear();

  QMap<QString, QFormLayout *> group_forms;
  for (const auto &spec : param_specs_) {
    const QString group = QString::fromStdString(spec.group.empty() ? "General" : spec.group);
    QFormLayout *form = group_forms.value(group, nullptr);
    if (!form) {
      auto *box = new QGroupBox(group, this);
      form = new QFormLayout(box);
      form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
      box->setLayout(form);
      param_groups_layout_->addWidget(box);
      group_forms.insert(group, form);
    }

    const QString key = QString::fromStdString(spec.name);
    const QString label = QString::fromStdString(spec.label);
    const QString description = QString::fromStdString(spec.description);
    QWidget *input = nullptr;

    if (spec.type == ParamValueType::kBool) {
      auto *checkbox = new QCheckBox(this);
      checkbox->setEnabled(false);
      connect(checkbox, &QCheckBox::toggled, this,
              [this, key](bool) { onParameterInputChanged(key); });
      input = checkbox;
    } else if (spec.type == ParamValueType::kInteger) {
      auto *spin = new QSpinBox(this);
      spin->setRange(static_cast<int>(std::lround(spec.min)),
                     static_cast<int>(std::lround(spec.max)));
      spin->setSingleStep(std::max(1, static_cast<int>(std::lround(spec.step))));
      spin->setEnabled(false);
      connect(spin, qOverload<int>(&QSpinBox::valueChanged), this,
              [this, key](int) { onParameterInputChanged(key); });
      input = spin;
                } else if (spec.type == ParamValueType::kString) {
                  auto *line_edit = new QLineEdit(this);
                  line_edit->setEnabled(false);
                  connect(line_edit, &QLineEdit::textChanged, this,
                    [this, key](const QString &) { onParameterInputChanged(key); });
                  input = line_edit;
                } else if (spec.type == ParamValueType::kStringArray ||
                     spec.type == ParamValueType::kDoubleArray) {
                  auto *text_edit = new QPlainTextEdit(this);
                  text_edit->setEnabled(false);
                  text_edit->setMinimumHeight(72);
                  text_edit->setPlaceholderText(spec.type == ParamValueType::kStringArray
                      ? "YAML sequence or [item1, item2]"
                      : "YAML numeric sequence, e.g. [0.0, 0.0]");
                  connect(text_edit, &QPlainTextEdit::textChanged, this,
                    [this, key]() { onParameterInputChanged(key); });
                  input = text_edit;
    } else {
      auto *spin = new QDoubleSpinBox(this);
      spin->setRange(spec.min, spec.max);
      spin->setSingleStep(spec.step);
      spin->setDecimals(spec.decimals);
      spin->setEnabled(false);
      connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
              [this, key](double) { onParameterInputChanged(key); });
      input = spin;
    }

    auto *label_widget = new QLabel(label, this);
    if (!description.isEmpty()) {
      label_widget->setToolTip(description);
      input->setToolTip(description);
    }
    form->addRow(label_widget, input);
    param_inputs_.insert(key, input);
  }

  param_groups_layout_->addStretch(1);
}

bool MainWindow::setWidgetParameterValue(const QString &name,
                                         const rclcpp::Parameter &param) {
  const auto spec_it = param_spec_map_.find(name);
  if (spec_it == param_spec_map_.end()) {
    return false;
  }

  QWidget *widget = param_inputs_.value(name, nullptr);
  if (!widget) {
    return false;
  }

  QSignalBlocker blocker(widget);
  if (spec_it->type == ParamValueType::kBool) {
    auto *checkbox = qobject_cast<QCheckBox *>(widget);
    if (!checkbox || param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
      return false;
    }
    checkbox->setChecked(param.as_bool());
    return true;
  }
  if (spec_it->type == ParamValueType::kInteger) {
    auto *spin = qobject_cast<QSpinBox *>(widget);
    if (!spin || param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
      return false;
    }
    spin->setValue(static_cast<int>(param.as_int()));
    return true;
  }

  if (spec_it->type == ParamValueType::kString) {
    auto *line_edit = qobject_cast<QLineEdit *>(widget);
    if (!line_edit || param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      return false;
    }
    line_edit->setText(QString::fromStdString(param.as_string()));
    return true;
  }

  if (spec_it->type == ParamValueType::kStringArray) {
    auto *text_edit = qobject_cast<QPlainTextEdit *>(widget);
    if (!text_edit || param.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
      return false;
    }
    text_edit->setPlainText(yamlNodeToEditorText(parameterToYamlNode(param)));
    return true;
  }

  if (spec_it->type == ParamValueType::kDoubleArray) {
    auto *text_edit = qobject_cast<QPlainTextEdit *>(widget);
    if (!text_edit || param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      return false;
    }
    text_edit->setPlainText(yamlNodeToEditorText(parameterToYamlNode(param)));
    return true;
  }

  auto *spin = qobject_cast<QDoubleSpinBox *>(widget);
  if (!spin || param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return false;
  }
  spin->setValue(param.as_double());
  return true;
}

bool MainWindow::readWidgetParameter(const QString &name,
                                     rclcpp::Parameter *out_param) const {
  if (!out_param) {
    return false;
  }

  const auto spec_it = param_spec_map_.find(name);
  if (spec_it == param_spec_map_.end()) {
    return false;
  }

  QWidget *widget = param_inputs_.value(name, nullptr);
  if (!widget) {
    return false;
  }

  if (spec_it->type == ParamValueType::kBool) {
    const auto *checkbox = qobject_cast<const QCheckBox *>(widget);
    if (!checkbox) {
      return false;
    }
    *out_param = rclcpp::Parameter(spec_it->name, checkbox->isChecked());
    return true;
  }

  if (spec_it->type == ParamValueType::kInteger) {
    const auto *spin = qobject_cast<const QSpinBox *>(widget);
    if (!spin) {
      return false;
    }
    *out_param = rclcpp::Parameter(spec_it->name, static_cast<int64_t>(spin->value()));
    return true;
  }

  if (spec_it->type == ParamValueType::kString) {
    const auto *line_edit = qobject_cast<const QLineEdit *>(widget);
    if (!line_edit) {
      return false;
    }
    *out_param = rclcpp::Parameter(spec_it->name, line_edit->text().toStdString());
    return true;
  }

  if (spec_it->type == ParamValueType::kStringArray ||
      spec_it->type == ParamValueType::kDoubleArray) {
    const auto *text_edit = qobject_cast<const QPlainTextEdit *>(widget);
    if (!text_edit) {
      return false;
    }

    const QString text = text_edit->toPlainText().trimmed();
    YAML::Node value_node;
    if (text.isEmpty()) {
      value_node = YAML::Node(YAML::NodeType::Sequence);
    } else {
      try {
        value_node = YAML::Load(text.toStdString());
      } catch (const YAML::Exception &) {
        return false;
      }
    }
    return makeParameterFromYamlNode(name, value_node, out_param);
  }

  const auto *spin = qobject_cast<const QDoubleSpinBox *>(widget);
  if (!spin) {
    return false;
  }
  *out_param = rclcpp::Parameter(spec_it->name, spin->value());
  return true;
}

bool MainWindow::makeParameterFromYamlNode(const QString &name,
                                           const YAML::Node &value_node,
                                           rclcpp::Parameter *out_param) const {
  if (!out_param) {
    return false;
  }

  const auto spec_it = param_spec_map_.find(name);
  if (spec_it == param_spec_map_.end()) {
    return false;
  }

  switch (spec_it->type) {
    case ParamValueType::kBool: {
      bool value = false;
      if (!parseScalarBool(value_node, &value)) {
        return false;
      }
      *out_param = rclcpp::Parameter(spec_it->name, value);
      return true;
    }
    case ParamValueType::kInteger: {
      int64_t value = 0;
      if (parseScalarInteger(value_node, &value)) {
        *out_param = rclcpp::Parameter(spec_it->name, value);
        return true;
      }
      double double_value = 0.0;
      if (!parseScalarDouble(value_node, &double_value)) {
        return false;
      }
      *out_param =
          rclcpp::Parameter(spec_it->name, static_cast<int64_t>(std::lround(double_value)));
      return true;
    }
    case ParamValueType::kDouble: {
      double value = 0.0;
      if (!parseScalarDouble(value_node, &value)) {
        return false;
      }
      *out_param = rclcpp::Parameter(spec_it->name, value);
      return true;
    }
    case ParamValueType::kString: {
      std::string value;
      if (!parseScalarString(value_node, &value)) {
        return false;
      }
      *out_param = rclcpp::Parameter(spec_it->name, value);
      return true;
    }
    case ParamValueType::kStringArray: {
      if (!value_node.IsSequence()) {
        if (value_node.IsScalar()) {
          std::string value;
          if (!parseScalarString(value_node, &value)) {
            return false;
          }
          *out_param = rclcpp::Parameter(spec_it->name, std::vector<std::string>{value});
          return true;
        }
        return false;
      }
      std::vector<std::string> values;
      values.reserve(value_node.size());
      for (const auto &entry : value_node) {
        std::string value;
        if (!parseScalarString(entry, &value)) {
          return false;
        }
        values.push_back(value);
      }
      *out_param = rclcpp::Parameter(spec_it->name, values);
      return true;
    }
    case ParamValueType::kDoubleArray: {
      if (!value_node.IsSequence()) {
        if (value_node.IsScalar()) {
          double value = 0.0;
          if (!parseScalarDouble(value_node, &value)) {
            return false;
          }
          *out_param = rclcpp::Parameter(spec_it->name, std::vector<double>{value});
          return true;
        }
        return false;
      }
      std::vector<double> values;
      values.reserve(value_node.size());
      for (const auto &entry : value_node) {
        double value = 0.0;
        if (!parseScalarDouble(entry, &value)) {
          return false;
        }
        values.push_back(value);
      }
      *out_param = rclcpp::Parameter(spec_it->name, values);
      return true;
    }
  }

  return false;
}

MainWindow::SubmitMode MainWindow::currentSubmitMode() const {
  if (!submit_mode_combo_) {
    return SubmitMode::Batch;
  }

  return static_cast<SubmitMode>(submit_mode_combo_->currentData().toInt());
}

void MainWindow::spinRosOnce() { rclcpp::spin_some(node_); }

}  // namespace rm2_dynamic_config
