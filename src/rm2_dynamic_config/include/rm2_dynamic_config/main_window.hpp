#pragma once

#include <QMainWindow>
#include <QMap>
#include <QSet>
#include <QString>

#include <string>
#include <vector>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rclcpp/rclcpp.hpp>

class QLabel;
class QLineEdit;
class QPushButton;
class QDoubleSpinBox;
class QSpinBox;
class QComboBox;
class QTimer;
class QVBoxLayout;
class QWidget;

namespace rm2_dynamic_config {

enum class ParamValueType { kDouble, kInteger };

struct ParamSpec {
  std::string name;
  std::string label;
  std::string group;
  ParamValueType type;
  double min;
  double max;
  double step;
  int decimals;
};

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow() override = default;

private slots:
  void onConnectClicked();
  void onRefreshClicked();
  void onApplyClicked();
  void onExportYamlClicked();
  void onImportYamlClicked();
  void spinRosOnce();
  void onImmediateApplyTimeout();

private:
  enum class SubmitMode { Batch = 0, Immediate = 1 };

  void buildUi();
  void setStatus(const QString &text);
  void setControlsEnabled(bool enabled);
  void updateUiState();
  void requestRefresh();
  void subscribeParameterEvents();
  void onParameterInputChanged(const QString &name);
  void updateWidgets(const std::vector<rclcpp::Parameter> &params);
  void handleParameterEvents(
      const rcl_interfaces::msg::ParameterEvent::SharedPtr msg);
  void loadDefaultSchema();
  bool loadSchemaFile(const QString &schema_path, QString *error);
  void rebuildParameterEditors();
  bool setWidgetNumericValue(const QString &name, double value);
  bool readWidgetParameter(const QString &name, rclcpp::Parameter *out_param) const;
  bool makeParameterFromDouble(const QString &name, double value,
                               rclcpp::Parameter *out_param) const;
  bool exportSnapshot(const QString &file_path);
  bool importSnapshot(const QString &file_path,
                      std::vector<rclcpp::Parameter> *params,
                      QString *warning);
  std::vector<rclcpp::Parameter> buildParameterList(bool only_dirty) const;
  bool applyParameters(const std::vector<rclcpp::Parameter> &params,
                       const QString &source);
  SubmitMode currentSubmitMode() const;

  std::vector<ParamSpec> param_specs_;
  QMap<QString, ParamSpec> param_spec_map_;
  QMap<QString, QWidget *> param_inputs_;
  QVBoxLayout *param_groups_layout_{nullptr};

  QLineEdit *target_node_edit_{nullptr};
  QPushButton *connect_btn_{nullptr};
  QPushButton *refresh_btn_{nullptr};
  QPushButton *apply_btn_{nullptr};
  QPushButton *export_yaml_btn_{nullptr};
  QPushButton *import_yaml_btn_{nullptr};
  QComboBox *submit_mode_combo_{nullptr};
  QLabel *status_label_{nullptr};
  QTimer *spin_timer_{nullptr};
  QTimer *immediate_apply_timer_{nullptr};

  rclcpp::Node::SharedPtr node_;
  rclcpp::AsyncParametersClient::SharedPtr param_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
      parameter_event_sub_;
  bool connected_{false};
  bool updating_widgets_{false};
  QString connected_target_node_;
  QString schema_path_;
  QString schema_target_node_default_;
  QString last_yaml_path_;
  QSet<QString> dirty_params_;
  QSet<QString> immediate_pending_params_;
};

}  // namespace rm2_dynamic_config
