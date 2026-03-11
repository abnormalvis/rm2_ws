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
class QComboBox;
class QTimer;

namespace rm2_dynamic_config {

struct ParamSpec {
  std::string name;
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
  bool exportSnapshot(const QString &file_path);
  bool importSnapshot(const QString &file_path,
                      std::vector<rclcpp::Parameter> *params,
                      QString *warning);
  std::vector<rclcpp::Parameter> buildParameterList(bool only_dirty) const;
  bool applyParameters(const std::vector<rclcpp::Parameter> &params,
                       const QString &source);
  SubmitMode currentSubmitMode() const;

  std::vector<ParamSpec> param_specs_;
  QMap<QString, QDoubleSpinBox *> param_inputs_;

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
  QString last_yaml_path_;
  QSet<QString> dirty_params_;
  QSet<QString> immediate_pending_params_;
};

}  // namespace rm2_dynamic_config
