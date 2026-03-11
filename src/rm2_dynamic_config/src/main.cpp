#include "rm2_dynamic_config/main_window.hpp"

#include <QApplication>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  QApplication app(argc, argv);
  rm2_dynamic_config::MainWindow window;
  window.show();

  const int ret = app.exec();
  rclcpp::shutdown();
  return ret;
}
