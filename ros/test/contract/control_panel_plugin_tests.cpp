#include "../../ros2/open_lmm_ros/rviz_control/open_lmm_control_panel.hpp"

#include <QApplication>

#include <cstdlib>

int main(int argc, char** argv) {
  QApplication application(argc, argv);
  open_lmm_ros::control::OpenLmmControlPanel panel;
  panel.resize(480, 640);
  return panel.isEnabled() ? EXIT_SUCCESS : EXIT_FAILURE;
}
