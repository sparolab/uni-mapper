#include <iostream>
#include <rclcpp/rclcpp.hpp>
#ifdef OPEN_LMM_ROS_GUI_COMPOSITION
#include "open_lmm_ros_gui.hpp"
#else
#include "open_lmm_ros.hpp"
#endif

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  try {
#ifdef OPEN_LMM_ROS_GUI_COMPOSITION
    auto open_lmm = std::make_shared<open_lmm::OpenLMMROSGui>(options);
#else
    auto open_lmm = std::make_shared<open_lmm::OpenLMMROS>(options);
#endif
    exec.add_node(open_lmm);
    exec.spin();
  } catch (const std::exception& error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();

  return 0;
}
