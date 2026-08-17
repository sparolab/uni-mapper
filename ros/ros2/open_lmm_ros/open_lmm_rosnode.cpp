#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "open_lmm_ros.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  try {
    auto open_lmm = std::make_shared<open_lmm::OpenLMMROS>(options);
    if (open_lmm->GuiEnabled()) {
      exec.add_node(open_lmm);
      exec.spin();
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();

  return 0;
}
