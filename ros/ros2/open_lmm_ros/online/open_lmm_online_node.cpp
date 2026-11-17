#include "open_lmm_online.hpp"

#include <iostream>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    auto node = std::make_shared<open_lmm::OpenLMMOnline>(rclcpp::NodeOptions{});
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& error) {
    std::cerr << error.what() << std::endl;
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
