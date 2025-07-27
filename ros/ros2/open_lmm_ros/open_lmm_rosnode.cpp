// #include <iostream>
// #include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>
#include "open_lmm_ros.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto open_lmm = std::make_shared<open_lmm::OpenLMMROS>(options);

  rclcpp::spin(open_lmm);
  rclcpp::shutdown();

  // uni_mapper->wait();
  // uni_mapper->save("/home/gil/uni_mapper/output");

  return 0;
}