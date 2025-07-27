#pragma once

#include <rclcpp/rclcpp.hpp>

namespace open_lmm {

class OpenLMMROS : public rclcpp::Node {
public:
  OpenLMMROS(const rclcpp::NodeOptions &options);
  ~OpenLMMROS() = default;

private:
};

} // namespace open_lmm
