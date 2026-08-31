#pragma once

#include "../runtime_adapter/open_lmm_ros.hpp"

#include <memory>

namespace open_lmm {

class GuiRuntimeHost;

// Optional composition root. The ROS runtime adapter remains usable with only
// the public client component; this class adds the GUI plugin host when the
// separate GUI component is requested.
class OpenLMMROSGui final : public OpenLMMROS {
 public:
  explicit OpenLMMROSGui(const rclcpp::NodeOptions& options);
  ~OpenLMMROSGui() override;

  [[nodiscard]] bool GuiEnabled() const { return gui_host_ != nullptr; }

 private:
  std::unique_ptr<GuiRuntimeHost> gui_host_;
};

}  // namespace open_lmm
