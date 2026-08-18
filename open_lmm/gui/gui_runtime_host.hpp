#pragma once

#include <open_lmm/common/result.hpp>

#include <memory>
#include <string>

namespace open_lmm {

class RuntimeClient;

// Stable, package-facing GUI host. Dynamic plugin loading and the internal GUI
// service bridge remain hidden behind the PImpl boundary.
class GuiRuntimeHost {
 public:
  static Result<std::unique_ptr<GuiRuntimeHost>> LoadAndStart(
      const std::string& plugin_path,
      std::shared_ptr<RuntimeClient> runtime,
      std::string config_file_path);
  ~GuiRuntimeHost();

  GuiRuntimeHost(const GuiRuntimeHost&) = delete;
  GuiRuntimeHost& operator=(const GuiRuntimeHost&) = delete;
  GuiRuntimeHost(GuiRuntimeHost&&) noexcept;
  GuiRuntimeHost& operator=(GuiRuntimeHost&&) noexcept;

  void Stop();
  [[nodiscard]] bool IsOpen() const;

 private:
  struct Impl;
  explicit GuiRuntimeHost(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace open_lmm
