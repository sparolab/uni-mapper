#pragma once

#include <open_lmm/gui/gui_plugin.hpp>
#include <open_lmm/server/runtime_client.hpp>

#include <memory>

namespace open_lmm {

// Bind one GUI to the session-free RuntimeClient contract. No core
// runner/controller escapes through this bridge.
GuiServices MakeGuiServices(
    const std::shared_ptr<RuntimeClient>& runtime,
    std::string config_file_path = {});

}  // namespace open_lmm
