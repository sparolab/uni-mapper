#pragma once

#include <open_lmm/gui/gui_plugin.hpp>
#include <open_lmm/server/runtime_session_client.hpp>

#include <memory>

namespace open_lmm {

// Bind one GUI to the same session-keyed RuntimeClient contract used by all
// external adapters. No core runner/controller escapes through this bridge.
GuiServices MakeGuiServices(
    const std::shared_ptr<RuntimeSessionClient>& session,
    std::string config_file_path = {});

}  // namespace open_lmm
