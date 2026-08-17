#pragma once

#include <open_lmm/gui/gui_plugin.hpp>

#include <memory>

namespace open_lmm {

// PipelineController의 public 계약만 GUI용 copyable function port로 변환한다.
GuiServices MakeGuiServices(const std::shared_ptr<PipelineController>& controller);

}  // namespace open_lmm
