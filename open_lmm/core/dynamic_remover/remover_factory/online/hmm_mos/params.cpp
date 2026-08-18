#include "params.hpp"

HmmMosParams::HmmMosParams(const open_lmm::Config& config) {
  replace_intensity = config.param_cast<bool>("dynamic_remover", "replace_intensity");
  voxel_size = config.param_cast<double>("dynamic_remover", "voxel_size");
  occupancy_sigma = config.param_cast<double>("dynamic_remover", "occupancy_sigma");
  free_sigma = config.param_cast<double>("dynamic_remover", "free_sigma");
  belief_threshold = config.param_cast<double>("dynamic_remover", "belief_threshold");
  conv_size = config.param_cast<int>("dynamic_remover", "conv_size");
  local_window_size = config.param_cast<int>("dynamic_remover", "local_window_size");
  global_window_size = config.param_cast<int>("dynamic_remover", "global_window_size");
  min_otsu = config.param_cast<int>("dynamic_remover", "min_otsu");
  max_range = config.param_cast<double>("dynamic_remover", "max_range");
  min_range = config.param_cast<double>("dynamic_remover", "min_range");
}
