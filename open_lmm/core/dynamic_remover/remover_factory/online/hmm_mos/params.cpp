#include "params.hpp"

HmmMosParams::HmmMosParams() {
  open_lmm::Config config = open_lmm::Config(
      open_lmm::GlobalConfig::get_global_config_path("config_dynamic_remover"));
  replace_intensity = config.param<bool>("dynamic_remover", "replace_intensity", true);
  voxel_size = config.param<double>("dynamic_remover", "voxel_size", 0.2);
  occupancy_sigma = config.param<double>("dynamic_remover", "occupancy_sigma", 0.2);
  free_sigma = config.param<double>("dynamic_remover", "free_sigma", 0.2);
  belief_threshold = config.param<double>("dynamic_remover", "belief_threshold", 0.99);
  conv_size = config.param<int>("dynamic_remover", "conv_size", 5);
  local_window_size = config.param<int>("dynamic_remover", "local_window_size", 3);
  global_window_size = config.param<int>("dynamic_remover", "global_window_size", 300);
  min_otsu = config.param<int>("dynamic_remover", "min_otsu", 3);
  max_range = config.param<double>("dynamic_remover", "max_range", 50);
  min_range = config.param<double>("dynamic_remover", "min_range", 0.5);
}

