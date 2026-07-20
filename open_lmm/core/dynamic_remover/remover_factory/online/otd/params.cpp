#include "params.hpp"

OTDParams::OTDParams() {
  open_lmm::Config config = open_lmm::Config(
      open_lmm::GlobalConfig::get_global_config_path("config_dynamic_remover"));
  replace_intensity =
      config.param<bool>("dynamic_remover", "replace_intensity", true);
  // groundseparate
  sensor_height =
      config.param<double>("dynamic_remover", "sensor_height", 1.73);
  tau_seeds = config.param<double>("dynamic_remover", "tau_seeds", 0.6);
  tau_dis = config.param<double>("dynamic_remover", "tau_dis", 0.14);
  // otd
  tau_ratio = config.param<double>("dynamic_remover", "tau_ratio", 0.8);
  voxel_size = config.param<double>("dynamic_remover", "voxel_size", 0.5);
}
