#include "params.hpp"

OTDParams::OTDParams(const open_lmm::Config& config) {
  replace_intensity =
      config.param_cast<bool>("dynamic_remover", "replace_intensity");
  // groundseparate
  sensor_height =
      config.param_cast<double>("dynamic_remover", "sensor_height");
  tau_seeds = config.param_cast<double>("dynamic_remover", "tau_seeds");
  tau_dis = config.param_cast<double>("dynamic_remover", "tau_dis");
  // otd
  tau_ratio = config.param_cast<double>("dynamic_remover", "tau_ratio");
  voxel_size = config.param_cast<double>("dynamic_remover", "voxel_size");
}
