#include "params.hpp"

OTDParams::OTDParams(const open_lmm::Config& config) {
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
  if (sensor_height <= 0.0 || tau_seeds < 0.0 || tau_dis < 0.0 ||
      tau_ratio < 0.0 || tau_ratio > 1.0 || voxel_size <= 0.0) {
    throw std::invalid_argument("OTD parameters are out of range");
  }
}
