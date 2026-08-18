#include "params.hpp"

DUFOMapParams::DUFOMapParams(const open_lmm::Config& config) {
  replace_intensity = config.param_cast<bool>("dynamic_remover", "replace_intensity");
  map.resolution = config.param_cast<double>("dynamic_remover", "resolution");
  map.levels = config.param_cast<int>("dynamic_remover", "levels");
  down_sampling_method = config.param_cast<std::string>("dynamic_remover", "down_sampling_method");
  if(down_sampling_method == "centroid") {
    integration.down_sampling_method = ufo::DownSamplingMethod::CENTROID;
  } else if(down_sampling_method == "uniform") {
    integration.down_sampling_method = ufo::DownSamplingMethod::UNIFORM;
  } else {
    integration.down_sampling_method = ufo::DownSamplingMethod::CENTER;
  }
  integration.hit_depth = config.param_cast<int>("dynamic_remover", "hit_depth");
  integration.miss_depth = config.param_cast<int>("dynamic_remover", "miss_depth");
  integration.max_range = config.param_cast<double>("dynamic_remover", "max_range");
  integration.inflate_unknown = config.param_cast<double>("dynamic_remover", "inflate_unknown");
  integration.inflate_unknown_compensation = config.param_cast<bool>("dynamic_remover", "inflate_unknown_compensation");
  integration.ray_passthrough_hits = config.param_cast<bool>("dynamic_remover", "ray_passthrough_hits");
  integration.inflate_hits_dist = config.param_cast<double>("dynamic_remover", "inflate_hits_dist");
  simple_ray_casting = config.param_cast<bool>("dynamic_remover", "simple_ray_casting");
  if(simple_ray_casting) {
    integration.ray_casting_method = ufo::RayCastingMethod::SIMPLE;
  } else {
    integration.ray_casting_method = ufo::RayCastingMethod::PROPER;
  }
  integration.simple_ray_casting_factor = config.param_cast<double>("dynamic_remover", "simple_ray_casting_factor");
  integration.parallel = config.param_cast<bool>("dynamic_remover", "parallel");
  integration.num_threads = config.param_cast<int>("dynamic_remover", "num_threads");
  integration.only_valid = config.param_cast<bool>("dynamic_remover", "only_valid");
  
  // integration.ray_casting_depth = config.param_cast<int>("dynamic_remover", "ray_casting_depth")
  // propagate = config.param_cast<bool>("dynamic_remover", "propagate");
  // ray_casting_method = config.param_cast<std::string>("dynamic_remover", "ray_casting_method");
}
