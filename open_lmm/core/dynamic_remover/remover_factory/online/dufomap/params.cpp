#include "params.hpp"

DUFOMapParams::DUFOMapParams() {
  open_lmm::Config config = open_lmm::Config(
      open_lmm::GlobalConfig::get_global_config_path("config_dynamic_remover"));
  replace_intensity =
      config.param<bool>("dynamic_remover", "replace_intensity", false);
  map.resolution = config.param<double>("dynamic_remover", "resolution", 0.2);
  map.levels = config.param<int>("dynamic_remover", "levels", 20);
  down_sampling_method = config.param<std::string>(
      "dynamic_remover", "down_sampling_method", "center");
  if (down_sampling_method == "centroid") {
    integration.down_sampling_method = ufo::DownSamplingMethod::CENTROID;
  } else if (down_sampling_method == "uniform") {
    integration.down_sampling_method = ufo::DownSamplingMethod::UNIFORM;
  } else {
    integration.down_sampling_method = ufo::DownSamplingMethod::CENTER;
  }
  integration.hit_depth = config.param<int>("dynamic_remover", "hit_depth", 0);
  integration.miss_depth =
      config.param<int>("dynamic_remover", "miss_depth", 0);
  integration.max_range =
      config.param<double>("dynamic_remover", "max_range", 80.0);
  integration.inflate_unknown =
      config.param<double>("dynamic_remover", "inflate_unknown", 1);
  integration.inflate_unknown_compensation = config.param<bool>(
      "dynamic_remover", "inflate_unknown_compensation", true);
  integration.ray_passthrough_hits =
      config.param<bool>("dynamic_remover", "ray_passthrough_hits", false);
  integration.inflate_hits_dist =
      config.param<double>("dynamic_remover", "inflate_hits_dist", 0.2);
  simple_ray_casting =
      config.param<bool>("dynamic_remover", "simple_ray_casting", false);
  if (simple_ray_casting) {
    integration.ray_casting_method = ufo::RayCastingMethod::SIMPLE;
  } else {
    integration.ray_casting_method = ufo::RayCastingMethod::PROPER;
  }
  integration.simple_ray_casting_factor =
      config.param<double>("dynamic_remover", "simple_ray_casting_factor", 1.0);
  integration.parallel =
      config.param<bool>("dynamic_remover", "parallel", true);
  integration.num_threads =
      config.param<int>("dynamic_remover", "num_threads", 16);
  integration.only_valid =
      config.param<bool>("dynamic_remover", "only_valid", false);

  // integration.ray_casting_depth = config.param<int>("dynamic_remover",
  // "ray_casting_depth", 0) propagate = config.param<bool>("dynamic_remover",
  // "propagate", false); ray_casting_method =
  // config.param<std::string>("dynamic_remover", "ray_casting_method",
  // "simple");
}
