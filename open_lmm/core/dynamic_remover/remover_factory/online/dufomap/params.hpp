#include <open_lmm/utils/config.hpp>
#include <ufo/map/integration/integration_parameters.hpp>

#include <string>

struct Map {
  ufo::node_size_t resolution = 0.1;  // In meters
  ufo::depth_t levels = 17;  // Levels of the octree
};

struct Clustering {
  bool cluster = false;
  float max_distance = 0.2f;
  std::size_t min_points = 0;
  ufo::depth_t depth = 0;
};

struct DUFOMapParams {
  DUFOMapParams();
  ~DUFOMapParams() = default;
  ufo::IntegrationParams integration;
  Map map;
  Clustering clustering;
  bool replace_intensity;

  //[core]
  double resolution{0.2};  // voxel size (meters), v in the paper, default 0.1
  double inflate_unknown{1};  // d_p in the paper, default 1
  double inflate_hits_dist{0.2};  // d_s in the paper, default 0.2
  //[integration]
  bool inflate_unknown_compensation{true};
  bool ray_passthrough_hits{false};
  std::string down_sampling_method{
      "center"};  // ["none", "center", "centroid", "uniform"]
  double max_range{-1};  // Maximum range to integrate
  bool only_valid{false};  // Only do ray casting for points within 'max_range'
  int hit_depth{0};  // Level of the octree to integrate hits
  int miss_depth{0};  // Level of the octree to integrate misses
  int ray_casting_depth{0};  // Level of the octree to perform ray casting
  bool simple_ray_casting{false};
  double simple_ray_casting_factor{1.0};
  bool parallel{true};  // Use parallel execution for integration
  int num_threads{
      16};  // Number of threads to use, 8 times number of physical cores seems
            // to be a good number and it is chosen if 0 is given
  //[map]
  int levels{20};  // Levels of the octree
  // TODO(gil) :
  //  bool propagate{false};
  // std::string ray_casting_method{"simple"};   // ["simple", "proper"]
};