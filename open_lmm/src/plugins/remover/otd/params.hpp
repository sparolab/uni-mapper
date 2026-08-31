#include <config/document/config.hpp>
#include <string>

struct OTDParams {
  explicit OTDParams(const open_lmm::Config& config);
  ~OTDParams() = default;

  bool replace_intensity;
  // groundseparate
  double sensor_height;
  double tau_seeds;
  double tau_dis;
  // otd
  double tau_ratio;
  double voxel_size;
};
