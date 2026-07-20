#include <open_lmm/utils/config.hpp>
#include <string>

struct OTDParams {
  OTDParams();
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