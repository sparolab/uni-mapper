#include <open_lmm/utils/config.hpp>
#include <string>

struct HmmMosParams {
  HmmMosParams();
  ~HmmMosParams() = default;

  bool replace_intensity;
  double voxel_size;
  double occupancy_sigma;
  double free_sigma;
  double belief_threshold;
  int conv_size;
  int local_window_size;
  int global_window_size;
  int min_otsu;
  double max_range;
  double min_range;
};