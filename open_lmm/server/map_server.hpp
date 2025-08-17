#pragma once

#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/server/map_aligner/map_aligner.hpp>
#include <open_lmm/server/map_updater/map_updater.hpp>

#include <filesystem>
#include <vector>

namespace fs = std::filesystem;
namespace open_lmm {

class MapServer {
 public:
  MapServer();
  ~MapServer();
  void parseConfig();
  void process();
  void allocateAgentMemory(const char);
  void saveOptimizedPoses(const std::string& save_dir);
  void saveOptimizedMap(const std::string& save_dir);

 private:
  std::shared_ptr<SharedDatabase> shared_data_ =
      std::make_shared<SharedDatabase>();
  int agent_num_;
  std::vector<fs::path> data_dir_list_;
  std::string output_save_dir_;
  std::optional<Config> config_map_server_;
  bool enable_map_updater_;
};

}  // namespace open_lmm