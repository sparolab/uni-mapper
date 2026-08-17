#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <memory>
#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/agent_data.hpp>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/utils/config.hpp>
#include <string>

namespace fs = std::filesystem;
namespace open_lmm {

class DataLoaderBase {
 public:
  DataLoaderBase() = default;
  explicit DataLoaderBase(Config config);
  virtual ~DataLoaderBase() = default;

  // 순수 함수 — SharedDatabase 없이 AgentRawData 반환
  virtual AgentRawData Process(const AgentContext& ctx,
                               const fs::path& data_dir) = 0;

  // MapUpdater가 raw 스캔만 다시 읽을 때 사용
  virtual ScanVec loadRawScanData(fs::path data_dir_path) = 0;

  virtual void parseConfig(Config config) = 0;
  static std::unique_ptr<DataLoaderBase> createInstance(Config config);
};

}  // namespace open_lmm