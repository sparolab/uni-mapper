#include "dynamic_remover_base.hpp"

#include <domain/support/validation.hpp>

#include <algorithm>
#include <set>

namespace open_lmm {

Result<DynamicRemoverBase::PointCloud::Ptr>
DynamicRemoverBase::ProcessStreaming(
    const AlgorithmExecutionContext& context,
    const DynamicRemoverStreamingInput& input) {
  AlgorithmExecutionTimer timer(context);
  auto cancellation =
      CheckAlgorithmCancellation(context, "before streaming map removal");
  if (!cancellation) {
    return Result<PointCloud::Ptr>::Failure(cancellation.GetError());
  }
  try {
    std::vector<std::pair<std::size_t, PointCloud::Ptr>> indexed_scans;
    std::set<std::size_t> seen;
    auto loaded = input.source(
        [&](std::size_t index, const PointCloud::Ptr& scan) {
          auto active = CheckAlgorithmCancellation(
              context, "while loading streaming map-removal input");
          if (!active) return active;
          if (!seen.insert(index).second) {
            return Result<void>::Failure(Error::InvalidArgument(
                "dynamic remover source returned duplicate frame " +
                std::to_string(index)));
          }
          auto valid = ValidatePointCloud(
              scan, "dynamic remover source frame " + std::to_string(index));
          if (!valid) return valid;
          indexed_scans.emplace_back(index, scan);
          return Result<void>::Ok();
        });
    if (!loaded) {
      return Result<PointCloud::Ptr>::Failure(
          WithAlgorithmContext(loaded.GetError(), context));
    }
    if (loaded.Value() != indexed_scans.size()) {
      return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
          Error::InvalidArgument(
              "dynamic remover source count differs from visited scan count"),
          context));
    }
    std::sort(indexed_scans.begin(), indexed_scans.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.first < rhs.first;
              });
    ScanVec scans;
    scans.reserve(indexed_scans.size());
    for (std::size_t expected = 0; expected < indexed_scans.size(); ++expected) {
      if (indexed_scans[expected].first != expected) {
        return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
            Error::InvalidArgument(
                "dynamic remover source is missing frame " +
                std::to_string(expected)),
            context));
      }
      scans.push_back(std::move(indexed_scans[expected].second));
    }
    AlgorithmExecutionContext process_context = context;
    process_context.profiler = {};
    auto processed = Process(process_context,
                             DynamicRemoverInput{
                                 std::move(scans), input.optimized_poses});
    if (!processed) return processed;
    if (!processed.Value()) {
      return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
          Error::InvalidArgument(
              "dynamic remover returned a null point cloud"),
          context));
    }
    cancellation =
        CheckAlgorithmCancellation(context, "after streaming map removal");
    if (!cancellation) {
      return Result<PointCloud::Ptr>::Failure(cancellation.GetError());
    }
    return processed;
  } catch (const std::exception& error) {
    return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
        Error::InvalidArgument(
            std::string("dynamic remover streaming exception: ") +
            error.what()),
        context));
  } catch (...) {
    return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
        Error::InvalidArgument("unknown dynamic remover streaming exception"),
        context));
  }
}

}  // namespace open_lmm
