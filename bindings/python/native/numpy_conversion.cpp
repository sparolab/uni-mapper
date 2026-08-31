#include "numpy_conversion.hpp"

#include <pybind11/numpy.h>

#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

namespace open_lmm::python {
namespace py = pybind11;
namespace {

static_assert(std::is_standard_layout_v<VisualizationPoint>);
static_assert(sizeof(VisualizationPoint) == sizeof(float) * 4);
static_assert(offsetof(VisualizationPoint, x) == sizeof(float) * 0);
static_assert(offsetof(VisualizationPoint, y) == sizeof(float) * 1);
static_assert(offsetof(VisualizationPoint, z) == sizeof(float) * 2);
static_assert(offsetof(VisualizationPoint, intensity) == sizeof(float) * 3);

py::array PointArray(std::vector<VisualizationPoint> points) {
  auto* owner = new std::vector<VisualizationPoint>(std::move(points));
  py::capsule lifetime(owner, [](void* value) {
    delete static_cast<std::vector<VisualizationPoint>*>(value);
  });
  py::array result(
      py::dtype::of<float>(),
      {static_cast<py::ssize_t>(owner->size()), static_cast<py::ssize_t>(4)},
      {static_cast<py::ssize_t>(sizeof(VisualizationPoint)),
       static_cast<py::ssize_t>(sizeof(float))},
      owner->data(), lifetime);
  result.attr("setflags")(false);
  return result;
}

py::array PoseArray(const std::vector<VisualizationPose>& poses) {
  py::array_t<float> result(
      {static_cast<py::ssize_t>(poses.size()), static_cast<py::ssize_t>(4),
       static_cast<py::ssize_t>(4)});
  auto target = result.mutable_unchecked<3>();
  for (py::ssize_t index = 0; index < target.shape(0); ++index) {
    const auto matrix = poses[static_cast<std::size_t>(index)].transform.matrix();
    for (py::ssize_t row = 0; row < 4; ++row) {
      for (py::ssize_t column = 0; column < 4; ++column) {
        target(index, row, column) = matrix(row, column);
      }
    }
  }
  result.attr("setflags")(false);
  return result;
}

py::array BoundArray(const Eigen::Vector3f& bound) {
  py::array_t<float> result(static_cast<py::ssize_t>(3));
  auto target = result.mutable_unchecked<1>();
  for (py::ssize_t index = 0; index < 3; ++index) target(index) = bound(index);
  result.attr("setflags")(false);
  return result;
}

}  // namespace

py::dict VisualizationPayload(VisualizationSnapshot snapshot) {
  py::list edges;
  for (const auto& edge : snapshot.edges) {
    py::dict value;
    value["from_agent"] = edge.from_agent.Value();
    value["from_index"] = edge.from_index;
    value["to_agent"] = edge.to_agent.Value();
    value["to_index"] = edge.to_index;
    value["type"] = static_cast<int>(edge.type);
    edges.append(std::move(value));
  }

  py::dict result;
  result["agent"] = snapshot.agent.Value();
  result["revision"] = snapshot.revision;
  result["phase"] = static_cast<int>(snapshot.phase);
  result["pose_kind"] = static_cast<int>(snapshot.pose_kind);
  result["point_kind"] = static_cast<int>(snapshot.point_kind);
  result["poses"] = PoseArray(snapshot.poses);
  result["edges"] = std::move(edges);
  result["points"] = PointArray(std::move(snapshot.points));
  result["min_bound"] = BoundArray(snapshot.min_bound);
  result["max_bound"] = BoundArray(snapshot.max_bound);
  result["has_bounds"] = snapshot.has_bounds;
  result["points_available"] = snapshot.points_available;
  result["points_complete"] = snapshot.points_complete;
  result["map_available"] = snapshot.map_available;
  result["displayed_point_count"] = snapshot.displayed_point_count;
  result["source_point_count"] = snapshot.source_point_count;
  return result;
}

}  // namespace open_lmm::python
