#include "model_conversion.hpp"

#include <open_lmm/common/agent_id.hpp>
#include <open_lmm/common/algorithm_progress.hpp>
#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/alignment_types.hpp>
#include <open_lmm/common/runtime_contracts.hpp>

#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <exception>
#include <string>
#include <utility>

namespace open_lmm::python {
namespace {

template <typename Enum>
int EnumValue(Enum value) {
  return static_cast<int>(value);
}

template <typename T>
py::object OptionalNumber(const std::optional<T>& value) {
  return value ? py::cast(*value) : py::none();
}

py::object OptionalAgent(const std::optional<AgentId>& agent) {
  return agent ? py::cast(agent->Value()) : py::none();
}

py::dict CancellationCapabilityPayload(
    const CancellationCapability& capability) {
  py::dict result;
  result["cooperative"] = capability.cooperative;
  result["mode"] = EnumValue(capability.mode);
  result["non_interruptible_operations"] =
      py::cast(capability.non_interruptible_operations);
  result["requires_process_isolation"] =
      capability.requires_process_isolation;
  return result;
}

py::dict CancellationPayload(const CancellationTelemetry& telemetry) {
  py::dict result;
  result["capability"] = CancellationCapabilityPayload(telemetry.capability);
  result["cancel_requested_at_unix_ns"] =
      OptionalNumber(telemetry.cancel_requested_at_unix_ns);
  result["cancel_observed_at_unix_ns"] =
      OptionalNumber(telemetry.cancel_observed_at_unix_ns);
  result["cancel_completed_at_unix_ns"] =
      OptionalNumber(telemetry.cancel_completed_at_unix_ns);
  return result;
}

py::dict AlgorithmProgressPayload(const AlgorithmProgress& progress) {
  py::dict result;
  result["agent"] = progress.agent.Value();
  result["operation"] = progress.operation;
  result["phase"] = EnumValue(progress.phase);
  result["current"] = progress.current;
  result["total"] = OptionalNumber(progress.total);
  return result;
}

py::dict ArtifactPayload(const ArtifactMetadata& artifact) {
  py::dict result;
  result["type"] = EnumValue(artifact.key.type);
  result["agent"] = OptionalAgent(artifact.key.agent);
  result["state"] = EnumValue(artifact.state);
  result["revision"] = artifact.revision;
  result["producer"] = artifact.producer;
  result["detail"] = artifact.detail;
  result["external_path"] = artifact.external_path;
  result["fingerprint"] = artifact.fingerprint;
  return result;
}

py::dict JobSnapshotPayload(const JobSnapshot& job) {
  py::dict result;
  result["id"] = job.id;
  result["state"] = EnumValue(job.state);
  result["active_stage"] =
      job.active_stage ? py::cast(EnumValue(*job.active_stage)) : py::none();
  result["message"] = job.message;
  result["cancellation"] = CancellationPayload(job.cancellation);
  return result;
}

py::array_t<float> AlignmentPointArray(
    const std::vector<AlignmentVisualizationPoint>& points) {
  py::array_t<float> result(
      {static_cast<py::ssize_t>(points.size()), static_cast<py::ssize_t>(3)});
  auto target = result.mutable_unchecked<2>();
  for (py::ssize_t index = 0; index < target.shape(0); ++index) {
    const auto& point = points[static_cast<std::size_t>(index)];
    target(index, 0) = point.x;
    target(index, 1) = point.y;
    target(index, 2) = point.z;
  }
  result.attr("setflags")(false);
  return result;
}

py::array_t<double> TransformArray(const Eigen::Isometry3d& transform) {
  py::array_t<double> result(
      {static_cast<py::ssize_t>(4), static_cast<py::ssize_t>(4)});
  auto target = result.mutable_unchecked<2>();
  const Eigen::Matrix4d matrix = transform.matrix();
  for (py::ssize_t row = 0; row < 4; ++row) {
    for (py::ssize_t column = 0; column < 4; ++column) {
      target(row, column) = matrix(row, column);
    }
  }
  result.attr("setflags")(false);
  return result;
}

py::dict ConstraintDiagnosticsPayload(
    const LoopConstraintBuildDiagnostics& diagnostics) {
  py::dict result;
  result["sampled_source_frames"] = diagnostics.sampled_source_frames;
  result["target_frames"] = diagnostics.target_frames;
  result["within_radius"] = diagnostics.within_radius;
  result["nearest_distance_m"] = diagnostics.nearest_distance_m;
  result["threshold_m"] = diagnostics.threshold_m;
  result["search_completed"] = diagnostics.search_completed;
  return result;
}

py::dict AttemptStatusPayload(const AlignmentAttemptStatus& status) {
  py::dict result;
  result["method"] = EnumValue(status.method);
  result["state"] = EnumValue(status.state);
  result["reason"] = status.reason
                         ? py::cast(EnumValue(*status.reason))
                         : py::none();
  result["message"] = status.message;
  result["attempt"] = status.attempt;
  if (status.constraint_diagnostics) {
    result["constraint_diagnostics"] =
        ConstraintDiagnosticsPayload(*status.constraint_diagnostics);
  } else {
    result["constraint_diagnostics"] = py::none();
  }
  return result;
}

py::dict MetricsPayload(const AlignmentMetrics& metrics) {
  py::dict result;
  result["correspondence_count"] = metrics.correspondence_count;
  result["rotation_inliers"] = metrics.rotation_inliers;
  result["final_inliers"] = metrics.final_inliers;
  result["consensus_size"] = metrics.consensus_size;
  result["fitness"] = OptionalNumber(metrics.fitness);
  result["overlap_ratio"] = OptionalNumber(metrics.overlap_ratio);
  return result;
}

}  // namespace

py::dict ErrorPayload(const Error& error) {
  py::dict context;
  context["runtime_revision"] =
      OptionalNumber(error.context.runtime_revision);
  context["stage"] = error.context.stage;
  context["node"] = error.context.node;
  context["agent"] = OptionalAgent(error.context.agent);
  context["plugin"] = error.context.plugin;
  context["config"] = error.context.config;
  context["json_pointer"] = error.context.json_pointer;
  context["expected"] = error.context.expected;
  context["actual"] = error.context.actual;
  context["schema_version"] = OptionalNumber(error.context.schema_version);

  py::dict result;
  result["code"] = EnumValue(error.code);
  result["message"] = error.message;
  result["severity"] = EnumValue(error.severity);
  result["context"] = std::move(context);
  return result;
}

py::dict ExecutionEventPayload(const ExecutionEvent& event) {
  py::dict result;
  result["job_id"] = event.job_id;
  result["type"] = EnumValue(event.type);
  result["stage"] =
      event.stage ? py::cast(EnumValue(*event.stage)) : py::none();
  result["message"] = event.message;
  result["sequence"] = event.sequence;
  result["node"] = event.node ? py::cast(EnumValue(*event.node)) : py::none();
  result["agent"] = OptionalAgent(event.agent);
  result["progress_current"] = event.progress_current;
  result["progress_total"] = event.progress_total;
  if (event.error) {
    result["error"] = ErrorPayload(*event.error);
  } else {
    result["error"] = py::none();
  }
  if (event.cancellation) {
    result["cancellation"] = CancellationPayload(*event.cancellation);
  } else {
    result["cancellation"] = py::none();
  }
  result["affected_agents"] = py::list();
  auto affected = result["affected_agents"].cast<py::list>();
  for (const auto& agent : event.affected_agents) affected.append(agent.Value());
  if (event.algorithm_progress) {
    result["algorithm_progress"] =
        AlgorithmProgressPayload(*event.algorithm_progress);
  } else {
    result["algorithm_progress"] = py::none();
  }
  return result;
}

py::dict RuntimeSnapshotPayload(const RuntimeSnapshot& snapshot) {
  py::dict pipeline;
  if (snapshot.pipeline.job) {
    pipeline["job"] = JobSnapshotPayload(*snapshot.pipeline.job);
  } else {
    pipeline["job"] = py::none();
  }
  pipeline["runtime_revision"] = snapshot.pipeline.runtime_revision;
  pipeline["config_revision"] = snapshot.pipeline.config_revision;
  py::list agents;
  for (const auto& agent : snapshot.pipeline.agents) agents.append(agent.Value());
  pipeline["agents"] = std::move(agents);
  py::list artifacts;
  for (const auto& artifact : snapshot.pipeline.artifacts) {
    artifacts.append(ArtifactPayload(artifact));
  }
  pipeline["artifacts"] = std::move(artifacts);
  py::list events;
  for (const auto& event : snapshot.pipeline.recent_events) {
    events.append(ExecutionEventPayload(event));
  }
  pipeline["recent_events"] = std::move(events);

  py::dict result;
  result["label"] = snapshot.label;
  result["status"] = EnumValue(snapshot.state);
  result["output_directory"] = snapshot.output_directory.string();
  result["pipeline"] = std::move(pipeline);
  return result;
}

py::dict ConfigApplyReceiptPayload(const ConfigApplyReceipt& receipt) {
  py::dict result;
  result["previous_config_revision"] = receipt.previous_config_revision;
  result["config_revision"] = receipt.config_revision;
  result["base_runtime_revision"] = receipt.base_runtime_revision;
  result["runtime_revision"] = receipt.runtime_revision;
  py::list agents;
  for (const auto& agent : receipt.affected_agents) agents.append(agent.Value());
  result["affected_agents"] = std::move(agents);
  return result;
}

py::dict RuntimeReplaceReceiptPayload(const RuntimeReplaceReceipt& receipt) {
  py::dict result;
  result["previous_runtime_revision"] = receipt.previous_runtime_revision;
  result["previous_config_revision"] = receipt.previous_config_revision;
  result["runtime_revision"] = receipt.runtime_revision;
  result["config_revision"] = receipt.config_revision;
  return result;
}

py::dict ConfigDocumentsPayload(const CommittedConfigDocuments& documents) {
  py::dict result;
  result["runtime_revision"] = documents.runtime_revision;
  result["config_revision"] = documents.config_revision;
  py::list entries;
  for (const auto& document : documents.documents) {
    py::dict entry;
    entry["domain"] = EnumValue(document.domain);
    entry["canonical_json"] = document.canonical_json;
    entry["selected_document"] = document.selected_document
                                     ? py::cast(*document.selected_document)
                                     : py::none();
    entries.append(std::move(entry));
  }
  result["documents"] = std::move(entries);
  return result;
}

py::dict ConfigCandidatesPayload(const ConfigCandidateCatalog& catalog) {
  py::dict result;
  result["runtime_revision"] = catalog.runtime_revision;
  result["config_revision"] = catalog.config_revision;
  py::list entries;
  for (const auto& candidate : catalog.candidates) {
    py::dict entry;
    entry["domain"] = EnumValue(candidate.domain);
    entry["model"] = candidate.model;
    entry["selected_document"] = candidate.selected_document;
    entry["canonical_json"] = candidate.canonical_json;
    entries.append(std::move(entry));
  }
  result["candidates"] = std::move(entries);
  return result;
}

py::list NodeDescriptorsPayload(
    const std::vector<NodeDescriptor>& descriptors) {
  py::list result;
  for (const auto& descriptor : descriptors) {
    py::dict value;
    value["id"] = EnumValue(descriptor.id);
    value["name"] = std::string(descriptor.name);
    value["stage"] = EnumValue(descriptor.stage);
    value["scope"] = EnumValue(descriptor.scope);
    py::list required;
    for (const auto artifact : descriptor.required_artifacts) {
      required.append(EnumValue(artifact));
    }
    value["required_artifacts"] = std::move(required);
    py::list produced;
    for (const auto artifact : descriptor.produced_artifacts) {
      produced.append(EnumValue(artifact));
    }
    value["produced_artifacts"] = std::move(produced);
    value["ordered"] = descriptor.ordered;
    value["supports_cancellation"] = descriptor.supports_cancellation;
    result.append(std::move(value));
  }
  return result;
}

py::object AlignmentFeedbackPayload(
    std::optional<AlignmentFeedbackSnapshot> snapshot) {
  if (!snapshot) return py::none();
  constexpr std::size_t kMaximumAlignmentPointCount = 2'000'000;
  if (snapshot->target_points.size() > kMaximumAlignmentPointCount ||
      snapshot->source_points.size() >
          kMaximumAlignmentPointCount - snapshot->target_points.size()) {
    RaiseError(Error::ResourceExhausted(
        "alignment feedback exceeds the 2000000-point conversion limit"));
  }

  py::dict proposal;
  proposal["request_id"] = snapshot->proposal.request_id;
  proposal["target_agent"] = snapshot->proposal.target_agent.Value();
  proposal["source_agent"] = snapshot->proposal.source_agent.Value();
  proposal["method"] = EnumValue(snapshot->proposal.method);
  proposal["target_T_source"] =
      TransformArray(snapshot->proposal.target_T_source);
  proposal["metrics"] = MetricsPayload(snapshot->proposal.metrics);

  py::dict diagnostics;
  diagnostics["target_trajectory"] =
      AlignmentPointArray(snapshot->diagnostics.target_trajectory);
  diagnostics["source_trajectory"] =
      AlignmentPointArray(snapshot->diagnostics.source_trajectory);
  py::list loops;
  for (const auto& loop : snapshot->diagnostics.descriptor_loops) {
    py::dict value;
    value["target"] = py::make_tuple(loop.target.x, loop.target.y, loop.target.z);
    value["source"] = py::make_tuple(loop.source.x, loop.source.y, loop.source.z);
    value["inlier"] = loop.inlier;
    loops.append(std::move(value));
  }
  diagnostics["descriptor_loops"] = std::move(loops);

  py::list history;
  for (const auto& status : snapshot->attempt_history) {
    history.append(AttemptStatusPayload(status));
  }
  py::dict result;
  result["proposal"] = std::move(proposal);
  result["target_points"] = AlignmentPointArray(snapshot->target_points);
  result["source_points"] = AlignmentPointArray(snapshot->source_points);
  result["diagnostics"] = std::move(diagnostics);
  result["attempt_status"] = AttemptStatusPayload(snapshot->attempt_status);
  result["attempt_history"] = std::move(history);
  result["session_revision"] = snapshot->session_revision;
  result["review_state"] = EnumValue(snapshot->review_state);
  result["terminal_message"] = snapshot->terminal_message;
  return result;
}

[[noreturn]] void RaiseError(const Error& error) {
  py::module_::import("open_lmm._errors")
      .attr("_raise_from_native")(ErrorPayload(error));
  throw std::runtime_error("Python error translator returned unexpectedly");
}

[[noreturn]] void RaiseInternal(std::exception_ptr exception) {
  std::string message = "unknown C++ exception escaped the RuntimeClient";
  if (exception) {
    try {
      std::rethrow_exception(exception);
    } catch (const std::exception& error) {
      message = std::string("C++ exception escaped the RuntimeClient: ") +
                error.what();
    } catch (...) {
    }
  }
  py::module_::import("open_lmm._errors").attr("_raise_internal")(message);
  throw std::runtime_error("Python internal-error translator returned unexpectedly");
}

}  // namespace open_lmm::python
