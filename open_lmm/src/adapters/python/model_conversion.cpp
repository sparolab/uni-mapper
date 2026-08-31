#include "model_conversion.hpp"

#include <open_lmm/common/agent_id.hpp>
#include <open_lmm/common/algorithm_progress.hpp>
#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/runtime_contracts.hpp>

#include <pybind11/stl.h>

#include <exception>
#include <string>

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
