include(cmake/OpenLmmPythonTest.cmake)

if(OPEN_LMM_PYTHON_WHEEL_ONLY)
  if(NOT OPEN_LMM_PYTHON_WHEEL_TEST_PYTHON OR
     NOT EXISTS "${OPEN_LMM_PYTHON_WHEEL_TEST_PYTHON}" OR
     NOT OPEN_LMM_PYTHON_WHEEL_FILE OR
     NOT EXISTS "${OPEN_LMM_PYTHON_WHEEL_FILE}" OR
     NOT OPEN_LMM_REPOSITORY_ROOT OR
     NOT OPEN_LMM_CORE_TEST_BUILD_ROOT)
    message(FATAL_ERROR
      "OPEN_LMM_PYTHON_WHEEL_ONLY requires the wheel, installed-wheel Python, repository, and core build")
  endif()
  openlmm_add_python_test(
    NAME open_lmm_python_wheel_tests FILE test_wheel_install.py WHEEL LAYER L3
    MODULE package.python OWNER PythonWheel INVARIANTS INV-13 INV-14)
  add_test(NAME open_lmm_python_wheel_closure_contract
    COMMAND "${OPEN_LMM_PYTHON_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/package/wheel_audit.py"
      "${OPEN_LMM_PYTHON_WHEEL_FILE}"
      "${OPEN_LMM_REPOSITORY_ROOT}"
      "${OPEN_LMM_CORE_TEST_BUILD_ROOT}")
  set_tests_properties(open_lmm_python_wheel_closure_contract PROPERTIES
    TIMEOUT 180
    LABELS "layer:L3;module:package.python;owner:PythonWheel;lane:pr;invariant:INV-13;invariant:INV-14")
  file(APPEND "${OPEN_LMM_PYTHON_TEST_MANIFEST}"
    "open_lmm_python_wheel_closure_contract\tL3\tpackage.python\tPythonWheel\tINV-13,INV-14\tpr\n")
  return()
endif()

if(NOT OPEN_LMM_PYTHON_CORE_CONTRACT_ONLY)
  foreach(required_test_input IN ITEMS
      OPEN_LMM_REPOSITORY_ROOT
      OPEN_LMM_CORE_TEST_BUILD_ROOT
      OPEN_LMM_REPLAY_COMPARE)
    if(NOT ${required_test_input})
      message(FATAL_ERROR
        "BUILD_TESTING requires ${required_test_input} for the Goal 03/05 integration contracts")
    endif()
  endforeach()
  if(NOT EXISTS "${OPEN_LMM_REPLAY_COMPARE}")
    message(FATAL_ERROR
      "OPEN_LMM_REPLAY_COMPARE is missing: ${OPEN_LMM_REPLAY_COMPARE}")
  endif()
endif()

openlmm_add_python_test(
  NAME open_lmm_python_public_api_tests FILE test_public_api.py LAYER L2
  MODULE adapters.python.api OWNER PythonBinding INVARIANTS INV-13 INV-14)
openlmm_add_python_test(
  NAME open_lmm_python_error_tests FILE test_errors.py LAYER L2
  MODULE adapters.python.errors OWNER PythonBinding
  INVARIANTS INV-07 INV-14 INV-18)
openlmm_add_python_test(
  NAME open_lmm_python_runtime_tests FILE test_runtime.py LAYER L3
  MODULE adapters.python.runtime OWNER RuntimeClient
  INVARIANTS INV-05 INV-06 INV-07 INV-08)
openlmm_add_python_test(
  NAME open_lmm_python_config_tests FILE test_config.py LAYER L3
  MODULE adapters.python.config OWNER RuntimeClient
  INVARIANTS INV-03 INV-04 INV-07)
openlmm_add_python_test(
  NAME open_lmm_python_callback_tests FILE test_callbacks.py LAYER L5
  MODULE adapters.python.events OWNER PythonSubscription
  INVARIANTS INV-08 INV-17)
openlmm_add_python_test(
  NAME open_lmm_python_numpy_tests FILE test_numpy.py LAYER L3
  MODULE adapters.python.visualization OWNER PythonBufferOwner
  INVARIANTS INV-09 INV-10 INV-16)
openlmm_add_python_test(
  NAME open_lmm_python_lifetime_tests FILE test_lifetime.py LAYER L5
  MODULE adapters.python.lifetime OWNER PythonRuntimeHolder
  INVARIANTS INV-08 INV-17)

if(OPEN_LMM_PYTHON_CORE_CONTRACT_ONLY)
  return()
endif()

openlmm_add_python_test(
  NAME open_lmm_python_experiment_api_tests
  FILE experiments/test_public_api.py LAYER L2
  MODULE experiments.api OWNER ExperimentAPI INVARIANTS INV-13 INV-14)
openlmm_add_python_test(
  NAME open_lmm_python_experiment_manifest_tests
  FILE experiments/test_manifest.py LAYER L2
  MODULE experiments.manifest OWNER ExperimentManifest
  INVARIANTS INV-02 INV-14 INV-18)
openlmm_add_python_test(
  NAME open_lmm_python_experiment_planner_tests
  FILE experiments/test_planner.py LAYER L2
  MODULE experiments.planner OWNER TrialPlanner INVARIANTS INV-08 INV-16 INV-18)
openlmm_add_python_test(
  NAME open_lmm_python_experiment_config_tests
  FILE experiments/test_config_materializer.py LAYER L3
  MODULE experiments.config OWNER ConfigMaterializer
  INVARIANTS INV-01 INV-02 INV-03 INV-04)
openlmm_add_python_test(
  NAME open_lmm_python_experiment_metric_tests
  FILE experiments/test_metrics.py LAYER L3
  MODULE experiments.metrics OWNER MetricCollector
  INVARIANTS INV-09 INV-10 INV-16 INV-18)
openlmm_add_python_test(
  NAME open_lmm_python_experiment_export_tests
  FILE experiments/test_export.py LAYER L2
  MODULE experiments.export OWNER EvidenceWriter INVARIANTS INV-14 INV-16 INV-18)
openlmm_add_python_test(
  NAME open_lmm_python_experiment_runner_tests
  FILE experiments/test_runner.py LAYER L4
  MODULE experiments.workflow OWNER ExperimentWorker
  INVARIANTS INV-01 INV-02 INV-03 INV-04 INV-05 INV-06 INV-07 INV-08 INV-17)
openlmm_add_python_test(
  NAME open_lmm_python_experiment_lifetime_tests
  FILE experiments/test_process_lifetime.py LAYER L5
  MODULE experiments.lifetime OWNER ExperimentWorker INVARIANTS INV-08 INV-17)
openlmm_add_python_test(
  NAME open_lmm_python_experiment_replay_tests
  FILE experiments/test_replay_adapter.py LAYER L4
  MODULE experiments.replay OWNER ReplayAdapter INVARIANTS INV-02 INV-03 INV-04 INV-14)
openlmm_add_python_test(
  NAME open_lmm_python_experiment_benchmark_tests
  FILE experiments/test_benchmark_adapter.py LAYER L3
  MODULE experiments.benchmark OWNER BenchmarkAdapter INVARIANTS INV-13 INV-16 INV-18)

if(OPEN_LMM_PYTHON_WHEEL_TEST_PYTHON)
  if(NOT EXISTS "${OPEN_LMM_PYTHON_WHEEL_TEST_PYTHON}")
    message(FATAL_ERROR
      "OPEN_LMM_PYTHON_WHEEL_TEST_PYTHON is missing: "
      "${OPEN_LMM_PYTHON_WHEEL_TEST_PYTHON}")
  endif()
  openlmm_add_python_test(
    NAME open_lmm_python_wheel_tests FILE test_wheel_install.py WHEEL LAYER L3
    MODULE package.python OWNER PythonWheel INVARIANTS INV-13 INV-14)
endif()
