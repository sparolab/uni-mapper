if(NOT TARGET open_lmm_python_native)
  return()
endif()

function(openlmm_add_python_test name file layer module owner invariants)
  openlmm_add_test(
    NAME ${name}
    COMMAND ${CMAKE_COMMAND}
    COMMAND_ARGS -E env
      "PYTHONPATH=${OPEN_LMM_PYTHON_STAGE_DIR}"
      "OPEN_LMM_PYTHON_TEST_ROOT=${PROJECT_BINARY_DIR}/python-test"
      "OPEN_LMM_SOURCE_ROOT=${PROJECT_SOURCE_DIR}"
      "OPEN_LMM_BUILD_ROOT=${PROJECT_BINARY_DIR}"
      "OPEN_LMM_REPLAY_COMPARE=$<TARGET_FILE:open_lmm_replay_compare>"
      ${OPEN_LMM_PYTHON_EXECUTABLE}
      "${PROJECT_SOURCE_DIR}/test/adapters/python/${file}"
    LAYER ${layer} MODULE ${module} OWNER ${owner}
    INVARIANTS ${invariants}
    LANES pr)
endfunction()

openlmm_add_python_test(
  open_lmm_python_public_api_tests test_public_api.py L2
  adapters.python.api PythonBinding "INV-13;INV-14")
openlmm_add_python_test(
  open_lmm_python_error_tests test_errors.py L2
  adapters.python.errors PythonBinding "INV-07;INV-14;INV-18")
openlmm_add_python_test(
  open_lmm_python_runtime_tests test_runtime.py L3
  adapters.python.runtime RuntimeClient "INV-05;INV-06;INV-07;INV-08")
openlmm_add_python_test(
  open_lmm_python_config_tests test_config.py L3
  adapters.python.config RuntimeClient "INV-03;INV-04;INV-07")
openlmm_add_python_test(
  open_lmm_python_callback_tests test_callbacks.py L5
  adapters.python.events PythonSubscription "INV-08;INV-17")
openlmm_add_python_test(
  open_lmm_python_numpy_tests test_numpy.py L3
  adapters.python.visualization PythonBufferOwner "INV-09;INV-10;INV-16")
openlmm_add_python_test(
  open_lmm_python_lifetime_tests test_lifetime.py L5
  adapters.python.lifetime PythonRuntimeHolder "INV-08;INV-17")

openlmm_add_python_test(
  open_lmm_python_experiment_api_tests experiments/test_public_api.py L2
  experiments.api ExperimentAPI "INV-13;INV-14")
openlmm_add_python_test(
  open_lmm_python_experiment_cli_tests experiments/test_cli.py L2
  experiments.cli ExperimentCLI "INV-13;INV-14;INV-18")
openlmm_add_python_test(
  open_lmm_python_experiment_manifest_tests experiments/test_manifest.py L2
  experiments.manifest ExperimentManifest "INV-02;INV-14;INV-18")
openlmm_add_python_test(
  open_lmm_python_experiment_planner_tests experiments/test_planner.py L2
  experiments.planner TrialPlanner "INV-08;INV-16;INV-18")
openlmm_add_python_test(
  open_lmm_python_experiment_config_tests experiments/test_config_materializer.py L3
  experiments.config ConfigMaterializer "INV-01;INV-02;INV-03;INV-04")
openlmm_add_python_test(
  open_lmm_python_experiment_metric_tests experiments/test_metrics.py L3
  experiments.metrics MetricCollector "INV-09;INV-10;INV-16;INV-18")
openlmm_add_python_test(
  open_lmm_python_experiment_export_tests experiments/test_export.py L2
  experiments.export EvidenceWriter "INV-14;INV-16;INV-18")
openlmm_add_python_test(
  open_lmm_python_experiment_runner_tests experiments/test_runner.py L4
  experiments.workflow ExperimentWorker "INV-01;INV-02;INV-03;INV-04;INV-05;INV-06;INV-07;INV-08;INV-17")
openlmm_add_python_test(
  open_lmm_python_experiment_lifetime_tests experiments/test_process_lifetime.py L5
  experiments.lifetime ExperimentWorker "INV-08;INV-17")
openlmm_add_python_test(
  open_lmm_python_experiment_replay_tests experiments/test_replay_adapter.py L4
  experiments.replay ReplayAdapter "INV-02;INV-03;INV-04;INV-14")
openlmm_add_python_test(
  open_lmm_python_experiment_benchmark_tests experiments/test_benchmark_adapter.py L3
  experiments.benchmark BenchmarkAdapter "INV-13;INV-16;INV-18")

if(OPEN_LMM_PYTHON_WHEEL_TEST_PYTHON)
  if(NOT EXISTS "${OPEN_LMM_PYTHON_WHEEL_TEST_PYTHON}")
    message(FATAL_ERROR
      "OPEN_LMM_PYTHON_WHEEL_TEST_PYTHON does not exist: "
      "${OPEN_LMM_PYTHON_WHEEL_TEST_PYTHON}")
  endif()
  openlmm_add_test(
    NAME open_lmm_python_wheel_tests
    COMMAND ${CMAKE_COMMAND}
    COMMAND_ARGS -E env
      --unset=LD_LIBRARY_PATH
      --unset=PYTHONPATH
      --unset=ROS_VERSION
      --unset=AMENT_PREFIX_PATH
      --unset=COLCON_PREFIX_PATH
      ${OPEN_LMM_PYTHON_WHEEL_TEST_PYTHON}
      -I
      "${PROJECT_SOURCE_DIR}/test/adapters/python/test_wheel_install.py"
    LAYER L3 MODULE package.python OWNER PythonWheel
    INVARIANTS INV-13 INV-14 LANES pr
    WORKING_DIRECTORY /tmp)
endif()
