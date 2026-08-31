SHELL := /bin/bash
.SHELLFLAGS := -eu -o pipefail -c
.ONESHELL:
.DEFAULT_GOAL := help

REPOSITORY_ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
CORE_SOURCE := $(REPOSITORY_ROOT)/open_lmm
GUI_SOURCE := $(REPOSITORY_ROOT)/applications/gui
CLI_SOURCE := $(REPOSITORY_ROOT)/applications/cli
PYTHON_SOURCE := $(REPOSITORY_ROOT)/bindings/python
ROS_SOURCE := $(REPOSITORY_ROOT)/ros
DEV_BUILD_ROOT := $(REPOSITORY_ROOT)/build/dev
CORE_BUILD := $(DEV_BUILD_ROOT)/core
GUI_BUILD := $(DEV_BUILD_ROOT)/gui
CLI_BUILD := $(DEV_BUILD_ROOT)/cli
PYTHON_CORE_BUILD := $(DEV_BUILD_ROOT)/python-core
PYTHON_BUILD_VENV := $(DEV_BUILD_ROOT)/python-build-venv
PYTHON_WHEEL_DIR := $(DEV_BUILD_ROOT)/python-wheel
ROS_BUILD := $(DEV_BUILD_ROOT)/ros
ROS_LOG := $(DEV_BUILD_ROOT)/ros-log
DEV_PREFIX := $(REPOSITORY_ROOT)/install/dev
PYTHON_CORE_PREFIX := $(DEV_PREFIX)/python-core
PYTHON_VENV := $(DEV_PREFIX)/python-venv
PYTHON_WHEEL_FILE := $(PYTHON_WHEEL_DIR)/open_lmm-3.0.0-cp310-cp310-linux_x86_64.whl
ROS_PREFIX := $(DEV_PREFIX)/ros-overlay
GUI_EXECUTABLE := $(DEV_PREFIX)/bin/open_lmm_gui
CLI_EXECUTABLE := $(DEV_PREFIX)/bin/open_lmm_batch
ROS_EXECUTABLE := $(ROS_PREFIX)/open_lmm_ros/lib/open_lmm_ros/open_lmm_rosnode
ROS_OVERLAY_SETUP := $(ROS_PREFIX)/setup.bash
INSTALL_MANIFEST_REMOVER := $(REPOSITORY_ROOT)/scripts/dev/remove_install_manifest.sh

CONFIG ?= $(REPOSITORY_ROOT)/open_lmm/config
BUILD_TYPE ?= Release
JOBS ?= $(shell nproc)
CC ?= cc
CXX ?= c++
GUI_USE_SYSTEM_IRIDESCENCE ?= ON
GUI_PLUGIN ?= $(DEV_PREFIX)/lib/libopen_lmm_iridescence_gui.so
ROS_DISTRO ?= humble
ROS_SYSTEM_SETUP ?= /opt/ros/$(ROS_DISTRO)/setup.bash
ROS_USE_RVIZ ?= true
PYTHON ?= python3.10
PYTHON_JOBS ?= 2
PYTHON_EXAMPLE ?= $(PYTHON_SOURCE)/examples/basic_runtime.py

.PHONY: help core-build core-clean gui gui-build gui-run gui-clean \
  cli cli-build cli-run cli-clean python python-build python-install \
  python-run python-clean ros ros-build ros-run ros-clean dev-clean

help:
	@printf '%s\n' \
	  'OpenLMM core + application developer commands' \
	  '' \
	  '  make core-build  Build/install only the C++ core' \
	  '  make core-clean  Remove core-owned developer artifacts' \
	  '  make gui         Build core + GUI, then run the GUI' \
	  '  make gui-build   Build/install core + GUI without running it' \
	  '  make gui-run     Run the existing installed developer GUI' \
	  '  make gui-clean   Remove GUI-owned developer artifacts' \
	  '  make cli         Build core + CLI, then run the batch pipeline' \
	  '  make cli-build   Build/install core + CLI without running it' \
	  '  make cli-run     Run the existing installed batch application' \
	  '  make cli-clean   Remove CLI-owned developer artifacts' \
	  '  make python      Build/install the local wheel, then run the example' \
	  '  make python-build Build the wheel-profile core and local wheel' \
	  '  make python-install Install the existing wheel into its developer venv' \
	  '  make python-run  Run the example from the existing developer venv' \
	  '  make python-clean Remove Python-owned developer artifacts' \
	  '  make ros         Build core + ROS, then launch ROS with RViz' \
	  '  make ros-build   Build/install core + ROS without launching it' \
	  '  make ros-run     Launch existing installed ROS artifacts' \
	  '  make ros-clean   Remove ROS-owned developer artifacts' \
	  '  make dev-clean   Remove the complete generated developer tree' \
	  '  make help        Show this help' \
	  '' \
	  'Overrides:' \
	  '  CONFIG=/path                 (default: open_lmm/config)' \
	  '  BUILD_TYPE=Release           CMake build type' \
	  '  JOBS=N                       parallel build jobs' \
	  '  CC=/path CXX=/path           C/C++ compilers' \
	  '  GUI_USE_SYSTEM_IRIDESCENCE=ON|OFF' \
	  '  GUI_PLUGIN=/path/to/plugin.so' \
	  '  PYTHON=/path/to/python3.10      CPython 3.10 interpreter' \
	  '  PYTHON_JOBS=N                   wheel-profile core jobs (default: 2)' \
	  '  PYTHON_EXAMPLE=/path/to/example.py' \
	  '  ROS_DISTRO=humble             ROS 2 distribution' \
	  '  ROS_SYSTEM_SETUP=/path        ROS 2 setup.bash' \
	  '  ROS_USE_RVIZ=true|false       launch RViz with the ROS node'

core-build:
	@for tool in cmake "$(CC)" "$(CXX)"; do
	  if ! command -v "$$tool" >/dev/null 2>&1; then
	    printf 'Required build tool was not found: %s\n' "$$tool" >&2
	    exit 1
	  fi
	done
	if [[ ! "$(JOBS)" =~ ^[1-9][0-9]*$$ ]]; then
	  printf 'JOBS must be a positive integer, got: %s\n' "$(JOBS)" >&2
	  exit 1
	fi
	printf '==> configuring OpenLMM core (%s)\n' "$(BUILD_TYPE)"
	CC="$(CC)" CXX="$(CXX)" cmake \
	  -S "$(CORE_SOURCE)" -B "$(CORE_BUILD)" \
	  -DCMAKE_BUILD_TYPE="$(BUILD_TYPE)" \
	  -DCMAKE_INSTALL_PREFIX="$(DEV_PREFIX)" \
	  -DBUILD_TESTING=OFF
	cmake --build "$(CORE_BUILD)" --parallel "$(JOBS)"
	cmake --install "$(CORE_BUILD)"
	if [[ ! -f "$(DEV_PREFIX)/share/open_lmm/cmake/open_lmmConfig.cmake" ]]; then
	  printf 'Installed core package config is missing under: %s\n' \
	    "$(DEV_PREFIX)" >&2
	  exit 1
	fi
	printf '==> developer core installed in %s\n' "$(DEV_PREFIX)"

gui:
	@$(MAKE) --no-print-directory gui-build \
	  CONFIG="$(CONFIG)" BUILD_TYPE="$(BUILD_TYPE)" JOBS="$(JOBS)" \
	  CC="$(CC)" CXX="$(CXX)" \
	  GUI_USE_SYSTEM_IRIDESCENCE="$(GUI_USE_SYSTEM_IRIDESCENCE)" \
	  GUI_PLUGIN="$(GUI_PLUGIN)"
	@$(MAKE) --no-print-directory gui-run \
	  CONFIG="$(CONFIG)" GUI_PLUGIN="$(GUI_PLUGIN)"

gui-build:
	@config_dir="$(CONFIG)"
	if [[ ! -d "$$config_dir" ]]; then
	  printf 'OpenLMM config directory does not exist: %s\n' "$$config_dir" >&2
	  exit 1
	fi
	if [[ ! -f "$$config_dir/config.json" ]]; then
	  printf 'OpenLMM config file does not exist: %s/config.json\n' "$$config_dir" >&2
	  exit 1
	fi
	case "$(GUI_USE_SYSTEM_IRIDESCENCE)" in
	  ON|OFF) ;;
	  *)
	    printf 'GUI_USE_SYSTEM_IRIDESCENCE must be ON or OFF, got: %s\n' \
	      "$(GUI_USE_SYSTEM_IRIDESCENCE)" >&2
	    exit 1
	    ;;
	esac
	$(MAKE) --no-print-directory core-build \
	  BUILD_TYPE="$(BUILD_TYPE)" JOBS="$(JOBS)" CC="$(CC)" CXX="$(CXX)"
	printf '==> configuring standalone GUI against installed core\n'
	CC="$(CC)" CXX="$(CXX)" cmake \
	  -S "$(GUI_SOURCE)" -B "$(GUI_BUILD)" \
	  -DCMAKE_BUILD_TYPE="$(BUILD_TYPE)" \
	  -DCMAKE_INSTALL_PREFIX="$(DEV_PREFIX)" \
	  -DCMAKE_PREFIX_PATH="$(DEV_PREFIX)" \
	  -DOPEN_LMM_GUI_BUILD_IRIDESCENCE=ON \
	  -DOPEN_LMM_GUI_USE_SYSTEM_IRIDESCENCE="$(GUI_USE_SYSTEM_IRIDESCENCE)" \
	  -DBUILD_TESTING=OFF
	cmake --build "$(GUI_BUILD)" --parallel "$(JOBS)"
	cmake --install "$(GUI_BUILD)"
	if [[ ! -x "$(GUI_EXECUTABLE)" ]]; then
	  printf 'Installed GUI executable is missing: %s\n' "$(GUI_EXECUTABLE)" >&2
	  exit 1
	fi
	if [[ ! -f "$(GUI_PLUGIN)" ]]; then
	  printf 'Installed GUI plugin is missing: %s\n' "$(GUI_PLUGIN)" >&2
	  exit 1
	fi
	if ! env -u LD_LIBRARY_PATH "$(GUI_EXECUTABLE)" --help >/dev/null; then
	  printf 'Installed GUI does not run from its RPATH: %s\n' \
	    "$(GUI_EXECUTABLE)" >&2
	  exit 1
	fi
	if command -v ldd >/dev/null 2>&1; then
	  unresolved=$$(ldd "$(GUI_PLUGIN)" | sed -n '/not found/p')
	  if [[ -n "$$unresolved" ]]; then
	    printf 'Installed GUI plugin has unresolved runtime dependencies:\n%s\n' \
	      "$$unresolved" >&2
	    exit 1
	  fi
	fi
	printf '==> developer GUI installed in %s\n' "$(DEV_PREFIX)"

gui-run:
	@config_dir="$(CONFIG)"
	if [[ ! -d "$$config_dir" || ! -f "$$config_dir/config.json" ]]; then
	  printf 'A config directory containing config.json is required: %s\n' \
	    "$$config_dir" >&2
	  exit 1
	fi
	if [[ ! -x "$(GUI_EXECUTABLE)" ]]; then
	  printf 'Developer GUI is not built; run make gui-build first: %s\n' \
	    "$(GUI_EXECUTABLE)" >&2
	  exit 1
	fi
	if [[ ! -f "$(GUI_PLUGIN)" ]]; then
	  printf 'GUI plugin was not found; run make gui-build first: %s\n' \
	    "$(GUI_PLUGIN)" >&2
	  exit 1
	fi
	if [[ -z "$${DISPLAY:-}" && -z "$${WAYLAND_DISPLAY:-}" ]]; then
	  printf '%s\n' \
	    'No display is available; set DISPLAY or WAYLAND_DISPLAY before running the GUI.' >&2
	  exit 1
	fi
	config_dir=$$(realpath "$$config_dir")
	plugin_path=$$(realpath "$(GUI_PLUGIN)")
	printf '==> starting %s with config %s\n' "$(GUI_EXECUTABLE)" "$$config_dir"
	exec "$(GUI_EXECUTABLE)" "$$config_dir" "$$plugin_path"

cli:
	@$(MAKE) --no-print-directory cli-build \
	  CONFIG="$(CONFIG)" BUILD_TYPE="$(BUILD_TYPE)" JOBS="$(JOBS)" \
	  CC="$(CC)" CXX="$(CXX)"
	@$(MAKE) --no-print-directory cli-run CONFIG="$(CONFIG)"

cli-build:
	@config_dir="$(CONFIG)"
	if [[ ! -d "$$config_dir" || ! -f "$$config_dir/config.json" ]]; then
	  printf 'A config directory containing config.json is required: %s\n' \
	    "$$config_dir" >&2
	  exit 1
	fi
	$(MAKE) --no-print-directory core-build \
	  BUILD_TYPE="$(BUILD_TYPE)" JOBS="$(JOBS)" CC="$(CC)" CXX="$(CXX)"
	printf '==> configuring CLI against installed core\n'
	CC="$(CC)" CXX="$(CXX)" cmake \
	  -S "$(CLI_SOURCE)" -B "$(CLI_BUILD)" \
	  -DCMAKE_BUILD_TYPE="$(BUILD_TYPE)" \
	  -DCMAKE_INSTALL_PREFIX="$(DEV_PREFIX)" \
	  -DCMAKE_PREFIX_PATH="$(DEV_PREFIX)" \
	  -DBUILD_TESTING=OFF
	cmake --build "$(CLI_BUILD)" --parallel "$(JOBS)"
	cmake --install "$(CLI_BUILD)" --component Tools
	if [[ ! -x "$(CLI_EXECUTABLE)" ]]; then
	  printf 'Installed CLI executable is missing: %s\n' "$(CLI_EXECUTABLE)" >&2
	  exit 1
	fi
	if ! env -u LD_LIBRARY_PATH "$(CLI_EXECUTABLE)" --help >/dev/null; then
	  printf 'Installed CLI does not run from its RPATH: %s\n' \
	    "$(CLI_EXECUTABLE)" >&2
	  exit 1
	fi
	if command -v ldd >/dev/null 2>&1; then
	  unresolved=$$(ldd "$(CLI_EXECUTABLE)" | sed -n '/not found/p')
	  if [[ -n "$$unresolved" ]]; then
	    printf 'Installed CLI has unresolved runtime dependencies:\n%s\n' \
	      "$$unresolved" >&2
	    exit 1
	  fi
	fi
	printf '==> developer CLI installed in %s\n' "$(DEV_PREFIX)"

cli-run:
	@config_dir="$(CONFIG)"
	if [[ ! -d "$$config_dir" || ! -f "$$config_dir/config.json" ]]; then
	  printf 'A config directory containing config.json is required: %s\n' \
	    "$$config_dir" >&2
	  exit 1
	fi
	if [[ ! -x "$(CLI_EXECUTABLE)" ]]; then
	  printf 'Developer CLI is not built; run make cli-build first: %s\n' \
	    "$(CLI_EXECUTABLE)" >&2
	  exit 1
	fi
	config_dir=$$(realpath "$$config_dir")
	printf '==> starting %s with config %s\n' "$(CLI_EXECUTABLE)" "$$config_dir"
	exec "$(CLI_EXECUTABLE)" "$$config_dir"

python:
	@$(MAKE) --no-print-directory python-build \
	  CC="$(CC)" CXX="$(CXX)" PYTHON="$(PYTHON)" \
	  PYTHON_JOBS="$(PYTHON_JOBS)"
	@$(MAKE) --no-print-directory python-install PYTHON="$(PYTHON)"
	@$(MAKE) --no-print-directory python-run \
	  CONFIG="$(CONFIG)" PYTHON_EXAMPLE="$(PYTHON_EXAMPLE)"

python-build:
	@for tool in cmake "$(CC)" "$(CXX)" "$(PYTHON)"; do
	  if ! command -v "$$tool" >/dev/null 2>&1; then
	    printf 'Required Python build tool was not found: %s\n' "$$tool" >&2
	    exit 1
	  fi
	done
	python_version=$$("$(PYTHON)" -c \
	  'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
	if [[ "$$python_version" != 3.10 ]]; then
	  printf 'OpenLMM local wheel requires CPython 3.10, got: %s\n' \
	    "$$python_version" >&2
	  exit 1
	fi
	if [[ ! "$(PYTHON_JOBS)" =~ ^[1-9][0-9]*$$ ]]; then
	  printf 'PYTHON_JOBS must be a positive integer, got: %s\n' \
	    "$(PYTHON_JOBS)" >&2
	  exit 1
	fi
	if [[ ! -x "$(PYTHON_BUILD_VENV)/bin/python" ]]; then
	  "$(PYTHON)" -m venv --system-site-packages "$(PYTHON_BUILD_VENV)"
	fi
	"$(PYTHON_BUILD_VENV)/bin/python" -m pip install \
	  --disable-pip-version-check \
	  --requirement "$(PYTHON_SOURCE)/build-constraints.txt"
	printf '%s\n' '==> configuring exact Python wheel-profile core (Release)'
	CC="$(CC)" CXX="$(CXX)" cmake \
	  -S "$(CORE_SOURCE)" -B "$(PYTHON_CORE_BUILD)" \
	  -DCMAKE_BUILD_TYPE=Release \
	  -DCMAKE_INSTALL_PREFIX="$(PYTHON_CORE_PREFIX)" \
	  -DUSE_CCACHE=OFF \
	  -DFETCHCONTENT_UPDATES_DISCONNECTED=ON \
	  -DBUILD_TESTING=OFF \
	  -DOPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT=ON \
	  -DOPEN_LMM_BUILD_DESCRIPTOR_SOLID=OFF \
	  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_HMM_MOS=OFF \
	  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_DUFOMAP=OFF \
	  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_OTD=OFF \
	  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM=ON \
	  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_ERASOR=OFF
	cmake --build "$(PYTHON_CORE_BUILD)" --parallel "$(PYTHON_JOBS)"
	cmake --install "$(PYTHON_CORE_BUILD)"
	if [[ ! -f "$(PYTHON_CORE_PREFIX)/share/open_lmm/cmake/open_lmmConfig.cmake" ]]; then
	  printf 'Python wheel-profile core install is incomplete: %s\n' \
	    "$(PYTHON_CORE_PREFIX)" >&2
	  exit 1
	fi
	if [[ "$(PYTHON_WHEEL_DIR)" != "$(REPOSITORY_ROOT)/build/dev/python-wheel" ]]; then
	  printf '%s\n' 'Refusing to replace a wheel directory outside the fixed developer root.' >&2
	  exit 1
	fi
	cmake -E remove_directory "$(PYTHON_WHEEL_DIR)"
	"$(PYTHON_SOURCE)/build_local_wheel.sh" \
	  "$(PYTHON_BUILD_VENV)/bin/python" \
	  "$(PYTHON_CORE_PREFIX)" "$(PYTHON_WHEEL_DIR)"
	if [[ ! -f "$(PYTHON_WHEEL_FILE)" ]]; then
	  printf 'Expected local wheel was not produced: %s\n' \
	    "$(PYTHON_WHEEL_FILE)" >&2
	  exit 1
	fi
	printf '==> developer Python wheel built: %s\n' "$(PYTHON_WHEEL_FILE)"

python-install:
	@if [[ ! -f "$(PYTHON_WHEEL_FILE)" ]]; then
	  printf 'Developer Python wheel is not built; run make python-build first: %s\n' \
	    "$(PYTHON_WHEEL_FILE)" >&2
	  exit 1
	fi
	if [[ ! -x "$(PYTHON)" ]]; then
	  if ! command -v "$(PYTHON)" >/dev/null 2>&1; then
	    printf 'CPython 3.10 executable was not found: %s\n' "$(PYTHON)" >&2
	    exit 1
	  fi
	fi
	python_version=$$("$(PYTHON)" -c \
	  'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
	if [[ "$$python_version" != 3.10 ]]; then
	  printf 'OpenLMM local wheel requires CPython 3.10, got: %s\n' \
	    "$$python_version" >&2
	  exit 1
	fi
	if [[ ! -x "$(PYTHON_VENV)/bin/python" ]]; then
	  "$(PYTHON)" -m venv --system-site-packages "$(PYTHON_VENV)"
	fi
	env -u PYTHONPATH -u LD_LIBRARY_PATH \
	  "$(PYTHON_VENV)/bin/python" -m pip install \
	  --disable-pip-version-check --no-index --no-deps --force-reinstall \
	  "$(PYTHON_WHEEL_FILE)"
	env -u PYTHONPATH -u LD_LIBRARY_PATH \
	  "$(PYTHON_VENV)/bin/python" -c \
	  'import open_lmm; assert open_lmm.__version__ == "3.0.0"'
	printf '==> developer Python wheel installed in %s\n' "$(PYTHON_VENV)"

python-run:
	@config_dir="$(CONFIG)"
	if [[ ! -d "$$config_dir" || ! -f "$$config_dir/config.json" ]]; then
	  printf 'A config directory containing config.json is required: %s\n' \
	    "$$config_dir" >&2
	  exit 1
	fi
	if [[ ! -x "$(PYTHON_VENV)/bin/python" ]]; then
	  printf 'Developer Python wheel is not installed; run make python-install first: %s\n' \
	    "$(PYTHON_VENV)" >&2
	  exit 1
	fi
	if [[ ! -f "$(PYTHON_EXAMPLE)" ]]; then
	  printf 'Python example does not exist: %s\n' "$(PYTHON_EXAMPLE)" >&2
	  exit 1
	fi
	config_dir=$$(realpath "$$config_dir")
	example_path=$$(realpath "$(PYTHON_EXAMPLE)")
	printf '==> starting Python example %s with config %s\n' \
	  "$$example_path" "$$config_dir"
	exec env -u PYTHONPATH -u LD_LIBRARY_PATH \
	  "$(PYTHON_VENV)/bin/python" "$$example_path" "$$config_dir"

ros:
	@$(MAKE) --no-print-directory ros-build \
	  BUILD_TYPE="$(BUILD_TYPE)" JOBS="$(JOBS)" CC="$(CC)" CXX="$(CXX)" \
	  ROS_DISTRO="$(ROS_DISTRO)" ROS_SYSTEM_SETUP="$(ROS_SYSTEM_SETUP)"
	@$(MAKE) --no-print-directory ros-run \
	  CONFIG="$(CONFIG)" ROS_DISTRO="$(ROS_DISTRO)" \
	  ROS_SYSTEM_SETUP="$(ROS_SYSTEM_SETUP)" ROS_USE_RVIZ="$(ROS_USE_RVIZ)"

ros-build:
	@if ! command -v colcon >/dev/null 2>&1; then
	  printf '%s\n' 'Required ROS build tool was not found: colcon' >&2
	  exit 1
	fi
	if [[ ! -f "$(ROS_SYSTEM_SETUP)" ]]; then
	  printf 'ROS setup file does not exist: %s\n' "$(ROS_SYSTEM_SETUP)" >&2
	  exit 1
	fi
	$(MAKE) --no-print-directory core-build \
	  BUILD_TYPE="$(BUILD_TYPE)" JOBS="$(JOBS)" CC="$(CC)" CXX="$(CXX)"
	printf '==> building ROS %s against installed core\n' "$(ROS_DISTRO)"
	set +u
	source "$(ROS_SYSTEM_SETUP)"
	set -u
	CC="$(CC)" CXX="$(CXX)" colcon --log-base "$(ROS_LOG)" build \
	  --base-paths "$(ROS_SOURCE)" \
	  --build-base "$(ROS_BUILD)" \
	  --install-base "$(ROS_PREFIX)" \
	  --symlink-install \
	  --cmake-args \
	    -DCMAKE_BUILD_TYPE="$(BUILD_TYPE)" \
	    -DCMAKE_PREFIX_PATH="$(DEV_PREFIX)" \
	    -DBUILD_TESTING=OFF
	if [[ ! -f "$(ROS_OVERLAY_SETUP)" || ! -x "$(ROS_EXECUTABLE)" ]]; then
	  printf 'ROS developer overlay is incomplete under: %s\n' \
	    "$(ROS_PREFIX)" >&2
	  exit 1
	fi
	printf '==> developer ROS overlay installed in %s\n' "$(ROS_PREFIX)"

ros-run:
	@config_dir="$(CONFIG)"
	if [[ ! -d "$$config_dir" || ! -f "$$config_dir/config.json" ]]; then
	  printf 'A config directory containing config.json is required: %s\n' \
	    "$$config_dir" >&2
	  exit 1
	fi
	case "$(ROS_USE_RVIZ)" in
	  true|false) ;;
	  *)
	    printf 'ROS_USE_RVIZ must be true or false, got: %s\n' \
	      "$(ROS_USE_RVIZ)" >&2
	    exit 1
	    ;;
	esac
	if [[ ! -f "$(ROS_SYSTEM_SETUP)" ]]; then
	  printf 'ROS setup file does not exist: %s\n' "$(ROS_SYSTEM_SETUP)" >&2
	  exit 1
	fi
	if [[ ! -f "$(ROS_OVERLAY_SETUP)" || ! -x "$(ROS_EXECUTABLE)" ]]; then
	  printf 'Developer ROS is not built; run make ros-build first: %s\n' \
	    "$(ROS_PREFIX)" >&2
	  exit 1
	fi
	set +u
	source "$(ROS_SYSTEM_SETUP)"
	source "$(ROS_OVERLAY_SETUP)"
	set -u
	if [[ "$(ROS_USE_RVIZ)" == true ]] && ! command -v rviz2 >/dev/null 2>&1; then
	  printf '%s\n' \
	    'RViz is unavailable; install ros-'"$(ROS_DISTRO)"'-rviz2 or use ROS_USE_RVIZ=false.' >&2
	  exit 1
	fi
	config_dir=$$(realpath "$$config_dir")
	export LD_LIBRARY_PATH="$(DEV_PREFIX)/lib$${LD_LIBRARY_PATH:+:$$LD_LIBRARY_PATH}"
	printf '==> launching OpenLMM ROS with config %s (RViz=%s)\n' \
	  "$$config_dir" "$(ROS_USE_RVIZ)"
	exec ros2 launch open_lmm_ros open_lmm_rviz.launch.py \
	  config_path:="$$config_dir" use_rviz:="$(ROS_USE_RVIZ)"

core-clean:
	@"$(INSTALL_MANIFEST_REMOVER)" "$(DEV_PREFIX)" \
	  "$(CORE_BUILD)/install_manifest.txt" core
	cmake -E remove_directory "$(CORE_BUILD)"
	printf '%s\n' '==> removed core-owned developer artifacts'

gui-clean:
	@"$(INSTALL_MANIFEST_REMOVER)" "$(DEV_PREFIX)" \
	  "$(GUI_BUILD)/install_manifest.txt" gui
	cmake -E remove_directory "$(GUI_BUILD)"
	printf '%s\n' '==> removed GUI-owned developer artifacts'

cli-clean:
	@"$(INSTALL_MANIFEST_REMOVER)" "$(DEV_PREFIX)" \
	  "$(CLI_BUILD)/install_manifest_Tools.txt" cli
	cmake -E remove_directory "$(CLI_BUILD)"
	printf '%s\n' '==> removed CLI-owned developer artifacts'

python-clean:
	@if [[ "$(PYTHON_CORE_BUILD)" != "$(REPOSITORY_ROOT)/build/dev/python-core" || \
	      "$(PYTHON_BUILD_VENV)" != "$(REPOSITORY_ROOT)/build/dev/python-build-venv" || \
	      "$(PYTHON_WHEEL_DIR)" != "$(REPOSITORY_ROOT)/build/dev/python-wheel" || \
	      "$(PYTHON_CORE_PREFIX)" != "$(REPOSITORY_ROOT)/install/dev/python-core" || \
	      "$(PYTHON_VENV)" != "$(REPOSITORY_ROOT)/install/dev/python-venv" ]]; then
	  printf '%s\n' 'Refusing to clean paths outside the fixed Python developer roots.' >&2
	  exit 1
	fi
	cmake -E remove_directory "$(PYTHON_CORE_BUILD)"
	cmake -E remove_directory "$(PYTHON_BUILD_VENV)"
	cmake -E remove_directory "$(PYTHON_WHEEL_DIR)"
	cmake -E remove_directory "$(PYTHON_CORE_PREFIX)"
	cmake -E remove_directory "$(PYTHON_VENV)"
	printf '%s\n' '==> removed Python-owned developer artifacts'

ros-clean:
	@if [[ "$(ROS_BUILD)" != "$(REPOSITORY_ROOT)/build/dev/ros" || \
	      "$(ROS_LOG)" != "$(REPOSITORY_ROOT)/build/dev/ros-log" || \
	      "$(ROS_PREFIX)" != "$(REPOSITORY_ROOT)/install/dev/ros-overlay" ]]; then
	  printf '%s\n' 'Refusing to clean paths outside the fixed ROS developer roots.' >&2
	  exit 1
	fi
	cmake -E remove_directory "$(ROS_BUILD)"
	cmake -E remove_directory "$(ROS_LOG)"
	cmake -E remove_directory "$(ROS_PREFIX)"
	printf '%s\n' '==> removed ROS-owned developer artifacts'

dev-clean:
	@if [[ "$(DEV_BUILD_ROOT)" != "$(REPOSITORY_ROOT)/build/dev" || \
	      "$(DEV_PREFIX)" != "$(REPOSITORY_ROOT)/install/dev" ]]; then
	  printf '%s\n' 'Refusing to clean paths outside the fixed developer roots.' >&2
	  exit 1
	fi
	cmake -E remove_directory "$(DEV_BUILD_ROOT)"
	cmake -E remove_directory "$(DEV_PREFIX)"
	printf '%s\n' '==> removed complete generated developer artifacts'
