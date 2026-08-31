SHELL := /bin/bash
.SHELLFLAGS := -eu -o pipefail -c
.ONESHELL:
.DEFAULT_GOAL := help

REPOSITORY_ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
CORE_SOURCE := $(REPOSITORY_ROOT)/open_lmm
GUI_SOURCE := $(REPOSITORY_ROOT)/applications/gui
CLI_SOURCE := $(REPOSITORY_ROOT)/applications/cli
DEV_BUILD_ROOT := $(REPOSITORY_ROOT)/build/dev
CORE_BUILD := $(DEV_BUILD_ROOT)/core
GUI_BUILD := $(DEV_BUILD_ROOT)/gui
CLI_BUILD := $(DEV_BUILD_ROOT)/cli
DEV_PREFIX := $(REPOSITORY_ROOT)/install/dev
GUI_EXECUTABLE := $(DEV_PREFIX)/bin/open_lmm_gui
CLI_EXECUTABLE := $(DEV_PREFIX)/bin/open_lmm_batch
INSTALL_MANIFEST_REMOVER := $(REPOSITORY_ROOT)/scripts/dev/remove_install_manifest.sh

CONFIG ?= $(REPOSITORY_ROOT)/open_lmm/config
BUILD_TYPE ?= Release
JOBS ?= $(shell nproc)
CC ?= cc
CXX ?= c++
GUI_USE_SYSTEM_IRIDESCENCE ?= ON
GUI_PLUGIN ?= $(DEV_PREFIX)/lib/libopen_lmm_iridescence_gui.so

.PHONY: help core-build core-clean gui gui-build gui-run gui-clean \
  cli cli-build cli-run cli-clean dev-clean

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
	  '  make dev-clean   Remove the complete generated developer tree' \
	  '  make help        Show this help' \
	  '' \
	  'Overrides:' \
	  '  CONFIG=/path                 (default: open_lmm/config)' \
	  '  BUILD_TYPE=Release           CMake build type' \
	  '  JOBS=N                       parallel build jobs' \
	  '  CC=/path CXX=/path           C/C++ compilers' \
	  '  GUI_USE_SYSTEM_IRIDESCENCE=ON|OFF' \
	  '  GUI_PLUGIN=/path/to/plugin.so'

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

dev-clean:
	@if [[ "$(DEV_BUILD_ROOT)" != "$(REPOSITORY_ROOT)/build/dev" || \
	      "$(DEV_PREFIX)" != "$(REPOSITORY_ROOT)/install/dev" ]]; then
	  printf '%s\n' 'Refusing to clean paths outside the fixed developer roots.' >&2
	  exit 1
	fi
	cmake -E remove_directory "$(DEV_BUILD_ROOT)"
	cmake -E remove_directory "$(DEV_PREFIX)"
	printf '%s\n' '==> removed complete generated developer artifacts'
