# OpenLMM release and compatibility policy

OpenLMM uses semantic versioning for its installed packages. The current package version is
`3.0.0`; `open_lmm`, `open-lmm-cli`, `open_lmm_ros`, the generated CMake package version and
shared-library versions must remain synchronized.

## Compatibility contracts

| Contract | Compatibility guarantee |
|---|---|
| Config schema | Within one major release, migration or an explicit deprecation period is provided. |
| Plugin ABI v1 | Same compiler, standard library and OpenLMM release family only. |
| C++ runtime API | Source/rebuild compatibility only within the documented compiler and standard-library matrix. |
| Artifact files | Only formats carrying an explicit schema/version field are long-term contracts. |

Shared libraries use `SOVERSION=3` and full `VERSION=3.0.0`. A breaking installed C++ or plugin
ABI change requires a major-version and SONAME-major change. Additive source changes still require
the normal compiler matrix and source-free package consumer checks.

Plugin entry ABI v1 remains version 1. GUI plugins are the exception to the
otherwise descriptive capability metadata: the GUI host requires the exact
`gui:services-v3` capability before it calls `create()`, so a DSO compiled for
the previous `GuiServices` layout is rejected safely.

## Supported release matrix

- Ubuntu 22.04, x86-64
- ROS 2 Humble for the ROS adapter
- GCC 12 and GCC 13 with libstdc++
- Clang 15 with the documented libstdc++ 12 compatibility include set
- CMake 3.25.3 in the release/CI image
- GTSAM 4.2a9 as built by `docker/open_lmm.Dockerfile`

Other environments may work but are not release gates until added to the required-check contract.

## Install components and file ownership

The `open_lmm` core artifact owns these components:

- `Runtime`: versioned OpenLMM runtime shared libraries and release/license metadata
- `Development`: public headers, static archives and CMake package metadata
- `PluginSDK`: the toolchain-compatible plugin ABI v1 header and interface target
- `Plugins`: runtime-loaded plugin entry libraries and support DSOs

The separate, exact-version `open-lmm-cli` artifact owns the `Tools` component and
`bin/open_lmm_batch`. The separate, exact-version `open-lmm Python` artifact owns the
`Python` component: the CPython package, native extension, wheel metadata and its reviewed
same-image runtime closure. Core does not own or install either adapter path. An unqualified
install of each artifact contains all components owned by that artifact; it does not acquire
ownership from another artifact.

The separate, exact-version `OpenLmmGui` / `open_lmm_gui` artifact owns GUI
runtime, development, plugin and application files:

- `GuiRuntime`: `libopen_lmm_gui_core.so.3` and GUI release metadata
- `GuiDevelopment`: the `open_lmm_gui::gui` export and `open_lmm/gui/*.hpp` public contracts
- `GuiPlugins`: `libopen_lmm_iridescence_gui.so` when enabled
- `GuiApplication`: `bin/open_lmm_gui`

For the remainder of v3, Compat-A preserves
`find_package(open_lmm 3.0 CONFIG REQUIRED COMPONENTS gui)` and
`open_lmm::gui` as a deprecated config-time alias of the canonical
`open_lmm_gui::gui` imported shared target. It is not a shim DSO or a second
implementation owner. Core-only client discovery does not require GUI,
Iridescence, OpenGL or spdlog; explicit GUI discovery requires the exact
`OpenLmmGui 3.0.0` artifact and fails closed on absence or version skew.

The supported combined distribution installs matching `open_lmm` core, `open_lmm_gui` and `open-lmm-cli`
artifacts into the same clean prefix. Their normalized install manifests must have an
`empty intersection`. Validation first installs core, proves that `bin/open_lmm_batch` is absent, then
installs CLI and exercises the executable without source-tree or build-tree access. Version skew
between core and either application artifact fails closed because each uses an exact-version
CMake package lookup.

Repository-wide composition policy has one private owner under `distribution/`.
It consumes already-built leaf install artifacts and never builds leaf source with
`add_subdirectory`. The schema-v1 topology is fixed as follows:

| Artifact | Namespace | Install mode | Exact dependency |
|---|---|---|---|
| `core` | native prefix | CMake install | none |
| `cli` | native prefix | CMake install | `core=3.0.0` |
| `gui` | native prefix | CMake install | `core=3.0.0` |
| `python` | Python venv | wheel | reviewed `core-runtime-closure=3.0.0` |
| `ros` | ROS overlay | ament install | `core=3.0.0`, and `gui=3.0.0` when enabled |

Every installed path has exactly one artifact owner inside its namespace. Native
composition order is `core -> CLI -> GUI`; ROS consumes that native prefix from a
separate overlay and Python remains in a separate venv. Manifest uninstall simulation
uses the reverse order `ROS -> GUI -> CLI -> core`, followed separately by Python venv
removal, and must preserve other-owner and unknown files. Legacy transfer cleanup is
limited to the exact paths recorded from baseline `cdda354`; recursive directory globs
are forbidden. Canonical publication composes an empty staging prefix and then switches
the published prefix. The in-place upgrade fixture proves file convergence only and
does not claim crash-atomic package-manager behavior.

This changes the source-build installation workflow from one core install to explicit adapter
steps: install core, then configure `applications/cli`, `applications/gui` and `bindings/python` against that installed
core. A combined native prefix is published only after its ownership checks succeed; the Python
wheel remains a separate artifact and must pass its isolated install/runtime audit. Rollback
restores matching artifact versions as one distribution transaction. OS package-manager transfer
metadata, public archives, signing, SBOM and provenance remain separate release-engineering work
and are not implied by this CMake ownership split.

Official archives or images, SBOM and CVE gates, signing, provenance, immutable registry
publication, and RC-to-stable same-bytes promotion remain Goal 09 supply-chain work.
Passing this ownership contract is not evidence that those release gates are complete.

## Required release evidence

The stable required checks documented in `.github/workflows/compiler-matrix.yml`, including the
exact-profile CPython 3.10 wheel contract, must pass.
Branch protection must require those exact names; source workflow changes alone do not establish
that policy. Release evidence also includes the CTest JUnit/log artifacts, source-free consumer
result, installed license/notice and the repository-policy record.

OpenLMM is distributed under GPL-3.0-only as declared in `LICENCE` and the package manifests.
