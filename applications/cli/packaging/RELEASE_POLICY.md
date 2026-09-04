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

The official v3 artifacts are deliberately narrow:

- `ghcr.io/sparolab/uni-mapper:3.0.0[-rc.N]`: Ubuntu 22.04, x86-64, Headless Core + CLI
- `open_lmm-3.0.0-cp310-*-linux_x86_64.whl`: CPython 3.10 on Linux x86-64

ROS 2 Humble and the native GUI remain source-build regression checks, but are
not official binary artifacts. GCC 12, GCC 13 and Clang 15 remain the required
compiler validation matrix. The release image uses CMake 3.25.3 and GTSAM
4.2a9 from pinned, hash-verified inputs. Other environments may work but are
not release gates until added to this contract.

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

The ROS 2 v3 artifact is a headless `open_lmm::client` consumer. It publishes
read-only RViz PointCloud2, Path and MarkerArray presentation and has no
dependency on `OpenLmmGui` or the Iridescence plugin. The former
`OpenLMMROSGui`, `gui_enabled` and `gui_plugin_path` composition surface was
removed by the approved P7 product cutover. The standalone GUI remains a
separate native artifact and runtime owner.

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
| `ros` | ROS overlay | ament install | `core=3.0.0` |

Every installed path has exactly one artifact owner inside its namespace. Native
composition order is `core -> CLI -> GUI`; ROS consumes the core native prefix from a
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
restores matching artifact versions as one distribution transaction. OS
package-manager transfer metadata remains separate work and is not implied by
this CMake ownership split. Official publication is defined below.

The candidate workflow publishes the image to GHCR and attaches the CPython
wheel, SHA256SUMS, CycloneDX image/wheel SBOMs and release-manifest v1 to a
GitHub prerelease. Trivy blocks HIGH and CRITICAL findings. A vulnerability
exception must name the CVE, state its reason in review and carry an
`exp:YYYY-MM-DD` expiry; expired exceptions fail the gate. GitHub artifact
attestations cover the image digest and attached files.

Stable promotion pulls and retags the exact candidate image digest and reuses
the exact candidate files. It never rebuilds and never publishes a mutable
`latest` tag. Rollback selects a previously attested digest.

Runtime transactions provide process-level candidate → validate → commit
atomicity. They do not claim crash- or power-loss durability: there is no WAL,
filesystem-wide fsync protocol or package-manager crash-atomic publication.
After a process or host failure, operators must reconcile against authoritative
runtime state and committed files.

## Required release evidence

The stable required checks documented in `.github/workflows/compiler-matrix.yml`, including the
exact-profile CPython 3.10 wheel contract, must pass.
Branch protection must require those exact names; source workflow changes alone do not establish
that policy. Release evidence also includes the CTest JUnit/log artifacts, source-free consumer
result, installed license/notice and the repository-policy record.

OpenLMM is distributed under GPL-3.0-only as declared in `LICENCE` and the package manifests.
