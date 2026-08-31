# OpenLMM release and compatibility policy

OpenLMM uses semantic versioning for its installed packages. The current package version is
`3.0.0`; `open_lmm`, `open_lmm_ros`, the generated CMake package version and shared-library
versions must remain synchronized.

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

## Install components

- `Runtime`: versioned OpenLMM runtime shared libraries and release/license metadata
- `Development`: public headers, static archives and CMake package metadata
- `Plugins`: runtime-loaded plugin entry libraries and support DSOs
- `Tools`: the `open_lmm_batch` executable

A normal unqualified install contains all components. Release validation consumes that installation
without access to the source tree and verifies the batch launcher, exported targets and SONAMEs.

## Required release evidence

The seven stable required checks documented in `.github/workflows/compiler-matrix.yml` must pass.
Branch protection must require those exact names; source workflow changes alone do not establish
that policy. Release evidence also includes the CTest JUnit/log artifacts, source-free consumer
result, installed license/notice and the repository-policy record.

OpenLMM is distributed under GPL-3.0-only as declared in `LICENCE` and the package manifests.
