# OpenLMM distribution contract

This directory is the private repository-wide composition and verification
owner. It is not an installed package and does not provide a user-facing
installer.

The verifier consumes completed leaf build directories only. It composes core,
CLI and GUI into an empty native staging prefix, installs ROS into a separate
overlay, and treats the Python wheel/venv as a separate namespace. It rejects
same-namespace path collisions, exact-version skew, unsafe legacy migration
rows, non-idempotent reinstall, and manifest uninstall that removes another
owner or an unknown file.

Canonical ownership manifests live in `manifests/`. Migration entries are
exact paths; recursive cleanup patterns are intentionally unsupported.

Official publication is narrower than the complete source distribution. The
candidate workflow builds Ubuntu 22.04 x86-64 Headless Core + CLI and the
CPython 3.10 wheel, verifies reproducibility and source-free consumption,
blocks HIGH/CRITICAL Trivy findings, emits CycloneDX SBOMs and GitHub
attestations, and publishes immutable RC artifacts. Stable promotion retags
the exact RC image digest and reuses its wheel, checksums, SBOMs and manifest;
it does not rebuild or publish `latest`. GUI and ROS stay in regression lanes.
