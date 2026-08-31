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

Canonical manifests live in `manifests/`. Migration entries are exact paths;
recursive cleanup patterns are intentionally unsupported. Official artifacts,
SBOM, vulnerability gates, signing, provenance, registry publication and
RC-to-stable promotion remain Goal 09 work.
