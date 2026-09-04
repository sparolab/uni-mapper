# OpenLMM local Python wheel policy

This artifact is a same-image CPython 3.10 wheel for Ubuntu 22.04 x86-64.
It consumes an exact OpenLMM 3.0.0 installed core and bundles only the
OpenLMM-owned runtime and the complete seven-plugin built-in closure recorded
in `open_lmm-python-runtime-closure.tsv`.

The wheel is not a manylinux, PyPI or portable external-plugin ABI claim.
System C/C++ dependencies remain part of the reviewed build/runtime image.
Release candidates attach the exact wheel with SHA-256, a CycloneDX SBOM and
GitHub artifact attestation; stable releases reuse those bytes unchanged.
