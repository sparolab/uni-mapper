# OpenLMM local Python wheel policy

This artifact is a same-image CPython 3.10 wheel for Ubuntu 22.04 x86-64.
It consumes an exact OpenLMM 3.0.0 installed core and bundles only the
OpenLMM-owned runtime and selected ScanContext/FreeDOM plugin closure recorded
in `open_lmm-python-runtime-closure.tsv`.

The wheel is not a manylinux, PyPI, portable external-plugin ABI, SBOM,
signature, provenance, or stable-release claim. System C/C++ dependencies
remain part of the reviewed build/runtime image. Those publication and supply
chain guarantees remain Goal 09 work.
