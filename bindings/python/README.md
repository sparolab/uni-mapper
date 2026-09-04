# OpenLMM Python API v1

This package is the optional CPython leaf adapter for OpenLMM's public
`RuntimeClient`. It does not expose runtime internals or PCL objects.

The official v3 wheel targets CPython 3.10 on Ubuntu 22.04 x86-64 and is
attached to each GitHub release with checksums, a CycloneDX SBOM and artifact
attestation. It is not a manylinux/PyPI or wider-platform compatibility claim.

## Local wheel build

For a repository developer checkout, the root Makefile owns the convenience
orchestration while this directory remains the wheel source owner:

```bash
make python-build    # exact wheel-profile core + local CPython 3.10 wheel
make python-install  # install that wheel into install/dev/python-venv
make python-run      # run examples/basic_runtime.py with open_lmm/config
make python          # build, install, and run in sequence
make python-clean    # remove only the generated Python developer roots
```

Use `CONFIG=/absolute/config/path`, `PYTHON=/path/to/python3.10`,
`PYTHON_JOBS=N`, or `PYTHON_EXAMPLE=/absolute/example.py` as needed. The first
`python-build` installs the reviewed build constraints into
`build/dev/python-build-venv`; it never installs into the caller's Python.
The runtime wheel is installed separately under `install/dev/python-venv`.

The equivalent lower-level flow is retained for CI and custom prefixes.

Create a dedicated CPython 3.10 environment and install the reviewed build
set before invoking the helper:

```bash
python3.10 -m venv /tmp/openlmm-python-build
/tmp/openlmm-python-build/bin/pip install \
  -r bindings/python/build-constraints.txt
bindings/python/build_local_wheel.sh \
  /tmp/openlmm-python-build/bin/python \
  /path/to/wheel-profile/open_lmm/install \
  /tmp/openlmm-wheelhouse
```

The helper deliberately uses `--no-build-isolation`: the caller owns the
exact build environment. A normal PEP 517 isolated build is also supported;
its ranges are declared in `pyproject.toml`, while the constraints file is the
reviewed local baseline.

The core prefix must be a source-free OpenLMM 3.0.0 install built with the
reviewed wheel profile containing all seven built-in algorithm plugins:
ScanContext, SOLiD, HMM-MOS, DUFOMap, OTD, FreeDOM and ERASOR. The local wheel
contains the native bridge and the
exact OpenLMM-owned runtime/plugin closure recorded in
`packaging/open_lmm-python-runtime-closure.tsv`. The bridge itself links only
to installed `open_lmm::client`; packaging owns DSO staging. The wheel must not
be treated as a portable external-plugin ABI promise.

## Use

```python
import open_lmm

with open_lmm.Runtime() as runtime:
    runtime.open("config", label="trial-01")
    runtime.run_all().wait()
    snapshot = runtime.snapshot()
    points = runtime.visualization("agent1").points
```

Long-running native calls release the GIL. Event callbacks run with the GIL,
may issue read-only queries, and must be released with `Subscription.close()`
or a context manager. Visualization points are read-only `float32` NumPy
arrays with shape `(N, 4)` in `x, y, z, intensity` order.

See `examples/` for cancellation, event, and visualization workflows.

## Reproducible experiments

`open_lmm.experiments` is the pure-Python evaluation layer. It verifies a
closed dataset/config/software manifest, materializes a new config snapshot,
and runs every trial in a fresh worker process. Results are published as
no-overwrite JSON, long-form CSV, and SHA-256 evidence. It never imports the
native extension directly and does not replace RuntimeClient authority.

The user-facing command is owned by the separate `open-lmm-experiment`
application wheel in `applications/python/experiment`. Install it alongside
the exact `open-lmm==3.0.0` SDK wheel before using these commands.

```bash
open-lmm-experiment validate --manifest experiment.json
open-lmm-experiment run \
  --manifest experiment.json \
  --dataset-root /immutable/datasets/tiny-v1 \
  --config-root /reviewed/config \
  --evidence-root /new/results/tiny-run-001
```

The generic local runner supports a closed `sha256-index-v1` dataset. Goal 03
replay and Goal 05 performance evidence are accessed through explicit
`ReplayToolchain` and `BenchmarkToolchain` paths; Python delegates their
comparison semantics and never rewrites a baseline. Pandas is optional and is
only imported by `ExperimentResult.to_pandas()`.

The optional browser viewer is likewise a separate application wheel under
`applications/python/viser`; it does not add Viser to the SDK dependency or
native runtime closure.

## Supported surface

- Supported: CPython 3.10, Ubuntu 22.04, x86-64, same-image local wheel.
- Unsupported in v1: Python 3.11+, PyPy, free-threaded CPython, Windows,
  macOS, public PyPI/source-only installation, and Python algorithm plugins.
- Experiment v1 is local and serial. Distributed/parallel sweeps, portable
  Tools artifacts, ground-truth ATE/RPE policy, and baseline promotion remain
  outside this package.
