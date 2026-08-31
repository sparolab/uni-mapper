# OpenLMM Python API v1

This package is the optional CPython leaf adapter for OpenLMM's public
`RuntimeClient`. It does not expose runtime internals or PCL objects.

Goal 06 supports a local CPython 3.10 wheel on Ubuntu 22.04 x86-64. Portable
public wheel publication, manylinux repair, signing, and a wider
Python/platform matrix are deferred to Goal 09.

## Local wheel build

Create a dedicated CPython 3.10 environment and install the reviewed build
set before invoking the helper:

```bash
python3.10 -m venv /tmp/openlmm-python-build
/tmp/openlmm-python-build/bin/pip install \
  -r open_lmm/src/adapters/python/build-constraints.txt
open_lmm/src/adapters/python/build_local_wheel.sh \
  /tmp/openlmm-python-build/bin/python /tmp/openlmm-wheelhouse
```

The helper deliberately uses `--no-build-isolation`: the caller owns the
exact build environment. A normal PEP 517 isolated build is also supported;
its ranges are declared in `pyproject.toml`, while the constraints file is the
reviewed local baseline.

The local wheel contains the native bridge, OpenLMM-owned runtime DSOs, and
the ScanContext and FreeDOM built-in plugins. The bridge itself links only to
`open_lmm_client`; packaging owns DSO staging. The wheel must not be treated as
a portable external-plugin ABI promise.

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

## Supported surface

- Supported: CPython 3.10, Ubuntu 22.04, x86-64, same-image local wheel.
- Unsupported in v1: Python 3.11+, PyPy, free-threaded CPython, Windows,
  macOS, public PyPI/source-only installation, and Python algorithm plugins.
- Experiment v1 is local and serial. Distributed/parallel sweeps, portable
  Tools artifacts, ground-truth ATE/RPE policy, and baseline promotion remain
  outside this package.
