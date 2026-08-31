# OpenLMM Viser

`open-lmm-viser` is a local browser viewer for committed OpenLMM point clouds
and trajectories. It consumes only the public `open_lmm` Python SDK.

```bash
open-lmm-viser /path/to/config \
  --host 127.0.0.1 --port 8080 --preview-voxel-size-m 0.2
```

The command prints the server URL, runs the pipeline, and keeps the viewer
available until Ctrl-C. It does not open a browser automatically. The default
loopback bind is unauthenticated; do not expose it to a public network.

Supported delivery: OpenLMM 3.0.0, CPython 3.10, Ubuntu 22.04 x86-64,
same-image local wheels.
