# OpenLMM Python Iridescence

Native desktop visualization and runtime control for OpenLMM, implemented with
the public `open_lmm.Runtime` Python API and `pyridescence`.

## Requirements

- Ubuntu 22.04 x86_64
- CPython 3.10
- An X11 or Wayland OpenGL display
- An installed `open-lmm==3.0.0` wheel

## Usage

```bash
open-lmm-iridescence CONFIG_DIR \
  --label iridescence \
  --output-root /path/to/output
```

`--auto-run` starts the full pipeline after the first authoritative runtime
snapshot. `--preview-voxel-size-m` requests a downsampled visualization without
changing committed runtime state.

The application is an adapter: configuration changes and commands always pass
through `Runtime`; it never edits the configuration directory directly.
