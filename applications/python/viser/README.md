# OpenLMM Viser

`open-lmm-viser` is a local browser viewer for committed OpenLMM point clouds
and trajectories. It consumes only the public `open_lmm` Python SDK.

```bash
make viser
```

By default the viewer starts idle so work can be started from the browser
controls:

```bash
make viser-run
```

Use `make viser-run VISER_AUTO_RUN=true` to opt into Run All on startup.

The root developer entrypoint builds and installs the exact OpenLMM Python
wheel, installs this application and its Viser dependency into the same
developer venv, and starts the command below. Existing artifacts can be run
without rebuilding with `make viser-run`.

```bash
open-lmm-viser /path/to/config \
  --host 127.0.0.1 --port 8080 --preview-voxel-size-m 0.2
```

The command prints the server URL and keeps the viewer available until Ctrl-C.
Its controls are split into independent movable panels instead of sharing one
GUI column:

- **Runtime Control** is docked left and provides Run All, the four pipeline
  stages, all six pipeline nodes, per-agent Optimize Through, active-job
  cancellation, runtime/job revisions and live progress.
- **Configuration** is docked right and provides revision-bound transactional
  edits for the dataset root, ordered agents, output root, alignment settings,
  Loop Detector and Dynamic Remover selectors. `Refresh Agents` scans only the
  immediate child directories of the local Dataset Root and exposes them as
  checkboxes. Selection order is execution order; uncheck and recheck an agent
  to move it last.
- **Alignment Review** is a separate right-side panel that remains hidden until
  an interactive alignment review is active.

Agent-scoped commands use the authoritative agent catalog from the runtime
snapshot. Selector choices come only from the runtime's bounded,
schema-validated candidate catalog and submit its canonical document and
relative logical path in the same transaction. The application performs only
read-only dataset-directory discovery; it never reads or writes config files.
Pass `--auto-run` to opt into Run All on startup.

Interactive alignment reviews appear automatically while an Alignment job is
waiting for feedback. Target and source clouds use the same deterministic
per-agent colors as their committed map and trajectory. The review supports
Accept, KISS retry, Descriptor retry, 6-DoF manual transform, agent exclusion,
and cancellation. Click `Manual Align` to reveal the unobstructed transform
gizmo, edit the source pose, then click `Apply Manual Transform`. `Reset
Proposal`, `Reset Identity`, and `Cancel Manual` do not submit a runtime
response. The review source cloud and trajectory live under an independent
presentation frame, so every gizmo update moves the downsampled source map
immediately while hiding the gizmo never hides the map. During review the
normal committed map is hidden but retained, leaving only
the fixed target review cloud and movable source review cloud on screen. It is
restored if review ends without a committed replacement. Responses are enabled
only when the panel still owns the exact active Python `Job`. Accepted review
geometry remains visible until the committed presentation for the source agent
replaces it.

It does not open a browser automatically. The default loopback bind is
unauthenticated; do not expose it to a public network.

Supported delivery: OpenLMM 3.0.0, CPython 3.10, Ubuntu 22.04 x86-64,
same-image local wheels.
