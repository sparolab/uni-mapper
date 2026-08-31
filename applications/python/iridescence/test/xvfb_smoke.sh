#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 PACKAGE_PATH" >&2
  exit 2
fi
if ! command -v xvfb-run >/dev/null 2>&1; then
  echo "xvfb-run is unavailable; skipping Python Iridescence smoke test"
  exit 77
fi
if ! python3 -c 'import pyridescence' >/dev/null 2>&1; then
  echo "pyridescence is unavailable; skipping Python Iridescence smoke test"
  exit 77
fi

PYTHONPATH=$1 xvfb-run -a python3 - <<'PY'
import numpy as np
from pyridescence import glk, guik, imgui

viewer = guik.LightViewer.instance(title="OpenLMM Iridescence smoke")
assert viewer.ok()
viewer.set_point_shape(0.025, True, True)
points = np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32)
viewer.update_drawable("smoke/points", glk.PointCloudBuffer(points), guik.Rainbow())
viewer.update_thin_lines(
    "smoke/line",
    points,
    np.asarray([[1.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0, 1.0]], dtype=np.float32),
    line_strip=True,
    shader_setting=guik.VertexColor(),
)
frames = 0
def ui():
    global frames
    imgui.begin("OpenLMM smoke", True)
    imgui.text("ready")
    imgui.end()
    frames += 1
    if frames >= 3:
        viewer.close()
viewer.register_ui_callback("smoke", ui)
viewer.spin()
guik.destroy()
assert frames >= 3
PY

PYTHONPATH=$1 xvfb-run -a python3 "$(dirname "$0")/application_smoke.py"
