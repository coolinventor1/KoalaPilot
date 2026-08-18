#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
OPENPILOT_DIR="$(realpath "$SCRIPT_DIR/../../")"
REPO_DIR="$(realpath "$OPENPILOT_DIR/..")"
VENV_PYTHON="$REPO_DIR/.venv/bin/python"
if [[ -x "$VENV_PYTHON" ]]; then
  PYTHON_BIN="$VENV_PYTHON"
else
  PYTHON_CMD=$(command -v python3)
  PYTHON_BIN="$(cd "$(dirname "$PYTHON_CMD")" && pwd)/$(basename "$PYTHON_CMD")"
fi
export PYTHONPATH="$REPO_DIR${PYTHONPATH:+:$PYTHONPATH}"

export PASSIVE="0"
export NOBOARD="1"
export SIMULATION="1"
export SKIP_FW_QUERY="1"
export FINGERPRINT="HONDA_CIVIC_2022"

# Desktop simulation defaults. Keep the UI smooth without spending most CPU
# time drawing it, and use WebGPU for modeld when the Dawn runtime is installed.
export FPS="${FPS:-30}"
# WSLg can report a bogus monitor height, and the old 0.35 default made the
# rendered dashboard itself tiny even when its outer window was maximized.
# Render the normal PC UI at 2x so its contents stay readable.
export SCALE="${SCALE:-2.0}"
if [[ -e /dev/dxg ]]; then
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
fi
if "$PYTHON_BIN" -c "import onnxruntime as ort; assert 'OpenVINOExecutionProvider' in ort.get_available_providers()" 2>/dev/null; then
  export MODEL_ONNX_RUNTIME="${MODEL_ONNX_RUNTIME:-1}"
  export DEV="${DEV:-CPU}"
elif ldconfig -p 2>/dev/null | grep -q libwebgpu_dawn; then
  export DEV="${DEV:-WEBGPU}"
  export WEBGPU_BACKEND="${WEBGPU_BACKEND:-OpenGL}"
fi

export BLOCK="${BLOCK},camerad,loggerd,encoderd,micd,logmessaged,manage_athenad"
if [[ "$CI" ]]; then
  # TODO: offscreen UI should work
  export BLOCK="${BLOCK},ui"
fi

"$PYTHON_BIN" -c "from openpilot.selfdrive.test.helpers import set_params_enabled; set_params_enabled()"
"$PYTHON_BIN" -c "from openpilot.common.params import Params; Params().put_bool('ExperimentalMode', False)"

cd $OPENPILOT_DIR/system/manager && exec "$PYTHON_BIN" ./manager.py
