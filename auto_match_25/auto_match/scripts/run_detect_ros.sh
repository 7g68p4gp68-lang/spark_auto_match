#!/usr/bin/env bash
set -euo pipefail

VENV_PYTHON="${AUTO_MATCH_VENV_PYTHON:-${HOME}/yolo_venv/bin/python}"
if [[ ! -x "${VENV_PYTHON}" ]]; then
  echo "[detect_ros] Python 不存在或不可执行: ${VENV_PYTHON}" >&2
  echo "[detect_ros] 请创建 ~/yolo_venv，或设置 AUTO_MATCH_VENV_PYTHON" >&2
  exit 2
fi

PACKAGE_PATH="$(rospack find auto_match)"
exec "${VENV_PYTHON}" "${PACKAGE_PATH}/nodes/detect_ros.py" "$@"
