#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_DIR="${HOME}/.config/onplant"
SYSTEMD_DIR="${HOME}/.config/systemd/user"

mkdir -p "${CONFIG_DIR}" "${SYSTEMD_DIR}"

if [ ! -f "${CONFIG_DIR}/onplant-robot.env" ]; then
  cp "${PROJECT_DIR}/hardware/onplant-robot.env.example" "${CONFIG_DIR}/onplant-robot.env"
  sed -i "s|ONPLANT_PROJECT_DIR=.*|ONPLANT_PROJECT_DIR=${PROJECT_DIR}|" "${CONFIG_DIR}/onplant-robot.env"
  echo "Created ${CONFIG_DIR}/onplant-robot.env"
  echo "Edit ONPLANT_SERVER if the server PC IP is different."
fi

ensure_env() {
  local key="$1"
  local value="$2"
  if ! grep -q "^${key}=" "${CONFIG_DIR}/onplant-robot.env"; then
    printf '%s=%s\n' "${key}" "${value}" >> "${CONFIG_DIR}/onplant-robot.env"
  fi
}

ensure_env ONPLANT_SERVER_INTERVAL 600
ensure_env ONPLANT_WAKE_CONFIDENCE 0.70
ensure_env ONPLANT_VOSK_MODEL "${PROJECT_DIR}/models/vosk-model-small-ko-0.22"
ensure_env ONPLANT_LOCAL_COMMAND_FILE /tmp/onplant-local-commands.jsonl
ensure_env ONPLANT_SENSOR_CACHE_FILE /tmp/onplant-latest-sensor.json
ensure_env ONPLANT_DISPLAY_STATE_FILE /tmp/onplant-display-state.json
ensure_env ONPLANT_RUNTIME_STATE_FILE /tmp/onplant-robot-runtime.json

cp "${PROJECT_DIR}/hardware/systemd/onplant-display.service" "${SYSTEMD_DIR}/onplant-display.service"
cp "${PROJECT_DIR}/hardware/systemd/onplant-drive.service" "${SYSTEMD_DIR}/onplant-drive.service"
cp "${PROJECT_DIR}/hardware/systemd/onplant-sensor.service" "${SYSTEMD_DIR}/onplant-sensor.service"
cp "${PROJECT_DIR}/hardware/systemd/onplant-voice.service" "${SYSTEMD_DIR}/onplant-voice.service"

systemctl --user daemon-reload
systemctl --user enable onplant-display.service
systemctl --user enable onplant-sensor.service

echo "Display and sensor services installed and enabled."
echo "Start display and sensor now:"
echo "  systemctl --user start onplant-display.service"
echo "  systemctl --user start onplant-sensor.service"
echo
echo "After STT/TTS is ready, enable voice assistant:"
echo "  systemctl --user enable --now onplant-voice.service"
echo
echo "After lidar_fsm_drive.py is ready, enable drive:"
echo "  systemctl --user enable --now onplant-drive.service"
