#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_DIR="${HOME}/.config/onplant"
SYSTEMD_DIR="${HOME}/.config/systemd/user"
ENV_FILE="${CONFIG_DIR}/onplant-robot.env"

REQUIRED_FILES=(
  "Raspbot_Lib.py"
  "local_runtime.py"
  "local_tts.py"
  "sensor_sender.py"
  "robot_main.py"
  "lidar_fsm_drive.py"
  "voice_assistant_local.py"
  "usb_camera_sender.py"
  "movement/movement_controller.py"
  "hardware/raspbot_driver.py"
  "hardware/front_display_app.py"
)

for relative_path in "${REQUIRED_FILES[@]}"; do
  if [ ! -f "${PROJECT_DIR}/${relative_path}" ]; then
    echo "Missing runtime file: ${PROJECT_DIR}/${relative_path}" >&2
    exit 1
  fi
done

mkdir -p "${CONFIG_DIR}" "${SYSTEMD_DIR}"

if [ ! -f "${ENV_FILE}" ]; then
  cp "${PROJECT_DIR}/hardware/onplant-robot.env.example" "${ENV_FILE}"
  sed -i "s|ONPLANT_PROJECT_DIR=.*|ONPLANT_PROJECT_DIR=${PROJECT_DIR}|" "${ENV_FILE}"
  echo "Created ${ENV_FILE}"
  echo "Edit ONPLANT_SERVER if the server PC IP is different."
fi

ensure_env() {
  local key="$1"
  local value="$2"
  if ! grep -q "^${key}=" "${ENV_FILE}"; then
    printf '%s=%s\n' "${key}" "${value}" >> "${ENV_FILE}"
  fi
}

ensure_env ONPLANT_SERVER_INTERVAL 5
ensure_env ONPLANT_WAKE_CONFIDENCE 0.70
ensure_env ONPLANT_VOSK_MODEL "${PROJECT_DIR}/models/vosk-model-small-ko-0.22"
ensure_env ONPLANT_TTS_VOICE ko-KR-InJoonNeural
ensure_env ONPLANT_LOCAL_COMMAND_FILE /tmp/onplant-local-commands.jsonl
ensure_env ONPLANT_SENSOR_CACHE_FILE /tmp/onplant-latest-sensor.json
ensure_env ONPLANT_DISPLAY_STATE_FILE /tmp/onplant-display-state.json
ensure_env ONPLANT_RUNTIME_STATE_FILE /tmp/onplant-robot-runtime.json
ensure_env ONPLANT_CAMERA_DEVICE /dev/video0
ensure_env ONPLANT_CAMERA_WIDTH 960
ensure_env ONPLANT_CAMERA_HEIGHT 540
ensure_env ONPLANT_CAMERA_INTERVAL 0.2

SERVICES=(display sensor drive voice camera)
for service in "${SERVICES[@]}"; do
  cp \
    "${PROJECT_DIR}/hardware/systemd/onplant-${service}.service" \
    "${SYSTEMD_DIR}/onplant-${service}.service"
done

systemctl --user daemon-reload
systemctl --user enable "${SERVICES[@]/%/.service}"

echo "Installed and enabled: ${SERVICES[*]}"
echo "Review secrets and device settings before restart:"
echo "  nano ${ENV_FILE}"
echo "Restart all OnPlant services:"
echo "  systemctl --user restart onplant-sensor.service onplant-drive.service onplant-voice.service onplant-display.service onplant-camera.service"
echo "Check status:"
echo "  systemctl --user status onplant-sensor.service onplant-drive.service onplant-voice.service onplant-display.service onplant-camera.service --no-pager"
