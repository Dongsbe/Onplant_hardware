# OnPlant Raspberry Pi Runtime

This repository contains the Raspberry Pi side of OnPlant. The normal runtime is
five independent user services. A failure in one service does not intentionally
stop the other services.

## Runtime flow

```text
power on / user login
  systemd --user
    onplant-sensor.service  -> sensor_sender.py
    onplant-display.service -> hardware/front_display_app.py
    onplant-voice.service   -> voice_assistant_local.py
    onplant-drive.service   -> robot_main.py
    onplant-camera.service  -> usb_camera_sender.py

voice or display command
  local_runtime.py queue
    robot_main.py
      start_light_search -> lidar_fsm_drive.py child process
      stop               -> terminate child and stop motors

sensor_sender.py
  I2C/GPIO sensors -> /tmp/onplant-latest-sensor.json
                   -> server /api/sensors

voice_assistant_local.py
  Vosk wake word and STT
    status / light search / stop -> local processing
    weather / general question  -> server LLM text response
  all replies -> local_tts.py -> Raspberry Pi speaker
```

`lidar_fsm_drive.py` owns the active driving loop. `robot_main.py` only receives
commands, starts or stops that child process, and performs a final motor stop.

## Required files

```text
Raspbot_Lib.py
local_runtime.py
local_tts.py
sensor_sender.py
robot_main.py
lidar_fsm_drive.py
voice_assistant_local.py
usb_camera_sender.py
requirements.txt
requirements-local-voice.txt

movement/
  movement_controller.py

hardware/
  front_display_app.py
  raspbot_driver.py
  onplant-robot.env.example
  install_user_services.sh
  systemd/
    onplant-camera.service
    onplant-display.service
    onplant-drive.service
    onplant-sensor.service
    onplant-voice.service
```

Files under `tools/diagnostics/` are manual hardware checks and are not required
for automatic operation.

## Install

```bash
cd /home/ai/onplant_robot
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
pip install -r requirements-local-voice.txt
sudo apt update
sudo apt install -y alsa-utils espeak-ng mpg123 unzip

mkdir -p models
cd models
wget https://alphacephei.com/vosk/models/vosk-model-small-ko-0.22.zip
unzip vosk-model-small-ko-0.22.zip
rm vosk-model-small-ko-0.22.zip

cd /home/ai/onplant_robot
chmod +x hardware/install_user_services.sh
./hardware/install_user_services.sh
nano ~/.config/onplant/onplant-robot.env
```

Set real values for at least `ONPLANT_SERVER`, `ONPLANT_ROBOT_API_KEY`, and
`ONPLANT_CAMERA_KEY`. The two keys must match the server configuration.

Restart after editing the environment:

```bash
systemctl --user daemon-reload
systemctl --user restart \
  onplant-sensor.service \
  onplant-drive.service \
  onplant-voice.service \
  onplant-display.service \
  onplant-camera.service
```

The services are installed as user services. They start when the `ai` user
session starts. To start them before an interactive login, enable lingering once:

```bash
sudo loginctl enable-linger ai
```

## Verification

```bash
systemctl --user status \
  onplant-sensor.service \
  onplant-drive.service \
  onplant-voice.service \
  onplant-display.service \
  onplant-camera.service \
  --no-pager

journalctl --user -u onplant-voice.service -n 80 --no-pager
journalctl --user -u onplant-drive.service -n 80 --no-pager
```

Expected voice startup output includes `local STT ready`. A missing Vosk package
or model is treated as a startup error; it does not silently switch wake-word
recognition to the server.

The front display is local at `http://127.0.0.1:8765`. It reads:

```text
/tmp/onplant-latest-sensor.json
/tmp/onplant-display-state.json
/tmp/onplant-robot-runtime.json
```

## Diagnostic tools

Run these from the project root while the conflicting service is stopped.

```bash
cd /home/ai/onplant_robot
source venv/bin/activate

python tools/diagnostics/sensor_check.py --bus 3
python tools/diagnostics/dht_check.py
python tools/diagnostics/lidar_capture_snapshot.py --help
python tools/diagnostics/motor_id_test.py
python tools/diagnostics/send_remote_key.py --help
```

Do not run a LiDAR or motor diagnostic while `onplant-drive.service` owns the
same serial or motor device.

## Straight-line calibration

`straight_calibration.py` runs one forward-only trial, stops at the configured
LiDAR distance, appends one row to a course CSV, and exits. Stop the normal
drive service before each trial.

```bash
cd /home/ai/onplant_robot
source venv/bin/activate
systemctl --user stop onplant-drive.service

python -u straight_calibration.py --course 285 --direction X --stop-mm 250
python -u straight_calibration.py --course 325 --direction Y --stop-mm 250

systemctl --user restart onplant-drive.service
```

The default output files are `calibration_285.csv` and
`calibration_325.csv`. Repeated runs append rows and increment the `run`
column; they do not create a new file for every run.
