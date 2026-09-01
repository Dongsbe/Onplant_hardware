# Onplant Hardware

Raspberry Pi robot-side code goes here.

Planned files:

```text
lidar_drive_only.py
lidar_fsm_drive.py
front_display_app.py
send_remote_key.py
sensor_check.py
dht_check.py
raspi_picamera2_to_pc.py
raspi_send_camera_to_pc.py
```

Robot-side responsibilities:

- LiDAR scan and FSM driving
- Raspbot expansion board motor/remote control
- Camera, front display, microphone, and speaker
- Optional direct I2C sensor checks
- Read local sensors and execute safety-critical commands without the server

## Hybrid local/server runtime

The Raspberry Pi handles these commands locally:

```text
show_status         read /tmp/onplant-latest-sensor.json
start_light_search  append to /tmp/onplant-local-commands.jsonl
stop                append to /tmp/onplant-local-commands.jsonl
```

Weather and general conversation are sent to the FastAPI/NVIDIA LLM server.
When the server is unavailable, local status, light search, stop, and the
front display continue to work. The offline Korean STT uses the official
82 MB `vosk-model-small-ko-0.22` model, which is intended for lightweight
devices including Raspberry Pi.

Install local voice dependencies on Raspberry Pi:

```bash
cd ~/onplant_robot
source venv/bin/activate
pip install -r requirements-local-voice.txt
sudo apt update
sudo apt install -y unzip espeak-ng mpg123
mkdir -p models
cd models
wget https://alphacephei.com/vosk/models/vosk-model-small-ko-0.22.zip
unzip vosk-model-small-ko-0.22.zip
rm vosk-model-small-ko-0.22.zip
```

Copy the revised services and configuration:

```bash
cd ~/onplant_robot
chmod +x hardware/install_user_services.sh
./hardware/install_user_services.sh
nano ~/.config/onplant/onplant-robot.env
systemctl --user daemon-reload
systemctl --user enable --now onplant-sensor.service
systemctl --user enable --now onplant-drive.service
systemctl --user enable --now onplant-voice.service
```

The voice service must print `local STT ready`. If it prints `Using server
STT fallback`, the Vosk package or model path is missing.

## Front display

The 5-inch display always opens the Raspberry Pi local page at
`http://127.0.0.1:8765`. Sensor values come from the local cache. The server
address is used only for optional remote display instructions and metadata.

Manual start:

```bash
cd ~/onplant_robot
python3 hardware/front_display_app.py --server http://192.168.100.6:5050 --robot-id raspbot-a
```

The page primarily reads:

```text
/tmp/onplant-latest-sensor.json
/tmp/onplant-display-state.json
/tmp/onplant-robot-runtime.json
```

Remote key behavior:

```text
3: show plant status report
4: show camera area
5: hide camera area
```

Manual test from Raspberry Pi:

```bash
python3 hardware/send_remote_key.py 3 --server http://192.168.100.6:5050 --robot-id raspbot-a
python3 hardware/send_remote_key.py 4 --server http://192.168.100.6:5050 --robot-id raspbot-a
python3 hardware/send_remote_key.py 5 --server http://192.168.100.6:5050 --robot-id raspbot-a
```

The display remains available if the FastAPI server or Wi-Fi is unavailable.
Its `ONLINE` state means the local sensor cache is current, not that the
server PC is reachable.

## Recommended runtime split

Run the robot as four separate processes:

```text
onplant-display.service  opens the 5-inch display page in kiosk mode
onplant-drive.service    runs LiDAR/FSM driving code
onplant-sensor.service   refreshes local sensors every 5 seconds and DB every 10 minutes
onplant-voice.service    handles local wake/core commands and server conversation
```

This keeps the display alive even if the drive loop restarts, and the drive
loop can call `send_remote_key.py` or import `send_remote_key.send_remote_key`
when the physical remote receives keys 3, 4, or 5.

Install on Raspberry Pi:

```bash
cd hardware
chmod +x install_user_services.sh
./install_user_services.sh
```

Edit the server address if needed:

```bash
nano ~/.config/onplant/onplant-robot.env
```

Start only the display:

```bash
systemctl --user start onplant-display.service
```

When `lidar_fsm_drive.py` is ready:

```bash
systemctl --user enable --now onplant-drive.service
```

Check logs:

```bash
journalctl --user -u onplant-display.service -f
journalctl --user -u onplant-drive.service -f
```
