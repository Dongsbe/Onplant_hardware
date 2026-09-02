from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import tempfile
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.error import URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from local_runtime import (  # noqa: E402
    append_local_command,
    format_sensor_reply,
    read_display_state,
    read_runtime_state,
    read_sensor_cache,
    write_display_state,
)


DISPLAY_BUILD = "local-first-touch-20260902-1"

DEFAULT_STATUS = {
    "online": False,
    "server_online": False,
    "screen": "face",
    "emotion": "happy",
    "message": "OnPlant",
    "sub_message": "Ready",
    "report_message": "센서 데이터를 확인하고 있습니다.",
    "recommendation": "현재 환경을 유지해 주세요.",
    "lux": 0,
    "temperature": 0,
    "humidity": 0,
    "soil_moisture": 0,
    "robot_state": "IDLE",
    "camera_visible": False,
    "updated_at": 0,
}

status = DEFAULT_STATUS.copy()
status_lock = threading.Lock()

CONTROL_SETTINGS_FILE = Path(
    os.getenv("ONPLANT_CONTROL_SETTINGS_FILE", "~/.config/onplant/display-controls.json")
).expanduser()
control_settings = {
    "speaker_volume": 60,
    "display_brightness": 80,
    "pending_sync": False,
}
control_settings_lock = threading.Lock()
remote_base_url = ""
remote_robot_id = ""
settings_syncing = False


def clamp_percent(value: object, default: int) -> int:
    try:
        return max(0, min(100, int(float(value))))
    except (TypeError, ValueError):
        return default


def save_control_settings() -> None:
    CONTROL_SETTINGS_FILE.parent.mkdir(parents=True, exist_ok=True)
    temporary = CONTROL_SETTINGS_FILE.with_suffix(CONTROL_SETTINGS_FILE.suffix + ".tmp")
    with control_settings_lock:
        payload = control_settings.copy()
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(CONTROL_SETTINGS_FILE)


def load_control_settings() -> None:
    try:
        saved = json.loads(CONTROL_SETTINGS_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        saved = {}

    with control_settings_lock:
        control_settings["speaker_volume"] = clamp_percent(saved.get("speaker_volume"), 60)
        control_settings["display_brightness"] = clamp_percent(saved.get("display_brightness"), 80)
        control_settings["pending_sync"] = bool(saved.get("pending_sync", False))


def public_control_settings() -> dict:
    with control_settings_lock:
        return {
            "speaker_volume": control_settings["speaker_volume"],
            "display_brightness": control_settings["display_brightness"],
        }


def speaker_card() -> str:
    explicit = os.getenv("ONPLANT_SPEAKER_CARD", "").strip()
    if explicit:
        return explicit
    device = os.getenv("ONPLANT_SPEAKER_DEVICE", "plughw:3,0")
    match = re.search(r"(?:plughw|hw):([^,]+)", device)
    return match.group(1) if match else "3"


def apply_speaker_volume(volume: int) -> bool:
    amixer = shutil.which("amixer")
    if not amixer:
        print("speaker volume not applied: amixer not found", file=sys.stderr)
        return False
    result = subprocess.run(
        [amixer, "-c", speaker_card(), "sset", "PCM", f"{volume}%"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        print(f"speaker volume not applied: {result.stderr.strip()}", file=sys.stderr)
        return False
    return True


def update_control_settings(payload: dict, *, mark_pending: bool) -> dict:
    with control_settings_lock:
        old_volume = int(control_settings["speaker_volume"])
        control_settings["speaker_volume"] = clamp_percent(
            payload.get("speaker_volume"), old_volume
        )
        control_settings["display_brightness"] = clamp_percent(
            payload.get("display_brightness"), int(control_settings["display_brightness"])
        )
        if mark_pending:
            control_settings["pending_sync"] = True
        new_volume = int(control_settings["speaker_volume"])
    save_control_settings()
    if new_volume != old_volume or mark_pending:
        apply_speaker_volume(new_volume)
    return public_control_settings()


def fetch_remote_config() -> dict:
    robot_path = quote(remote_robot_id, safe="")
    return fetch_json(f"{remote_base_url}/api/robots/{robot_path}/config")


def sync_settings_to_server() -> None:
    global settings_syncing
    with control_settings_lock:
        if settings_syncing or not control_settings["pending_sync"]:
            return
        settings_syncing = True
        local = control_settings.copy()
    try:
        remote = fetch_remote_config()
        remote.update(
            {
                "speaker_volume": local["speaker_volume"],
                "display_brightness": local["display_brightness"],
            }
        )
        robot_path = quote(remote_robot_id, safe="")
        body = json.dumps(remote).encode("utf-8")
        request = Request(
            f"{remote_base_url}/api/robots/{robot_path}/config",
            data=body,
            method="PATCH",
            headers={"Content-Type": "application/json; charset=utf-8"},
        )
        with urlopen(request, timeout=2.0):
            pass
        with control_settings_lock:
            control_settings["pending_sync"] = False
        save_control_settings()
    except Exception as exc:
        print(f"display settings sync pending: {exc}", file=sys.stderr)
    finally:
        with control_settings_lock:
            settings_syncing = False


def accept_remote_settings(remote: dict) -> None:
    if not remote:
        return
    with control_settings_lock:
        pending = bool(control_settings["pending_sync"])
    if pending:
        threading.Thread(target=sync_settings_to_server, daemon=True).start()
        return
    current = public_control_settings()
    incoming = {
        "speaker_volume": clamp_percent(remote.get("speaker_volume"), current["speaker_volume"]),
        "display_brightness": clamp_percent(
            remote.get("display_brightness"), current["display_brightness"]
        ),
    }
    if incoming != current:
        update_control_settings(incoming, mark_pending=False)


def play_message_async(text: str) -> None:
    threading.Thread(target=play_message, args=(text,), daemon=True).start()


def play_message(text: str) -> bool:
    text = str(text or "").strip()
    if not text:
        return False
    with tempfile.TemporaryDirectory() as temp_dir:
        output = Path(temp_dir) / "display-message.mp3"
        try:
            result = post_json(f"{remote_base_url}/api/tts", {"text": text}, timeout=20.0)
            audio_url = str(result.get("audio_url") or "")
            if audio_url:
                audio_url = audio_url if audio_url.startswith("http") else f"{remote_base_url}{audio_url}"
                request = Request(audio_url, headers={"Accept": "audio/mpeg,*/*"})
                with urlopen(request, timeout=10.0) as response:
                    output.write_bytes(response.read())
                player = shutil.which("mpg123")
                if player and output.is_file() and output.stat().st_size > 0:
                    subprocess.run([player, "-q", "-a", os.getenv("ONPLANT_SPEAKER_DEVICE", "plughw:3,0"), str(output)], check=False)
                    return True
        except Exception as exc:
            print(f"display server TTS unavailable: {exc}", file=sys.stderr)

        fallback = shutil.which("espeak-ng") or shutil.which("espeak")
        if not fallback:
            return False
        wav = Path(temp_dir) / "display-message.wav"
        result = subprocess.run([fallback, "-v", "ko", "-s", "155", "-w", str(wav), text], check=False)
        if result.returncode != 0 or not wav.is_file():
            return False
        player = shutil.which("aplay")
        if not player:
            return False
        subprocess.run([player, "-D", os.getenv("ONPLANT_SPEAKER_DEVICE", "plughw:3,0"), str(wav)], check=False)
        return True


def play_test_sound() -> bool:
    speaker_test = shutil.which("speaker-test")
    if not speaker_test:
        return False
    subprocess.Popen(
        [
            speaker_test,
            "-D",
            os.getenv("ONPLANT_SPEAKER_DEVICE", "plughw:3,0"),
            "-t",
            "sine",
            "-f",
            "880",
            "-l",
            "1",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return True


HTML = r"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>OnPlant Display</title>
  <style>
:root {
  --ink: #1f3b1f;
  --leaf: #93aa74;
  --leaf-dark: #6e8757;
  --cream: #fffdf1;
  --cream-2: #f8f8e8;
  --blush: rgba(190, 215, 147, .46);
  --card: rgba(255, 255, 255, .78);
}

* { box-sizing: border-box; }

html,
body {
  width: 100%;
  height: 100%;
  margin: 0;
  overflow: hidden;
  cursor: none;
  background: #030303;
  color: var(--ink);
  font-family: "Noto Sans CJK KR", "Noto Sans KR", "NanumGothic", "Malgun Gothic", sans-serif;
}

.tablet {
  position: fixed;
  inset: 0;
  width: 100vw;
  width: 100dvw;
  height: 100vh;
  height: 100dvh;
  padding: clamp(24px, 5vw, 58px) clamp(34px, 6vw, 72px);
  display: grid;
  place-items: center;
  background:
    radial-gradient(circle at 50% 4.8%, #272727 0 5px, #0b0b0b 6px 11px, transparent 12px),
    linear-gradient(180deg, #242424 0, #060606 12%, #050505 86%, #1c1c1c 100%);
  border-radius: clamp(34px, 7vw, 76px);
  box-shadow:
    inset 0 0 0 5px #2a2a2a,
    inset 0 0 0 11px #0a0a0a,
    inset 0 22px 50px rgba(255,255,255,.08),
    inset 0 -22px 42px rgba(0,0,0,.72);
}

.screen {
  position: relative;
  width: 100%;
  height: 100%;
  min-width: 0;
  min-height: 0;
  overflow: hidden;
  display: none;
  border-radius: clamp(18px, 3vw, 34px);
  background:
    radial-gradient(circle at 50% 48%, rgba(255,255,255,.78) 0 22%, transparent 48%),
    radial-gradient(circle at 18% 84%, rgba(207, 222, 166, .22), transparent 34%),
    linear-gradient(180deg, var(--cream) 0%, var(--cream-2) 100%);
  box-shadow:
    inset 0 0 46px rgba(201, 210, 166, .2),
    0 0 0 1px rgba(255,255,255,.18);
}

.screen.active {
  display: grid;
  place-items: center;
}

.screen::before,
.screen::after {
  content: "";
  position: absolute;
  pointer-events: none;
  opacity: .44;
  z-index: 0;
}

.screen::before {
  width: min(270px, 34vw);
  height: min(250px, 46vh);
  left: -18px;
  top: -18px;
  background:
    radial-gradient(ellipse at 24% 20%, rgba(147,170,116,.36) 0 10%, transparent 11%),
    radial-gradient(ellipse at 40% 30%, rgba(147,170,116,.3) 0 13%, transparent 14%),
    radial-gradient(ellipse at 58% 14%, rgba(147,170,116,.28) 0 9%, transparent 10%),
    linear-gradient(110deg, transparent 0 22%, rgba(110,135,87,.38) 23% 24%, transparent 25%),
    linear-gradient(42deg, transparent 0 36%, rgba(110,135,87,.26) 37% 38%, transparent 39%);
  border-radius: 0 0 70% 0;
  transform: rotate(-7deg);
}

.screen::after {
  width: min(360px, 44vw);
  height: min(230px, 38vh);
  right: -10px;
  bottom: -8px;
  background:
    radial-gradient(ellipse at 78% 58%, rgba(147,170,116,.38) 0 11%, transparent 12%),
    radial-gradient(ellipse at 66% 70%, rgba(147,170,116,.34) 0 13%, transparent 14%),
    radial-gradient(ellipse at 54% 80%, rgba(147,170,116,.28) 0 11%, transparent 12%),
    radial-gradient(circle at 72% 92%, rgba(157,185,135,.32) 0 18%, transparent 19%),
    linear-gradient(145deg, transparent 0 48%, rgba(110,135,87,.34) 49% 50%, transparent 51%);
  border-radius: 70% 0 0 0;
}

.sprig {
  position: absolute;
  pointer-events: none;
  z-index: 1;
  opacity: .44;
}

.sprig::before,
.sprig::after {
  content: "";
  position: absolute;
  border-radius: 70% 0 70% 0;
  background: rgba(141, 164, 108, .36);
}

.sprig-a {
  left: 5%;
  bottom: 7%;
  width: 150px;
  height: 180px;
}

.sprig-a::before {
  width: 54px;
  height: 96px;
  left: 8px;
  bottom: 0;
  transform: rotate(-28deg);
}

.sprig-a::after {
  width: 38px;
  height: 72px;
  left: 76px;
  bottom: 18px;
  transform: rotate(34deg);
}

.sprig-b {
  right: 5%;
  top: 4%;
  width: 150px;
  height: 115px;
}

.sprig-b::before {
  width: 58px;
  height: 86px;
  right: 0;
  top: 0;
  transform: rotate(34deg);
}

.sprig-b::after {
  width: 34px;
  height: 58px;
  right: 66px;
  top: 10px;
  transform: rotate(-36deg);
}

.face-core {
  position: relative;
  z-index: 3;
  width: min(520px, 70vw);
  height: min(230px, 42vh);
  display: grid;
  place-items: center;
  animation: idle-alive 4.2s ease-in-out infinite;
}

.eyes,
.sleep-eyes {
  position: absolute;
  top: 14%;
  display: flex;
  gap: clamp(96px, 22vw, 190px);
  align-items: center;
}

.eyes i {
  position: relative;
  width: clamp(46px, 8vw, 64px);
  --eye-h: clamp(64px, 12vw, 92px);
  height: var(--eye-h);
  border-radius: 50%;
  background:
    radial-gradient(circle at 43% 30%, transparent 0 12%, var(--ink) 13%),
    linear-gradient(160deg, #24481f, #142b14);
  transition: height 120ms ease, border-radius 120ms ease, transform 120ms ease;
  transition-delay: 0ms;
  box-shadow: inset -8px -10px 12px rgba(0,0,0,.18);
}

.gentle-eyes i::before,
.happy-eyes i::before {
  content: "";
  position: absolute;
  left: 24%;
  top: 16%;
  width: 38%;
  height: 30%;
  border-radius: 50%;
  background: #fff;
}

.gentle-eyes i::after,
.happy-eyes i::after {
  content: "";
  position: absolute;
  right: 30%;
  bottom: 34%;
  width: 13%;
  height: 11%;
  border-radius: 50%;
  background: rgba(255,255,255,.92);
}

.screen.active .eyes.blink-now i,
.screen.active .happy-eyes.blink-now i {
  height: 8px !important;
  border-radius: 999px !important;
  transform: translateY(35px) !important;
}

.screen.active .eyes.blink-now i::before,
.screen.active .eyes.blink-now i::after {
  opacity: 0 !important;
}

.bored i {
  --eye-h: clamp(18px, 4vw, 30px);
  height: var(--eye-h);
  border-radius: 999px;
}

.sharp i {
  --eye-h: clamp(34px, 7vw, 50px);
  height: var(--eye-h);
  border-radius: 50% 50% 38% 38%;
  transform: skew(-10deg) rotate(-2deg);
}

.happy-eyes i {
  width: clamp(52px, 9vw, 72px);
  --eye-h: clamp(72px, 13vw, 100px);
}

.sleep-eyes i {
  width: clamp(62px, 11vw, 88px);
  height: 34px;
  border-bottom: 8px solid var(--ink);
  border-radius: 0 0 80px 80px;
}

.brows {
  position: absolute;
  top: 1%;
  display: flex;
  gap: clamp(92px, 22vw, 182px);
}

.brows i {
  width: 62px;
  height: 8px;
  border-radius: 8px;
  background: var(--ink);
}

.brows i:first-child { transform: rotate(17deg); }
.brows i:last-child { transform: rotate(-17deg); }

.cheeks {
  position: absolute;
  top: 62%;
  width: min(430px, 66vw);
  height: 58px;
}

.cheeks::before,
.cheeks::after {
  content: "";
  position: absolute;
  width: clamp(70px, 13vw, 112px);
  height: clamp(36px, 7vw, 54px);
  border-radius: 50%;
  background: var(--blush);
  filter: blur(.2px);
}

.cheeks::before { left: 0; }
.cheeks::after { right: 0; }

.mouth {
  position: absolute;
  top: 67%;
}

.smile,
.sleep-mouth {
  width: clamp(68px, 12vw, 96px);
  height: 42px;
  border-bottom: 8px solid var(--ink);
  border-radius: 0 0 90px 90px;
  animation: smile-soft 3.4s ease-in-out infinite;
}

.pout {
  width: 52px;
  height: 12px;
  border-radius: 999px;
  background: var(--ink);
}

.angry-mouth {
  width: 58px;
  height: 9px;
  border-radius: 999px;
  background: var(--ink);
  transform: rotate(-4deg);
}

.big-smile {
  width: 106px;
  height: 58px;
  border-radius: 0 0 120px 120px;
  background: var(--ink);
}

.sound {
  position: absolute;
  top: 37%;
  display: flex;
  gap: 10px;
  align-items: center;
  z-index: 4;
}

.sound.left { left: 16%; }
.sound.right { right: 16%; }

.sound i {
  width: 10px;
  height: 42px;
  border-radius: 20px;
  background: #7fa36d;
  animation: sound 820ms ease-in-out infinite;
}

.sound i:nth-child(2) { animation-delay: 90ms; }
.sound i:nth-child(3) { animation-delay: 180ms; }
.sound i:nth-child(4) { animation-delay: 270ms; }

.scan {
  position: absolute;
  width: min(310px, 52vw);
  aspect-ratio: 1;
  border-radius: 50%;
  z-index: 2;
}

.scan-a {
  border: 2px solid rgba(114, 151, 94, .3);
  box-shadow: 0 0 0 34px rgba(114,151,94,.07), 0 0 0 70px rgba(114,151,94,.04);
  animation: spin 3.4s linear infinite;
}

.scan-b {
  width: min(210px, 42vw);
  border-top: 4px solid rgba(114,151,94,.52);
  border-right: 4px solid transparent;
  animation: spin 1.6s linear infinite reverse;
}

.bubble {
  position: absolute;
  left: 50%;
  bottom: clamp(22px, 6vh, 46px);
  transform: translateX(-50%);
  min-width: min(390px, 82vw);
  border-radius: 26px;
  background: rgba(255, 255, 255, .84);
  border: 1px solid rgba(117, 156, 105, .28);
  box-shadow: 0 18px 44px rgba(54, 74, 47, .13);
  padding: 16px 26px;
  text-align: center;
  z-index: 5;
}

.bubble strong {
  display: block;
  font-size: clamp(20px, 4vw, 28px);
  line-height: 1.1;
}

.bubble span {
  display: block;
  margin-top: 6px;
  color: #60715d;
  font-size: clamp(14px, 2.6vw, 18px);
}

.bubble em {
  display: flex;
  justify-content: center;
  gap: 8px;
  margin-top: 10px;
  font-style: normal;
}

.bubble em i {
  width: 9px;
  height: 9px;
  border-radius: 50%;
  background: var(--leaf-dark);
  animation: dot 1s ease-in-out infinite;
}

.bubble em i:nth-child(2) { animation-delay: 150ms; }
.bubble em i:nth-child(3) { animation-delay: 300ms; }

.report {
  padding: clamp(8px, 2vh, 14px) clamp(14px, 3vw, 26px);
}

.report.active {
  display: grid;
  place-items: stretch;
}

.report-shell {
  width: min(720px, 100%);
  height: 100%;
  margin: 0 auto;
  display: grid;
  grid-template-rows: auto auto minmax(0, 1fr);
  gap: clamp(6px, 1.4vh, 10px);
  align-content: center;
  animation: report-in .5s ease both;
}

.report h1 {
  margin: 0;
  text-align: center;
  color: #60715d;
  font-size: clamp(15px, 2.6vw, 20px);
  line-height: 1.15;
}

.report-card {
  display: grid;
  gap: 3px;
  width: 100%;
  margin: 0;
  padding: clamp(6px, 1.5vh, 10px) 14px;
  text-align: center;
}

.report-card strong {
  color: #477640;
  font-size: clamp(23px, 4.5vw, 32px);
  line-height: 1.1;
}

.report-card span {
  font-size: clamp(14px, 2.7vw, 19px);
  line-height: 1.2;
}

.report-card b {
  color: #52684c;
  font-size: clamp(13px, 2.4vw, 17px);
  line-height: 1.2;
}

.metric-grid {
  width: 100%;
  min-height: 0;
  margin: 0;
  display: grid;
  grid-template-columns: 1fr 1fr;
  grid-template-rows: 1fr 1fr;
  gap: clamp(6px, 1.4vw, 10px);
}

.metric-grid div {
  min-height: 0;
  border-radius: 14px;
  border: 1px solid rgba(117, 156, 105, .3);
  background: rgba(255, 255, 255, .72);
  padding: clamp(7px, 1.7vw, 12px) clamp(12px, 2.6vw, 20px);
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
  font-weight: 800;
}

.metric-grid span {
  color: #5f6f5d;
  font-size: clamp(14px, 2.8vw, 19px);
}

.metric-grid strong {
  margin: 0;
  color: #477640;
  font-size: clamp(21px, 4.5vw, 30px);
  white-space: nowrap;
}

.notice {
  padding: clamp(18px, 4vh, 42px) clamp(18px, 5vw, 56px);
}

.notice.active {
  display: grid;
  place-items: center;
}

.notice-shell {
  width: min(760px, 100%);
  display: grid;
  gap: clamp(12px, 2.5vh, 22px);
  text-align: center;
  animation: report-in .5s ease both;
}

.notice-icon {
  width: clamp(76px, 15vw, 116px);
  height: clamp(76px, 15vw, 116px);
  margin: 0 auto;
  border-radius: 999px;
  border: 2px solid rgba(75, 118, 64, .24);
  background: rgba(255, 255, 255, .72);
  display: grid;
  place-items: center;
  color: #477640;
  font-size: clamp(38px, 8vw, 60px);
  font-weight: 900;
}

.notice h1 {
  margin: 0;
  color: #17351c;
  font-size: clamp(34px, 7vw, 58px);
  line-height: 1.12;
}

.notice p {
  margin: 0;
  color: #52684c;
  font-size: clamp(18px, 3.4vw, 28px);
  line-height: 1.35;
  white-space: pre-line;
}

@keyframes idle-alive {
  0%, 100% { transform: translateY(0) scale(1); }
  48% { transform: translateY(-4px) scale(1.01); }
}

@keyframes smile-soft {
  0%, 100% { transform: translateY(0) scaleX(1); }
  50% { transform: translateY(2px) scaleX(1.06); }
}

@keyframes sound {
  0%, 100% { transform: scaleY(.45); opacity: .52; }
  50% { transform: scaleY(1.35); opacity: 1; }
}

@keyframes spin { to { transform: rotate(360deg); } }

@keyframes dot {
  0%, 100% { opacity: .35; transform: translateY(0); }
  50% { opacity: 1; transform: translateY(-5px); }
}

@keyframes report-in {
  from { opacity: 0; transform: translateY(12px); }
  to { opacity: 1; transform: translateY(0); }
}

    * { cursor: none !important; }
    .tablet { border-radius: 0; padding: clamp(12px, 3vw, 32px); }
    .status-pill {
      position: fixed;
      right: 18px;
      top: 14px;
      z-index: 20;
      padding: 7px 12px;
      border-radius: 999px;
      background: rgba(255,255,255,.72);
      border: 1px solid rgba(117,156,105,.26);
      font: 700 14px "Noto Sans CJK KR", "Noto Sans KR", "NanumGothic", sans-serif;
      color: #52684c;
    }
    .status-pill.offline { color: #80584a; }

    .menu-layer,
    .settings-layer {
      position: fixed;
      inset: 0;
      z-index: 80;
      display: none;
      align-items: center;
      justify-content: center;
      padding: 24px;
      background: rgba(8, 14, 8, .54);
    }

    .menu-layer.open,
    .settings-layer.open { display: flex; }

    .touch-menu {
      width: min(720px, 92vw);
      padding: 26px;
      border-radius: 8px;
      background: #fffef7;
      color: #203820;
      box-shadow: 0 22px 70px rgba(0,0,0,.32);
    }

    .touch-menu-header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 20px;
    }

    .touch-menu-header h2 { margin: 0; font-size: 30px; }

    .touch-menu-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 14px;
    }

    .touch-menu-button {
      min-height: 116px;
      padding: 16px 10px;
      border: 1px solid #b8c9ae;
      border-radius: 8px;
      background: #f8fbf4;
      color: #294529;
      font-size: 22px;
      font-weight: 800;
      touch-action: manipulation;
    }

    .touch-menu-button.primary {
      border-color: #668a5b;
      background: #e9f4e4;
      color: #176c31;
    }

    .touch-menu-button:active { transform: scale(.98); }

    .menu-result {
      min-height: 24px;
      margin: 16px 0 0;
      color: #587054;
      text-align: center;
      font-size: 17px;
      font-weight: 700;
    }

    .settings-panel {
      width: min(620px, 92vw);
      padding: 28px;
      border-radius: 8px;
      background: #fffef7;
      color: #203820;
      box-shadow: 0 22px 70px rgba(0,0,0,.32);
    }

    .settings-header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 22px;
    }

    .settings-header h2 { margin: 0; font-size: 28px; }

    .close-menu,
    .close-settings,
    .test-sound {
      min-height: 48px;
      border: 1px solid #b8c9ae;
      border-radius: 6px;
      background: #fff;
      color: #294529;
      font-size: 18px;
      font-weight: 700;
      touch-action: manipulation;
    }

    .close-menu,
    .close-settings { width: 48px; font-size: 28px; }
    .test-sound { width: 100%; margin-top: 22px; }

    .setting-row { margin-top: 22px; }
    .setting-label {
      display: flex;
      justify-content: space-between;
      gap: 16px;
      margin-bottom: 12px;
      font-size: 20px;
      font-weight: 700;
    }

    .setting-row input[type="range"] {
      width: 100%;
      height: 34px;
      accent-color: #6e8757;
    }

    .touch-stepper {
      display: grid;
      grid-template-columns: 64px minmax(0, 1fr) 64px;
      align-items: center;
      gap: 14px;
    }

    .step-button {
      width: 64px;
      height: 58px;
      border: 1px solid #9db494;
      border-radius: 8px;
      background: #edf6e9;
      color: #1d6732;
      font-size: 34px;
      font-weight: 800;
      line-height: 1;
      touch-action: manipulation;
    }

    .step-button:active { transform: scale(.96); background: #dcebd6; }

    .settings-result {
      min-height: 24px;
      margin: 14px 0 0;
      color: #587054;
      text-align: center;
      font-size: 16px;
      font-weight: 700;
    }

    .brightness-shade {
      position: fixed;
      inset: 0;
      z-index: 100;
      pointer-events: none;
      background: #000;
      opacity: 0;
    }
  </style>
</head>
<body>
  <div class="status-pill offline" id="netState">OFFLINE</div>
  <main class="tablet" aria-label="OnPlant front display">
    <section class="screen face idle active" data-screen="idle">
      <div class="sprig sprig-a"></div><div class="sprig sprig-b"></div>
      <div class="face-core gentle-core"><div class="eyes glossy gentle-eyes"><i></i><i></i></div><div class="cheeks"></div><div class="mouth smile"></div></div>
    </section>
    <section class="screen face sulk" data-screen="sulk">
      <div class="sprig sprig-a"></div><div class="sprig sprig-b"></div>
      <div class="face-core sulk-core"><div class="eyes bored"><i></i><i></i></div><div class="cheeks"></div><div class="mouth pout"></div></div>
    </section>
    <section class="screen face angry" data-screen="angry">
      <div class="sprig sprig-a"></div><div class="sprig sprig-b"></div>
      <div class="face-core angry-core"><div class="brows"><i></i><i></i></div><div class="eyes sharp"><i></i><i></i></div><div class="cheeks"></div><div class="mouth angry-mouth"></div></div>
    </section>
    <section class="screen face sleep" data-screen="sleep">
      <div class="sprig sprig-a"></div><div class="sprig sprig-b"></div>
      <div class="face-core sleep-core"><div class="sleep-eyes"><i></i><i></i></div><div class="cheeks"></div><div class="mouth sleep-mouth"></div><div class="sleep-mark">z</div></div>
    </section>
    <section class="screen face happy" data-screen="happy">
      <div class="sprig sprig-a"></div><div class="sprig sprig-b"></div>
      <div class="face-core happy-core"><div class="eyes glossy happy-eyes"><i></i><i></i></div><div class="cheeks"></div><div class="mouth big-smile"></div></div>
    </section>
    <section class="screen face listening" data-screen="listening">
      <div class="sound left"><i></i><i></i><i></i><i></i></div><div class="sound right"><i></i><i></i><i></i><i></i></div>
      <div class="face-core gentle-core"><div class="eyes glossy gentle-eyes"><i></i><i></i></div><div class="cheeks"></div><div class="mouth smile"></div></div>
      <div class="bubble"><strong>Listening</strong><span>Waiting for voice command</span></div>
    </section>
    <section class="screen face analyzing" data-screen="analyzing">
      <div class="scan scan-a"></div><div class="scan scan-b"></div>
      <div class="face-core gentle-core"><div class="eyes glossy gentle-eyes"><i></i><i></i></div><div class="cheeks"></div><div class="mouth smile"></div></div>
      <div class="bubble"><strong>Checking plant status</strong><span>Reading sensor data</span><em><i></i><i></i><i></i></em></div>
    </section>
    <section class="screen report" data-screen="report">
      <div class="report-shell">
        <h1>상태 리포트</h1>
        <div class="report-card"><strong id="reportTitle">현재 상태</strong><span id="reportMessage">센서 데이터를 확인하고 있습니다.</span><b id="reportRecommend">현재 환경을 유지해 주세요.</b></div>
        <div class="metric-grid"><div><span>온도</span><strong id="dTemp">--C</strong></div><div><span>습도</span><strong id="dHum">--%</strong></div><div><span>조도</span><strong id="dLux">-- lux</strong></div><div><span>토양수분</span><strong id="dSoil">--%</strong></div></div>
      </div>
    </section>
    <section class="screen notice" data-screen="notice">
      <div class="notice-shell">
        <div class="notice-icon" id="noticeIcon">...</div>
        <h1 id="noticeTitle">응답하고 있습니다</h1>
        <p id="noticeMessage">서버 LLM에 연결하고 있습니다.</p>
      </div>
    </section>
  </main>
  <div class="menu-layer" id="menuLayer" aria-hidden="true">
    <section class="touch-menu" role="dialog" aria-modal="true" aria-labelledby="menuTitle">
      <div class="touch-menu-header">
        <h2 id="menuTitle">메뉴</h2>
        <button class="close-menu" id="closeMenu" type="button" aria-label="메뉴 닫기">&times;</button>
      </div>
      <div class="touch-menu-grid">
        <button class="touch-menu-button primary" id="startLightSearch" type="button">조도 탐색</button>
        <button class="touch-menu-button" id="showStatus" type="button">상태 확인</button>
        <button class="touch-menu-button" id="openSettings" type="button">설정</button>
      </div>
      <p class="menu-result" id="menuResult">원하는 기능을 선택하세요.</p>
    </section>
  </div>
  <div class="settings-layer" id="settingsLayer" aria-hidden="true">
    <section class="settings-panel" role="dialog" aria-modal="true" aria-labelledby="settingsTitle">
      <div class="settings-header">
        <h2 id="settingsTitle">디스플레이 설정</h2>
        <button class="close-settings" id="closeSettings" type="button" aria-label="설정 닫기">&times;</button>
      </div>
      <div class="setting-row">
        <label class="setting-label" for="speakerVolume"><span>스피커 음량</span><strong id="speakerVolumeValue">60%</strong></label>
        <div class="touch-stepper">
          <button class="step-button" id="speakerDown" type="button" aria-label="스피커 음량 줄이기">&minus;</button>
          <input id="speakerVolume" type="range" min="0" max="100" step="5" value="60">
          <button class="step-button" id="speakerUp" type="button" aria-label="스피커 음량 높이기">+</button>
        </div>
      </div>
      <div class="setting-row">
        <label class="setting-label" for="displayBrightness"><span>화면 밝기</span><strong id="displayBrightnessValue">80%</strong></label>
        <div class="touch-stepper">
          <button class="step-button" id="brightnessDown" type="button" aria-label="화면 밝기 낮추기">&minus;</button>
          <input id="displayBrightness" type="range" min="0" max="100" step="5" value="80">
          <button class="step-button" id="brightnessUp" type="button" aria-label="화면 밝기 높이기">+</button>
        </div>
      </div>
      <button class="test-sound" id="testSound" type="button">소리 테스트</button>
      <p class="settings-result" id="settingsResult">변경값은 즉시 적용됩니다.</p>
    </section>
  </div>
  <div class="brightness-shade" id="brightnessShade"></div>
  <script>
    const screens = new Set(["idle", "sulk", "angry", "sleep", "happy", "listening", "analyzing", "report", "notice"]);
    let currentScreen = "idle";
    let lastStatus = null;
    let forcedScreenUntil = 0;
    function show(screen) { const next = screens.has(screen) ? screen : "idle"; currentScreen = next; document.querySelectorAll(".screen").forEach((node) => node.classList.toggle("active", node.dataset.screen === next)); }
    function setMetric(id, value, suffix, digits = 0) { const node = document.getElementById(id); if (!node) return; const n = Number(value); node.textContent = Number.isFinite(n) ? `${n.toFixed(digits)}${suffix}` : `--${suffix}`; }
    function mapScreen(data) { if (!data.online) return "idle"; if (data.screen === "notice") return "notice"; if (data.screen === "report") return "report"; if (data.emotion === "warn" || data.emotion === "alert") return "sulk"; if (data.emotion === "angry") return "angry"; if (data.emotion === "sleep") return "sleep"; if (data.emotion === "listening") return "listening"; if (data.emotion === "analyzing") return "analyzing"; if (data.emotion === "happy") return "happy"; return "idle"; }
    function updateNotice(data) {
      const message = data.message || "응답하고 있습니다";
      const offline = message.includes("서버 연결") || message.includes("연결할 수 없습니다");
      const pending = message.includes("연결하고 있습니다") || message.includes("준비하고 있습니다") || message.includes("처리하고 있습니다");
      document.getElementById("noticeIcon").textContent = offline ? "!" : "...";
      document.getElementById("noticeTitle").textContent = offline ? "서버 연결이 필요합니다" : pending ? "응답 준비 중" : "동스비 응답";
      document.getElementById("noticeMessage").textContent = data.report_message || message;
    }
    async function refresh() { try { const res = await fetch("/api/status?ts=" + Date.now(), { cache: "no-store" }); const data = await res.json(); lastStatus = data; if (Date.now() >= forcedScreenUntil && !menuOpen && !settingsOpen) show(mapScreen(data)); const net = document.getElementById("netState"); const serverOnline = Boolean(data.server_online); net.textContent = serverOnline ? "ONLINE" : "OFFLINE"; net.className = "status-pill" + (serverOnline ? "" : " offline"); setMetric("dTemp", data.temperature, "C", 1); setMetric("dHum", data.humidity, "%"); setMetric("dLux", data.lux, " lux"); setMetric("dSoil", data.soil_moisture, "%"); document.getElementById("reportTitle").textContent = `현재 상태: ${data.sub_message || "확인 중"}`; document.getElementById("reportMessage").textContent = data.report_message || data.message || "센서 데이터를 확인하고 있습니다."; document.getElementById("reportRecommend").textContent = data.recommendation || "현재 환경을 유지해 주세요."; updateNotice(data); } catch { if (!menuOpen && !settingsOpen) show("idle"); } }
    function blinkActiveFace() { if (currentScreen === "sleep" || currentScreen === "report" || currentScreen === "notice") return; const eyes = document.querySelector(".screen.active .eyes"); if (!eyes) return; eyes.classList.remove("blink-now"); void eyes.offsetWidth; eyes.classList.add("blink-now"); window.setTimeout(() => eyes.classList.remove("blink-now"), 220); }
    const menuLayer = document.getElementById("menuLayer");
    const menuResult = document.getElementById("menuResult");
    const settingsLayer = document.getElementById("settingsLayer");
    const speakerVolume = document.getElementById("speakerVolume");
    const displayBrightness = document.getElementById("displayBrightness");
    const settingsResult = document.getElementById("settingsResult");
    let menuOpen = false;
    let settingsOpen = false;
    let menuTimer = 0;
    let settingsTimer = 0;

    function closeMenu() {
      menuOpen = false;
      window.clearTimeout(menuTimer);
      menuLayer.classList.remove("open");
      menuLayer.setAttribute("aria-hidden", "true");
    }

    function openMenu() {
      if (settingsOpen) return;
      menuOpen = true;
      menuResult.textContent = "원하는 기능을 선택하세요.";
      menuLayer.classList.add("open");
      menuLayer.setAttribute("aria-hidden", "false");
      window.clearTimeout(menuTimer);
      menuTimer = window.setTimeout(closeMenu, 12000);
    }

    function openSettingsPanel() {
      closeMenu();
      settingsOpen = true;
      settingsLayer.classList.add("open");
      settingsLayer.setAttribute("aria-hidden", "false");
      resetSettingsTimer();
      loadSettings();
    }

    function closeSettingsPanel() {
      settingsOpen = false;
      window.clearTimeout(settingsTimer);
      settingsLayer.classList.remove("open");
      settingsLayer.setAttribute("aria-hidden", "true");
      if (lastStatus) show(mapScreen(lastStatus));
    }

    function resetSettingsTimer() {
      window.clearTimeout(settingsTimer);
      settingsTimer = window.setTimeout(closeSettingsPanel, 20000);
    }

    function applyBrightness(value) {
      const raw = Number(value);
      const brightness = Math.max(0, Math.min(100, Number.isFinite(raw) ? raw : 80));
      const minimumVisibleBrightness = 15;
      const effectiveBrightness = minimumVisibleBrightness
        + (brightness * (100 - minimumVisibleBrightness) / 100);
      document.getElementById("brightnessShade").style.opacity = String(
        (100 - effectiveBrightness) / 100
      );
      document.getElementById("displayBrightnessValue").textContent = `${brightness}%`;
    }

    function showSettings(settings) {
      speakerVolume.value = String(settings.speaker_volume ?? 60);
      displayBrightness.value = String(settings.display_brightness ?? 80);
      document.getElementById("speakerVolumeValue").textContent = `${speakerVolume.value}%`;
      applyBrightness(displayBrightness.value);
    }

    async function loadSettings() {
      try {
        const response = await fetch("/api/settings?ts=" + Date.now(), { cache: "no-store" });
        if (!response.ok) return;
        showSettings(await response.json());
      } catch {}
    }

    async function saveSettings() {
      settingsResult.textContent = "적용 중...";
      try {
        const response = await fetch("/api/settings", {
          method: "POST",
          headers: { "Content-Type": "application/json; charset=utf-8" },
          body: JSON.stringify({
            speaker_volume: Number(speakerVolume.value),
            display_brightness: Number(displayBrightness.value),
          }),
        });
        if (!response.ok) throw new Error("save failed");
        showSettings(await response.json());
        settingsResult.textContent = "설정을 적용했습니다.";
      } catch {
        settingsResult.textContent = "설정을 적용하지 못했습니다.";
      }
    }

    document.querySelector(".tablet").addEventListener("click", () => {
      if (!menuOpen && !settingsOpen) openMenu();
    });
    document.getElementById("closeMenu").addEventListener("click", closeMenu);
    document.getElementById("openSettings").addEventListener("click", openSettingsPanel);
    document.getElementById("showStatus").addEventListener("click", async () => {
      closeMenu();
      forcedScreenUntil = Date.now() + 12000;
      if (lastStatus) {
        setMetric("dTemp", lastStatus.temperature, "C", 1);
        setMetric("dHum", lastStatus.humidity, "%");
        setMetric("dLux", lastStatus.lux, " lux");
        setMetric("dSoil", lastStatus.soil_moisture, "%");
        document.getElementById("reportTitle").textContent = `현재 상태: ${lastStatus.sub_message || "확인 중"}`;
        document.getElementById("reportMessage").textContent = lastStatus.report_message || lastStatus.message || "센서 데이터를 확인하고 있습니다.";
        document.getElementById("reportRecommend").textContent = lastStatus.recommendation || "현재 환경을 유지해 주세요.";
      }
      show("report");
      try { await fetch("/api/actions/show-status", { method: "POST" }); } catch {}
    });
    document.getElementById("startLightSearch").addEventListener("click", async () => {
      window.clearTimeout(menuTimer);
      menuResult.textContent = "조도 탐색을 요청하고 있습니다...";
      try {
        const response = await fetch("/api/actions/start-light-search", { method: "POST" });
        const result = await response.json();
        if (!response.ok || !result.ok) throw new Error(result.error || "request failed");
        menuResult.textContent = "조도 탐색을 시작합니다.";
        window.setTimeout(closeMenu, 900);
      } catch {
        menuResult.textContent = "로봇 주행 서비스를 확인해 주세요.";
        menuTimer = window.setTimeout(closeMenu, 5000);
      }
    });
    document.getElementById("closeSettings").addEventListener("click", closeSettingsPanel);
    settingsLayer.addEventListener("pointerdown", resetSettingsTimer);
    speakerVolume.addEventListener("input", () => {
      resetSettingsTimer();
      document.getElementById("speakerVolumeValue").textContent = `${speakerVolume.value}%`;
    });
    speakerVolume.addEventListener("change", saveSettings);
    displayBrightness.addEventListener("input", () => {
      resetSettingsTimer();
      applyBrightness(displayBrightness.value);
    });
    displayBrightness.addEventListener("change", saveSettings);
    function stepSetting(input, delta, onChange) {
      const minimum = Number(input.min);
      const maximum = Number(input.max);
      const step = Math.abs(delta);
      const current = Number(input.value);
      const next = delta > 0
        ? Math.ceil((current + step * 0.000001) / step) * step
        : Math.floor((current - step * 0.000001) / step) * step;
      input.value = String(Math.max(minimum, Math.min(maximum, next)));
      onChange(input.value);
      saveSettings();
    }
    document.getElementById("speakerDown").addEventListener("click", () => stepSetting(speakerVolume, -5, (value) => document.getElementById("speakerVolumeValue").textContent = `${value}%`));
    document.getElementById("speakerUp").addEventListener("click", () => stepSetting(speakerVolume, 5, (value) => document.getElementById("speakerVolumeValue").textContent = `${value}%`));
    document.getElementById("brightnessDown").addEventListener("click", () => stepSetting(displayBrightness, -5, applyBrightness));
    document.getElementById("brightnessUp").addEventListener("click", () => stepSetting(displayBrightness, 5, applyBrightness));
    document.getElementById("testSound").addEventListener("click", async () => {
      settingsResult.textContent = "테스트 소리를 재생합니다.";
      try {
        const response = await fetch("/api/settings/test-sound", { method: "POST" });
        if (!response.ok) throw new Error("test failed");
      } catch {
        settingsResult.textContent = "테스트 소리를 재생하지 못했습니다.";
      }
    });

    refresh(); loadSettings();
    setInterval(refresh, 1000);
    setInterval(() => { if (!settingsOpen) loadSettings(); }, 3000);
    setInterval(blinkActiveFace, 1800);
    window.setTimeout(blinkActiveFace, 600);
  </script>
</body>
</html>"""


def now() -> float:
    return time.time()


def fetch_json(url: str, timeout: float = 2.0) -> dict:
    request = Request(url, headers={"Accept": "application/json"})
    with urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def post_json(url: str, payload: dict, timeout: float = 5.0) -> dict:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    request = Request(
        url,
        data=body,
        method="POST",
        headers={
            "Accept": "application/json",
            "Content-Type": "application/json; charset=utf-8",
        },
    )
    with urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def update_status(payload: dict) -> None:
    with status_lock:
        status.update(payload)
        status["updated_at"] = now()


def fallback_status() -> None:
    update_status(local_status())


def clamp_number(value: object, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def local_status() -> dict:
    sensor = read_sensor_cache(max_age=30.0)
    local_display = read_display_state()
    runtime = read_runtime_state()
    sensor_online = not sensor.get("stale", True)
    if not sensor_online:
        return {
            **DEFAULT_STATUS,
            "online": False,
            "server_online": False,
            "emotion": "offline",
            "message": "OnPlant",
            "sub_message": "Sensor standby",
            "build": DISPLAY_BUILD,
        }

    lux = clamp_number(sensor.get("lux"), 0)
    if lux < 300:
        emotion = "warn"
        level = "조도 부족"
        report_message = "조도가 조금 부족해요."
        recommendation = "밝은 곳으로 이동하면 좋아요."
    elif lux >= 1100:
        emotion = "warn"
        level = "조도 과다"
        report_message = "조도가 너무 강해요."
        recommendation = "직사광선을 피해 조금 어두운 곳으로 이동해 주세요."
    else:
        emotion = "happy"
        level = "양호"
        report_message = "현재 환경이 안정적이에요."
        recommendation = "지금 상태를 유지해 주세요."
    report_until = clamp_number(local_display.get("report_until"), 0)
    local_screen = str(local_display.get("screen") or "")
    show_report = local_screen == "report" and report_until > now()
    show_notice = local_screen == "notice" and report_until > now()
    display_message = str(local_display.get("message") or "").strip()
    message = display_message[:80] if (show_report or show_notice) and display_message else "OnPlant"
    if (show_report or show_notice) and display_message:
        report_message = display_message[:100]
        recommendation = ""

    return {
        "online": True,
        "server_online": False,
        "screen": "notice" if show_notice else "report" if show_report else "face",
        "emotion": emotion,
        "message": message[:80],
        "sub_message": level,
        "report_message": report_message,
        "recommendation": recommendation,
        "lux": lux,
        "temperature": clamp_number(sensor.get("temperature"), 0),
        "humidity": clamp_number(sensor.get("humidity"), 0),
        "soil_moisture": clamp_number(sensor.get("soil_moisture"), 0),
        "robot_state": str(runtime.get("state") or "IDLE"),
        "robot_action": str(runtime.get("action") or "STOP"),
        "camera_visible": False,
        "build": DISPLAY_BUILD,
    }


def merge_local_status(remote: dict) -> dict:
    local = local_status()
    if not local.get("online"):
        return {**remote, "server_online": True, "build": DISPLAY_BUILD}
    merged = {**remote, **local, "server_online": True, "build": DISPLAY_BUILD}
    if local.get("screen") not in {"report", "notice"} and remote.get("screen") == "report":
        merged["screen"] = "report"
        merged["message"] = remote.get("message") or merged["message"]
    return merged


def status_from_summary(summary: dict, display: dict) -> dict:
    latest = summary.get("latest") or {}
    robot = summary.get("robot") or {}
    status_info = summary.get("status") or {}

    screen = display.get("screen") or "idle"
    if screen == "notice":
        screen = "notice"
    elif screen in {"report", "dashboard", "status"}:
        screen = "report"
    else:
        screen = "face"

    tone = str(status_info.get("tone") or "").lower()
    level = str(status_info.get("level") or "").lower()
    emotion = "happy"
    if tone in {"warn", "danger", "alert"} or "warn" in level or "danger" in level:
        emotion = "warn"

    robot_state = "IDLE"
    message = display.get("message") or ""
    if not message:
        message = robot.get("plant_name") or "OnPlant"

    return {
        "online": True,
        "screen": screen,
        "emotion": emotion,
        "message": str(message)[:80],
        "sub_message": str(status_info.get("level") or robot_state)[:80],
        "report_message": str(status_info.get("message") or "현재 센서 상태를 확인했습니다.")[:100],
        "recommendation": str(status_info.get("recommendation") or "현재 환경을 유지해 주세요.")[:120],
        "lux": clamp_number(latest.get("lux"), 0),
        "temperature": clamp_number(latest.get("temperature"), 0),
        "humidity": clamp_number(latest.get("humidity"), 0),
        "soil_moisture": clamp_number(latest.get("soil_moisture"), 0),
        "robot_state": robot_state,
        "camera_visible": bool(display.get("camera_visible", False)),
    }


def poll_server(base_url: str, robot_id: str, interval: float) -> None:
    base_url = base_url.rstrip("/")
    robot_path = quote(robot_id, safe="")
    failed_count = 0
    was_online = False
    last_error_print = 0.0

    while True:
        try:
            summary = fetch_json(f"{base_url}/api/robots/{robot_path}/summary")
            accept_remote_settings(summary.get("config") or {})
            try:
                display = fetch_json(f"{base_url}/api/robots/{robot_path}/display")
            except Exception as exc:
                display = {}
                now_ts = now()
                if now_ts - last_error_print > 10:
                    print(f"display endpoint unavailable, using summary only: {exc}", file=sys.stderr)
                    last_error_print = now_ts

            update_status(merge_local_status(status_from_summary(summary, display)))
            failed_count = 0
            if not was_online:
                print(f"display server online: {base_url} robot_id={robot_id}")
            was_online = True
        except Exception as exc:
            failed_count += 1
            now_ts = now()
            if now_ts - last_error_print > 10:
                print(f"display server offline/retry {failed_count}: {exc}", file=sys.stderr)
                last_error_print = now_ts
            if failed_count >= 2:
                update_status(local_status())
                was_online = False
        time.sleep(interval)


class DisplayHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args: object) -> None:
        return

    def send_json(self, payload: dict, code: int = 200) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if self.path.startswith("/api/status"):
            with status_lock:
                payload = status.copy()
            self.send_json(payload)
            return

        if self.path.startswith("/api/diagnostics"):
            self.send_json(
                {
                    "build": DISPLAY_BUILD,
                    "source_file": str(Path(__file__).resolve()),
                    "pid": os.getpid(),
                    "project_root": str(PROJECT_ROOT),
                    "local_command_file": str(
                        os.getenv("ONPLANT_LOCAL_COMMAND_FILE", "/tmp/onplant-local-commands.jsonl")
                    ),
                }
            )
            return

        if self.path.startswith("/api/settings"):
            self.send_json(public_control_settings())
            return

        body = HTML.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def read_json_body(self) -> dict:
        try:
            length = int(self.headers.get("Content-Length", "0"))
            value = json.loads(self.rfile.read(length).decode("utf-8"))
            return value if isinstance(value, dict) else {}
        except (ValueError, UnicodeDecodeError):
            return {}

    def do_POST(self) -> None:
        if self.path == "/api/actions/show-status":
            try:
                reply = format_sensor_reply(read_sensor_cache())
                write_display_state("report", reply, duration=12.0)
                play_message_async(reply)
                self.send_json({"ok": True, "reply": reply})
            except Exception as exc:
                print(f"display status action failed: {exc}", file=sys.stderr)
                self.send_json({"ok": False, "error": "status action failed"}, 503)
            return

        if self.path == "/api/actions/start-light-search":
            runtime = read_runtime_state()
            if runtime.get("drive_running"):
                message = "이미 조도 탐색 중입니다."
                write_display_state("report", message, duration=5.0)
                play_message_async(message)
                self.send_json({"ok": False, "error": "robot is already running"}, 409)
                return
            try:
                command = append_local_command(
                    "start_light_search",
                    "전면 디스플레이 터치",
                    source="display-touch",
                )
                write_display_state("report", "최적 조도 탐색을 시작합니다.", duration=6.0)
                play_message_async("최적 조도 탐색을 시작합니다.")
                self.send_json({"ok": True, "command": command})
            except Exception as exc:
                print(f"display action failed: {exc}", file=sys.stderr)
                self.send_json({"ok": False, "error": "local command failed"}, 503)
            return

        if self.path == "/api/settings":
            payload = self.read_json_body()
            saved = update_control_settings(payload, mark_pending=True)
            threading.Thread(target=sync_settings_to_server, daemon=True).start()
            self.send_json(saved)
            return

        if self.path == "/api/settings/test-sound":
            if play_test_sound():
                self.send_json({"ok": True})
            else:
                self.send_json({"ok": False, "error": "speaker-test not found"}, 503)
            return

        self.send_json({"error": "not found"}, 404)


def find_browser() -> str:
    for candidate in ("chromium", "chromium-browser", "google-chrome", "firefox"):
        path = shutil.which(candidate)
        if path:
            return path
    raise RuntimeError("no browser found. Install chromium-browser on Raspberry Pi.")


def build_browser_command(browser: str, url: str, width: int, height: int) -> list[str]:
    name = os.path.basename(browser).lower()
    if "firefox" in name:
        return [browser, "--kiosk", url]
    return [
        browser,
        "--kiosk",
        "--window-position=0,0",
        f"--window-size={width},{height}",
        "--start-maximized",
        "--start-fullscreen",
        "--force-device-scale-factor=1",
        "--hide-scrollbars",
        "--app=" + url,
        "--noerrdialogs",
        "--no-first-run",
        "--no-default-browser-check",
        "--disable-infobars",
        "--disable-translate",
        "--disable-features=Translate,TranslateUI,PasswordManagerOnboarding",
        "--disable-session-crashed-bubble",
        "--password-store=basic",
        "--disable-gpu",
        "--disable-dev-shm-usage",
        "--user-data-dir=/tmp/onplant-display-chromium",
        "--check-for-update-interval=31536000",
        "--autoplay-policy=no-user-gesture-required",
    ]


def start_local_server(host: str, port: int) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, port), DisplayHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def main() -> int:
    global remote_base_url, remote_robot_id
    parser = argparse.ArgumentParser(description="Run OnPlant 5-inch display with offline fallback.")
    parser.add_argument("--server", default=os.getenv("ONPLANT_SERVER", "http://192.168.100.6:5050"))
    parser.add_argument("--robot-id", default=os.getenv("ONPLANT_ROBOT_ID", "raspbot-a"))
    parser.add_argument("--host", default=os.getenv("ONPLANT_DISPLAY_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.getenv("ONPLANT_DISPLAY_PORT", "8765")))
    parser.add_argument("--poll", type=float, default=float(os.getenv("ONPLANT_DISPLAY_POLL", "2.0")))
    parser.add_argument("--width", type=int, default=int(os.getenv("ONPLANT_DISPLAY_WIDTH", "1024")))
    parser.add_argument("--height", type=int, default=int(os.getenv("ONPLANT_DISPLAY_HEIGHT", "600")))
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    remote_base_url = args.server.rstrip("/")
    remote_robot_id = args.robot_id
    load_control_settings()
    apply_speaker_volume(public_control_settings()["speaker_volume"])
    fallback_status()
    start_local_server(args.host, args.port)

    poll_thread = threading.Thread(
        target=poll_server,
        args=(args.server, args.robot_id, args.poll),
        daemon=True,
    )
    poll_thread.start()

    local_url = f"http://{args.host}:{args.port}/?build={DISPLAY_BUILD}"
    print(f"local display: {local_url}")
    print(f"remote server: {args.server.rstrip('/')} robot_id={args.robot_id}")

    if args.no_browser:
        try:
            while True:
                time.sleep(3600)
        except KeyboardInterrupt:
            return 0

    browser = find_browser()
    command = build_browser_command(browser, local_url, args.width, args.height)
    subprocess.Popen(command)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"front display failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
