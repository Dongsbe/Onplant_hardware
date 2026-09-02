from __future__ import annotations

import json
import os
import re
import tempfile
import time
from pathlib import Path
from typing import Any


DEFAULT_COMMAND_FILE = "/tmp/onplant-local-commands.jsonl"
DEFAULT_SENSOR_FILE = "/tmp/onplant-latest-sensor.json"
DEFAULT_DISPLAY_FILE = "/tmp/onplant-display-state.json"
DEFAULT_RUNTIME_FILE = "/tmp/onplant-robot-runtime.json"

WAKE_ALIASES = {
    "동스비",
    "동시비",
    "동습이",
    "동습비",
    "동수비",
    "동쓰비",
    "동스피",
    "동스뷔",
    "동시뷔",
    "동십이",
    "동씹이",
    "동스베",
    "동수베",
    "통스비",
    "통시비",
    "돈스비",
    "돈시비",
    "돈습이",
    "둥스비",
    "둥시비",
}


def command_file() -> Path:
    return Path(os.getenv("ONPLANT_LOCAL_COMMAND_FILE", DEFAULT_COMMAND_FILE))


def sensor_file() -> Path:
    return Path(os.getenv("ONPLANT_SENSOR_CACHE_FILE", DEFAULT_SENSOR_FILE))


def display_file() -> Path:
    return Path(os.getenv("ONPLANT_DISPLAY_STATE_FILE", DEFAULT_DISPLAY_FILE))


def runtime_file() -> Path:
    return Path(os.getenv("ONPLANT_RUNTIME_STATE_FILE", DEFAULT_RUNTIME_FILE))


def normalize_text(value: object) -> str:
    text = str(value or "").strip().lower()
    return re.sub(r"[^0-9a-z가-힣]+", "", text)


def is_wake_word(text: object) -> bool:
    normalized = normalize_text(text)
    if normalized in WAKE_ALIASES:
        return True
    return any(normalized in {f"{alias}야", f"{alias}아"} for alias in WAKE_ALIASES)


def classify_local_command(text: object) -> str | None:
    normalized = normalize_text(text)
    if not normalized:
        return None

    if any(word in normalized for word in ("멈춰", "멈추", "정지", "스톱", "중지")):
        return "stop"
    if (
        ("조도" in normalized and any(word in normalized for word in ("찾", "탐색", "이동")))
        or "밝은곳찾" in normalized
        or "빛찾" in normalized
    ):
        return "start_light_search"
    if any(word in normalized for word in ("상태", "센서", "온도", "습도", "토양수분", "조도알려")):
        return "show_status"
    return None


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, ensure_ascii=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_name, path)
    finally:
        try:
            os.unlink(temp_name)
        except FileNotFoundError:
            pass


def read_json(path: Path, default: dict[str, Any] | None = None) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream)
        return value if isinstance(value, dict) else (default or {})
    except (OSError, ValueError, TypeError):
        return default or {}


def append_local_command(name: str, value: str = "", source: str = "voice-local") -> dict[str, Any]:
    payload = {
        "command": name,
        "value": value,
        "source": source,
        "created_at": time.time(),
    }
    path = command_file()
    path.parent.mkdir(parents=True, exist_ok=True)
    line = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
    descriptor = os.open(path, os.O_APPEND | os.O_CREAT | os.O_WRONLY, 0o600)
    try:
        os.write(descriptor, line)
    finally:
        os.close(descriptor)
    return payload


def write_sensor_cache(robot_id: str, values: dict[str, Any]) -> None:
    atomic_write_json(
        sensor_file(),
        {
            "robot_id": robot_id,
            "lux": values.get("lux"),
            "temperature": values.get("temperature"),
            "humidity": values.get("humidity"),
            "soil_moisture": values.get("soil_moisture"),
            "updated_at": time.time(),
        },
    )


def read_sensor_cache(max_age: float = 30.0) -> dict[str, Any]:
    payload = read_json(sensor_file())
    updated_at = float(payload.get("updated_at") or 0)
    payload["stale"] = updated_at <= 0 or time.time() - updated_at > max_age
    return payload


def write_display_state(screen: str, message: str = "", duration: float = 12.0) -> None:
    now = time.time()
    timed_screens = {"report", "notice"}
    atomic_write_json(
        display_file(),
        {
            "screen": screen,
            "message": message,
            "updated_at": now,
            "report_until": now + duration if screen in timed_screens else None,
        },
    )


def read_display_state() -> dict[str, Any]:
    return read_json(display_file())


def write_runtime_state(
    drive_running: bool,
    state: str,
    action: str,
    **details: Any,
) -> None:
    payload = read_json(runtime_file())
    payload.update(details)
    payload.update(
        {
            "drive_running": bool(drive_running),
            "state": state,
            "action": action,
            "updated_at": time.time(),
        }
    )
    atomic_write_json(runtime_file(), payload)


def read_runtime_state() -> dict[str, Any]:
    return read_json(runtime_file(), {"drive_running": False, "state": "IDLE", "action": "STOP"})


def format_sensor_reply(values: dict[str, Any]) -> str:
    if not values or values.get("stale"):
        return "최신 센서값을 확인할 수 없어요. 센서 서비스를 확인해 주세요."

    def value(name: str, suffix: str) -> str:
        raw = values.get(name)
        if raw is None:
            return "측정 안 됨"
        try:
            return f"{float(raw):.1f}{suffix}"
        except (TypeError, ValueError):
            return f"{raw}{suffix}"

    return (
        f"현재 조도는 {value('lux', '럭스')}, 온도는 {value('temperature', '도')}, "
        f"습도는 {value('humidity', '퍼센트')}, 토양 수분은 "
        f"{value('soil_moisture', '퍼센트')}입니다."
    )
