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

CHOSEONG = (
    "ㄱ",
    "ㄲ",
    "ㄴ",
    "ㄷ",
    "ㄸ",
    "ㄹ",
    "ㅁ",
    "ㅂ",
    "ㅃ",
    "ㅅ",
    "ㅆ",
    "ㅇ",
    "ㅈ",
    "ㅉ",
    "ㅊ",
    "ㅋ",
    "ㅌ",
    "ㅍ",
    "ㅎ",
)
JUNGSEONG = (
    "ㅏ",
    "ㅐ",
    "ㅑ",
    "ㅒ",
    "ㅓ",
    "ㅔ",
    "ㅕ",
    "ㅖ",
    "ㅗ",
    "ㅘ",
    "ㅙ",
    "ㅚ",
    "ㅛ",
    "ㅜ",
    "ㅝ",
    "ㅞ",
    "ㅟ",
    "ㅠ",
    "ㅡ",
    "ㅢ",
    "ㅣ",
)
JONGSEONG = (
    "",
    "ㄱ",
    "ㄲ",
    "ㄳ",
    "ㄴ",
    "ㄵ",
    "ㄶ",
    "ㄷ",
    "ㄹ",
    "ㄺ",
    "ㄻ",
    "ㄼ",
    "ㄽ",
    "ㄾ",
    "ㄿ",
    "ㅀ",
    "ㅁ",
    "ㅂ",
    "ㅄ",
    "ㅅ",
    "ㅆ",
    "ㅇ",
    "ㅈ",
    "ㅊ",
    "ㅋ",
    "ㅌ",
    "ㅍ",
    "ㅎ",
)
PHONETIC_GROUPS = (
    {"ㅡ", "ㅜ"},
    {"ㅗ", "ㅜ"},
    {"ㅐ", "ㅔ"},
    {"ㅂ", "ㅍ"},
    {"ㄱ", "ㅋ"},
    {"ㄷ", "ㅌ"},
    {"ㅈ", "ㅊ"},
    {"ㅅ", "ㅆ"},
    {"ㄴ", "ㅇ"},
    {"ㅁ", "ㅂ"},
)


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


def wake_name() -> str:
    return normalize_text(os.getenv("ONPLANT_WAKE_NAME", "동스비")) or "동스비"


def wake_similarity_threshold() -> float:
    try:
        return float(os.getenv("ONPLANT_WAKE_SIMILARITY", "0.80"))
    except ValueError:
        return 0.80


def wake_max_length_diff() -> int:
    try:
        return int(os.getenv("ONPLANT_WAKE_MAX_LENGTH_DIFF", "1"))
    except ValueError:
        return 1


def wake_phrases() -> set[str]:
    names = {wake_name(), *WAKE_ALIASES}
    phrases: set[str] = set()
    for name in names:
        if not name:
            continue
        phrases.add(name)
        phrases.add(f"{name}야")
        phrases.add(f"{name}아")
    return phrases


def strip_wake_suffix(text: str) -> str:
    if len(text) > 1 and text[-1] in {"야", "아"}:
        return text[:-1]
    return text


def decompose_hangul(value: str) -> list[str]:
    parts: list[str] = []
    for char in value:
        code = ord(char)
        if 0xAC00 <= code <= 0xD7A3:
            syllable = code - 0xAC00
            choseong_index = syllable // 588
            jungseong_index = (syllable % 588) // 28
            jongseong_index = syllable % 28
            parts.append(CHOSEONG[choseong_index])
            parts.append(JUNGSEONG[jungseong_index])
            jongseong = JONGSEONG[jongseong_index]
            if jongseong:
                parts.append(jongseong)
        else:
            parts.append(char)
    return parts


def phonetic_substitution_cost(left: str, right: str) -> float:
    if left == right:
        return 0.0
    for group in PHONETIC_GROUPS:
        if left in group and right in group:
            return 0.25
    return 1.0


def phonetic_gap_cost(value: str) -> float:
    return 0.25 if value == "ㅇ" else 1.0


def jamo_similarity(left: str, right: str) -> float:
    left_parts = decompose_hangul(left)
    right_parts = decompose_hangul(right)
    if not left_parts and not right_parts:
        return 1.0
    if not left_parts or not right_parts:
        return 0.0

    previous = [0.0]
    for part in right_parts:
        previous.append(previous[-1] + phonetic_gap_cost(part))

    for left_part in left_parts:
        current = [previous[0] + phonetic_gap_cost(left_part)]
        for index, right_part in enumerate(right_parts, start=1):
            delete = previous[index] + phonetic_gap_cost(left_part)
            insert = current[index - 1] + phonetic_gap_cost(right_part)
            substitute = previous[index - 1] + phonetic_substitution_cost(left_part, right_part)
            current.append(min(delete, insert, substitute))
        previous = current

    max_len = max(len(left_parts), len(right_parts))
    return max(0.0, 1.0 - previous[-1] / max_len)


def wake_word_similarity(text: object) -> float:
    normalized = normalize_text(text)
    if not normalized:
        return 0.0
    candidates = {normalized, strip_wake_suffix(normalized)}
    target = wake_name()
    max_diff = wake_max_length_diff()
    scores = [
        jamo_similarity(candidate, target)
        for candidate in candidates
        if abs(len(candidate) - len(target)) <= max_diff
    ]
    return max(scores, default=0.0)


def is_wake_word(text: object) -> bool:
    normalized = normalize_text(text)
    if normalized in wake_phrases():
        return True
    return wake_word_similarity(normalized) >= wake_similarity_threshold()


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

    # Weather and other daily questions must reach the server even when they
    # contain words such as "temperature" or "status".
    if "날씨" in normalized or (normalized.startswith("오늘") and "상태" not in normalized):
        return None

    request_words = ("알려", "알려줘", "어때", "확인", "보여", "말해", "보고")
    explicit_status_phrases = (
        "상태확인",
        "상태알려",
        "상태어때",
        "오늘상태",
        "현재상태",
        "식물상태",
    )
    sensor_words = ("센서", "온도", "습도", "토양수분", "조도")
    if any(phrase in normalized for phrase in explicit_status_phrases):
        return "show_status"
    if any(sensor in normalized for sensor in sensor_words) and any(
        request in normalized for request in request_words
    ):
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
