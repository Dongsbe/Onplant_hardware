from __future__ import annotations

import hashlib
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator

try:
    import fcntl
except ImportError:  # pragma: no cover - only used for Windows-side syntax checks
    fcntl = None


DEFAULT_SPEAKER_DEVICE = "plughw:3,0"
DEFAULT_VOICE = "ko-KR-InJoonNeural"
_thread_lock = threading.Lock()


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _speaker_device(explicit: str | None = None) -> str:
    return explicit or os.getenv("ONPLANT_SPEAKER_DEVICE", DEFAULT_SPEAKER_DEVICE)


def _cache_path(text: str) -> Path:
    cache_dir = Path(
        os.getenv("ONPLANT_VOICE_CACHE_DIR", "~/.cache/onplant-voice")
    ).expanduser()
    cache_dir.mkdir(parents=True, exist_ok=True)
    voice = os.getenv("ONPLANT_TTS_VOICE", DEFAULT_VOICE)
    rate = os.getenv("ONPLANT_TTS_RATE", "+0%")
    pitch = os.getenv("ONPLANT_TTS_PITCH", "+0Hz")
    volume = os.getenv("ONPLANT_TTS_VOLUME", "+0%")
    cache_key = "\n".join((voice, rate, pitch, volume, text))
    digest = hashlib.sha256(cache_key.encode("utf-8")).hexdigest()[:24]
    return cache_dir / f"{digest}.mp3"


@contextmanager
def _audio_lock() -> Iterator[None]:
    lock_path = Path(os.getenv("ONPLANT_TTS_LOCK_FILE", "/tmp/onplant-tts.lock"))
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with _thread_lock:
        with lock_path.open("a+") as lock_file:
            if fcntl is not None:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                if fcntl is not None:
                    fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


def _edge_tts_command(text: str, output_path: Path) -> list[str]:
    executable = shutil.which("edge-tts")
    command = [executable] if executable else [sys.executable, "-m", "edge_tts"]
    return [
        *command,
        "--voice",
        os.getenv("ONPLANT_TTS_VOICE", DEFAULT_VOICE),
        "--rate",
        os.getenv("ONPLANT_TTS_RATE", "+0%"),
        "--pitch",
        os.getenv("ONPLANT_TTS_PITCH", "+0Hz"),
        "--volume",
        os.getenv("ONPLANT_TTS_VOLUME", "+0%"),
        "--text",
        text,
        "--write-media",
        str(output_path),
    ]


def _make_edge_audio(text: str, output_path: Path) -> bool:
    temporary = output_path.with_suffix(".tmp.mp3")
    try:
        temporary.unlink(missing_ok=True)
        result = subprocess.run(
            _edge_tts_command(text, temporary),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            timeout=45,
            check=False,
        )
        if result.returncode != 0 or not temporary.is_file() or temporary.stat().st_size == 0:
            detail = result.stderr.strip()
            if detail:
                print(f"local edge-tts unavailable: {detail}", file=sys.stderr)
            return False
        temporary.replace(output_path)
        return True
    except (OSError, subprocess.TimeoutExpired) as exc:
        print(f"local edge-tts unavailable: {exc}", file=sys.stderr)
        return False
    finally:
        temporary.unlink(missing_ok=True)


def _play_mp3(path: Path, speaker_device: str) -> bool:
    player = shutil.which("mpg123")
    if not player:
        return False
    result = subprocess.run(
        [player, "-q", "-a", speaker_device, str(path)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        print(f"mp3 playback failed: {result.stderr.strip()}", file=sys.stderr)
    return result.returncode == 0


def _speak_espeak(text: str, speaker_device: str) -> bool:
    synthesizer = shutil.which("espeak-ng") or shutil.which("espeak")
    player = shutil.which("aplay")
    if not synthesizer or not player:
        return False
    with tempfile.TemporaryDirectory() as temp_dir:
        output = Path(temp_dir) / "onplant-offline-tts.wav"
        generated = subprocess.run(
            [synthesizer, "-v", "ko", "-s", "155", "-w", str(output), text],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
        if generated.returncode != 0 or not output.is_file() or output.stat().st_size == 0:
            return False
        played = subprocess.run(
            [player, "-D", speaker_device, str(output)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
        if played.returncode != 0:
            print(f"wav playback failed: {played.stderr.strip()}", file=sys.stderr)
        return played.returncode == 0


def speak(text: str, speaker_device: str | None = None) -> bool:
    message = str(text or "").strip()
    if not message:
        return False
    device = _speaker_device(speaker_device)
    with _audio_lock():
        cached = _cache_path(message)
        edge_ready = cached.is_file() and cached.stat().st_size > 0
        if not edge_ready:
            try:
                retries = max(1, int(os.getenv("ONPLANT_TTS_EDGE_RETRIES", "2")))
            except ValueError:
                retries = 2
            for attempt in range(retries):
                if _make_edge_audio(message, cached):
                    edge_ready = True
                    break
                if attempt + 1 < retries:
                    time.sleep(0.5)
        if edge_ready:
            if _play_mp3(cached, device):
                return True
        if _env_bool("ONPLANT_TTS_ESPEAK_FALLBACK") and _speak_espeak(message, device):
            return True
    print("Neural TTS output failed; robotic espeak fallback is disabled.", file=sys.stderr)
    return False


def speak_async(text: str, speaker_device: str | None = None) -> threading.Thread:
    worker = threading.Thread(
        target=speak,
        args=(text, speaker_device),
        daemon=True,
        name="onplant-local-tts",
    )
    worker.start()
    return worker
