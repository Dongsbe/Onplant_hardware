from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
import wave
from pathlib import Path

import requests

from local_runtime import (
    WAKE_ALIASES,
    append_local_command,
    classify_local_command,
    format_sensor_reply,
    is_wake_word,
    read_runtime_state,
    read_sensor_cache,
    write_display_state,
)
from voice_assistant import download_audio, play_audio, post_voice


DEFAULT_SERVER_URL = "http://192.168.100.6:5050"
DEFAULT_ROBOT_ID = "raspbot-a"
DEFAULT_MIC_DEVICE = "plughw:2,0"
DEFAULT_SPEAKER_DEVICE = "plughw:3,0"
DEFAULT_MODEL_PATH = Path(__file__).resolve().parent / "models" / "vosk-model-small-ko-0.22"
_last_blocked_notice_at = 0.0


def record_voice(path: Path, seconds: float, mic_device: str) -> None:
    duration = max(1, int(math.ceil(seconds)))
    subprocess.run(
        [
            "arecord",
            "-D",
            mic_device,
            "-f",
            "S16_LE",
            "-r",
            "16000",
            "-c",
            "1",
            "-d",
            str(duration),
            str(path),
        ],
        check=True,
    )


class VoskTranscriber:
    def __init__(self, model_path: Path) -> None:
        from vosk import KaldiRecognizer, Model, SetLogLevel

        SetLogLevel(-1)
        self._recognizer_class = KaldiRecognizer
        self._model = Model(str(model_path))
        self.last_confidence = 0.0

    def transcribe(self, audio_path: Path, grammar: list[str] | None = None) -> str:
        with wave.open(str(audio_path), "rb") as source:
            if source.getnchannels() != 1 or source.getsampwidth() != 2:
                raise RuntimeError("Local STT requires mono 16-bit PCM WAV")
            if grammar:
                vocabulary = json.dumps([*grammar, "[unk]"], ensure_ascii=False)
                recognizer = self._recognizer_class(self._model, source.getframerate(), vocabulary)
            else:
                recognizer = self._recognizer_class(self._model, source.getframerate())
            recognizer.SetWords(True)
            parts: list[str] = []
            confidences: list[float] = []
            while True:
                chunk = source.readframes(4000)
                if not chunk:
                    break
                if recognizer.AcceptWaveform(chunk):
                    result = json.loads(recognizer.Result())
                    text = result.get("text", "").strip()
                    if text:
                        parts.append(text)
                    confidences.extend(float(word.get("conf", 0)) for word in result.get("result", []))
            final_result = json.loads(recognizer.FinalResult())
            final_text = final_result.get("text", "").strip()
            if final_text:
                parts.append(final_text)
            confidences.extend(float(word.get("conf", 0)) for word in final_result.get("result", []))
        self.last_confidence = sum(confidences) / len(confidences) if confidences else 0.0
        return " ".join(parts).strip()


def make_local_transcriber(model_path: str) -> VoskTranscriber | None:
    path = Path(model_path).expanduser()
    if not path.is_dir():
        print(f"local STT model not found: {path}")
        return None
    try:
        transcriber = VoskTranscriber(path)
    except (ImportError, OSError, RuntimeError) as exc:
        print("local STT unavailable:", exc)
        return None
    print(f"local STT ready: {path}")
    return transcriber


def post_text_chat(server: str, robot_id: str, username: str, message: str) -> dict:
    response = requests.post(
        f"{server.rstrip('/')}/api/robots/{robot_id}/llm/chat",
        json={"message": message, "username": username, "speak": False},
        timeout=(2, 90),
    )
    response.raise_for_status()
    return response.json()


def fetch_server_commands(args: argparse.Namespace) -> list[dict]:
    response = requests.get(
        f"{args.server.rstrip('/')}/api/robots/{args.robot_id}/commands?limit=40",
        timeout=(1.5, 3),
    )
    response.raise_for_status()
    commands = response.json()
    return commands if isinstance(commands, list) else []


def initial_server_command_cursor(args: argparse.Namespace) -> int:
    try:
        return max((int(item.get("id", 0)) for item in fetch_server_commands(args)), default=0)
    except (TypeError, ValueError, requests.RequestException):
        return 0


def play_pending_server_replies(args: argparse.Namespace, cursor: int) -> tuple[int, bool]:
    try:
        commands = fetch_server_commands(args)
    except (TypeError, ValueError, requests.RequestException):
        return cursor, False

    played = False
    for item in sorted(commands, key=lambda command: int(command.get("id", 0))):
        try:
            command_id = int(item.get("id", 0))
        except (TypeError, ValueError):
            continue
        if command_id <= cursor:
            continue
        cursor = command_id
        if item.get("command") != "speak":
            continue
        reply = str(item.get("value") or "").strip()
        if not reply:
            continue
        speak_text(args, reply)
        played = True
    return cursor, played


def cached_tts_path(text: str) -> Path:
    cache_dir = Path(os.getenv("ONPLANT_VOICE_CACHE_DIR", "~/.cache/onplant-voice")).expanduser()
    cache_dir.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha256(text.encode("utf-8")).hexdigest()[:24]
    return cache_dir / f"{digest}.mp3"


def fetch_tts(server: str, text: str, output_path: Path) -> bool:
    try:
        health = requests.get(f"{server.rstrip('/')}/api/health", timeout=1.5)
        health.raise_for_status()
        response = requests.post(
            f"{server.rstrip('/')}/api/tts",
            json={"text": text},
            timeout=(2, 45),
        )
        response.raise_for_status()
        audio_url = response.json().get("audio_url")
        if not audio_url:
            return False
        download_audio(server, audio_url, output_path)
        return True
    except (OSError, ValueError, requests.RequestException) as exc:
        print("server TTS unavailable:", exc)
        return False


def speak_local_fallback(text: str, speaker_device: str) -> bool:
    player = shutil.which("espeak-ng") or shutil.which("espeak")
    if not player:
        return False
    with tempfile.TemporaryDirectory() as temp_dir:
        output = Path(temp_dir) / "offline-reply.wav"
        result = subprocess.run(
            [player, "-v", "ko", "-s", "155", "-w", str(output), text],
            check=False,
        )
        if result.returncode != 0 or not output.is_file():
            return False
        play_audio(output, speaker_device)
        return True


def speak_text(args: argparse.Namespace, text: str) -> None:
    print("reply:", text)
    if args.no_play:
        return
    cached = cached_tts_path(text)
    if cached.is_file() and cached.stat().st_size > 0:
        play_audio(cached, args.speaker_device)
        return
    if fetch_tts(args.server, text, cached):
        play_audio(cached, args.speaker_device)
        return
    if not speak_local_fallback(text, args.speaker_device):
        print("No local TTS engine. Install espeak-ng for offline fallback.")


def handle_local_transcript(args: argparse.Namespace, transcript: str) -> None:
    global _last_blocked_notice_at
    print("transcript:", transcript)
    command = classify_local_command(transcript)
    drive_running = bool(read_runtime_state().get("drive_running"))

    if drive_running and command != "stop":
        now_ts = time.time()
        if now_ts - _last_blocked_notice_at > 10:
            speak_text(args, "현재 이동 중이라 정지 명령만 받을 수 있어요.")
            _last_blocked_notice_at = now_ts
        return
    if command == "stop":
        append_local_command("stop", transcript)
        speak_text(args, "정지할게요.")
        return
    if command == "start_light_search":
        append_local_command("start_light_search", transcript)
        speak_text(args, "최적 조도 탐색을 시작할게요.")
        return
    if command == "show_status":
        reply = format_sensor_reply(read_sensor_cache())
        write_display_state("report", reply)
        append_local_command("show_status", transcript)
        speak_text(args, reply)
        return

    write_display_state("report", "응답 처리 중", duration=8.0)
    try:
        result = post_text_chat(args.server, args.robot_id, args.username, transcript)
        reply = str(result.get("reply") or "응답을 만들지 못했어요.")
        write_display_state("report", reply, duration=12.0)
        speak_text(args, reply)
    except requests.RequestException:
        reply = "서버와 연결할 수 없습니다. 상태 조회와 정지 명령은 사용할 수 있습니다."
        write_display_state("report", reply, duration=12.0)
        speak_text(args, reply)


def server_voice_turn(
    args: argparse.Namespace,
    audio_path: Path,
    phase: str,
    play_intents: set[str] | None = None,
) -> dict | None:
    try:
        result = post_voice(args.server, args.robot_id, args.username, audio_path, phase)
    except requests.RequestException as exc:
        print("voice request failed:", exc)
        return None
    print("transcript:", result.get("transcript"))
    print("reply:", result.get("reply"))
    print("intent:", result.get("intent"))
    audio_url = result.get("audio_url")
    if audio_url and not args.no_play and (play_intents is None or result.get("intent") in play_intents):
        reply_path = audio_path.parent / "server-reply.mp3"
        download_audio(args.server, audio_url, reply_path)
        play_audio(reply_path, args.speaker_device)
    return result


def record_and_transcribe(
    args: argparse.Namespace,
    transcriber: VoskTranscriber,
    path: Path,
    seconds: float,
    prompt: str,
    grammar: list[str] | None = None,
) -> str:
    print(prompt)
    record_voice(path, seconds, args.mic_device)
    return transcriber.transcribe(path, grammar)


def run_local_loop(args: argparse.Namespace, transcriber: VoskTranscriber) -> int:
    print("Local-first voice loop started. Say '동스비' first. Press Ctrl+C to stop.")
    server_command_cursor = initial_server_command_cursor(args)
    wake_grammar = sorted(
        {
            *WAKE_ALIASES,
            *(f"{alias}야" for alias in WAKE_ALIASES),
            *(f"{alias}아" for alias in WAKE_ALIASES),
        }
    )
    with tempfile.TemporaryDirectory() as temp_dir:
        audio_path = Path(temp_dir) / "voice.wav"
        while True:
            server_command_cursor, played_reply = play_pending_server_replies(
                args, server_command_cursor
            )
            if played_reply:
                time.sleep(args.cooldown)
            transcript = record_and_transcribe(
                args,
                transcriber,
                audio_path,
                args.wake_seconds,
                f"Waiting wake word {args.wake_seconds:.1f}s.",
                wake_grammar,
            )
            wake_accepted = is_wake_word(transcript) and transcriber.last_confidence >= args.wake_confidence
            if not wake_accepted:
                if transcript:
                    print(
                        "ambient speech ignored:",
                        transcript,
                        f"confidence={transcriber.last_confidence:.2f}",
                    )
                continue

            speak_text(args, "네, 말씀하세요.")
            time.sleep(args.cooldown)
            command_text = record_and_transcribe(
                args,
                transcriber,
                audio_path,
                args.seconds,
                f"Listening command {args.seconds:.1f}s.",
            )
            if command_text:
                handle_local_transcript(args, command_text)
            else:
                speak_text(args, "잘 듣지 못했어요. 다시 불러 주세요.")
            time.sleep(args.cooldown)


def run_server_fallback_loop(args: argparse.Namespace) -> int:
    print("Local STT is unavailable. Using server STT fallback.")
    server_command_cursor = initial_server_command_cursor(args)
    with tempfile.TemporaryDirectory() as temp_dir:
        audio_path = Path(temp_dir) / "voice.wav"
        while True:
            server_command_cursor, played_reply = play_pending_server_replies(
                args, server_command_cursor
            )
            if played_reply:
                time.sleep(args.cooldown)
            print(f"Waiting wake word {args.wake_seconds:.1f}s.")
            record_voice(audio_path, args.wake_seconds, args.mic_device)
            wake = server_voice_turn(args, audio_path, "wake", {"wake"})
            if not wake or wake.get("intent") != "wake":
                continue
            time.sleep(args.cooldown)
            print(f"Listening command {args.seconds:.1f}s.")
            record_voice(audio_path, args.seconds, args.mic_device)
            server_voice_turn(args, audio_path, "command")
            time.sleep(args.cooldown)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OnPlant local-first voice assistant")
    parser.add_argument("--server", default=os.getenv("ONPLANT_SERVER", DEFAULT_SERVER_URL))
    parser.add_argument("--robot-id", default=os.getenv("ONPLANT_ROBOT_ID", DEFAULT_ROBOT_ID))
    parser.add_argument("--username", default="demo")
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--wake-seconds", type=float, default=2.0)
    parser.add_argument("--mic-device", default=DEFAULT_MIC_DEVICE)
    parser.add_argument("--speaker-device", default=DEFAULT_SPEAKER_DEVICE)
    parser.add_argument("--model", default=os.getenv("ONPLANT_VOSK_MODEL", str(DEFAULT_MODEL_PATH)))
    parser.add_argument(
        "--wake-confidence",
        type=float,
        default=float(os.getenv("ONPLANT_WAKE_CONFIDENCE", "0.82")),
    )
    parser.add_argument("--cooldown", type=float, default=0.8)
    parser.add_argument("--no-play", action="store_true")
    parser.add_argument("--loop", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    transcriber = make_local_transcriber(args.model)
    if args.loop:
        if transcriber is not None:
            return run_local_loop(args, transcriber)
        return run_server_fallback_loop(args)

    with tempfile.TemporaryDirectory() as temp_dir:
        audio_path = Path(temp_dir) / "voice.wav"
        if transcriber is None:
            record_voice(audio_path, args.seconds, args.mic_device)
            server_voice_turn(args, audio_path, "direct")
            return 0
        transcript = record_and_transcribe(
            args,
            transcriber,
            audio_path,
            args.seconds,
            f"Recording {args.seconds:.1f}s.",
        )
        handle_local_transcript(args, transcript)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("Stopped")
        raise SystemExit(0)
    except subprocess.CalledProcessError as exc:
        print(f"command failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
