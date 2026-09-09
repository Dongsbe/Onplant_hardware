from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import wave
from array import array
from pathlib import Path

import requests

from local_runtime import (
    append_local_command,
    classify_local_command,
    format_sensor_reply,
    is_wake_word,
    read_runtime_state,
    read_sensor_cache,
    wake_name,
    wake_word_similarity,
    write_display_state,
)
from local_tts import speak


DEFAULT_SERVER_URL = "http://192.168.100.6:5050"
DEFAULT_ROBOT_ID = "raspbot-a"
DEFAULT_MIC_DEVICE = "plughw:2,0"
DEFAULT_SPEAKER_DEVICE = "plughw:3,0"
DEFAULT_MODEL_PATH = Path(__file__).resolve().parent / "models" / "vosk-model-small-ko-0.22"
_last_blocked_notice_at = 0.0


def normalize_recording(path: Path) -> bool:
    if os.getenv("ONPLANT_MIC_NORMALIZE", "1").strip().lower() not in {
        "1",
        "true",
        "yes",
        "on",
    }:
        return True

    try:
        target_peak = max(1000, int(os.getenv("ONPLANT_MIC_TARGET_PEAK", "12000")))
        max_gain = max(1.0, float(os.getenv("ONPLANT_MIC_MAX_GAIN", "10.0")))
        min_peak = max(0, int(os.getenv("ONPLANT_MIC_MIN_PEAK", "500")))
    except ValueError:
        target_peak = 12000
        max_gain = 10.0
        min_peak = 500

    with wave.open(str(path), "rb") as source:
        params = source.getparams()
        if source.getnchannels() != 1 or source.getsampwidth() != 2:
            return True
        frames = source.readframes(source.getnframes())

    samples = array("h")
    samples.frombytes(frames)
    if not samples:
        return False
    peak = max(abs(sample) for sample in samples)
    if peak < min_peak:
        print(f"microphone input too quiet: peak={peak} minimum={min_peak}")
        return False

    gain = min(max_gain, target_peak / peak)
    if gain <= 1.05:
        return True
    amplified = array(
        "h",
        (max(-32768, min(32767, round(sample * gain))) for sample in samples),
    )
    temporary = path.with_suffix(".normalized.wav")
    with wave.open(str(temporary), "wb") as destination:
        destination.setparams(params)
        destination.writeframes(amplified.tobytes())
    temporary.replace(path)
    print(f"microphone normalized: peak={peak} gain={gain:.1f}x")
    return True


def record_voice(path: Path, seconds: float, mic_device: str) -> bool:
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
    return normalize_recording(path)


class VoskTranscriber:
    def __init__(self, model_path: Path) -> None:
        from vosk import KaldiRecognizer, Model, SetLogLevel

        SetLogLevel(-1)
        self._recognizer_class = KaldiRecognizer
        self._model = Model(str(model_path))
        self.last_confidence = 0.0

    def make_recognizer(self, sample_rate: int = 16000):
        recognizer = self._recognizer_class(self._model, sample_rate)
        recognizer.SetWords(True)
        return recognizer

    def transcribe(self, audio_path: Path) -> str:
        with wave.open(str(audio_path), "rb") as source:
            if source.getnchannels() != 1 or source.getsampwidth() != 2:
                raise RuntimeError("Local STT requires mono 16-bit PCM WAV")
            recognizer = self.make_recognizer(source.getframerate())
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


def post_voice_chat(server: str, robot_id: str, username: str, audio_path: Path) -> dict:
    with audio_path.open("rb") as audio_stream:
        response = requests.post(
            f"{server.rstrip('/')}/api/robots/{robot_id}/voice/chat",
            data={"username": username, "phase": "direct"},
            files={"audio": (audio_path.name, audio_stream, "audio/wav")},
            timeout=(3, 120),
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


def speak_text(args: argparse.Namespace, text: str) -> None:
    print("reply:", text)
    if args.no_play:
        return
    if not speak(text, args.speaker_device):
        print("local TTS failed", file=sys.stderr)


def handle_command_audio(
    args: argparse.Namespace,
    transcript: str,
    confidence: float,
    audio_path: Path,
) -> None:
    global _last_blocked_notice_at
    print("local transcript:", transcript or "<empty>", f"confidence={confidence:.2f}")
    try:
        command_confidence = float(os.getenv("ONPLANT_COMMAND_CONFIDENCE", "0.30"))
    except ValueError:
        command_confidence = 0.30
    command = classify_local_command(transcript) if confidence >= command_confidence else None
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

    write_display_state("notice", "서버 LLM에 연결하고 있습니다.", duration=8.0)
    try:
        result = post_voice_chat(args.server, args.robot_id, args.username, audio_path)
        server_transcript = str(result.get("transcript") or "").strip()
        print(
            "server transcript:",
            server_transcript or "<empty>",
            f"intent={result.get('intent') or 'unknown'}",
        )
        if not server_transcript:
            reply = "잘 듣지 못했어요. 다시 불러 주세요."
        else:
            reply = str(result.get("reply") or "응답을 만들지 못했어요.")
        write_display_state("notice", reply, duration=12.0)
        speak_text(args, reply)
    except requests.RequestException:
        reply = "서버 연결이 필요합니다.\n상태 확인, 조도 탐색, 정지는 로컬에서 사용할 수 있습니다."
        write_display_state("notice", reply, duration=12.0)
        speak_text(args, reply)


def record_and_transcribe(
    args: argparse.Namespace,
    transcriber: VoskTranscriber,
    path: Path,
    seconds: float,
    prompt: str,
) -> str:
    print(prompt)
    if not record_voice(path, seconds, args.mic_device):
        transcriber.last_confidence = 0.0
        return ""
    return transcriber.transcribe(path)


def listen_for_wake_continuous(
    args: argparse.Namespace,
    transcriber: VoskTranscriber,
) -> tuple[str, float, float] | None:
    try:
        stream_seconds = max(5.0, float(os.getenv("ONPLANT_WAKE_STREAM_SECONDS", "15")))
    except ValueError:
        stream_seconds = 15.0

    command = [
        "arecord",
        "-q",
        "-D",
        args.mic_device,
        "-t",
        "raw",
        "-f",
        "S16_LE",
        "-r",
        "16000",
        "-c",
        "1",
        "-",
    ]
    print(f"Waiting wake word continuously for {stream_seconds:.0f}s.")
    process = subprocess.Popen(command, stdout=subprocess.PIPE)
    recognizer = transcriber.make_recognizer(16000)
    deadline = time.monotonic() + stream_seconds

    def evaluate(result: dict) -> tuple[str, float, float] | None:
        transcript = str(result.get("text") or "").strip()
        words = result.get("result") or []
        confidences = [float(word.get("conf", 0)) for word in words]
        confidence = sum(confidences) / len(confidences) if confidences else 0.0
        transcriber.last_confidence = confidence
        if not transcript:
            return None

        wake_score = wake_word_similarity(transcript)
        strong_wake_match = wake_score >= 0.90
        wake_accepted = is_wake_word(transcript) and (
            confidence >= args.wake_confidence or strong_wake_match
        )
        if wake_accepted:
            return transcript, confidence, wake_score
        print(
            "ambient speech ignored:",
            transcript,
            f"confidence={confidence:.2f}",
            f"wake_similarity={wake_score:.2f}",
        )
        return None

    try:
        if process.stdout is None:
            raise RuntimeError("arecord stdout is unavailable")
        while time.monotonic() < deadline:
            chunk = process.stdout.read(4000)
            if not chunk:
                if process.poll() is not None:
                    raise subprocess.CalledProcessError(process.returncode or 1, command)
                continue
            if recognizer.AcceptWaveform(chunk):
                accepted = evaluate(json.loads(recognizer.Result()))
                if accepted is not None:
                    return accepted

        return evaluate(json.loads(recognizer.FinalResult()))
    finally:
        if process.stdout is not None:
            process.stdout.close()
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1)


def run_local_loop(args: argparse.Namespace, transcriber: VoskTranscriber) -> int:
    name = wake_name()
    print(f"Local-first voice loop started. Say '{name}' first. Press Ctrl+C to stop.")
    server_command_cursor = initial_server_command_cursor(args)
    print("local wake recognition: unrestricted Vosk + phonetic similarity")
    with tempfile.TemporaryDirectory() as temp_dir:
        audio_path = Path(temp_dir) / "voice.wav"
        while True:
            server_command_cursor, played_reply = play_pending_server_replies(
                args, server_command_cursor
            )
            if played_reply:
                time.sleep(args.cooldown)
            wake_result = listen_for_wake_continuous(args, transcriber)
            if wake_result is None:
                continue

            transcript, wake_confidence, wake_score = wake_result

            print(
                "wake accepted:",
                transcript,
                f"confidence={wake_confidence:.2f}",
                f"wake_similarity={wake_score:.2f}",
            )
            speak_text(args, "네, 말씀하세요.")
            time.sleep(args.cooldown)
            command_text = record_and_transcribe(
                args,
                transcriber,
                audio_path,
                args.seconds,
                f"Listening command {args.seconds:.1f}s.",
            )
            handle_command_audio(
                args,
                command_text,
                transcriber.last_confidence,
                audio_path,
            )
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
    if transcriber is None:
        print(
            "Local Vosk STT is required. Install requirements-local-voice.txt and check ONPLANT_VOSK_MODEL.",
            file=sys.stderr,
        )
        return 2
    if args.loop:
        return run_local_loop(args, transcriber)

    with tempfile.TemporaryDirectory() as temp_dir:
        audio_path = Path(temp_dir) / "voice.wav"
        transcript = record_and_transcribe(
            args,
            transcriber,
            audio_path,
            args.seconds,
            f"Recording {args.seconds:.1f}s.",
        )
        handle_command_audio(args, transcript, transcriber.last_confidence, audio_path)
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
