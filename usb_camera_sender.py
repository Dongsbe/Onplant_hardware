from __future__ import annotations

import argparse
import os
import time

import cv2
import requests


DEFAULT_SERVER_URL = "http://192.168.100.6:5050"
DEFAULT_ROBOT_ID = "raspbot-a"


def env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)))
    except ValueError:
        return default


def env_float(name: str, default: float) -> float:
    try:
        return float(os.getenv(name, str(default)))
    except ValueError:
        return default


def parse_device(value: str) -> int | str:
    text = str(value).strip()
    if text.startswith("/dev/video"):
        suffix = text.removeprefix("/dev/video")
        if suffix.isdigit():
            return int(suffix)
    try:
        return int(text)
    except ValueError:
        return text


def create_capture(device: int | str, width: int, height: int) -> cv2.VideoCapture:
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open camera: {device}")
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    return capture


def main() -> int:
    parser = argparse.ArgumentParser(description="Send USB camera frames to OnPlant server.")
    parser.add_argument("--server", default=os.getenv("ONPLANT_SERVER", DEFAULT_SERVER_URL))
    parser.add_argument("--robot-id", default=os.getenv("ONPLANT_ROBOT_ID", DEFAULT_ROBOT_ID))
    parser.add_argument("--device", default=os.getenv("ONPLANT_CAMERA_DEVICE", "/dev/video0"))
    parser.add_argument("--width", type=int, default=env_int("ONPLANT_CAMERA_WIDTH", 960))
    parser.add_argument("--height", type=int, default=env_int("ONPLANT_CAMERA_HEIGHT", 540))
    parser.add_argument("--interval", type=float, default=env_float("ONPLANT_CAMERA_INTERVAL", 0.2))
    parser.add_argument("--timeout", type=float, default=env_float("ONPLANT_CAMERA_TIMEOUT", 2.0))
    parser.add_argument("--quality", type=int, default=env_int("ONPLANT_CAMERA_JPEG_QUALITY", 80))
    parser.add_argument("--camera-key", default=os.getenv("ONPLANT_CAMERA_KEY", ""))
    args = parser.parse_args()

    if not args.camera_key:
        raise SystemExit("ONPLANT_CAMERA_KEY is required.")

    device = parse_device(args.device)
    url = f"{args.server.rstrip('/')}/api/robots/{args.robot_id}/camera/frame"
    capture = create_capture(device, args.width, args.height)
    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), max(40, min(95, args.quality))]
    headers = {
        "Content-Type": "image/jpeg",
        "X-OnPlant-Camera-Key": args.camera_key,
    }

    print("USB camera sender started")
    print(f"device={args.device} size={args.width}x{args.height} interval={args.interval}s")
    print(f"send to: {url}")

    sent_frames = 0
    try:
        while True:
            started = time.monotonic()
            ok, frame = capture.read()
            if not ok:
                print("camera read failed")
                time.sleep(0.5)
                continue

            ok, encoded = cv2.imencode(".jpg", frame, encode_params)
            if not ok:
                print("jpeg encode failed")
                time.sleep(0.5)
                continue

            try:
                response = requests.post(
                    url,
                    data=encoded.tobytes(),
                    headers=headers,
                    timeout=args.timeout,
                )
                if response.status_code >= 400:
                    print(f"send failed: {response.status_code} {response.text[:120]}")
                else:
                    sent_frames += 1
                    if sent_frames == 1 or sent_frames % 50 == 0:
                        print(f"sent camera frame {sent_frames}")
            except Exception as exc:
                print(f"send failed: {exc}")

            elapsed = time.monotonic() - started
            time.sleep(max(0.0, args.interval - elapsed))
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        capture.release()
        print("Stopped")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
