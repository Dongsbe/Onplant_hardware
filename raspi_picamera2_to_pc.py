import argparse
import io
import time

import requests
from picamera2 import Picamera2


def create_camera(width, height):
    camera = Picamera2()
    config = camera.create_still_configuration(
        main={"size": (width, height)},
        buffer_count=2,
    )
    camera.configure(config)
    camera.start()
    time.sleep(1.0)
    return camera


def capture_jpeg(camera):
    stream = io.BytesIO()
    camera.capture_file(stream, format="jpeg")
    return stream.getvalue()


def main():
    parser = argparse.ArgumentParser(
        description="Send Raspberry Pi Camera Module frames to PC FastAPI"
    )
    parser.add_argument(
        "--server",
        required=True,
        help="PC FastAPI address, example: http://192.168.0.10:8000",
    )
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--interval", type=float, default=0.3)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()

    url = args.server.rstrip("/") + "/camera/frame"
    camera = create_camera(args.width, args.height)

    print("Picamera2 sender started")
    print(f"send to: {url}")
    print(f"size: {args.width}x{args.height}, interval: {args.interval}s")
    print("PC page: http://PC_IP:8000/camera")
    print("Press Ctrl+C to stop.")

    try:
        while True:
            started = time.monotonic()
            frame = capture_jpeg(camera)

            try:
                response = requests.post(
                    url,
                    data=frame,
                    headers={"Content-Type": "image/jpeg"},
                    timeout=args.timeout,
                )
                print(f"sent status={response.status_code} bytes={len(frame)}")
            except Exception as e:
                print(f"send failed: {e}")

            elapsed = time.monotonic() - started
            sleep_time = max(0.0, args.interval - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        camera.stop()
        print("Stopped")


if __name__ == "__main__":
    main()
