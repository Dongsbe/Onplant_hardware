from __future__ import annotations

import argparse
import csv
import math
import os
import time
from datetime import datetime
from pathlib import Path

from rplidar import RPLidar

from movement.movement_controller import MovementController


PORT = os.getenv("ONPLANT_LIDAR_PORT", "/dev/ttyUSB0")
LIDAR_FRONT_ANGLE = -4.0
LIDAR_MIRROR_ANGLE = True
MIN_VALID = 50.0
MAX_VALID = 2000.0
LIDAR_MAX_BUF_MEAS = 1000

LIDAR_TO_LEFT_OUTER = float(os.getenv("ONPLANT_LIDAR_TO_LEFT_OUTER", "80"))
LIDAR_TO_RIGHT_OUTER = float(os.getenv("ONPLANT_LIDAR_TO_RIGHT_OUTER", "80"))
SAFETY_MARGIN = float(os.getenv("ONPLANT_LIDAR_SAFETY_MARGIN", "30"))
LEFT_CLEARANCE = LIDAR_TO_LEFT_OUTER + SAFETY_MARGIN
RIGHT_CLEARANCE = LIDAR_TO_RIGHT_OUTER + SAFETY_MARGIN

SELF_IGNORE_ZONES = (
    (105.0, 125.0, 180.0, 280.0),
    (-125.0, -105.0, 180.0, 280.0),
    (-105.0, -80.0, 110.0, 180.0),
    (-45.0, -20.0, 150.0, 240.0),
)

CSV_FIELDS = (
    "run",
    "course_cm",
    "course_length_mm",
    "direction",
    "elapsed_seconds",
    "estimated_distance_mm",
    "reference_distance_mm",
    "pose_x",
    "pose_y",
    "nearest_front_distance_mm",
    "timestamp",
    "stop_reason",
    "pose_linear_mm_per_sec",
    "stop_distance_mm",
)


def patch_rplidar_health_compat() -> None:
    original_get_health = RPLidar.get_health
    if getattr(original_get_health, "_onplant_compat", False):
        return

    def compatible_get_health(self):
        health = original_get_health(self)
        if isinstance(health, (tuple, list)) and len(health) > 2:
            for index in range(len(health) - 1):
                if isinstance(health[index], str) and isinstance(health[index + 1], int):
                    return health[index], health[index + 1]
            return health[0], health[1]
        return health

    compatible_get_health._onplant_compat = True
    RPLidar.get_health = compatible_get_health


def normalize_angle(angle: float) -> float:
    angle %= 360.0
    if angle > 180.0:
        angle -= 360.0
    return angle


def normalize_lidar_angle(angle: float) -> float:
    angle = normalize_angle(angle - LIDAR_FRONT_ANGLE)
    if LIDAR_MIRROR_ANGLE:
        angle = -angle
    return normalize_angle(angle)


def angle_to_xy(angle: float, distance: float) -> tuple[float, float]:
    corrected_angle = normalize_lidar_angle(angle)
    radians = math.radians(corrected_angle)
    return distance * math.cos(radians), distance * math.sin(radians)


def is_self_noise(angle: float, distance: float) -> bool:
    return any(
        min_angle <= angle <= max_angle and min_distance <= distance <= max_distance
        for min_angle, max_angle, min_distance, max_distance in SELF_IGNORE_ZONES
    )


def get_scan_points(scan) -> list[tuple[float, float, float, float]]:
    points = []
    for _quality, raw_angle, distance in scan:
        if distance < MIN_VALID or distance > MAX_VALID:
            continue
        angle = normalize_lidar_angle(raw_angle)
        if is_self_noise(angle, distance):
            continue
        forward_x, lateral_y = angle_to_xy(raw_angle, distance)
        points.append((angle, distance, forward_x, lateral_y))
    return points


def in_front_lane(forward_x: float, lateral_y: float) -> bool:
    return forward_x > 0 and -RIGHT_CLEARANCE <= lateral_y <= LEFT_CLEARANCE


def get_front_points(points) -> list[tuple[float, float, float, float]]:
    return [point for point in points if in_front_lane(point[2], point[3])]


def nearest_front_distance(scan) -> float | None:
    front_points = get_front_points(get_scan_points(scan))
    if not front_points:
        return None
    return min(point[2] for point in front_points)


def close_lidar(device: RPLidar | None) -> None:
    if device is None:
        return
    for operation in ("stop", "stop_motor", "disconnect"):
        try:
            getattr(device, operation)()
        except Exception:
            pass


def open_lidar() -> RPLidar:
    last_error: Exception | None = None
    for attempt in range(1, 4):
        device = None
        try:
            print(f"LiDAR open attempt={attempt} port={PORT}")
            device = RPLidar(PORT)
            time.sleep(0.2)
            try:
                device.stop()
            except Exception:
                pass
            try:
                device.clean_input()
            except Exception:
                pass
            time.sleep(0.8)
            device.start_motor()
            time.sleep(2.0)
            return device
        except Exception as exc:
            last_error = exc
            print(f"LiDAR open failed attempt={attempt}: {exc}")
            close_lidar(device)
            time.sleep(1.2)
    raise RuntimeError(f"LiDAR open failed: {last_error}")


def next_run_number(log_path: Path) -> int:
    if not log_path.exists() or log_path.stat().st_size == 0:
        return 1
    highest = 0
    try:
        with log_path.open("r", encoding="utf-8-sig", newline="") as stream:
            for row in csv.DictReader(stream):
                try:
                    highest = max(highest, int(row.get("run", 0)))
                except (TypeError, ValueError):
                    continue
    except (OSError, csv.Error) as exc:
        raise RuntimeError(f"Cannot read calibration log {log_path}: {exc}") from exc
    return highest + 1


def append_result(log_path: Path, result: dict[str, object]) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    write_header = not log_path.exists() or log_path.stat().st_size == 0
    with log_path.open("a", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        if write_header:
            writer.writeheader()
        writer.writerow(result)
        stream.flush()
        os.fsync(stream.fileno())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OnPlant straight-line pose calibration")
    parser.add_argument(
        "--course",
        choices=("285", "325"),
        default=os.getenv("ONPLANT_CALIBRATION_COURSE", "").strip(),
        help="testbed course length in centimetres; choose 285 or 325",
    )
    parser.add_argument(
        "--direction",
        choices=("X", "Y"),
        default=os.getenv("ONPLANT_CALIBRATION_DIRECTION", "Y").strip().upper(),
    )
    parser.add_argument(
        "--speed-mm-per-sec",
        type=float,
        default=float(os.getenv("ONPLANT_POSE_LINEAR_MM_PER_SEC", "90")),
    )
    parser.add_argument(
        "--stop-mm",
        type=float,
        default=float(os.getenv("ONPLANT_CALIBRATION_STOP_MM", "250")),
    )
    parser.add_argument(
        "--log",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--max-seconds",
        type=float,
        default=float(os.getenv("ONPLANT_CALIBRATION_MAX_SECONDS", "60")),
    )
    args = parser.parse_args()
    if not args.course:
        parser.error("--course is required (285 or 325)")
    configured_log = os.getenv("ONPLANT_CALIBRATION_LOG", "").strip()
    if args.log is None:
        if configured_log:
            args.log = Path(configured_log).expanduser()
        else:
            args.log = Path(__file__).resolve().parent / f"calibration_{args.course}.csv"
    args.course_length_mm = float(args.course) * 10.0
    if args.speed_mm_per_sec <= 0:
        parser.error("--speed-mm-per-sec must be greater than zero")
    if not 100 <= args.stop_mm <= MAX_VALID:
        parser.error(f"--stop-mm must be between 100 and {MAX_VALID:.0f}")
    if args.max_seconds <= 0:
        parser.error("--max-seconds must be greater than zero")
    return args


def make_result(
    run_number: int,
    args: argparse.Namespace,
    elapsed: float,
    estimated_distance: float,
    nearest_front: float | None,
    stop_reason: str,
) -> dict[str, object]:
    pose_x = estimated_distance if args.direction == "X" else 0.0
    pose_y = estimated_distance if args.direction == "Y" else 0.0
    reference_distance = (
        max(0.0, args.course_length_mm - nearest_front)
        if nearest_front is not None
        else None
    )
    return {
        "run": run_number,
        "course_cm": args.course,
        "course_length_mm": f"{args.course_length_mm:.1f}",
        "direction": args.direction,
        "elapsed_seconds": f"{elapsed:.3f}",
        "estimated_distance_mm": f"{estimated_distance:.1f}",
        "reference_distance_mm": (
            "" if reference_distance is None else f"{reference_distance:.1f}"
        ),
        "pose_x": f"{pose_x:.1f}",
        "pose_y": f"{pose_y:.1f}",
        "nearest_front_distance_mm": "" if nearest_front is None else f"{nearest_front:.1f}",
        "timestamp": datetime.now().astimezone().isoformat(timespec="seconds"),
        "stop_reason": stop_reason,
        "pose_linear_mm_per_sec": f"{args.speed_mm_per_sec:.3f}",
        "stop_distance_mm": f"{args.stop_mm:.1f}",
    }


def run_calibration(args: argparse.Namespace) -> int:
    patch_rplidar_health_compat()
    controller = MovementController()
    lidar: RPLidar | None = None
    run_number = next_run_number(args.log)
    started_at: float | None = None
    elapsed = 0.0
    estimated_distance = 0.0
    nearest_front: float | None = None
    stop_reason = "startup_error"
    exit_code = 1

    print("=" * 60)
    print(
        f"Straight calibration run={run_number} course={args.course}cm "
        f"direction={args.direction} heading=0"
    )
    print(
        f"pose_rate={args.speed_mm_per_sec:.3f}mm/s stop={args.stop_mm:.1f}mm "
        f"max={args.max_seconds:.1f}s"
    )
    print(f"log={args.log}")
    print("The robot will only move FORWARD. Press Ctrl+C to stop.")
    print("=" * 60)

    try:
        lidar = open_lidar()
        scans = lidar.iter_scans(max_buf_meas=LIDAR_MAX_BUF_MEAS)

        first_scan = next(scans)
        first_points = get_scan_points(first_scan)
        first_front_points = get_front_points(first_points)
        nearest_front = (
            min(point[2] for point in first_front_points)
            if first_front_points
            else None
        )
        if not first_points:
            stop_reason = "invalid_first_scan"
            exit_code = 2
            print("Start cancelled: first LiDAR scan contained no valid points")
        elif nearest_front is not None and nearest_front <= args.stop_mm:
            stop_reason = "blocked_at_start"
            exit_code = 2
            print(f"Start blocked: front={nearest_front:.1f}mm")
        else:
            started_at = time.monotonic()
            last_print = 0.0
            controller.forward()
            print("FORWARD")

            for scan in scans:
                now = time.monotonic()
                elapsed = now - started_at
                estimated_distance = args.speed_mm_per_sec * elapsed
                nearest_front = nearest_front_distance(scan)

                if nearest_front is not None and nearest_front <= args.stop_mm:
                    stop_reason = "front_obstacle"
                    exit_code = 0
                    controller.stop()
                    print(f"STOP front={nearest_front:.1f}mm")
                    break

                if elapsed >= args.max_seconds:
                    stop_reason = "timeout"
                    exit_code = 2
                    controller.stop()
                    print("STOP calibration timeout; front wall was not detected")
                    break

                if now - last_print >= 0.5:
                    front_text = "--" if nearest_front is None else f"{nearest_front:.0f}"
                    print(
                        f"t={elapsed:.2f}s estimated={estimated_distance:.1f}mm "
                        f"front={front_text}mm"
                    )
                    last_print = now
    except KeyboardInterrupt:
        stop_reason = "interrupted"
        exit_code = 130
        print("Interrupted")
    except Exception as exc:
        stop_reason = f"lidar_error:{type(exc).__name__}"
        exit_code = 1
        print(f"Calibration failed: {type(exc).__name__}: {exc}")
    finally:
        stopped_at = time.monotonic()
        try:
            controller.stop()
        finally:
            close_lidar(lidar)

        if started_at is not None:
            # Exclude LiDAR shutdown time from dead-reckoning distance.
            if stop_reason not in {"front_obstacle", "timeout"}:
                elapsed = stopped_at - started_at
            estimated_distance = args.speed_mm_per_sec * elapsed
        result = make_result(
            run_number,
            args,
            elapsed,
            estimated_distance,
            nearest_front,
            stop_reason,
        )
        try:
            append_result(args.log, result)
            print(f"Saved run {run_number} to {args.log}")
        except Exception as exc:
            print(f"Failed to save calibration result: {exc}")
            exit_code = 1
        print(result)

    return exit_code


def main() -> int:
    return run_calibration(parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
