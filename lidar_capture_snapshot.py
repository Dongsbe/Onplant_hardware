import argparse
import csv
import math
import time
from pathlib import Path

from rplidar import RPLidar

PORT = "/dev/ttyUSB0"
LIDAR_FRONT_ANGLE = -4.0
LIDAR_MIRROR_ANGLE = True

SELF_IGNORE_ZONES = [
    (105, 125, 180, 280),
    (-125, -105, 180, 280),
    (-105, -80, 110, 180),
    (-45, -20, 150, 240),
]

MIN_VALID = 50
MAX_VALID = 2000

LIDAR_TO_FRONT_AXLE = 25
LIDAR_TO_LEFT_OUTER = 80
LIDAR_TO_RIGHT_OUTER = 80
SAFETY_MARGIN = 20

LEFT_CLEARANCE = LIDAR_TO_LEFT_OUTER + SAFETY_MARGIN
RIGHT_CLEARANCE = LIDAR_TO_RIGHT_OUTER + SAFETY_MARGIN

FRONT_WARN_X = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + 300
FRONT_DANGER_X = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + 180
FRONT_EMERGENCY_X = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + 80
FRONT_CLEAR_X = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + 380

POINT_LIMIT = 2
THIN_POINT_X = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + 160

LEFT_SECTOR = (15, 115)
RIGHT_SECTOR = (-115, -15)
SIDE_FRONT_MIN_X = 0
SIDE_FRONT_MAX_X = 650
SIDE_SCORE_DEADBAND = 60


def normalize_angle(angle):
    angle %= 360
    if angle > 180:
        angle -= 360
    return angle


def normalize_lidar_angle(angle):
    angle = normalize_angle(angle - LIDAR_FRONT_ANGLE)
    if LIDAR_MIRROR_ANGLE:
        angle = -angle
    return normalize_angle(angle)


def angle_to_xy(angle, distance):
    adjusted = normalize_lidar_angle(angle)
    rad = math.radians(adjusted)
    return distance * math.cos(rad), distance * math.sin(rad)


def is_self_noise(angle, distance):
    for min_angle, max_angle, min_distance, max_distance in SELF_IGNORE_ZONES:
        if min_angle <= angle <= max_angle and min_distance <= distance <= max_distance:
            return True
    return False


def classify_point(angle, distance, x, y):
    labels = []

    if x > 0 and -RIGHT_CLEARANCE <= y <= LEFT_CLEARANCE:
        labels.append("front_lane")
        if x <= FRONT_EMERGENCY_X:
            labels.append("emergency")
        elif x <= FRONT_DANGER_X:
            labels.append("danger")
        elif x <= FRONT_WARN_X:
            labels.append("warn")

        if x <= THIN_POINT_X:
            labels.append("thin_candidate")

    if LEFT_SECTOR[0] <= angle <= LEFT_SECTOR[1]:
        labels.append("left_sector")
    elif RIGHT_SECTOR[0] <= angle <= RIGHT_SECTOR[1]:
        labels.append("right_sector")

    return "|".join(labels) if labels else "other"


def collect_points(duration):
    lidar = RPLidar(PORT)
    points = []
    scans = 0
    start = time.monotonic()

    try:
        lidar.stop()
        lidar.clean_input()
        time.sleep(0.5)
        lidar.start_motor()
        time.sleep(1.5)

        for scan in lidar.iter_scans(max_buf_meas=1000):
            scans += 1
            now = time.monotonic()

            for quality, raw_angle, distance in scan:
                if distance < MIN_VALID or distance > MAX_VALID:
                    continue

                angle = normalize_lidar_angle(raw_angle)
                ignored = is_self_noise(angle, distance)
                x, y = angle_to_xy(raw_angle, distance)
                label = "self_noise" if ignored else classify_point(angle, distance, x, y)

                points.append(
                    {
                        "t": now - start,
                        "scan": scans,
                        "quality": quality,
                        "raw_angle": raw_angle,
                        "angle": angle,
                        "distance": distance,
                        "x": x,
                        "y": y,
                        "ignored": ignored,
                        "label": label,
                    }
                )

            if now - start >= duration:
                break

    finally:
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except Exception:
            pass

    return points


def analyze(points):
    active = [p for p in points if not p["ignored"]]
    front = [p for p in active if "front_lane" in p["label"] and p["x"] <= FRONT_WARN_X]
    nearest_front = min(front, key=lambda p: p["x"], default=None)

    left_points = [
        p for p in active
        if LEFT_SECTOR[0] <= p["angle"] <= LEFT_SECTOR[1]
        and SIDE_FRONT_MIN_X <= p["x"] <= SIDE_FRONT_MAX_X
    ]
    right_points = [
        p for p in active
        if RIGHT_SECTOR[0] <= p["angle"] <= RIGHT_SECTOR[1]
        and SIDE_FRONT_MIN_X <= p["x"] <= SIDE_FRONT_MAX_X
    ]

    left_nearest = min((p["distance"] for p in left_points), default=FRONT_CLEAR_X)
    right_nearest = min((p["distance"] for p in right_points), default=FRONT_CLEAR_X)

    danger = nearest_front is not None and nearest_front["x"] <= FRONT_DANGER_X
    emergency = nearest_front is not None and nearest_front["x"] <= FRONT_EMERGENCY_X
    front_blocked = len(front) >= POINT_LIMIT or any(p["x"] <= THIN_POINT_X for p in front)

    if abs(left_nearest - right_nearest) <= SIDE_SCORE_DEADBAND:
        recommended_turn = "BALANCED"
    elif left_nearest >= right_nearest:
        recommended_turn = "LEFT"
    else:
        recommended_turn = "RIGHT"

    return {
        "total_points": len(points),
        "active_points": len(active),
        "ignored_points": len(points) - len(active),
        "front_points": len(front),
        "left_points": len(left_points),
        "right_points": len(right_points),
        "left_nearest": left_nearest,
        "right_nearest": right_nearest,
        "front_blocked": front_blocked,
        "danger": danger,
        "emergency": emergency,
        "recommended_turn": recommended_turn,
        "nearest_front": nearest_front,
    }


def write_csv(points, path):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "t",
                "scan",
                "quality",
                "raw_angle",
                "angle",
                "distance",
                "x",
                "y",
                "ignored",
                "label",
            ],
        )
        writer.writeheader()
        writer.writerows(points)


def write_summary(summary, path, label):
    nearest = summary["nearest_front"]
    with path.open("w", encoding="utf-8") as f:
        f.write(f"label={label}\n")
        for key, value in summary.items():
            if key == "nearest_front":
                continue
            f.write(f"{key}={value}\n")

        if nearest:
            front_gap = nearest["x"] - LIDAR_TO_FRONT_AXLE
            f.write("nearest_front_angle={:.1f}\n".format(nearest["angle"]))
            f.write("nearest_front_distance={:.1f}\n".format(nearest["distance"]))
            f.write("nearest_front_x={:.1f}\n".format(nearest["x"]))
            f.write("nearest_front_y={:.1f}\n".format(nearest["y"]))
            f.write("front_gap={:.1f}\n".format(front_gap))
        else:
            f.write("nearest_front=None\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", default="test")
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--out-dir", default="lidar_captures")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = f"{stamp}_{args.label}"

    points = collect_points(args.duration)
    summary = analyze(points)

    csv_path = out_dir / f"{base}.csv"
    summary_path = out_dir / f"{base}_summary.txt"

    write_csv(points, csv_path)
    write_summary(summary, summary_path, args.label)

    print(f"saved_csv={csv_path}")
    print(f"saved_summary={summary_path}")
    print(f"front_points={summary['front_points']}")
    print(f"front_blocked={summary['front_blocked']}")
    print(f"danger={summary['danger']}")
    print(f"recommended_turn={summary['recommended_turn']}")


if __name__ == "__main__":
    main()
