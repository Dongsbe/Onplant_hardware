import argparse
import math
import time
from enum import Enum

from rplidar import RPLidar

from Raspbot_Lib import Raspbot
from movement.movement_controller import MovementController


PORT = "/dev/ttyUSB0"

LIDAR_FRONT_ANGLE = -4.0
LIDAR_MIRROR_ANGLE = True

SELF_IGNORE_ZONES = [
    (105, 125, 180, 280),
    (-125, -105, 180, 280),
    (-105, -80, 110, 180),
    (-45, -20, 150, 240),
]

START_KEY = 16
STOP_KEY = 17
IGNORE_KEYS = [0, 65, 255]

MIN_VALID = 50
MAX_VALID = 2000

LIDAR_TO_FRONT_AXLE = 25
LIDAR_TO_LEFT_OUTER = 80
LIDAR_TO_RIGHT_OUTER = 80
SAFETY_MARGIN = 20

LEFT_CLEARANCE = LIDAR_TO_LEFT_OUTER + SAFETY_MARGIN
RIGHT_CLEARANCE = LIDAR_TO_RIGHT_OUTER + SAFETY_MARGIN

FRONT_WARN_X = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + 300
FRONT_DANGER_X = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + 160
FRONT_EMERGENCY_X = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + 70
FRONT_CLEAR_X = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + 360

SIDE_FRONT_MIN_X = 0
SIDE_FRONT_MAX_X = 650
LEFT_SECTOR = (15, 115)
RIGHT_SECTOR = (-115, -15)
SIDE_SCORE_DEADBAND = 60

POINT_LIMIT = 2
THIN_POINT_X = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + 280

BACKWARD_X = 180
BACKWARD_WIDTH = 120

KEY_READ_INTERVAL = 0.2
LIDAR_MAX_BUF_MEAS = 1000
PRINT_INTERVAL = 0.3

TURN_PULSE_SECONDS = 0.20
BACKUP_SECONDS = 0.22
ESCAPE_TURN_SECONDS = 0.65
COMMAND_REFRESH_SECONDS = 0.6


class State(Enum):
    IDLE = "IDLE"
    DRIVE = "DRIVE"
    AVOID = "AVOID"
    BACKUP = "BACKUP"
    ESCAPE = "ESCAPE"


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


def angle_to_xy(raw_angle, distance):
    angle = normalize_lidar_angle(raw_angle)
    rad = math.radians(angle)
    return distance * math.cos(rad), distance * math.sin(rad)


def is_self_noise(angle, distance):
    for min_angle, max_angle, min_distance, max_distance in SELF_IGNORE_ZONES:
        if min_angle <= angle <= max_angle and min_distance <= distance <= max_distance:
            return True
    return False


def get_scan_points(scan):
    points = []
    for quality, raw_angle, distance in scan:
        if distance < MIN_VALID or distance > MAX_VALID:
            continue

        angle = normalize_lidar_angle(raw_angle)
        if is_self_noise(angle, distance):
            continue

        x, y = angle_to_xy(raw_angle, distance)
        points.append((angle, distance, x, y))
    return points


def in_front_lane(x, y):
    return x > 0 and -RIGHT_CLEARANCE <= y <= LEFT_CLEARANCE


def get_front_points(points):
    front = []
    for angle, distance, x, y in points:
        if in_front_lane(x, y) and x <= FRONT_WARN_X:
            front.append((angle, distance, x, y))
    return front


def get_rear_blocked(points):
    count = 0
    for angle, distance, x, y in points:
        if x < 0 and abs(x) <= BACKWARD_X and -BACKWARD_WIDTH <= y <= BACKWARD_WIDTH:
            count += 1
    return count >= 2


def get_side_scores(points):
    left_nearest = MAX_VALID
    right_nearest = MAX_VALID
    left_count = 0
    right_count = 0

    for angle, distance, x, y in points:
        if x < SIDE_FRONT_MIN_X or x > SIDE_FRONT_MAX_X:
            continue

        if LEFT_SECTOR[0] <= angle <= LEFT_SECTOR[1]:
            left_count += 1
            left_nearest = min(left_nearest, distance)
        elif RIGHT_SECTOR[0] <= angle <= RIGHT_SECTOR[1]:
            right_count += 1
            right_nearest = min(right_nearest, distance)

    left_score = left_nearest if left_count else FRONT_CLEAR_X
    right_score = right_nearest if right_count else FRONT_CLEAR_X
    return left_score, right_score, left_count, right_count


def choose_open_turn(points):
    left_score, right_score, left_count, right_count = get_side_scores(points)
    if abs(left_score - right_score) <= SIDE_SCORE_DEADBAND:
        return "LEFT", left_score, right_score
    if left_score >= right_score:
        return "LEFT", left_score, right_score
    return "RIGHT", left_score, right_score


def analyze_scan(scan):
    points = get_scan_points(scan)
    front = get_front_points(points)
    nearest = min(front, key=lambda p: p[2], default=None)
    left_score, right_score, left_count, right_count = get_side_scores(points)

    front_blocked = len(front) >= POINT_LIMIT
    thin_blocked = any(p[2] <= THIN_POINT_X for p in front)
    danger = nearest is not None and nearest[2] <= FRONT_DANGER_X
    emergency = nearest is not None and nearest[2] <= FRONT_EMERGENCY_X
    clear = nearest is None or nearest[2] > FRONT_CLEAR_X

    return {
        "points": points,
        "front": front,
        "nearest": nearest,
        "front_blocked": front_blocked or thin_blocked,
        "danger": danger,
        "emergency": emergency,
        "clear": clear,
        "rear_blocked": get_rear_blocked(points),
        "left_score": left_score,
        "right_score": right_score,
        "left_count": left_count,
        "right_count": right_count,
    }


class LidarDriveController:
    def __init__(self, speed, auto_start):
        self.bot = Raspbot()
        self.controller = MovementController()
        self.controller.speed = speed

        self.state = State.DRIVE if auto_start else State.IDLE
        self.state_until = 0.0
        self.avoid_turn = "LEFT"
        self.last_turn = "LEFT"
        self.turn_repeat_count = 0

        self.current_motion = "STOP"
        self.last_motion_send = 0.0
        self.last_key_read = 0.0
        self.last_print = 0.0

        try:
            self.bot.Ctrl_IR_Switch(1)
        except Exception as e:
            print("IR init skipped:", e)

    def read_ir_key(self):
        try:
            key = self.bot.read_data_array(0x0C, 1)[0]
            if key not in IGNORE_KEYS:
                return key
        except Exception:
            pass
        return 0

    def set_state(self, state, duration=0.0):
        self.state = state
        self.state_until = time.monotonic() + duration if duration > 0 else 0.0

    def send_motion(self, motion, force=False):
        now = time.monotonic()
        if (
            not force
            and motion == self.current_motion
            and now - self.last_motion_send < COMMAND_REFRESH_SECONDS
        ):
            return

        if motion == "FORWARD":
            self.controller.forward()
        elif motion == "BACKWARD":
            self.controller.backward()
        elif motion == "LEFT":
            self.controller.rotate_left()
        elif motion == "RIGHT":
            self.controller.rotate_right()
        else:
            self.controller.stop()
            motion = "STOP"

        self.current_motion = motion
        self.last_motion_send = now

    def handle_remote(self, now):
        if now - self.last_key_read < KEY_READ_INTERVAL:
            return None

        self.last_key_read = now
        key = self.read_ir_key()
        if key == START_KEY:
            self.turn_repeat_count = 0
            self.set_state(State.DRIVE)
            self.send_motion("FORWARD", force=True)
            return "START"
        if key == STOP_KEY:
            self.set_state(State.IDLE)
            self.send_motion("STOP", force=True)
            return "STOP"
        return None

    def step(self, scan_info, now):
        remote_event = self.handle_remote(now)
        if remote_event:
            nearest = scan_info["nearest"]
            front_gap = nearest[2] - LIDAR_TO_FRONT_AXLE if nearest else None
            return remote_event, front_gap

        nearest = scan_info["nearest"]
        front_gap = nearest[2] - LIDAR_TO_FRONT_AXLE if nearest else None

        if self.state == State.IDLE:
            self.send_motion("STOP")
            return "WAIT", front_gap

        if self.state == State.DRIVE:
            if scan_info["emergency"]:
                if scan_info["rear_blocked"]:
                    self.avoid_turn, _, _ = choose_open_turn(scan_info["points"])
                    self.set_state(State.ESCAPE, ESCAPE_TURN_SECONDS)
                    self.send_motion(self.avoid_turn, force=True)
                    return "EMERGENCY_TURN", front_gap

                self.set_state(State.BACKUP, BACKUP_SECONDS)
                self.send_motion("BACKWARD", force=True)
                return "EMERGENCY_BACKUP", front_gap

            if scan_info["danger"] or scan_info["front_blocked"]:
                self.avoid_turn, _, _ = choose_open_turn(scan_info["points"])
                if self.avoid_turn == self.last_turn:
                    self.turn_repeat_count += 1
                else:
                    self.turn_repeat_count = 0
                self.last_turn = self.avoid_turn

                self.set_state(State.AVOID, TURN_PULSE_SECONDS)
                self.send_motion(self.avoid_turn, force=True)
                return "AVOID_START", front_gap

            self.turn_repeat_count = 0
            self.send_motion("FORWARD")
            return "FORWARD", front_gap

        if self.state == State.AVOID:
            if now >= self.state_until:
                if scan_info["clear"] or not scan_info["danger"]:
                    self.set_state(State.DRIVE)
                    self.send_motion("FORWARD", force=True)
                    return "AVOID_DONE", front_gap

                self.avoid_turn, _, _ = choose_open_turn(scan_info["points"])
                if self.turn_repeat_count >= 5:
                    self.set_state(State.ESCAPE, ESCAPE_TURN_SECONDS)
                    self.send_motion(self.avoid_turn, force=True)
                    self.turn_repeat_count = 0
                    return "ESCAPE_REPEAT", front_gap

                self.set_state(State.AVOID, TURN_PULSE_SECONDS)
                self.send_motion(self.avoid_turn, force=True)
                return "AVOID_MORE", front_gap

            self.send_motion(self.avoid_turn)
            return "AVOIDING", front_gap

        if self.state == State.BACKUP:
            if now >= self.state_until:
                self.avoid_turn, _, _ = choose_open_turn(scan_info["points"])
                self.set_state(State.AVOID, TURN_PULSE_SECONDS)
                self.send_motion(self.avoid_turn, force=True)
                return "BACKUP_DONE", front_gap

            self.send_motion("BACKWARD")
            return "BACKING", front_gap

        if self.state == State.ESCAPE:
            if now >= self.state_until:
                self.set_state(State.DRIVE)
                self.send_motion("FORWARD", force=True)
                return "ESCAPE_DONE", front_gap

            self.send_motion(self.avoid_turn)
            return "ESCAPING", front_gap

        self.send_motion("STOP")
        return "UNKNOWN", front_gap

    def print_status(self, label, scan_info, front_gap):
        nearest = scan_info["nearest"]
        if nearest:
            nearest_text = f"x={nearest[2]:.0f} y={nearest[3]:.0f} gap={front_gap:.0f}"
        else:
            nearest_text = "front=clear"

        print(
            f"{self.state.value:6s} {label:16s} motion={self.current_motion:8s} "
            f"{nearest_text} L={scan_info['left_score']:.0f} "
            f"R={scan_info['right_score']:.0f} "
            f"front_points={len(scan_info['front'])}"
        )

    def stop(self):
        self.send_motion("STOP", force=True)


def parse_args():
    parser = argparse.ArgumentParser(description="Lidar-only obstacle avoidance drive")
    parser.add_argument("--port", default=PORT)
    parser.add_argument("--speed", type=int, default=18)
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="Start driving immediately instead of waiting for remote key 1.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    lidar = RPLidar(args.port)
    drive = LidarDriveController(speed=args.speed, auto_start=args.auto_start)

    print("=" * 50)
    print("Lidar-only FSM drive")
    print("Remote key 1: start, key 2: stop")
    print("Use --auto-start only when the robot is lifted or in a safe test area.")
    print(f"port={args.port} speed={args.speed}")
    print("=" * 50)

    try:
        for scan in lidar.iter_scans(max_buf_meas=LIDAR_MAX_BUF_MEAS):
            now = time.monotonic()
            scan_info = analyze_scan(scan)
            label, front_gap = drive.step(scan_info, now)

            if now - drive.last_print >= PRINT_INTERVAL:
                drive.print_status(label, scan_info, front_gap)
                drive.last_print = now

    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        drive.stop()
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except Exception:
            pass
        print("Stopped")


if __name__ == "__main__":
    main()
