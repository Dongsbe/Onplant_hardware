import math
import json
import os
import queue
import random
import signal
import threading
import time
import urllib.request
from enum import Enum
from pathlib import Path

try:
    import smbus
except ImportError:
    import smbus2 as smbus
from rplidar import RPLidar
from serial.serialutil import SerialException

from Raspbot_Lib import Raspbot
from local_runtime import read_sensor_cache, write_display_state, write_runtime_state
from local_tts import speak as play_local_tts
from movement.movement_controller import MovementController

DRIVE_BUILD = "testbed-learning-20260909-1"
PORT = "/dev/ttyUSB0"
LIDAR_FRONT_ANGLE = -4.0
LIDAR_MIRROR_ANGLE = True


def patch_rplidar_health_compat():
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


patch_rplidar_health_compat()

def env_bool(name, default=False):
    value = os.getenv(name)
    if value is None:
        return default
    return value.lower() in ("1", "true", "yes", "on")


USE_DUMMY_LUX = env_bool("ONPLANT_DUMMY_LUX", False)
USE_DUMMY_SOIL = env_bool("ONPLANT_DUMMY_SOIL", False)
AUTO_START = env_bool("ONPLANT_AUTO_START", False)
DISABLE_IR = env_bool("ONPLANT_DISABLE_IR", False)
USE_SENSOR_CACHE = env_bool("ONPLANT_USE_SENSOR_CACHE", True)
BH1750_BUS = int(os.getenv("ONPLANT_BH1750_BUS", os.getenv("ONPLANT_SENSOR_BUS", "3")))
SENSOR_CACHE_MAX_AGE = float(os.getenv("ONPLANT_SENSOR_CACHE_MAX_AGE", "10"))

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

LIDAR_TO_FRONT_AXLE = float(os.getenv("ONPLANT_LIDAR_TO_FRONT_AXLE", "25"))
LIDAR_TO_LEFT_OUTER = float(os.getenv("ONPLANT_LIDAR_TO_LEFT_OUTER", "80"))
LIDAR_TO_RIGHT_OUTER = float(os.getenv("ONPLANT_LIDAR_TO_RIGHT_OUTER", "80"))
SAFETY_MARGIN = float(os.getenv("ONPLANT_LIDAR_SAFETY_MARGIN", "30"))

LEFT_CLEARANCE = LIDAR_TO_LEFT_OUTER + SAFETY_MARGIN
RIGHT_CLEARANCE = LIDAR_TO_RIGHT_OUTER + SAFETY_MARGIN

FRONT_WARN_X = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + float(os.getenv("ONPLANT_FRONT_WARN_MM", "310"))
FRONT_DANGER_X = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + float(os.getenv("ONPLANT_FRONT_DANGER_MM", "180"))
FRONT_EMERGENCY_X = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + float(os.getenv("ONPLANT_FRONT_EMERGENCY_MM", "95"))
FRONT_CLEAR_X = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + float(os.getenv("ONPLANT_FRONT_CLEAR_MM", "380"))

SIDE_FRONT_MIN_X = 0
SIDE_FRONT_MAX_X = 650
LEFT_SECTOR = (15, 115)
RIGHT_SECTOR = (-115, -15)
SIDE_SCORE_DEADBAND = 60

POINT_LIMIT = int(os.getenv("ONPLANT_LIDAR_POINT_LIMIT", "3"))
THIN_POINT_X = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + float(os.getenv("ONPLANT_THIN_POINT_MM", "120"))

BACKWARD_X = float(os.getenv("ONPLANT_BACKWARD_WARN_MM", "120"))
BACKWARD_WIDTH = float(os.getenv("ONPLANT_BACKWARD_WIDTH_MM", "80"))
REAR_POINT_LIMIT = int(os.getenv("ONPLANT_REAR_POINT_LIMIT", "4"))

BH1750_ADDR = 0x23
BH1750_CONT_HIGH_RES = 0x10
LIGHT_READ_INTERVAL = 0.35
SOIL_READ_INTERVAL = 1.0

KEY_READ_INTERVAL = 0.2
LIDAR_MAX_BUF_MEAS = 1000
PRINT_INTERVAL = 0.3
LIDAR_POST_ENABLED = True
LIDAR_POST_DEBUG = True
LIDAR_POST_INTERVAL = 0.35
LIDAR_POST_MAX_POINTS = 360
ONPLANT_SERVER_URL = os.getenv("ONPLANT_SERVER_URL", "http://192.168.100.6:5050").rstrip("/")
ONPLANT_ROBOT_ID = os.getenv("ONPLANT_ROBOT_ID", "raspbot-a")
CONTINUOUS_LEARNING = env_bool("ONPLANT_CONTINUOUS_LEARNING", True)
LEARNING_LOG_ENABLED = env_bool("ONPLANT_LEARNING_LOG_ENABLED", True)
LEARNING_LOG_INTERVAL = max(0.1, float(os.getenv("ONPLANT_LEARNING_LOG_INTERVAL", "0.5")))
LEARNING_COURSE = os.getenv("ONPLANT_LEARNING_COURSE", "unknown").strip()
LEARNING_AXIS = os.getenv("ONPLANT_LEARNING_AXIS", "unknown").strip().upper()
LEARNING_MAX_MM = float(os.getenv("ONPLANT_LEARNING_MAX_MM", "0"))
LEARNING_LOG_PATH = Path(
    os.getenv(
        "ONPLANT_LEARNING_LOG",
        str(Path(__file__).resolve().parent / "autonomy_learning_log.txt"),
    )
).expanduser()

EXPLORE_SECONDS = 50.0
LIGHT_SEEK_SECONDS = 20.0
LIGHT_IMPROVE_MARGIN = 3.0
TARGET_LUX_MIN = float(os.getenv("ONPLANT_TARGET_LUX_MIN", "800"))
TARGET_LUX_MAX = float(os.getenv("ONPLANT_TARGET_LUX_MAX", "900"))
TARGET_LUX = float(os.getenv("ONPLANT_TARGET_LUX", str((TARGET_LUX_MIN + TARGET_LUX_MAX) / 2)))
LIGHT_FOUND_MARGIN = float(os.getenv("ONPLANT_LIGHT_FOUND_MARGIN", "50"))
RETURN_STOP_MARGIN = float(os.getenv("ONPLANT_RETURN_STOP_MARGIN", "50"))

TURN_PULSE_SECONDS = float(os.getenv("ONPLANT_TURN_PULSE_SECONDS", "0.12"))
BACKUP_SECONDS = float(os.getenv("ONPLANT_BACKUP_SECONDS", "0.30"))
ESCAPE_TURN_SECONDS = float(os.getenv("ONPLANT_ESCAPE_TURN_SECONDS", "0.42"))
DEADEND_BACKUP_SECONDS = float(os.getenv("ONPLANT_DEADEND_BACKUP_SECONDS", "0.65"))
ESCAPE_TURN_DEGREES = float(os.getenv("ONPLANT_ESCAPE_TURN_DEGREES", "85"))
ESCAPE_TURN_TOLERANCE_DEGREES = float(os.getenv("ONPLANT_ESCAPE_TURN_TOLERANCE_DEGREES", "7"))
ESCAPE_TURN_MAX_SECONDS = float(os.getenv("ONPLANT_ESCAPE_TURN_MAX_SECONDS", "0.60"))
ESCAPE_SIDE_BLOCK_MM = float(os.getenv("ONPLANT_ESCAPE_SIDE_BLOCK_MM", "230"))
CORNER_BACKUP_SIDE_SCORE_MM = float(os.getenv("ONPLANT_CORNER_BACKUP_SIDE_SCORE_MM", "360"))
ESCAPE_REPEAT_WINDOW_SECONDS = float(os.getenv("ONPLANT_ESCAPE_REPEAT_WINDOW_SECONDS", "5.0"))
ESCAPE_DIRECTION_LOCK_SECONDS = float(os.getenv("ONPLANT_ESCAPE_DIRECTION_LOCK_SECONDS", "4.0"))
ESCAPE_SAME_TURN_BACKUP_LIMIT = int(os.getenv("ONPLANT_ESCAPE_SAME_TURN_BACKUP_LIMIT", "1"))
DOOR_GUARD_ENABLED = env_bool("ONPLANT_DOOR_GUARD_ENABLED", True)
DOOR_HALF_WIDTH_MM = float(os.getenv("ONPLANT_DOOR_HALF_WIDTH_MM", "220"))
DOOR_EXIT_Y_MM = float(os.getenv("ONPLANT_DOOR_EXIT_Y_MM", "-20"))
DOOR_PREDICT_SECONDS = float(os.getenv("ONPLANT_DOOR_PREDICT_SECONDS", "0.55"))
COMMAND_REFRESH_SECONDS = 0.6
POSE_LINEAR_MM_PER_SEC = float(os.getenv("ONPLANT_POSE_LINEAR_MM_PER_SEC", "90"))
POSE_TURN_DEG_PER_SEC = float(os.getenv("ONPLANT_POSE_TURN_DEG_PER_SEC", "175"))
POSE_UPDATE_MAX_DT = float(os.getenv("ONPLANT_POSE_UPDATE_MAX_DT", "0.50"))
BLOCKED_GRID_MM = float(os.getenv("ONPLANT_BLOCKED_GRID_MM", "100"))
RETURN_MAX_SECONDS = float(os.getenv("ONPLANT_RETURN_MAX_SECONDS", "20.0"))
RETURN_HEADING_TOLERANCE_DEGREES = float(os.getenv("ONPLANT_RETURN_HEADING_TOLERANCE_DEGREES", "22"))
RETURN_TURN_MIN_SECONDS = float(os.getenv("ONPLANT_RETURN_TURN_MIN_SECONDS", "0.08"))
RETURN_DETOUR_DISTANCE_MM = float(os.getenv("ONPLANT_RETURN_DETOUR_DISTANCE_MM", "350"))
RETURN_DETOUR_STOP_MARGIN = float(os.getenv("ONPLANT_RETURN_DETOUR_STOP_MARGIN", "90"))
RETURN_DETOUR_MAX_SECONDS = float(os.getenv("ONPLANT_RETURN_DETOUR_MAX_SECONDS", "5.0"))

EXPLORE_NUDGE_INTERVAL = float(os.getenv("ONPLANT_EXPLORE_NUDGE_INTERVAL", "2.5"))
EXPLORE_NUDGE_SECONDS = float(os.getenv("ONPLANT_EXPLORE_NUDGE_SECONDS", "0.09"))
EXPLORE_BRANCH_ENABLED = env_bool("ONPLANT_EXPLORE_BRANCH_ENABLED", True)
EXPLORE_BRANCH_DELAY_SECONDS = float(os.getenv("ONPLANT_EXPLORE_BRANCH_DELAY_SECONDS", "4.0"))
EXPLORE_BRANCH_MIN_INTERVAL = float(os.getenv("ONPLANT_EXPLORE_BRANCH_MIN_INTERVAL", "3.0"))
EXPLORE_BRANCH_MAX_INTERVAL = float(os.getenv("ONPLANT_EXPLORE_BRANCH_MAX_INTERVAL", "7.0"))
EXPLORE_BRANCH_OPEN_MM = float(os.getenv("ONPLANT_EXPLORE_BRANCH_OPEN_MM", "450"))
EXPLORE_BRANCH_SECONDS = float(os.getenv("ONPLANT_EXPLORE_BRANCH_SECONDS", "0.10"))
EXPLORE_EDGE_BIAS_X = float(os.getenv("ONPLANT_EXPLORE_EDGE_BIAS_X", "220"))
EXPLORE_POST_ESCAPE_FORWARD_SECONDS = float(os.getenv("ONPLANT_EXPLORE_POST_ESCAPE_FORWARD_SECONDS", "2.0"))
EXPLORE_PASSAGE_SIDE_LIMIT = float(os.getenv("ONPLANT_EXPLORE_PASSAGE_SIDE_LIMIT", "320"))
PASSAGE_CENTER_DEADBAND = float(os.getenv("ONPLANT_PASSAGE_CENTER_DEADBAND_MM", "40"))
SIDE_TOO_CLOSE_X = float(os.getenv("ONPLANT_SIDE_TOO_CLOSE_MM", "300"))
SIDE_CRITICAL_X = float(os.getenv("ONPLANT_SIDE_CRITICAL_MM", "240"))
TURN_SPACE_MIN_X = float(os.getenv("ONPLANT_TURN_SPACE_MIN_MM", "250"))
TRAPPED_BACKUP_SECONDS = 0.45
AVOID_REPEAT_BACKUP_COUNT = 3
AVOID_REPEAT_ESCAPE_COUNT = 5


class State(Enum):
    IDLE = "IDLE"
    EXPLORE = "EXPLORE"
    AVOID = "AVOID"
    BACKUP = "BACKUP"
    ESCAPE = "ESCAPE"
    RETURN_TO_BEST = "RETURN_TO_BEST"
    RETURN_DETOUR = "RETURN_DETOUR"
    SEEK_LIGHT = "SEEK_LIGHT"


lidar = None
bot = Raspbot()
controller = MovementController()
light_bus = None if USE_DUMMY_LUX or USE_SENSOR_CACHE else smbus.SMBus(BH1750_BUS)

if not DISABLE_IR:
    bot.Ctrl_IR_Switch(1)

state = State.IDLE
state_until = 0.0
state_started = 0.0
explore_started = 0.0
seek_started = 0.0
return_until = 0.0
resume_state = None

current_motion = "STOP"
last_motion_send = 0.0
motion_log = []
active_motion = "STOP"
active_motion_started = 0.0
avoid_turn = "LEFT"
last_turn = "LEFT"
turn_repeat_count = 0
pending_escape_turn = None
pending_escape_resume_state = None
committed_escape_turn = None
committed_escape_start_heading = 0.0
committed_escape_resume_state = None
last_escape_done_turn = None
last_escape_done_time = 0.0
escape_lock_turn = None
escape_lock_until = 0.0
escape_same_turn_backup_count = 0

current_lux = 0.0
current_soil = 0.0
best_lux = -1.0
best_lux_error = float("inf")
best_lux_time = 0.0
best_motion_time = 0.0
pose_x = 0.0
pose_y = 0.0
heading_deg = 0.0
pose_last_update = 0.0
best_x = 0.0
best_y = 0.0
blocked_map = set()
last_lux_read = 0.0
last_soil_read = 0.0
last_print = 0.0
last_key_read = 0.0
dummy_sensor_started = time.monotonic()
explore_last_nudge = 0.0
explore_nudge_until = 0.0
explore_nudge_motion = "FORWARD"
last_explore_branch_turn = None
explore_next_branch_at = 0.0
explore_hold_forward_until = 0.0
last_lidar_post = 0.0
last_lidar_post_error = 0.0
lidar_post_ok_printed = False
lidar_post_queue = queue.Queue(maxsize=1)
auto_start_pending = AUTO_START
last_runtime_write = 0.0
return_started = 0.0
return_turn_motion = None
return_turn_until = 0.0
return_detour_started = 0.0
return_detour_until = 0.0
return_detour_x = 0.0
return_detour_y = 0.0
seek_last_error = float("inf")
seek_probe_motion = "FORWARD"
seek_probe_until = 0.0
search_finished = False
learning_session_id = None
learning_started_at = 0.0
learning_last_log_at = 0.0
learning_samples = 0
learning_bounds = {"x_min": 0.0, "x_max": 0.0, "y_min": 0.0, "y_max": 0.0}


def lidar_post_worker():
    global last_lidar_post_error, lidar_post_ok_printed

    while True:
        payload = lidar_post_queue.get()
        if payload is None:
            return

        url = f"{ONPLANT_SERVER_URL}/api/robots/{ONPLANT_ROBOT_ID}/lidar"
        data = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            url,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            urllib.request.urlopen(request, timeout=0.12).read()
            if LIDAR_POST_DEBUG and not lidar_post_ok_printed:
                print(f"LIDAR POST OK -> {url}")
                lidar_post_ok_printed = True
        except Exception:
            if LIDAR_POST_DEBUG:
                now = time.monotonic()
                if now - last_lidar_post_error >= 5.0:
                    print(f"LIDAR POST ERROR -> {url}")
                    last_lidar_post_error = now


def start_lidar_post_worker():
    if not LIDAR_POST_ENABLED:
        return
    worker = threading.Thread(target=lidar_post_worker, daemon=True)
    worker.start()


def search_result_message(label):
    if label in {"RETURN_TO_BEST_DONE", "SEEK_LIGHT_FOUND"}:
        return "최적 조도 위치에 도착했습니다. 최적 조도 탐색을 종료합니다."
    lux_text = f"{current_lux:.0f}럭스" if current_lux >= 0 else "측정되지 않았습니다"
    return f"현재 위치 조도는 {lux_text}입니다. 최적 조도 탐색을 종료합니다."


def obstacle_label(scan_info):
    if scan_info["emergency"]:
        return "emergency"
    if scan_info["danger"]:
        return "danger"
    if scan_info["front_blocked"]:
        return "front_blocked"
    return "clear"


def runtime_context_state():
    if state == State.RETURN_DETOUR:
        return State.RETURN_TO_BEST
    if state in (State.AVOID, State.BACKUP, State.ESCAPE):
        return committed_escape_resume_state or pending_escape_resume_state or resume_state or State.EXPLORE
    return state


def detail_state_label():
    if state == State.AVOID:
        return "장애물 회피"
    if state == State.BACKUP:
        return "후진 회피"
    if state == State.ESCAPE:
        return "탈출 회전"
    if state == State.RETURN_DETOUR:
        return "장애물 우회"
    if state == State.EXPLORE:
        return "자유 탐색"
    if state == State.RETURN_TO_BEST:
        return "좌표 기반 복귀"
    if state == State.SEEK_LIGHT:
        return "근거리 조도 확인"
    return "대기"


def runtime_phase(now):
    context = runtime_context_state()
    if CONTINUOUS_LEARNING and context in (
        State.EXPLORE,
        State.AVOID,
        State.BACKUP,
        State.ESCAPE,
    ):
        return "테스트베드 학습", None
    if context == State.EXPLORE:
        return "조도탐색(1차)", max(0.0, EXPLORE_SECONDS - elapsed_from_start(now))
    if context in (State.RETURN_TO_BEST, State.RETURN_DETOUR):
        elapsed = now - return_started if return_started > 0 else 0.0
        return "복귀탐색", max(0.0, RETURN_MAX_SECONDS - elapsed)
    if context == State.SEEK_LIGHT:
        elapsed = now - seek_started if seek_started > 0 else 0.0
        return "조도탐색(2차)", max(0.0, LIGHT_SEEK_SECONDS - elapsed)
    return "대기", None


def maybe_post_lidar(scan_info, now):
    global last_lidar_post

    if not LIDAR_POST_ENABLED:
        return
    if now - last_lidar_post < LIDAR_POST_INTERVAL:
        return
    last_lidar_post = now

    points = scan_info["points"]
    stride = max(1, len(points) // LIDAR_POST_MAX_POINTS)
    sampled = points[::stride][:LIDAR_POST_MAX_POINTS]
    phase, remaining = runtime_phase(now)
    payload = {
        "state": state.value,
        "action": current_motion,
        "phase": phase,
        "detail_state": detail_state_label(),
        "remaining_seconds": round(remaining, 1) if remaining is not None else None,
        "label": detail_state_label(),
        "obstacle": obstacle_label(scan_info),
        "current_lux": round(current_lux, 1),
        "best_lux": round(best_lux, 1) if best_lux >= 0 else None,
        "lux_error": round(lux_error_from_target(current_lux), 1) if current_lux >= 0 else None,
        "best_time": round(best_lux_time - explore_started, 1) if explore_started > 0 else 0.0,
        "explore_elapsed": round(elapsed_from_start(now), 1) if explore_started > 0 else 0.0,
        "return_elapsed": round(now - return_started, 1) if return_started > 0 else 0.0,
        "seek_elapsed": round(now - seek_started, 1) if seek_started > 0 else 0.0,
        "seek_seconds": LIGHT_SEEK_SECONDS,
        "front_blocked": scan_info["front_blocked"],
        "danger": scan_info["danger"],
        "emergency": scan_info["emergency"],
        "front_points": len(scan_info["front"]),
        "left_score": round(scan_info["left_score"], 1),
        "right_score": round(scan_info["right_score"], 1),
        "pose_x": round(pose_x, 1),
        "pose_y": round(pose_y, 1),
        "heading": heading_label(),
        "best_x": round(best_x, 1),
        "best_y": round(best_y, 1),
        "blocked_count": len(blocked_map),
        "source": "raspberry-pi",
        "points": [
            {
                "angle": round(angle, 1),
                "distance": round(distance, 1),
                "x": round(x, 1),
                "y": round(y, 1),
            }
            for angle, distance, x, y in sampled
        ],
    }

    try:
        if lidar_post_queue.full():
            lidar_post_queue.get_nowait()
        lidar_post_queue.put_nowait(payload)
    except Exception:
        pass


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
    angle = normalize_lidar_angle(angle)
    rad = math.radians(angle)
    return distance * math.cos(rad), distance * math.sin(rad)


def heading_label():
    angle = heading_deg % 360
    if angle < 45 or angle >= 315:
        return "NORTH"
    if angle < 135:
        return "EAST"
    if angle < 225:
        return "SOUTH"
    return "WEST"


def reset_pose(now):
    global pose_x, pose_y, heading_deg, pose_last_update, best_x, best_y, blocked_map
    pose_x = 0.0
    pose_y = 0.0
    heading_deg = 0.0
    pose_last_update = now
    best_x = 0.0
    best_y = 0.0
    blocked_map = set()


def predict_pose(motion, seconds):
    if motion not in ("FORWARD", "BACKWARD"):
        return pose_x, pose_y

    direction = 1.0 if motion == "FORWARD" else -1.0
    distance = direction * POSE_LINEAR_MM_PER_SEC * seconds
    rad = math.radians(heading_deg)
    return pose_x + math.sin(rad) * distance, pose_y + math.cos(rad) * distance


def door_exit_risk(motion):
    if not DOOR_GUARD_ENABLED:
        return False
    if motion not in ("FORWARD", "BACKWARD"):
        return False

    next_x, next_y = predict_pose(motion, DOOR_PREDICT_SECONDS)
    return abs(next_x) <= DOOR_HALF_WIDTH_MM and next_y <= DOOR_EXIT_Y_MM


def choose_door_recovery_turn():
    angle = normalize_angle(heading_deg)
    return "LEFT" if angle > 0 else "RIGHT"


def guard_motion_for_door(motion):
    if not door_exit_risk(motion):
        return motion
    if motion == "BACKWARD":
        return "STOP"
    return choose_door_recovery_turn()


def update_pose(now):
    global pose_x, pose_y, heading_deg, pose_last_update

    if pose_last_update <= 0:
        pose_last_update = now
        return

    dt = max(0.0, min(now - pose_last_update, POSE_UPDATE_MAX_DT))
    pose_last_update = now
    if dt <= 0:
        return

    if current_motion == "LEFT":
        heading_deg = (heading_deg - POSE_TURN_DEG_PER_SEC * dt) % 360
        return
    if current_motion == "RIGHT":
        heading_deg = (heading_deg + POSE_TURN_DEG_PER_SEC * dt) % 360
        return

    if current_motion not in ("FORWARD", "BACKWARD"):
        return

    direction = 1.0 if current_motion == "FORWARD" else -1.0
    distance = direction * POSE_LINEAR_MM_PER_SEC * dt
    rad = math.radians(heading_deg)
    pose_x += math.sin(rad) * distance
    pose_y += math.cos(rad) * distance


def remember_blocked(scan_info):
    if not (scan_info["front_blocked"] or scan_info["danger"] or scan_info["emergency"]):
        return
    grid_x = int(round(pose_x / BLOCKED_GRID_MM))
    grid_y = int(round(pose_y / BLOCKED_GRID_MM))
    blocked_map.add((grid_x, grid_y))


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
    return count >= REAR_POINT_LIMIT


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
        if left_count > right_count + 2:
            return "RIGHT", left_score, right_score
        if right_count > left_count + 2:
            return "LEFT", left_score, right_score
        return avoid_turn, left_score, right_score

    if left_score >= right_score:
        return "LEFT", left_score, right_score
    return "RIGHT", left_score, right_score


def opposite_turn(turn):
    return "RIGHT" if turn == "LEFT" else "LEFT"


def safe_turn_from_scan(turn, scan_info):
    if turn_side_blocked(turn, scan_info):
        other = opposite_turn(turn)
        if not turn_side_blocked(other, scan_info):
            return other
    return turn


def turn_side_blocked(turn, scan_info):
    if turn == "RIGHT":
        return scan_info["right_count"] >= 2 and scan_info["right_score"] < ESCAPE_SIDE_BLOCK_MM
    return scan_info["left_count"] >= 2 and scan_info["left_score"] < ESCAPE_SIDE_BLOCK_MM


def active_escape_lock(now=None):
    now = time.monotonic() if now is None else now
    if escape_lock_turn in ("LEFT", "RIGHT") and now < escape_lock_until:
        return escape_lock_turn
    return None


def lock_escape_direction(turn, now=None, reset_retry=True):
    global escape_lock_turn, escape_lock_until, escape_same_turn_backup_count
    if turn not in ("LEFT", "RIGHT"):
        return
    now = time.monotonic() if now is None else now
    escape_lock_turn = turn
    escape_lock_until = now + ESCAPE_DIRECTION_LOCK_SECONDS
    if reset_retry:
        escape_same_turn_backup_count = 0


def choose_corner_escape_turn(scan_info, fallback_turn, now=None):
    """Choose a stable escape direction for a corner/dead-end.

    1) If a recent successful escape direction is locked, keep using it.
    2) Otherwise, if an intentional EXPLORE branch led into the corner,
       prefer the opposite direction.
    3) Fall back to the LiDAR-open direction.

    A preferred direction is never forced when that turn side is physically
    blocked according to the LiDAR side clearance check.
    """
    now = time.monotonic() if now is None else now
    locked = active_escape_lock(now)
    if locked and not turn_side_blocked(locked, scan_info):
        return locked

    if last_explore_branch_turn in ("LEFT", "RIGHT"):
        opposite = opposite_turn(last_explore_branch_turn)
        if not turn_side_blocked(opposite, scan_info):
            return opposite

    return safe_turn_from_scan(fallback_turn, scan_info)


def retry_committed_escape_after_backup(now, scan_info, reason):
    """Back up without immediately reversing the committed turn.

    The first side-clearance/emergency interruption keeps the same committed
    direction. Only if that same committed direction fails again do we switch
    to the opposite side. This prevents RIGHT/BACKUP/RIGHT/BACKUP oscillation
    at corners while preserving a safety fallback.
    """
    global pending_escape_turn, pending_escape_resume_state
    global escape_same_turn_backup_count

    current_turn = committed_escape_turn
    resume_after = committed_escape_resume_state
    if current_turn not in ("LEFT", "RIGHT"):
        return None

    keep_same = escape_same_turn_backup_count < ESCAPE_SAME_TURN_BACKUP_LIMIT
    retry_turn = current_turn if keep_same else opposite_turn(current_turn)
    if keep_same:
        escape_same_turn_backup_count += 1
    else:
        escape_same_turn_backup_count = 0
        lock_escape_direction(retry_turn, now, reset_retry=False)

    clear_committed_escape()
    pending_escape_turn = retry_turn
    pending_escape_resume_state = resume_after
    enter_state(State.BACKUP, DEADEND_BACKUP_SECONDS)
    set_motion("BACKWARD", force=True)
    return f"{reason}_BACKUP_KEEP_{retry_turn}" if keep_same else f"{reason}_BACKUP_SWITCH_{retry_turn}"


def should_backup_before_escape(scan_info):
    """Back up first when the robot is genuinely boxed in near the front.

    A plain front obstacle with a wide side opening can be handled by pivoting
    directly.  When both side scores are also short, back up before committing
    to the turn so the chassis has room to rotate out of the corner.
    """
    if scan_info["rear_blocked"]:
        return False
    if scan_info["emergency"] or scan_info["danger"] or both_sides_tight(scan_info):
        return True
    if scan_info["front_blocked"]:
        side_opening = max(scan_info["left_score"], scan_info["right_score"])
        return side_opening <= CORNER_BACKUP_SIDE_SCORE_MM
    return False


def schedule_next_explore_branch(now, initial=False):
    global explore_next_branch_at

    min_delay = EXPLORE_BRANCH_DELAY_SECONDS if initial else EXPLORE_BRANCH_MIN_INTERVAL
    max_delay = max(min_delay, EXPLORE_BRANCH_MAX_INTERVAL)
    explore_next_branch_at = now + random.uniform(min_delay, max_delay)


def choose_free_explore_turn():
    # EXPLORE is intentionally free-running. Do not steer from pose_x.
    # Pick a short branch direction and avoid repeating the exact same
    # intentional branch twice in a row.
    preferred = random.choice(("LEFT", "RIGHT"))
    if last_explore_branch_turn in ("LEFT", "RIGHT") and preferred == last_explore_branch_turn:
        preferred = opposite_turn(preferred)
    return preferred


def should_try_branch(now):
    if not EXPLORE_BRANCH_ENABLED:
        return False
    if elapsed_from_start(now) < EXPLORE_BRANCH_DELAY_SECONDS:
        return False
    if explore_next_branch_at <= 0:
        return True
    if now < explore_next_branch_at:
        return False
    return True


def distance_between(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def target_heading_to(x, y):
    dx = x - pose_x
    dy = y - pose_y
    return normalize_angle(math.degrees(math.atan2(dx, dy)))


def heading_error_to(x, y):
    return normalize_angle(target_heading_to(x, y) - heading_deg)


def turn_for_heading_error(error):
    return "RIGHT" if error > 0 else "LEFT"


def best_lux_delta():
    if best_lux < 0 or current_lux < 0:
        return float("inf")
    return abs(current_lux - best_lux)


def begin_return_to_best(now):
    global return_started, return_turn_motion, return_turn_until
    global return_detour_started, return_detour_until

    return_started = now
    return_turn_motion = None
    return_turn_until = 0.0
    return_detour_started = 0.0
    return_detour_until = 0.0
    enter_state(State.RETURN_TO_BEST)
    set_motion("STOP", force=True)


def finish_light_search(label):
    global search_finished
    finalize_motion_log(time.monotonic())
    finish_learning_session(label, time.monotonic())
    enter_state(State.IDLE)
    set_motion("STOP", force=True)
    message = search_result_message(label)
    write_display_state("report", message, duration=12.0)
    print(
        f"{label}: current_lux={current_lux:.1f} "
        f"best_lux={best_lux:.1f} delta={best_lux_delta():.1f}"
    )
    try:
        play_local_tts(message)
    finally:
        search_finished = True


def begin_seek_light(now):
    global seek_started, seek_last_error, seek_probe_motion, seek_probe_until
    seek_started = now
    seek_last_error = best_lux_delta()
    seek_probe_motion = "FORWARD"
    seek_probe_until = 0.0
    enter_state(State.SEEK_LIGHT)
    set_motion("STOP", force=True)


def choose_return_motion(now):
    global return_turn_motion, return_turn_until

    error = heading_error_to(best_x, best_y)
    if abs(error) <= RETURN_HEADING_TOLERANCE_DEGREES:
        return_turn_motion = None
        return_turn_until = 0.0
        return "FORWARD"

    desired = turn_for_heading_error(error)
    if return_turn_motion == desired and now < return_turn_until:
        return desired

    return_turn_motion = desired
    return_turn_until = now + RETURN_TURN_MIN_SECONDS
    return desired


def begin_return_detour(now):
    global return_detour_started, return_detour_until, return_detour_x, return_detour_y
    global return_turn_motion, return_turn_until

    rad = math.radians(heading_deg)
    return_detour_x = pose_x + math.sin(rad) * RETURN_DETOUR_DISTANCE_MM
    return_detour_y = pose_y + math.cos(rad) * RETURN_DETOUR_DISTANCE_MM
    return_detour_started = now
    return_detour_until = now + RETURN_DETOUR_MAX_SECONDS
    return_turn_motion = None
    return_turn_until = 0.0
    enter_state(State.RETURN_DETOUR)
    set_motion("FORWARD", force=True)


def choose_detour_motion(now):
    global return_turn_motion, return_turn_until

    if distance_between(pose_x, pose_y, return_detour_x, return_detour_y) <= RETURN_DETOUR_STOP_MARGIN:
        return None

    error = heading_error_to(return_detour_x, return_detour_y)
    if abs(error) <= RETURN_HEADING_TOLERANCE_DEGREES:
        return_turn_motion = None
        return_turn_until = 0.0
        return "FORWARD"

    desired = turn_for_heading_error(error)
    if return_turn_motion == desired and now < return_turn_until:
        return desired

    return_turn_motion = desired
    return_turn_until = now + RETURN_TURN_MIN_SECONDS
    return desired


def choose_seek_light_motion(scan_info, now):
    global seek_last_error, seek_probe_motion, seek_probe_until, last_explore_branch_turn

    current_error = best_lux_delta()
    if now < seek_probe_until:
        return seek_probe_motion

    if current_error <= seek_last_error + LIGHT_IMPROVE_MARGIN:
        seek_last_error = current_error
        seek_probe_motion = "FORWARD"
        seek_probe_until = now + 0.6
        return "FORWARD"

    turn = choose_free_explore_turn()
    turn = safe_turn_from_scan(turn, scan_info)
    last_explore_branch_turn = turn
    seek_last_error = current_error
    seek_probe_motion = turn
    seek_probe_until = now + EXPLORE_NUDGE_SECONDS
    return turn


def choose_escape_turn(points):
    turn, left_score, right_score = choose_open_turn(points)
    if turn == last_turn and turn_repeat_count >= 2:
        alternate = opposite_turn(turn)
        turn = alternate
    return turn, left_score, right_score


def adjust_repeated_escape_turn(turn, scan_info, now):
    if not last_escape_done_turn:
        return turn
    if now - last_escape_done_time > ESCAPE_REPEAT_WINDOW_SECONDS:
        return turn
    if turn != last_escape_done_turn:
        return turn

    alternate = opposite_turn(turn)
    if turn_side_blocked(alternate, scan_info):
        return turn
    return alternate


def remember_turn_choice(turn):
    global last_turn, turn_repeat_count

    if turn == last_turn:
        turn_repeat_count += 1
    else:
        turn_repeat_count = 0
    last_turn = turn


def both_sides_tight(scan_info):
    return (
        scan_info["left_count"] >= 2
        and scan_info["right_count"] >= 2
        and scan_info["left_score"] < TURN_SPACE_MIN_X
        and scan_info["right_score"] < TURN_SPACE_MIN_X
    )


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


def init_lux_sensor():
    if USE_DUMMY_LUX or USE_SENSOR_CACHE:
        return

    try:
        light_bus.write_byte(BH1750_ADDR, BH1750_CONT_HIGH_RES)
    except Exception as e:
        print("BH1750 INIT ERROR", e)


def read_dummy_lux():
    t = time.monotonic() - dummy_sensor_started
    lux = random.uniform(420, 620)

    # Dummy bright zone for the current testbed.
    if 22.0 <= t <= 34.0:
        center = 28.0
        width = 6.0
        peak = max(0.0, 1.0 - abs(t - center) / width)
        lux = 800 + 90 * peak + random.uniform(-8, 8)

    return round(lux, 1)


def lux_error_from_target(lux):
    if TARGET_LUX_MIN <= lux <= TARGET_LUX_MAX:
        return 0.0
    return min(abs(lux - TARGET_LUX_MIN), abs(lux - TARGET_LUX_MAX), abs(lux - TARGET_LUX))


def read_lux_fast():
    if USE_DUMMY_LUX:
        return read_dummy_lux()

    if USE_SENSOR_CACHE:
        cached = read_sensor_cache(max_age=SENSOR_CACHE_MAX_AGE)
        if cached.get("stale") or cached.get("lux") is None:
            return -1
        try:
            return float(cached["lux"])
        except (TypeError, ValueError):
            return -1

    try:
        data = light_bus.read_i2c_block_data(BH1750_ADDR, 0x00, 2)
        return ((data[0] << 8) + data[1]) / 1.2
    except Exception as e:
        print("BH1750 READ ERROR", e)
        return -1


def read_dummy_soil():
    return round(random.uniform(38, 52), 1)


def read_soil_fast():
    if USE_DUMMY_SOIL:
        return read_dummy_soil()

    # Real soil sensor is intentionally disabled until it is inserted in soil
    # and its normal range is confirmed.
    return -1


def read_ir_key():
    if DISABLE_IR:
        return 0
    try:
        key = bot.read_data_array(0x0c, 1)[0]
        if key not in IGNORE_KEYS:
            return key
    except Exception:
        pass
    return 0


def should_log_motion():
    return state in (State.EXPLORE, State.AVOID, State.BACKUP, State.ESCAPE)


def log_motion_change(next_motion, now):
    global active_motion, active_motion_started

    if active_motion == next_motion:
        return

    if should_log_motion() and active_motion != "STOP" and active_motion_started > 0:
        duration = now - active_motion_started
        if duration >= 0.05:
            motion_log.append(
                {
                    "start": active_motion_started,
                    "end": now,
                    "motion": active_motion,
                    "duration": duration,
                }
            )

    active_motion = next_motion
    active_motion_started = now


def finalize_motion_log(now):
    log_motion_change("STOP", now)


def elapsed_from_start(now):
    if explore_started <= 0:
        return 0.0
    return now - explore_started


def append_learning_record(record):
    if not LEARNING_LOG_ENABLED:
        return
    try:
        LEARNING_LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        with LEARNING_LOG_PATH.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
    except OSError as exc:
        print("LEARNING LOG ERROR:", exc)


def learning_timestamp():
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")


def start_learning_session(now):
    global learning_session_id, learning_started_at, learning_last_log_at
    global learning_samples, learning_bounds

    if not LEARNING_LOG_ENABLED:
        return
    learning_session_id = f"{time.strftime('%Y%m%d-%H%M%S')}-{os.getpid()}"
    learning_started_at = now
    learning_last_log_at = 0.0
    learning_samples = 0
    learning_bounds = {"x_min": 0.0, "x_max": 0.0, "y_min": 0.0, "y_max": 0.0}
    append_learning_record(
        {
            "record_type": "start",
            "session_id": learning_session_id,
            "timestamp": learning_timestamp(),
            "build": DRIVE_BUILD,
            "course_cm": LEARNING_COURSE,
            "axis": LEARNING_AXIS,
            "expected_max_mm": LEARNING_MAX_MM,
            "pose_linear_mm_per_sec": POSE_LINEAR_MM_PER_SEC,
            "pose_turn_deg_per_sec": POSE_TURN_DEG_PER_SEC,
            "front_warn_mm": FRONT_WARN_X,
            "front_danger_mm": FRONT_DANGER_X,
            "front_emergency_mm": FRONT_EMERGENCY_X,
            "continuous": CONTINUOUS_LEARNING,
        }
    )
    print(f"LEARNING LOG START session={learning_session_id} path={LEARNING_LOG_PATH}")


def log_learning_sample(scan_info, label, now):
    global learning_last_log_at, learning_samples, learning_bounds

    if not LEARNING_LOG_ENABLED or learning_session_id is None:
        return

    learning_bounds["x_min"] = min(learning_bounds["x_min"], pose_x)
    learning_bounds["x_max"] = max(learning_bounds["x_max"], pose_x)
    learning_bounds["y_min"] = min(learning_bounds["y_min"], pose_y)
    learning_bounds["y_max"] = max(learning_bounds["y_max"], pose_y)
    if now - learning_last_log_at < LEARNING_LOG_INTERVAL:
        return

    learning_last_log_at = now
    learning_samples += 1
    nearest = scan_info["nearest"]
    obstacle_x = None
    obstacle_y = None
    if nearest:
        heading_rad = math.radians(heading_deg)
        forward_mm = nearest[2]
        lateral_mm = nearest[3]
        obstacle_x = pose_x + math.sin(heading_rad) * forward_mm + math.cos(heading_rad) * lateral_mm
        obstacle_y = pose_y + math.cos(heading_rad) * forward_mm - math.sin(heading_rad) * lateral_mm
    phase, remaining = runtime_phase(now)
    append_learning_record(
        {
            "record_type": "sample",
            "session_id": learning_session_id,
            "timestamp": learning_timestamp(),
            "elapsed_seconds": round(now - learning_started_at, 3),
            "course_cm": LEARNING_COURSE,
            "axis": LEARNING_AXIS,
            "expected_max_mm": LEARNING_MAX_MM,
            "phase": phase,
            "state": state.value,
            "detail_state": detail_state_label(),
            "action": current_motion,
            "label": label,
            "pose_x": round(pose_x, 1),
            "pose_y": round(pose_y, 1),
            "heading_degrees": round(heading_deg % 360.0, 1),
            "heading": heading_label(),
            "current_lux": round(current_lux, 1),
            "best_lux": round(best_lux, 1) if best_lux >= 0 else None,
            "best_x": round(best_x, 1),
            "best_y": round(best_y, 1),
            "front_range_mm": round(nearest[1], 1) if nearest else None,
            "front_distance_mm": round(nearest[2], 1) if nearest else None,
            "front_lateral_mm": round(nearest[3], 1) if nearest else None,
            "obstacle_estimated_x": round(obstacle_x, 1) if obstacle_x is not None else None,
            "obstacle_estimated_y": round(obstacle_y, 1) if obstacle_y is not None else None,
            "left_distance_mm": round(scan_info["left_score"], 1),
            "right_distance_mm": round(scan_info["right_score"], 1),
            "front_points": len(scan_info["front"]),
            "left_points": scan_info["left_count"],
            "right_points": scan_info["right_count"],
            "total_points": len(scan_info["points"]),
            "front_blocked": scan_info["front_blocked"],
            "danger": scan_info["danger"],
            "emergency": scan_info["emergency"],
            "rear_blocked": scan_info["rear_blocked"],
            "blocked_cells": len(blocked_map),
            "remaining_seconds": round(remaining, 1) if remaining is not None else None,
        }
    )


def finish_learning_session(reason, now):
    global learning_session_id

    if not LEARNING_LOG_ENABLED or learning_session_id is None:
        return
    append_learning_record(
        {
            "record_type": "end",
            "session_id": learning_session_id,
            "timestamp": learning_timestamp(),
            "reason": reason,
            "elapsed_seconds": round(max(0.0, now - learning_started_at), 3),
            "samples": learning_samples,
            "course_cm": LEARNING_COURSE,
            "axis": LEARNING_AXIS,
            "expected_max_mm": LEARNING_MAX_MM,
            "x_min": round(learning_bounds["x_min"], 1),
            "x_max": round(learning_bounds["x_max"], 1),
            "x_span_mm": round(learning_bounds["x_max"] - learning_bounds["x_min"], 1),
            "y_min": round(learning_bounds["y_min"], 1),
            "y_max": round(learning_bounds["y_max"], 1),
            "y_span_mm": round(learning_bounds["y_max"] - learning_bounds["y_min"], 1),
            "final_x": round(pose_x, 1),
            "final_y": round(pose_y, 1),
            "final_heading_degrees": round(heading_deg % 360.0, 1),
            "best_lux": round(best_lux, 1) if best_lux >= 0 else None,
            "best_x": round(best_x, 1),
            "best_y": round(best_y, 1),
        }
    )
    print(
        "LEARNING LOG END",
        f"session={learning_session_id}",
        f"x={learning_bounds['x_min']:.0f}..{learning_bounds['x_max']:.0f}",
        f"y={learning_bounds['y_min']:.0f}..{learning_bounds['y_max']:.0f}",
    )
    learning_session_id = None


def set_motion(motion, force=False):
    global current_motion, last_motion_send

    now = time.monotonic()
    motion = guard_motion_for_door(motion)
    if not force and motion == current_motion and now - last_motion_send < COMMAND_REFRESH_SECONDS:
        return

    if motion != current_motion:
        log_motion_change(motion, now)

    current_motion = motion
    last_motion_send = now

    if motion == "FORWARD":
        controller.forward()
    elif motion == "LEFT":
        controller.rotate_left()
    elif motion == "RIGHT":
        controller.rotate_right()
    elif motion == "BACKWARD":
        controller.backward()
    else:
        controller.stop()


def enter_state(new_state, duration=0.0):
    global state, state_started, state_until
    state = new_state
    state_started = time.monotonic()
    state_until = state_started + duration if duration > 0 else 0.0


def turn_progress_degrees(start_heading, current_heading, turn):
    if turn == "RIGHT":
        return normalize_angle(current_heading - start_heading)
    return normalize_angle(start_heading - current_heading)


def committed_escape_reached():
    if not committed_escape_turn:
        return False
    progress = turn_progress_degrees(
        committed_escape_start_heading,
        heading_deg,
        committed_escape_turn,
    )
    return progress >= ESCAPE_TURN_DEGREES - ESCAPE_TURN_TOLERANCE_DEGREES


def clear_committed_escape():
    global committed_escape_turn, committed_escape_start_heading, committed_escape_resume_state
    committed_escape_turn = None
    committed_escape_start_heading = 0.0
    committed_escape_resume_state = None


def begin_committed_escape(turn, resume_after=None):
    global avoid_turn, committed_escape_turn, committed_escape_start_heading, committed_escape_resume_state
    avoid_turn = turn
    committed_escape_turn = turn
    committed_escape_start_heading = heading_deg
    committed_escape_resume_state = resume_after
    enter_state(State.ESCAPE, ESCAPE_TURN_MAX_SECONDS)
    set_motion(turn, force=True)
    return f"ESCAPE_COMMIT_{turn}_{ESCAPE_TURN_DEGREES:.0f}DEG"


def start_committed_escape(turn, scan_info, backup_first=False, resume_after=None):
    global pending_escape_turn, pending_escape_resume_state

    now = time.monotonic()
    locked = active_escape_lock(now)

    # During EXPLORE, a danger/dead-end that requires BACKUP is treated as a
    # corner escape. If the robot recently branched RIGHT into a corner, prefer
    # LEFT after backing up (and vice versa). A recently successful escape
    # direction is temporarily locked so the robot does not alternate at the
    # same wall every few hundred milliseconds.
    if resume_after == State.EXPLORE and backup_first:
        turn = choose_corner_escape_turn(scan_info, turn, now)
        lock_escape_direction(turn, now, reset_retry=True)
    elif locked and not turn_side_blocked(locked, scan_info):
        turn = locked
    else:
        turn = safe_turn_from_scan(turn, scan_info)

    if backup_first and not scan_info["rear_blocked"]:
        pending_escape_turn = turn
        pending_escape_resume_state = resume_after
        enter_state(State.BACKUP, DEADEND_BACKUP_SECONDS)
        set_motion("BACKWARD", force=True)
        return f"ESCAPE_BACKUP_BEFORE_{turn}"
    return begin_committed_escape(turn, resume_after)


def resume_explore_after_escape(now, completed_turn=None):
    global explore_last_nudge, explore_nudge_until, explore_nudge_motion
    global last_explore_branch_turn, explore_hold_forward_until

    explore_last_nudge = now
    explore_nudge_until = 0.0
    explore_nudge_motion = "FORWARD"
    explore_hold_forward_until = now + EXPLORE_POST_ESCAPE_FORWARD_SECONDS
    if completed_turn in ("LEFT", "RIGHT"):
        last_explore_branch_turn = completed_turn
        # Keep the successful corner-exit direction for a short window. If the
        # same wall is detected again immediately, continue going around it
        # instead of flipping LEFT/RIGHT on each scan.
        lock_escape_direction(completed_turn, now, reset_retry=True)
    schedule_next_explore_branch(now, initial=True)
    enter_state(State.EXPLORE)
    set_motion("FORWARD", force=True)


def resume_after_escape(now):
    global resume_state, return_until, last_escape_done_turn, last_escape_done_time
    next_state = committed_escape_resume_state or resume_state or State.EXPLORE
    completed_turn = committed_escape_turn
    last_escape_done_turn = completed_turn
    last_escape_done_time = now
    resume_state = None
    clear_committed_escape()
    if next_state == State.RETURN_DETOUR:
        begin_return_detour(now)
        return next_state
    if next_state == State.RETURN_TO_BEST:
        enter_state(next_state)
        return_until = 0.0
        set_motion("STOP", force=True)
    elif next_state == State.EXPLORE:
        resume_explore_after_escape(now, completed_turn)
    else:
        enter_state(next_state)
        set_motion("FORWARD", force=True)
    return next_state


def update_light(now):
    global current_lux, best_lux, best_lux_error, best_lux_time, best_motion_time, best_x, best_y, last_lux_read

    if now - last_lux_read < LIGHT_READ_INTERVAL:
        return

    lux = read_lux_fast()
    last_lux_read = now
    if lux < 0:
        return

    current_lux = lux
    error = lux_error_from_target(lux)
    if best_lux < 0 or error < best_lux_error:
        best_lux = lux
        best_lux_error = error
        best_lux_time = now
        best_motion_time = now
        best_x = pose_x
        best_y = pose_y


def update_soil(now):
    global current_soil, last_soil_read

    if now - last_soil_read < SOIL_READ_INTERVAL:
        return

    soil = read_soil_fast()
    last_soil_read = now
    if soil < 0:
        return

    current_soil = soil


def choose_explore_clear_motion(scan_info, now):
    global explore_last_nudge, explore_nudge_until, explore_nudge_motion, last_explore_branch_turn

    if scan_info["emergency"] or scan_info["danger"] or scan_info["front_blocked"]:
        return None

    if now < explore_nudge_until:
        return explore_nudge_motion

    if now < explore_hold_forward_until:
        return "FORWARD"

    side_nudge_ready = now - explore_last_nudge >= EXPLORE_NUDGE_INTERVAL

    front_points = len(scan_info["front"])
    left_score = scan_info["left_score"]
    right_score = scan_info["right_score"]
    left_count = scan_info["left_count"]
    right_count = scan_info["right_count"]

    # Continue straight in a narrow passage only while the robot is centered.
    # An unbalanced passage must reach the side-correction checks below.
    if (
        front_points == 0
        and left_score < EXPLORE_PASSAGE_SIDE_LIMIT
        and right_score < EXPLORE_PASSAGE_SIDE_LIMIT
        and abs(left_score - right_score) <= PASSAGE_CENTER_DEADBAND
    ):
        return "FORWARD"

    # Side-only obstacles do not always enter the front lane. Handle them here
    # so the robot does not keep spinning or scraping near a box/wall.
    right_critical = right_count >= 2 and right_score < SIDE_CRITICAL_X
    if (
        (side_nudge_ready or right_critical)
        and right_count >= 2
        and right_score < SIDE_TOO_CLOSE_X
    ):
        explore_last_nudge = now
        explore_nudge_motion = "LEFT"
        explore_nudge_until = now + min(EXPLORE_NUDGE_SECONDS, 0.25)
        return "LEFT"

    left_critical = left_count >= 2 and left_score < SIDE_CRITICAL_X
    if (
        (side_nudge_ready or left_critical)
        and left_count >= 2
        and left_score < SIDE_TOO_CLOSE_X
    ):
        explore_last_nudge = now
        explore_nudge_motion = "RIGHT"
        explore_nudge_until = now + min(EXPLORE_NUDGE_SECONDS, 0.25)
        return "RIGHT"

    if should_try_branch(now):
        branch_motion = choose_free_explore_turn()
        branch_motion = safe_turn_from_scan(branch_motion, scan_info)
        explore_last_nudge = now
        last_explore_branch_turn = branch_motion
        explore_nudge_motion = branch_motion
        explore_nudge_until = now + EXPLORE_BRANCH_SECONDS
        schedule_next_explore_branch(now)
        return branch_motion

    return "FORWARD"


def fsm_step(scan_info, now):
    global avoid_turn, last_turn, turn_repeat_count, pending_escape_turn, pending_escape_resume_state
    global explore_started, seek_started, best_lux, best_lux_error, best_lux_time
    global return_until, resume_state, return_started

    nearest = scan_info["nearest"]
    front_gap = nearest[2] - LIDAR_TO_FRONT_AXLE if nearest else None

    if state == State.IDLE:
        set_motion("STOP")
        return "WAIT", front_gap

    if state in (State.EXPLORE, State.SEEK_LIGHT):
        if (
            state == State.EXPLORE
            and not CONTINUOUS_LEARNING
            and now - explore_started >= EXPLORE_SECONDS
        ):
            finalize_motion_log(now)
            begin_return_to_best(now)
            print(
                f"EXPLORE DONE: best_lux={best_lux:.1f} "
                f"best_time={best_lux_time - explore_started:.1f}s "
                f"best_coord=({best_x:.0f},{best_y:.0f}) "
                f"current=({pose_x:.0f},{pose_y:.0f})"
            )
            return "RETURN_START", front_gap

        if scan_info["emergency"]:
            avoid_turn, _, _ = choose_escape_turn(scan_info["points"])
            remember_turn_choice(avoid_turn)
            label = start_committed_escape(
                avoid_turn,
                scan_info,
                backup_first=should_backup_before_escape(scan_info),
                resume_after=state,
            )
            return f"EMERGENCY_{label}", front_gap

        if scan_info["danger"] or scan_info["front_blocked"]:
            avoid_turn, _, _ = choose_escape_turn(scan_info["points"])
            avoid_turn = adjust_repeated_escape_turn(avoid_turn, scan_info, now)
            remember_turn_choice(avoid_turn)
            backup_first = should_backup_before_escape(scan_info)
            label = start_committed_escape(avoid_turn, scan_info, backup_first=backup_first, resume_after=state)
            return label, front_gap

        if state == State.SEEK_LIGHT:
            if seek_started <= 0:
                seek_started = now

            if best_lux_delta() <= LIGHT_FOUND_MARGIN:
                finish_light_search("SEEK_LIGHT_FOUND")
                return "SEEK_LIGHT_FOUND", front_gap

            if now - seek_started >= LIGHT_SEEK_SECONDS:
                finish_light_search("SEEK_LIGHT_TIMEOUT")
                return "SEEK_LIGHT_TIMEOUT", front_gap

            seek_motion = choose_seek_light_motion(scan_info, now)
            if seek_motion is not None:
                set_motion(seek_motion)
                return f"SEEK_{seek_motion}", front_gap

            set_motion("FORWARD")
            return "SEEK_FORWARD", front_gap

        if state == State.EXPLORE:
            explore_motion = choose_explore_clear_motion(scan_info, now)
            if explore_motion is not None:
                set_motion(explore_motion)
                return f"EXPLORE_{explore_motion}", front_gap

        set_motion("FORWARD")
        return "FORWARD", front_gap

    if state == State.RETURN_TO_BEST:
        if best_lux_delta() <= LIGHT_FOUND_MARGIN:
            finish_light_search("RETURN_TO_BEST_DONE")
            return "RETURN_LUX_FOUND", front_gap

        distance_to_best = distance_between(pose_x, pose_y, best_x, best_y)
        return_timed_out = return_started > 0 and now - return_started >= RETURN_MAX_SECONDS
        if distance_to_best <= RETURN_STOP_MARGIN or return_timed_out:
            if best_lux_delta() <= LIGHT_FOUND_MARGIN:
                finish_light_search("RETURN_TO_BEST_DONE")
                return "RETURN_FINISH", front_gap
            begin_seek_light(now)
            return "RETURN_DONE_SEEK_LIGHT", front_gap

        if scan_info["emergency"] or scan_info["danger"] or scan_info["front_blocked"]:
            avoid_turn, _, _ = choose_escape_turn(scan_info["points"])
            avoid_turn = adjust_repeated_escape_turn(avoid_turn, scan_info, now)
            remember_turn_choice(avoid_turn)
            backup_first = should_backup_before_escape(scan_info)
            label = start_committed_escape(
                avoid_turn,
                scan_info,
                backup_first=backup_first,
                resume_after=State.RETURN_DETOUR,
            )
            return f"RETURN_{label}", front_gap

        return_motion = choose_return_motion(now)
        set_motion(return_motion)
        return f"RETURN_{return_motion}", front_gap

    if state == State.RETURN_DETOUR:
        if best_lux_delta() <= LIGHT_FOUND_MARGIN:
            finish_light_search("RETURN_TO_BEST_DONE")
            return "DETOUR_LUX_FOUND", front_gap

        if scan_info["emergency"] or scan_info["danger"] or scan_info["front_blocked"]:
            avoid_turn, _, _ = choose_escape_turn(scan_info["points"])
            avoid_turn = adjust_repeated_escape_turn(avoid_turn, scan_info, now)
            remember_turn_choice(avoid_turn)
            backup_first = should_backup_before_escape(scan_info)
            label = start_committed_escape(
                avoid_turn,
                scan_info,
                backup_first=backup_first,
                resume_after=State.RETURN_DETOUR,
            )
            return f"DETOUR_{label}", front_gap

        detour_done = (
            distance_between(pose_x, pose_y, return_detour_x, return_detour_y)
            <= RETURN_DETOUR_STOP_MARGIN
        )
        detour_timed_out = return_detour_until > 0 and now >= return_detour_until
        if detour_done or detour_timed_out:
            enter_state(State.RETURN_TO_BEST)
            set_motion("STOP", force=True)
            return "DETOUR_DONE_RETURN", front_gap

        detour_motion = choose_detour_motion(now)
        if detour_motion is None:
            enter_state(State.RETURN_TO_BEST)
            set_motion("STOP", force=True)
            return "DETOUR_REACHED_RETURN", front_gap

        set_motion(detour_motion)
        return f"DETOUR_{detour_motion}", front_gap

    if state == State.AVOID:
        if state_until and now >= state_until:
            if scan_info["clear"] or (not scan_info["danger"] and not scan_info["front_blocked"]):
                next_state = resume_state or State.EXPLORE
                resume_state = None
                enter_state(next_state)
                if next_state == State.RETURN_TO_BEST:
                    set_motion("STOP", force=True)
                    return_until = 0.0
                else:
                    set_motion("FORWARD", force=True)
                return "AVOID_DONE", front_gap

            avoid_turn, _, _ = choose_escape_turn(scan_info["points"])
            remember_turn_choice(avoid_turn)
            if both_sides_tight(scan_info) and not scan_info["rear_blocked"]:
                enter_state(State.BACKUP, TRAPPED_BACKUP_SECONDS)
                set_motion("BACKWARD", force=True)
                return "AVOID_TRAPPED_BACKUP", front_gap

            if turn_repeat_count >= AVOID_REPEAT_ESCAPE_COUNT:
                turn_repeat_count = 0
                if not scan_info["rear_blocked"]:
                    enter_state(State.BACKUP, TRAPPED_BACKUP_SECONDS)
                    set_motion("BACKWARD", force=True)
                    return "AVOID_REPEAT_BACKUP", front_gap

                enter_state(State.ESCAPE, ESCAPE_TURN_SECONDS)
                set_motion(avoid_turn, force=True)
                return "ESCAPE_REPEAT", front_gap

            if turn_repeat_count >= AVOID_REPEAT_BACKUP_COUNT and not scan_info["rear_blocked"]:
                enter_state(State.BACKUP, BACKUP_SECONDS)
                set_motion("BACKWARD", force=True)
                return "AVOID_SOFT_BACKUP", front_gap

            enter_state(State.AVOID, TURN_PULSE_SECONDS)
            set_motion(avoid_turn, force=True)
            return "AVOID_MORE", front_gap

        set_motion(avoid_turn)
        return "AVOIDING", front_gap

    if state == State.BACKUP:
        if state_until and now >= state_until:
            if pending_escape_turn:
                avoid_turn = pending_escape_turn
                resume_after = pending_escape_resume_state
                pending_escape_turn = None
                pending_escape_resume_state = None
                return begin_committed_escape(avoid_turn, resume_after), front_gap

            if (scan_info["front_blocked"] or scan_info["danger"]) and both_sides_tight(scan_info):
                if not scan_info["rear_blocked"]:
                    enter_state(State.BACKUP, TRAPPED_BACKUP_SECONDS)
                    set_motion("BACKWARD", force=True)
                    return "TRAPPED_BACKUP_MORE", front_gap
                return begin_committed_escape(avoid_turn), front_gap

            if resume_state == State.RETURN_TO_BEST:
                enter_state(State.RETURN_TO_BEST)
                set_motion("STOP", force=True)
                return_until = 0.0
                resume_state = None
                return "BACKUP_RETURN_RESUME", front_gap

            avoid_turn = safe_turn_from_scan(avoid_turn, scan_info)
            return begin_committed_escape(avoid_turn), front_gap

        set_motion("BACKWARD")
        return "BACKING", front_gap

    if state == State.ESCAPE:
        if committed_escape_turn:
            if turn_side_blocked(committed_escape_turn, scan_info) and not scan_info["rear_blocked"]:
                label = retry_committed_escape_after_backup(now, scan_info, "ESCAPE_SIDE_BLOCK")
                if label:
                    return label, front_gap

            if scan_info["emergency"] and not scan_info["rear_blocked"]:
                label = retry_committed_escape_after_backup(now, scan_info, "ESCAPE_EMERGENCY")
                if label:
                    return label, front_gap

            reached = committed_escape_reached()
            timed_out = bool(state_until and now >= state_until)

            # ESCAPE_TURN_DEGREES is a MINIMUM committed turn, not an
            # immediate switch point.  Once the minimum angle is reached,
            # keep rotating in the SAME direction while the front is still
            # blocked.  This prevents LEFT -> BACKUP -> RIGHT oscillation
            # when the chassis needs a little more than the nominal angle
            # to clear a wall/corner.
            if reached and not (scan_info["danger"] or scan_info["front_blocked"]):
                next_state = resume_after_escape(now)
                return f"ESCAPE_DONE_{next_state.value}", front_gap

            if timed_out:
                if not (scan_info["danger"] or scan_info["front_blocked"]):
                    next_state = resume_after_escape(now)
                    return f"ESCAPE_DONE_{next_state.value}", front_gap

                # At the hard timeout, back up and retry the SAME committed
                # direction once.  Only the following failure may switch to
                # the opposite direction via retry_committed_escape_after_backup().
                label = retry_committed_escape_after_backup(now, scan_info, "ESCAPE_TIMEOUT")
                if label:
                    return label, front_gap

            set_motion(committed_escape_turn)
            phase = "MIN_REACHED_KEEP" if reached else "TO"
            return f"ESCAPING_{phase}_{committed_escape_turn}_{ESCAPE_TURN_DEGREES:.0f}DEG", front_gap

        if state_until and now >= state_until:
            resume_explore_after_escape(now, avoid_turn)
            return "ESCAPE_DONE", front_gap

        set_motion(avoid_turn)
        return "ESCAPING", front_gap

    set_motion("STOP")
    return "UNKNOWN", front_gap


def print_status(label, scan_info, front_gap):
    nearest = scan_info["nearest"]
    if nearest:
        nearest_text = f"x={nearest[2]:.0f} y={nearest[3]:.0f} gap={front_gap:.0f}"
    else:
        nearest_text = "front=clear"
    flags = []
    if scan_info["front_blocked"]:
        flags.append("front_blocked")
    if scan_info["danger"]:
        flags.append("danger")
    if scan_info["emergency"]:
        flags.append("emergency")
    if scan_info["rear_blocked"]:
        flags.append("rear_blocked")
    flags_text = "flags=" + ",".join(flags or ["clear"])

    elapsed = elapsed_from_start(time.monotonic())
    best_elapsed = best_lux_time - explore_started if explore_started > 0 else 0.0
    lux_error = lux_error_from_target(current_lux)
    print(
        f"FSM={state.value} action={current_motion} {label} "
        f"t={elapsed:.1f}/{EXPLORE_SECONDS:.0f}s "
        f"LUX={current_lux:.1f} SOIL={current_soil:.1f}% "
        f"TARGET={TARGET_LUX_MIN:.0f}-{TARGET_LUX_MAX:.0f} "
        f"BEST={best_lux:.1f}@{best_elapsed:.1f}s ERR={lux_error:.1f} "
        f"POSE=({pose_x:.0f},{pose_y:.0f}) {heading_label()} "
        f"BEST_COORD=({best_x:.0f},{best_y:.0f}) BLOCKED={len(blocked_map)} "
        f"{nearest_text} {flags_text} "
        f"L={scan_info['left_score']:.0f} R={scan_info['right_score']:.0f} "
        f"front_points={len(scan_info['front'])}"
    )


def publish_runtime_state(scan_info, label, now):
    global last_runtime_write
    if now - last_runtime_write < 0.25:
        return
    last_runtime_write = now
    phase, remaining = runtime_phase(now)
    write_runtime_state(
        state != State.IDLE,
        state.value,
        current_motion,
        phase=phase,
        detail_state=detail_state_label(),
        label=label,
        remaining_seconds=round(remaining, 1) if remaining is not None else None,
        current_lux=round(current_lux, 1),
        best_lux=round(best_lux, 1) if best_lux >= 0 else None,
        target_lux_min=TARGET_LUX_MIN,
        target_lux_max=TARGET_LUX_MAX,
        pose_x=round(pose_x, 1),
        pose_y=round(pose_y, 1),
        heading=heading_label(),
        best_x=round(best_x, 1),
        best_y=round(best_y, 1),
        blocked_count=len(blocked_map),
        obstacle=obstacle_label(scan_info),
    )


def close_lidar_device(device):
    if device is None:
        return
    try:
        device.stop()
    except Exception:
        pass
    try:
        device.stop_motor()
    except Exception:
        pass
    try:
        device.disconnect()
    except Exception:
        pass


def open_lidar_device():
    last_error = None
    for attempt in range(1, 4):
        device = None
        try:
            print(f"LIDAR OPEN attempt={attempt} port={PORT}")
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
            print(f"LIDAR OPEN failed attempt={attempt}: {exc}")
            close_lidar_device(device)
            time.sleep(1.2)
    raise RuntimeError(f"LIDAR OPEN failed: {last_error}")


def handle_stop_signal(_signum, _frame):
    raise KeyboardInterrupt


signal.signal(signal.SIGTERM, handle_stop_signal)


try:
    print("=" * 50)
    print("FSM lidar drive + light exploration")
    print(f"DRIVE_BUILD={DRIVE_BUILD}")
    print("Remote key 1: start, key 2: stop")
    print(
        f"USE_DUMMY_LUX={USE_DUMMY_LUX} USE_DUMMY_SOIL={USE_DUMMY_SOIL} "
        f"AUTO_START={AUTO_START} DISABLE_IR={DISABLE_IR} "
        f"USE_SENSOR_CACHE={USE_SENSOR_CACHE} BH1750_BUS={BH1750_BUS}"
    )
    print(f"TARGET_LUX_RANGE={TARGET_LUX_MIN:.0f}-{TARGET_LUX_MAX:.0f} center={TARGET_LUX:.0f}")
    print(
        f"EXPLORE_BRANCH enabled={EXPLORE_BRANCH_ENABLED} "
        f"delay={EXPLORE_BRANCH_DELAY_SECONDS:.1f}s "
        f"random_interval={EXPLORE_BRANCH_MIN_INTERVAL:.1f}-{EXPLORE_BRANCH_MAX_INTERVAL:.1f}s "
        f"turn={EXPLORE_BRANCH_SECONDS:.2f}s open={EXPLORE_BRANCH_OPEN_MM:.0f}mm "
        f"edge_bias_x={EXPLORE_EDGE_BIAS_X:.0f}mm "
        f"post_escape_forward={EXPLORE_POST_ESCAPE_FORWARD_SECONDS:.1f}s"
    )
    print(
        f"RETURN coord_target enabled max={RETURN_MAX_SECONDS:.0f}s "
        f"tol={RETURN_HEADING_TOLERANCE_DEGREES:.0f}deg "
        f"detour={RETURN_DETOUR_DISTANCE_MM:.0f}mm/{RETURN_DETOUR_MAX_SECONDS:.1f}s"
    )
    print(
        f"DRIVE_SPEED forward={getattr(controller, 'forward_speed', 'legacy')} "
        f"turn={getattr(controller, 'turn_speed', 'legacy')} "
        f"backward={getattr(controller, 'backward_speed', 'legacy')}"
    )
    print(
        f"POSE_CAL linear={POSE_LINEAR_MM_PER_SEC:.0f}mm/s "
        f"turn={POSE_TURN_DEG_PER_SEC:.0f}deg/s max_dt={POSE_UPDATE_MAX_DT:.2f}s "
        f"avoid_pulse={TURN_PULSE_SECONDS:.2f}s escape_fallback={ESCAPE_TURN_SECONDS:.2f}s"
    )
    print(
        f"ESCAPE committed_turn={ESCAPE_TURN_DEGREES:.0f}deg(min) "
        f"max={ESCAPE_TURN_MAX_SECONDS:.2f}s backup={DEADEND_BACKUP_SECONDS:.2f}s "
        f"dir_lock={ESCAPE_DIRECTION_LOCK_SECONDS:.1f}s "
        f"corner_backup_side<={CORNER_BACKUP_SIDE_SCORE_MM:.0f}mm"
    )
    print(
        f"DOOR_GUARD enabled={DOOR_GUARD_ENABLED} "
        f"x<=+/-{DOOR_HALF_WIDTH_MM:.0f}mm y<={DOOR_EXIT_Y_MM:.0f}mm"
    )
    print(f"LIDAR_POST={LIDAR_POST_ENABLED} url={ONPLANT_SERVER_URL} robot={ONPLANT_ROBOT_ID}")
    print("=" * 50)

    start_lidar_post_worker()

    init_lux_sensor()
    time.sleep(0.2)
    first_lux = read_lux_fast()
    if first_lux >= 0:
        current_lux = first_lux
        best_lux = first_lux
        best_lux_error = lux_error_from_target(first_lux)
    current_soil = read_soil_fast()
    print(f"Initial light: {current_lux:.2f} lux, soil: {current_soil:.1f}%")

    lidar = open_lidar_device()
    lidar_retry_count = 0

    while True:
        try:
            for scan in lidar.iter_scans(max_buf_meas=LIDAR_MAX_BUF_MEAS):
                lidar_retry_count = 0
                now = time.monotonic()

                if now - last_key_read >= KEY_READ_INTERVAL:
                    key = read_ir_key()
                    last_key_read = now
                else:
                    key = 0

                if (key == START_KEY or auto_start_pending) and state == State.IDLE:
                    auto_start_pending = False
                    explore_started = now
                    dummy_sensor_started = now
                    explore_last_nudge = now
                    explore_next_branch_at = 0.0
                    explore_hold_forward_until = 0.0
                    seek_started = 0.0
                    return_until = 0.0
                    resume_state = None
                    pending_escape_turn = None
                    pending_escape_resume_state = None
                    last_escape_done_turn = None
                    last_escape_done_time = 0.0
                    last_explore_branch_turn = None
                    schedule_next_explore_branch(now, initial=True)
                    clear_committed_escape()
                    motion_log = []
                    active_motion = "STOP"
                    active_motion_started = now
                    reset_pose(now)
                    best_lux = current_lux
                    best_lux_error = lux_error_from_target(current_lux)
                    best_lux_time = now
                    best_motion_time = now
                    best_x = pose_x
                    best_y = pose_y
                    enter_state(State.EXPLORE)
                    set_motion("FORWARD", force=True)
                    start_learning_session(now)
                    print("AUTO START: EXPLORE")
                elif key == STOP_KEY and state != State.IDLE:
                    finalize_motion_log(now)
                    finish_learning_session("MANUAL_STOP", now)
                    pending_escape_turn = None
                    pending_escape_resume_state = None
                    last_escape_done_turn = None
                    last_escape_done_time = 0.0
                    explore_hold_forward_until = 0.0
                    clear_committed_escape()
                    enter_state(State.IDLE)
                    set_motion("STOP", force=True)
                    print("AUTO STOP")

                update_pose(now)
                update_light(now)
                update_soil(now)
                scan_info = analyze_scan(scan)
                remember_blocked(scan_info)
                label, front_gap = fsm_step(scan_info, now)
                maybe_post_lidar(scan_info, now)
                publish_runtime_state(scan_info, label, now)
                log_learning_sample(scan_info, label, now)

                if now - last_print >= PRINT_INTERVAL:
                    print_status(label, scan_info, front_gap)
                    last_print = now
                if search_finished:
                    break
            if search_finished:
                break
        except SerialException as exc:
            lidar_retry_count += 1
            print(f"LIDAR READ ERROR retry={lidar_retry_count}: {exc}")
            set_motion("STOP", force=True)
            if lidar_retry_count > 3:
                raise
            close_lidar_device(lidar)
            time.sleep(1.5)
            lidar = open_lidar_device()

except KeyboardInterrupt:
    print("\nInterrupted")

finally:
    set_motion("STOP", force=True)
    finish_learning_session("PROCESS_EXIT", time.monotonic())
    write_runtime_state(False, "IDLE", "STOP", phase="대기", remaining_seconds=None)
    if not DISABLE_IR:
        bot.Ctrl_IR_Switch(0)

    close_lidar_device(lidar)

    print("Finished")
