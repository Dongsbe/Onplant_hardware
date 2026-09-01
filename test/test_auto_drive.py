import time
from rplidar import RPLidar
from Raspbot_Lib import Raspbot
from movement.movement_controller import MovementController

PORT = "/dev/ttyUSB0"

START_KEY = 16
STOP_KEY = 17
IGNORE_KEYS = [0, 65, 255]

MIN_VALID = 50
MAX_VALID = 2000

FRONT_SAFE = 320
MIN_TURN_SPACE = 240

SECTOR_SIZE = 30
SECTOR_COUNT = 12

FORWARD_DURATION = 0.14
TURN_DURATION = 0.28
BACKWARD_DURATION = 0.35
UTURN_DURATION = 0.75
STOP_DURATION = 0.08

COMMAND_INTERVAL = 0.18
PRINT_INTERVAL = 0.3

STUCK_LIMIT = 4

lidar = RPLidar(PORT)
bot = Raspbot()
controller = MovementController()

bot.Ctrl_IR_Switch(1)

current_action = "STOP"
action_end_time = 0

last_action = None
stuck_count = 0
escape_mode = False
escape_step = 0


def read_ir_key():
    key = bot.read_data_array(0x0c, 1)[0]
    if key not in IGNORE_KEYS:
        return key
    return 0


def safe_distance(values):
    if not values:
        return 9999
    return min(values)


def get_sector_distances(scan):
    sectors = [[] for _ in range(SECTOR_COUNT)]

    for quality, angle, distance in scan:
        if distance < MIN_VALID or distance > MAX_VALID:
            continue

        idx = int(angle // SECTOR_SIZE)

        if 0 <= idx < SECTOR_COUNT:
            sectors[idx].append(distance)

    return [safe_distance(values) for values in sectors]


def get_best_direction(sectors):
    right_score = min(sectors[9], sectors[10])
    front_score = min(sectors[11], sectors[0], sectors[1])
    left_score = min(sectors[2], sectors[3])

    if front_score >= FRONT_SAFE:
        return "FORWARD", front_score, left_score, right_score

    if left_score < MIN_TURN_SPACE and right_score < MIN_TURN_SPACE:
        return "STOP", front_score, left_score, right_score

    if left_score > right_score and left_score >= MIN_TURN_SPACE:
        return "LEFT", front_score, left_score, right_score

    if right_score >= MIN_TURN_SPACE:
        return "RIGHT", front_score, left_score, right_score

    return "STOP", front_score, left_score, right_score


def start_action(action):
    global current_action, action_end_time

    now = time.time()
    current_action = action

    if action == "FORWARD":
        controller.forward()
        action_end_time = now + FORWARD_DURATION

    elif action == "LEFT":
        controller.rotate_left()
        action_end_time = now + TURN_DURATION

    elif action == "RIGHT":
        controller.rotate_right()
        action_end_time = now + TURN_DURATION

    elif action == "BACKWARD":
        controller.backward()
        action_end_time = now + BACKWARD_DURATION

    elif action == "UTURN":
        controller.rotate_left()
        action_end_time = now + UTURN_DURATION

    else:
        controller.stop()
        action_end_time = now + STOP_DURATION


def update_action():
    global current_action

    if current_action != "STOP" and time.time() >= action_end_time:
        controller.stop()
        current_action = "STOP"


def update_stuck(action):
    global last_action, stuck_count

    if action == "FORWARD":
        stuck_count = 0
        last_action = action
        return

    if action in ["LEFT", "RIGHT", "STOP"]:
        if last_action in ["LEFT", "RIGHT", "STOP"]:
            stuck_count += 1
        else:
            stuck_count = 1

    last_action = action


def start_escape():
    global escape_mode, escape_step, stuck_count

    escape_mode = True
    escape_step = 1
    stuck_count = 0

    print("ESCAPE START: 후진")


def run_escape():
    global escape_mode, escape_step

    if current_action != "STOP":
        return

    if escape_step == 1:
        start_action("BACKWARD")
        escape_step = 2
        print("ESCAPE: BACKWARD")

    elif escape_step == 2:
        start_action("UTURN")
        escape_step = 3
        print("ESCAPE: UTURN")

    elif escape_step == 3:
        start_action("STOP")
        escape_step = 0
        escape_mode = False
        print("ESCAPE END")


try:
    print("=" * 50)
    print("자율주행 테스트 - 헛돌이 탈출 FSM")
    print("1번: 시작")
    print("2번: 정지")
    print("=" * 50)

    lidar.start_motor()
    time.sleep(2)

    running = False
    last_command_time = 0
    last_print_time = 0

    for scan in lidar.iter_scans():
        now = time.time()

        key = read_ir_key()

        if key == START_KEY and not running:
            running = True
            print("AUTO START")

        elif key == STOP_KEY and running:
            running = False
            controller.stop()
            current_action = "STOP"
            print("AUTO STOP")

        update_action()

        if not running:
            controller.stop()
            continue

        if escape_mode:
            run_escape()
            continue

        sectors = get_sector_distances(scan)
        action, front_score, left_score, right_score = get_best_direction(sectors)

        update_stuck(action)

        if stuck_count >= STUCK_LIMIT:
            start_escape()
            continue

        if now - last_print_time >= PRINT_INTERVAL:
            print(
                f"FRONT:{front_score:.0f} "
                f"LEFT:{left_score:.0f} "
                f"RIGHT:{right_score:.0f} "
                f"=> {action} "
                f"STUCK:{stuck_count}"
            )
            last_print_time = now

        if current_action == "STOP" and now - last_command_time >= COMMAND_INTERVAL:
            start_action(action)
            last_command_time = now

except KeyboardInterrupt:
    print("\n강제 종료")

finally:
    controller.stop()
    bot.Ctrl_IR_Switch(0)

    try:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
    except:
        pass

    print("종료")