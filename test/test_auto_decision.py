import time
from rplidar import RPLidar
from Raspbot_Lib import Raspbot

PORT = "/dev/ttyUSB0"

START_KEY = 16   # 숫자 1
STOP_KEY = 17    # 숫자 2

IGNORE_KEYS = [0, 65, 255]

MIN_VALID = 50
OBSTACLE = 120
MAX_VALID = 2000

PRINT_INTERVAL = 0.2

lidar = RPLidar(PORT)
bot = Raspbot()

bot.Ctrl_IR_Switch(1)


def read_ir_key():
    for _ in range(3):
        key = bot.read_data_array(0x0c, 1)[0]

        if key not in IGNORE_KEYS:
            return key

        time.sleep(0.01)

    return 0


def get_zone(angle):
    if angle >= 330 or angle <= 30:
        return "front"
    elif 30 < angle <= 135:
        return "left"
    elif 225 <= angle < 330:
        return "right"
    return None


def safe_distance(values):
    if not values:
        return 9999

    values = sorted(values)
    close_values = values[:3]

    return sum(close_values) / len(close_values)


def get_distances(scan):
    zones = {
        "front": [],
        "left": [],
        "right": []
    }

    for quality, angle, distance in scan:
        if distance < MIN_VALID or distance > MAX_VALID:
            continue

        zone = get_zone(angle)

        if zone:
            zones[zone].append(distance)

    front = safe_distance(zones["front"])
    left = safe_distance(zones["left"])
    right = safe_distance(zones["right"])

    return front, left, right


def decide(front, left, right):
    if front <= OBSTACLE:
        if left == 9999 and right != 9999:
            return "RIGHT"

        if right == 9999 and left != 9999:
            return "LEFT"

        if left > right:
            return "LEFT"

        return "RIGHT"

    return "FORWARD"


try:
    print("=" * 50)
    print("자율주행 판단 테스트")
    print("1번(16): 시작")
    print("2번(17): 정지")
    print("모터는 아직 안 움직임")
    print("=" * 50)

    lidar.start_motor()
    time.sleep(2)

    running = False
    last_print_time = 0

    for scan in lidar.iter_scans():
        key = read_ir_key()

        if key == START_KEY:
            running = True
            print("AUTO START")

        elif key == STOP_KEY:
            running = False
            print("AUTO STOP")

        if running:
            front, left, right = get_distances(scan)
            action = decide(front, left, right)

            now = time.time()

            if now - last_print_time >= PRINT_INTERVAL:
                print(
                    f"FRONT:{front:.0f} "
                    f"LEFT:{left:.0f} "
                    f"RIGHT:{right:.0f} "
                    f"=> {action}"
                )
                last_print_time = now

except KeyboardInterrupt:
    print("\n강제 종료")

finally:
    bot.Ctrl_IR_Switch(0)

    try:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
    except:
        pass

    print("종료")