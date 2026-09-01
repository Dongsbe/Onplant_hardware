from rplidar import RPLidar
import time

PORT = "/dev/ttyUSB0"

MIN_VALID = 50       # 5cm 이하는 노이즈
OBSTACLE = 100       # 10cm 이하면 장애물
MAX_VALID = 2000     # 테스트베드 기준 2m 이상 무시

lidar = RPLidar(PORT)


def get_zone(angle):
    if angle >= 330 or angle <= 30:
        return "front"
    elif 30 < angle <= 135:
        return "left"
    elif 225 <= angle < 330:
        return "right"
    return None


try:
    print("LIDAR START")
    print("Info:", lidar.get_info())
    print("Health:", lidar.get_health())

    lidar.start_motor()
    time.sleep(2)

    for scan in lidar.iter_scans():
        distances = {
            "front": [],
            "left": [],
            "right": []
        }

        for quality, angle, distance in scan:
            if distance < MIN_VALID or distance > MAX_VALID:
                continue

            zone = get_zone(angle)

            if zone:
                distances[zone].append(distance)

        front = min(distances["front"]) if distances["front"] else 9999
        left = min(distances["left"]) if distances["left"] else 9999
        right = min(distances["right"]) if distances["right"] else 9999

        front_blocked = front <= OBSTACLE
        left_blocked = left <= OBSTACLE
        right_blocked = right <= OBSTACLE

        print(
            f"FRONT: {front:.0f} mm | "
            f"LEFT: {left:.0f} mm | "
            f"RIGHT: {right:.0f} mm | "
            f"F:{front_blocked} L:{left_blocked} R:{right_blocked}"
        )

        time.sleep(0.1)

except KeyboardInterrupt:
    print("STOP")

finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()