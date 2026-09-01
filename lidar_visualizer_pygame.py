import math
import time

import pygame
from rplidar import RPLidar

PORT = "/dev/ttyUSB0"

MIN_VALID = 50
MAX_VALID = 2000

LIDAR_TO_FRONT_AXLE = 150
LIDAR_TO_LEFT_OUTER = 76
LIDAR_TO_RIGHT_OUTER = 80
SAFETY_MARGIN = 20

LEFT_CLEARANCE = LIDAR_TO_LEFT_OUTER + SAFETY_MARGIN
RIGHT_CLEARANCE = LIDAR_TO_RIGHT_OUTER + SAFETY_MARGIN
FRONT_DANGER = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN
FRONT_WARN = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + 190
THIN_SINGLE_POINT_LIMIT = LIDAR_TO_FRONT_AXLE + SAFETY_MARGIN + 160

SCREEN_W = 800
SCREEN_H = 480
SCALE = 0.22


def angle_to_xy(angle, distance):
    angle = angle % 360
    if angle > 180:
        angle -= 360

    rad = math.radians(angle)
    x = distance * math.cos(rad)
    y = distance * math.sin(rad)
    return x, y


def world_to_screen(x, y):
    origin_x = SCREEN_W // 2
    origin_y = SCREEN_H - 70
    sx = int(origin_x + y * SCALE)
    sy = int(origin_y - x * SCALE)
    return sx, sy


def in_front_lane(x, y):
    return x > 0 and -RIGHT_CLEARANCE <= y <= LEFT_CLEARANCE


def draw_text(screen, font, text, x, y, color=(240, 240, 240)):
    screen.blit(font.render(text, True, color), (x, y))


def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("RPLidar Viewer")
    font = pygame.font.SysFont("monospace", 18)
    clock = pygame.time.Clock()

    lidar = RPLidar(PORT)
    running = True
    latest_scan = []
    last_status_time = 0
    status = "starting"

    try:
        lidar.stop()
        lidar.clean_input()
        time.sleep(0.5)
        lidar.start_motor()
        time.sleep(1.5)

        for scan in lidar.iter_scans(max_buf_meas=1000):
            latest_scan = scan

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False

            if not running:
                break

            screen.fill((15, 15, 18))

            origin = world_to_screen(0, 0)
            pygame.draw.circle(screen, (255, 255, 255), origin, 5)

            # Robot body clearance box, measured from lidar center.
            front_left = world_to_screen(FRONT_DANGER, LEFT_CLEARANCE)
            rear_right = world_to_screen(-80, -RIGHT_CLEARANCE)
            rect = pygame.Rect(
                rear_right[0],
                front_left[1],
                front_left[0] - rear_right[0],
                rear_right[1] - front_left[1],
            )
            pygame.draw.rect(screen, (80, 80, 90), rect, 2)

            # Front danger and warning lines.
            pygame.draw.line(
                screen,
                (255, 80, 80),
                world_to_screen(FRONT_DANGER, -RIGHT_CLEARANCE),
                world_to_screen(FRONT_DANGER, LEFT_CLEARANCE),
                3,
            )
            pygame.draw.line(
                screen,
                (255, 180, 50),
                world_to_screen(FRONT_WARN, -RIGHT_CLEARANCE),
                world_to_screen(FRONT_WARN, LEFT_CLEARANCE),
                2,
            )

            front_points = []
            thin_points = []

            for quality, angle, distance in latest_scan:
                if distance < MIN_VALID or distance > MAX_VALID:
                    continue

                x, y = angle_to_xy(angle, distance)
                sx, sy = world_to_screen(x, y)

                color = (90, 180, 255)

                if in_front_lane(x, y):
                    if x <= FRONT_DANGER:
                        color = (255, 40, 40)
                    elif x <= FRONT_WARN:
                        color = (255, 170, 40)
                    front_points.append((x, y, distance))

                if x > 0 and -LEFT_CLEARANCE <= y <= RIGHT_CLEARANCE and x <= THIN_SINGLE_POINT_LIMIT:
                    color = (255, 60, 180)
                    thin_points.append((x, y, distance))

                if 0 <= sx < SCREEN_W and 0 <= sy < SCREEN_H:
                    pygame.draw.circle(screen, color, (sx, sy), 2)

            nearest = min(front_points, key=lambda p: p[0], default=None)
            if nearest:
                front_gap = nearest[0] - LIDAR_TO_FRONT_AXLE
                status = (
                    f"front x={nearest[0]:.0f}mm "
                    f"gap={front_gap:.0f}mm "
                    f"y={nearest[1]:.0f}mm "
                    f"front_points={len(front_points)}"
                )
            elif time.time() - last_status_time > 0.2:
                status = f"clear front_points=0 thin_points={len(thin_points)}"
                last_status_time = time.time()

            draw_text(screen, font, "RPLidar viewer  q/esc: quit", 12, 10)
            draw_text(screen, font, status, 12, 34)
            draw_text(screen, font, "white dot=lidar, gray box=robot clearance", 12, SCREEN_H - 52)
            draw_text(screen, font, "red=danger, orange=warn, pink=thin/near, blue=other", 12, SCREEN_H - 28)

            pygame.display.flip()
            clock.tick(30)

    finally:
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except Exception:
            pass
        pygame.quit()


if __name__ == "__main__":
    main()
