import time
import signal
import sys
from Raspbot_Lib import Raspbot

bot = Raspbot()

SPEED = 25
RUN_SEC = 0.25
MOTOR_IDS = [1, 2, 3, 4]


def stop_all():
    for motor_id in MOTOR_IDS:
        try:
            bot.Ctrl_Car(motor_id, 0, 0)
        except Exception as e:
            print(f"stop error motor={motor_id}: {e}")


def safe_exit(*args):
    print("\nSAFE STOP")
    stop_all()
    time.sleep(0.1)
    stop_all()
    sys.exit(0)


signal.signal(signal.SIGINT, safe_exit)
signal.signal(signal.SIGTERM, safe_exit)

try:
    stop_all()
    time.sleep(0.5)

    for motor_id in MOTOR_IDS:
        for direction in [0, 1]:
            input(f"\nEnter 누르면 motor={motor_id}, dir={direction} 테스트")
            print(f"RUN motor={motor_id} dir={direction}")
            bot.Ctrl_Car(motor_id, direction, SPEED)
            time.sleep(RUN_SEC)
            stop_all()
            time.sleep(0.5)

finally:
    print("FINAL STOP")
    stop_all()
    time.sleep(0.1)
    stop_all()