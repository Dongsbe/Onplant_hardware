import time
import smbus

from Raspbot_Lib import Raspbot

# 리모컨 키값
START_LIGHT_KEY = 20  # 리모컨 4번 예상
STOP_LIGHT_KEY = 21   # 리모컨 5번 예상
IGNORE_KEYS = [0, 65, 255]

# BH1750
BH1750_ADDR = 0x23
bus = smbus.SMBus(1)

bot = Raspbot()
bot.Ctrl_IR_Switch(1)

reading_light = False


def read_ir_key():
    key = bot.read_data_array(0x0c, 1)[0]
    if key not in IGNORE_KEYS:
        return key
    return 0


def read_lux():
    try:
        bus.write_byte(BH1750_ADDR, 0x10)
        time.sleep(0.18)
        data = bus.read_i2c_block_data(BH1750_ADDR, 0x00, 2)
        lux = ((data[0] << 8) | data[1]) / 1.2
        return lux
    except Exception as e:
        print("BH1750 ERROR:", e)
        return -1


try:
    print("=" * 40)
    print("리모컨 조도 테스트")
    print("4번: 조도값 받기 시작")
    print("5번: 조도값 받기 정지")
    print("=" * 40)

    last_print_time = 0

    while True:
        key = read_ir_key()

        if key != 0:
            print("IR KEY =", key)

        if key == START_LIGHT_KEY:
            reading_light = True
            print("조도 측정 시작")

        elif key == STOP_LIGHT_KEY:
            reading_light = False
            print("조도 측정 정지")

        if reading_light and time.time() - last_print_time >= 1.0:
            lux = read_lux()
            print(f"현재 조도: {lux:.2f} lux")
            last_print_time = time.time()

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n강제 종료")

finally:
    bot.Ctrl_IR_Switch(0)
    print("종료")