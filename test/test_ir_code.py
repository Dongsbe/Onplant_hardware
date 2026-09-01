import time
from Raspbot_Lib import Raspbot

bot = Raspbot()
bot.Ctrl_IR_Switch(1)

IGNORE_KEYS = [0, 65]

print("IR KEY TEST")
print("1~9번 버튼 눌러봐")
print("0, 65는 무시")
print("Ctrl+C 종료")

last_key = None

try:
    while True:
        data = bot.read_data_array(0x0c, 1)
        key = data[0]

        if key not in IGNORE_KEYS and key != last_key:
            print("KEY:", key)
            last_key = key

        if key in IGNORE_KEYS:
            last_key = None

        time.sleep(0.08)

except KeyboardInterrupt:
    bot.Ctrl_IR_Switch(0)
    print("종료")