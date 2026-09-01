import time

from sensor.fake_light import FakeLightSensor
from movement.light_seek import LightSeekFSM


sensor = FakeLightSensor()
fsm = LightSeekFSM(sensor)

try:
    while True:
        fsm.tick()
        time.sleep(0.5)

except KeyboardInterrupt:
    print("FSM 테스트 종료")