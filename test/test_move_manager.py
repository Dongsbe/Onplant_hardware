import time

from sensor.fake_light import FakeLightSensor
from movement.light_seek import LightSeekFSM
from movement.move_manager import MoveManager


sensor = FakeLightSensor()
fsm = LightSeekFSM(sensor)
manager = MoveManager(fsm)

for i in range(15):
    if i == 3:
        manager.set_manual_command("right")

    if i == 8:
        manager.set_manual_command("stop")

    manager.tick()
    time.sleep(0.5)