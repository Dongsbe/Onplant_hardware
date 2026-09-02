import os

from hardware.raspbot_driver import RaspbotDriver


class MovementController:
    def __init__(self):
        self.driver = RaspbotDriver()
        base_speed = max(
            self._env_speed("ONPLANT_DRIVE_SPEED", 26),
            self._env_speed("ONPLANT_DRIVE_MIN_SPEED", 24),
        )
        self.forward_speed = self._env_speed("ONPLANT_FORWARD_SPEED", base_speed)
        self.backward_speed = self._env_speed("ONPLANT_BACKWARD_SPEED", max(18, base_speed - 4))
        self.turn_speed = self._env_speed("ONPLANT_TURN_SPEED", max(20, base_speed - 2))

    def _env_speed(self, name, default):
        try:
            value = int(os.getenv(name, str(default)))
        except ValueError:
            value = int(default)
        return max(0, min(255, value))

    def forward(self):
        self.driver.move_forward(self.forward_speed)

    def backward(self):
        self.driver.move_backward(self.backward_speed)

    def rotate_left(self):
        self.driver.rotate_left(self.turn_speed)

    def rotate_right(self):
        self.driver.rotate_right(self.turn_speed)

    def stop(self):
        self.driver.stop()
