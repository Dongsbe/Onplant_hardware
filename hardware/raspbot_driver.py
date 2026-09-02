from Raspbot_Lib import Raspbot


class RaspbotDriver:
    MOTOR_IDS = [0, 1, 2, 3]

    def __init__(self):
        self.bot = Raspbot()

    def set_motor(self, motor_id, direction, speed):
        speed = max(0, min(255, int(speed)))
        self.bot.Ctrl_Car(motor_id, direction, speed)

    def move_forward(self, speed):
        for motor_id in self.MOTOR_IDS:
            self.set_motor(motor_id, 0, speed)

    def move_backward(self, speed):
        for motor_id in self.MOTOR_IDS:
            self.set_motor(motor_id, 1, speed)

    def rotate_left(self, speed):
        self.set_motor(0, 0, speed)
        self.set_motor(1, 0, speed)
        self.set_motor(2, 1, speed)
        self.set_motor(3, 1, speed)

    def rotate_right(self, speed):
        self.set_motor(0, 1, speed)
        self.set_motor(1, 1, speed)
        self.set_motor(2, 0, speed)
        self.set_motor(3, 0, speed)

    def stop(self):
        for motor_id in self.MOTOR_IDS:
            self.set_motor(motor_id, 0, 0)
