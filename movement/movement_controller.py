from hardware.raspbot_driver import RaspbotDriver


class MovementController:
    def __init__(self):
        self.driver = RaspbotDriver()
        self.speed = 20

    def forward(self):
        self.driver.move_forward(self.speed)

    def backward(self):
        self.driver.move_backward(self.speed)

    def rotate_left(self):
        self.driver.rotate_left(self.speed)

    def rotate_right(self):
        self.driver.rotate_right(self.speed)

    def stop(self):
        self.driver.stop()