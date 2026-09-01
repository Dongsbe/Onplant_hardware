from enum import Enum


class LightSeekState(str, Enum):
    IDLE = "IDLE"
    SEARCH = "SEARCH"
    MOVE_TO_BEST = "MOVE_TO_BEST"
    STAY = "STAY"
    RESEARCH = "RESEARCH"


class LightSeekFSM:
    MIN_POS = 0
    MAX_POS = 10
    MAX_STEP = 12

    STAY_CHECK_LIMIT = 5
    LIGHT_DROP_THRESHOLD = 120
    RETURN_LIGHT_TOLERANCE = 150

    def __init__(self, light_sensor):
        self.light_sensor = light_sensor

        self.state = LightSeekState.IDLE

        self.current_light = 0
        self.best_light = -1

        self.current_pos = 5
        self.best_pos = 5

        self.step = 0
        self.search_direction = 1

        self.stay_count = 0

    def tick(self):
        self.current_light = self.light_sensor.read()

        if self.state == LightSeekState.IDLE:
            self.start_search()

        elif self.state == LightSeekState.SEARCH:
            self.search()

        elif self.state == LightSeekState.MOVE_TO_BEST:
            self.move_to_best()

        elif self.state == LightSeekState.STAY:
            self.stay()

        elif self.state == LightSeekState.RESEARCH:
            self.start_search()

    def start_search(self):
        self.state = LightSeekState.SEARCH
        self.best_light = -1
        self.best_pos = self.current_pos
        self.step = 0
        self.stay_count = 0
        self.search_direction = 1

        print("[FSM] IDLE/RESEARCH -> SEARCH")

    def search(self):
        if self.current_light > self.best_light:
            self.best_light = self.current_light
            self.best_pos = self.current_pos

        direction = self.get_search_direction()
        self.move_virtual(direction)

        print(
            f"[SEARCH] step={self.step}, "
            f"pos={self.current_pos}, "
            f"light={self.current_light}, "
            f"best={self.best_light}, "
            f"best_pos={self.best_pos}, "
            f"direction={direction}"
        )

        self.step += 1

        if self.step >= self.MAX_STEP:
            self.state = LightSeekState.MOVE_TO_BEST
            print("[FSM] SEARCH -> MOVE_TO_BEST")

    def move_to_best(self):
        if self.current_pos < self.best_pos:
            direction = "forward"
            self.move_virtual(direction)

        elif self.current_pos > self.best_pos:
            direction = "backward"
            self.move_virtual(direction)

        else:
            direction = "stop"

            print(
                f"[CHECK_BEST_POS] pos={self.current_pos}, "
                f"light={self.current_light}, "
                f"best={self.best_light}"
            )

            if self.current_light + self.RETURN_LIGHT_TOLERANCE >= self.best_light:
                self.state = LightSeekState.STAY
                self.stay_count = 0
                print("[FSM] MOVE_TO_BEST -> STAY")
            else:
                self.state = LightSeekState.RESEARCH
                print("[FSM] best_pos light check failed -> RESEARCH")

            return

        print(
            f"[MOVE_TO_BEST] pos={self.current_pos}, "
            f"target={self.best_pos}, "
            f"light={self.current_light}, "
            f"best={self.best_light}, "
            f"direction={direction}"
        )

    def stay(self):
        print(
            f"[STAY] count={self.stay_count}, "
            f"pos={self.current_pos}, "
            f"light={self.current_light}, "
            f"best={self.best_light}, "
            f"direction=stop"
        )

        if self.current_light + self.LIGHT_DROP_THRESHOLD < self.best_light:
            self.state = LightSeekState.RESEARCH
            print("[FSM] STAY light dropped -> RESEARCH")
            return

        self.stay_count += 1

        if self.stay_count >= self.STAY_CHECK_LIMIT:
            self.state = LightSeekState.RESEARCH
            print("[FSM] STAY recheck limit -> RESEARCH")

    def get_search_direction(self):
        if self.current_pos >= self.MAX_POS:
            self.search_direction = -1

        elif self.current_pos <= self.MIN_POS:
            self.search_direction = 1

        if self.search_direction == 1:
            return "forward"

        return "backward"

    def move_virtual(self, direction):
        if direction == "forward":
            self.current_pos += 1

        elif direction == "backward":
            self.current_pos -= 1

        if self.current_pos > self.MAX_POS:
            self.current_pos = self.MAX_POS

        if self.current_pos < self.MIN_POS:
            self.current_pos = self.MIN_POS