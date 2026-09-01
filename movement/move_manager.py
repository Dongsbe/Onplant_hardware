class MoveManager:
    def __init__(self, light_fsm):
        self.light_fsm = light_fsm

        self.emergency_stop = False
        self.manual_command = None

    def set_manual_command(self, direction):
        self.manual_command = direction

    def clear_manual_command(self):
        self.manual_command = None

    def request_emergency_stop(self):
        self.emergency_stop = True

    def clear_emergency_stop(self):
        self.emergency_stop = False

    def tick(self):
        if self.emergency_stop:
            print("[MOVE_MANAGER] priority=emergency_stop, direction=stop")
            return

        if self.manual_command is not None:
            print(
                f"[MOVE_MANAGER] priority=manual, "
                f"direction={self.manual_command}"
            )
            self.manual_command = None
            return

        # TODO: lidar obstacle avoidance
        obstacle_command = None
        if obstacle_command is not None:
            print(
                f"[MOVE_MANAGER] priority=obstacle, "
                f"direction={obstacle_command}"
            )
            return

        # TODO: LLM movement command
        llm_command = None
        if llm_command is not None:
            print(
                f"[MOVE_MANAGER] priority=llm, "
                f"direction={llm_command}"
            )
            return

        print("[MOVE_MANAGER] priority=light_seek")
        self.light_fsm.tick()