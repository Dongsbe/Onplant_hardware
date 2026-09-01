class FakeLightSensor:
    def __init__(self):
        self.values = [
            300,
            420,
            580,
            760,
            690,
            520,
            430,
            350,
            480,
            620,
            810,
            740,
        ]
        self.index = 0

    def read(self):
        value = self.values[self.index % len(self.values)]
        self.index += 1
        return value