class Servo:
    def __init__(self, maestro, channel, min_val=2000, max_val=8000, center_val=5000):
        self.maestro = maestro
        self.channel = channel
        self.min = min_val
        self.max = max_val
        self.center_val = center_val

        # enforce in maestro.py (expects quarter-microseconds style numbers)
        maestro.setRange(channel, self.min, self.max)
        print(f"[INIT] Servo ch{self.channel} range=({self.min},{self.max}) center={self.center_val}")

    def move(self, value):
        raw = value
        if value < self.min:
            value = self.min
        if value > self.max:
            value = self.max

        note = "" if raw == value else f" (clamped from {raw})"
        print(f"[SERVO] ch{self.channel} -> {value}{note}")
        self.maestro.setTarget(self.channel, value)

    def center(self):
        print(f"[SERVO] ch{self.channel} CENTER -> {self.center_val}")
        self.move(self.center_val)
