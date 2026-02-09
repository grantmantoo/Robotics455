import time

class Motor:
    def __init__(self, maestro, channel, neutral=5000, max_delta=1500):
        self.maestro = maestro
        self.channel = channel
        self.neutral = neutral
        self.max_delta = max_delta
        print(f"[INIT] Motor ch{self.channel} neutral={self.neutral} max_delta={self.max_delta}")

        # IMPORTANT: arm/start at neutral
        self.arm()

    def arm(self):
        print(f"[MOTOR] ch{self.channel} ARM -> {self.neutral}")
        self.maestro.setTarget(self.channel, self.neutral)
        time.sleep(0.2)

    def set(self, value):
        # clamp around neutral
        lo = self.neutral - self.max_delta
        hi = self.neutral + self.max_delta
        raw = value
        if value < lo:
            value = lo
        if value > hi:
            value = hi

        direction = "STOP" if value == self.neutral else ("FWD" if value > self.neutral else "REV")
        note = "" if raw == value else f" (clamped from {raw})"
        print(f"[MOTOR] ch{self.channel} {direction} -> {value}{note}")
        self.maestro.setTarget(self.channel, value)

    def forward(self, delta=200):
        self.set(self.neutral + abs(delta))

    def backward(self, delta=200):
        self.set(self.neutral - abs(delta))

    def stop_motor(self):
        self.set(self.neutral)
