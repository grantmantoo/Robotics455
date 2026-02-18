import time

class Motor:
    def __init__(
        self,
        maestro,
        channel,
        neutral=6000,
        forward_sign=+1,
        min_delta=800,
        max_delta=1600,
        arm_time=0.3
    ):
        """
        forward_sign:
            +1 -> value ABOVE neutral moves robot forward
            1 -> value BELOW neutral moves robot forward
        """
        self.maestro = maestro
        self.channel = channel
        self.neutral = neutral
        self.forward_sign = +1 if forward_sign >= 0 else -1
        self.min_delta = min_delta
        self.max_delta = max_delta

        print(
            f"[INIT] Motor ch{self.channel} "
            f"neutral={self.neutral} "
            f"forward_sign={self.forward_sign} "
            f"min_delta={self.min_delta}"
        )

        # Arm / initialize (this STOPS your motors)
        print(f"[MOTOR] ch{self.channel} ARM/STOP -> {self.neutral}")
        self.maestro.setTarget(self.channel, self.neutral)
        time.sleep(arm_time)

    def _clamp_delta(self, delta):
        delta = abs(int(delta))
        if delta < self.min_delta:
            print(
                f"[MOTOR WARN] ch{self.channel} delta {delta} < "
                f"{self.min_delta}, raising"
            )
            delta = self.min_delta
        if delta > self.max_delta:
            print(
                f"[MOTOR WARN] ch{self.channel} delta {delta} > "
                f"{self.max_delta}, clamping"
            )
            delta = self.max_delta
        return delta

    def _send(self, value, label=""):
        direction = (
            "STOP" if value == self.neutral
            else ("HIGH(+)" if value > self.neutral else "LOW(-)")
        )
        print(f"[MOTOR] {label} ch{self.channel} {direction} -> {value}")
        self.maestro.setTarget(self.channel, value)

    def forward(self, speed=800):
        d = self._clamp_delta(speed)
        self._send(self.neutral + (self.forward_sign * d), "FORWARD")

    def backward(self, speed=800):
        d = self._clamp_delta(speed)
        self._send(self.neutral - (self.forward_sign * d), "BACKWARD")

    def stop_motor(self):
        self._send(self.neutral, "STOP")
