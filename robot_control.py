from maestro import Controller
from robot import Robot

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class RobotControl:
    """
    Flask-independent control layer.
    Exposes required functions and enforces safe limits + STOP.
    """

    def __init__(self, port="/dev/ttyACM0", device=0x0C):
        self.maestro = Controller(port, device=device)
        self.robot = Robot(self.maestro)

        # ---- SAFE LIMITS (tune as needed) ----
        # Servo values are on your 2000..8000 scale, center 5000
        self.HEAD_PAN_MIN = 2000
        self.HEAD_PAN_MAX = 8000
        self.HEAD_TILT_MIN = 2000
        self.HEAD_TILT_MAX = 8000
        self.WAIST_MIN = 2000
        self.WAIST_MAX = 8000

        # Drive “speed” is delta from 6000; you said >= 800 moves
        self.DRIVE_MIN = 800
        self.DRIVE_MAX = 1600  # safety cap

        self.stop()  # start safe

    # -------------------------
    # STOP / neutral
    # -------------------------
    def stop(self):
        print("[CTRL] STOP/NEUTRAL")
        self.robot.stop()

    # -------------------------
    # Driving
    # -------------------------
    def drive(self, left_speed, right_speed):
        """
        left_speed/right_speed are deltas from neutral (6000).
        Positive means "robot forward" for that wheel, negative means backward.
        Example: +800 is minimum motion.
        """
        left_speed = int(clamp(left_speed, -self.DRIVE_MAX, self.DRIVE_MAX))
        right_speed = int(clamp(right_speed, -self.DRIVE_MAX, self.DRIVE_MAX))

        # enforce deadband: if nonzero magnitude < DRIVE_MIN, bump it up
        if left_speed != 0 and abs(left_speed) < self.DRIVE_MIN:
            left_speed = self.DRIVE_MIN if left_speed > 0 else -self.DRIVE_MIN
        if right_speed != 0 and abs(right_speed) < self.DRIVE_MIN:
            right_speed = self.DRIVE_MIN if right_speed > 0 else -self.DRIVE_MIN

        print(f"[CTRL] drive left={left_speed} right={right_speed}")

        # Robot methods accept "speed" as positive magnitude, so we route signs here
        if left_speed == 0:
            self.robot.left_wheel.stop_motor()
        elif left_speed > 0:
            self.robot.left_wheel.forward(abs(left_speed))
        else:
            self.robot.left_wheel.backward(abs(left_speed))

        if right_speed == 0:
            self.robot.right_wheel.stop_motor()
        elif right_speed > 0:
            self.robot.right_wheel.forward(abs(right_speed))
        else:
            self.robot.right_wheel.backward(abs(right_speed))

    def forward(self, speed=800):
        print(f"[CTRL] forward speed={speed}")
        self.drive(speed, speed)

    def backward(self, speed=800):
        print(f"[CTRL] backward speed={speed}")
        self.drive(-speed, -speed)

    def turn_left(self, speed=800):
        print(f"[CTRL] turn_left speed={speed}")
        self.drive(-speed, speed)

    def turn_right(self, speed=800):
        print(f"[CTRL] turn_right speed={speed}")
        self.drive(speed, -speed)

    # -------------------------
    # Head + Waist
    # -------------------------
    def head_pan(self, value):
        value = int(clamp(value, self.HEAD_PAN_MIN, self.HEAD_PAN_MAX))
        print(f"[CTRL] head_pan -> {value}")
        self.robot.head_pan.move(value)

    def head_tilt(self, value):
        value = int(clamp(value, self.HEAD_TILT_MIN, self.HEAD_TILT_MAX))
        print(f"[CTRL] head_tilt -> {value}")
        self.robot.head_tilt.move(value)

    def waist(self, value):
        value = int(clamp(value, self.WAIST_MIN, self.WAIST_MAX))
        print(f"[CTRL] waist -> {value}")
        self.robot.waist.move(value)

    # -------------------------
    # Cleanup
    # -------------------------
    def close(self):
        print("[CTRL] close")
        self.stop()
        self.maestro.close()


# -------------------------
# Direct testing (no Flask)
# -------------------------
if __name__ == "__main__":
    import time

    ctrl = RobotControl()

    try:
        ctrl.forward(800)
        time.sleep(1.5)
        ctrl.stop()
        time.sleep(0.8)

        ctrl.backward(800)
        time.sleep(1.5)
        ctrl.stop()
        time.sleep(0.8)

        ctrl.turn_left(900)
        time.sleep(1.0)
        ctrl.stop()
        time.sleep(0.8)

        ctrl.turn_right(900)
        time.sleep(1.0)
        ctrl.stop()
        time.sleep(0.8)

        ctrl.head_pan(3000)
        time.sleep(0.8)
        ctrl.head_pan(7000)
        time.sleep(0.8)
        ctrl.head_pan(5000)
        time.sleep(0.8)

        ctrl.head_tilt(3000)
        time.sleep(0.8)
        ctrl.head_tilt(7000)
        time.sleep(0.8)
        ctrl.head_tilt(5000)
        time.sleep(0.8)

        ctrl.waist(3000)
        time.sleep(0.8)
        ctrl.waist(7000)
        time.sleep(0.8)
        ctrl.waist(5000)
        time.sleep(0.8)

        ctrl.stop()

    finally:
        ctrl.close()
