from motor import Motor
from servo import Servo

class Robot:
    def __init__(self, maestro):
        print("[INIT] Robot initializing")

        # Wheels
        # LEFT wheel moves robot forward when value > 6000
        # RIGHT wheel moves robot forward when value < 6000
        self.left_wheel  = Motor(
            maestro,
            channel=0,
            neutral=6000,
            forward_sign=-1
        )
        self.right_wheel = Motor(
            maestro,
            channel=1,
            neutral=6000,
            forward_sign=+1
        )

        # Head / torso
        self.head_pan  = Servo(maestro, 2)
        self.waist     = Servo(maestro, 3)
        self.head_tilt = Servo(maestro, 4)

        print("[INIT] Robot ready")

    # -------- Drive --------

    def stop(self):
        print("[ROBOT] STOP")
        self.left_wheel.stop_motor()
        self.right_wheel.stop_motor()

    def drive_forward(self, speed=800):
        print(f"[ROBOT] DRIVE FORWARD speed={speed}")
        self.left_wheel.forward(speed)
        self.right_wheel.forward(speed)

    def drive_backward(self, speed=800):
        print(f"[ROBOT] DRIVE BACKWARD speed={speed}")
        self.left_wheel.backward(speed)
        self.right_wheel.backward(speed)

    def turn_left(self, speed=800):
        print(f"[ROBOT] TURN LEFT speed={speed}")
        self.left_wheel.backward(speed)
        self.right_wheel.forward(speed)

    def turn_right(self, speed=800):
        print(f"[ROBOT] TURN RIGHT speed={speed}")
        self.left_wheel.forward(speed)
        self.right_wheel.backward(speed)

    # -------- Head --------

    def look_left(self):
        self.head_pan.move(4000)

    def look_right(self):
        self.head_pan.move(6000)

    def center_head(self):
        self.head_pan.center_servo()
