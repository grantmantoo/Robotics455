from servo import Servo
from motor import Motor

class Robot:
    def __init__(self, maestro):
        print("[INIT] Robot initializing...")

        # Drive
        self.left_wheel  = Motor(maestro, 0)
        self.right_wheel = Motor(maestro, 1)

        # Head & torso
        self.head_pan    = Servo(maestro, 2)
        self.waist       = Servo(maestro, 3)
        self.head_tilt   = Servo(maestro, 4)

        # Right arm
        self.r_shoulder_ud = Servo(maestro, 5)
        self.r_shoulder_yaw = Servo(maestro, 6)
        self.r_elbow = Servo(maestro, 7)
        self.r_wrist_ud = Servo(maestro, 8)
        self.r_wrist_rot = Servo(maestro, 9)
        self.r_hand = Servo(maestro, 10)

        # Left arm
        self.l_wrist_rot = Servo(maestro, 11)
        self.l_shoulder_ud = Servo(maestro, 12)
        self.l_shoulder_yaw = Servo(maestro, 13)
        self.l_elbow = Servo(maestro, 14)
        self.l_wrist_ud = Servo(maestro, 15)
        self.l_hand = Servo(maestro, 16)

        print("[INIT] Robot ready.")

    # ---- Behaviors ----

    def stop(self):
        print("[ROBOT] STOP")
        self.left_wheel.stop_motor()
        self.right_wheel.stop_motor()

    def drive_forward(self, speed=100):
        print(f"[ROBOT] DRIVE FORWARD speed={speed}")
        self.left_wheel.forward(speed)
        self.right_wheel.forward(speed)

    def drive_backward(self, speed=100):
        print(f"[ROBOT] DRIVE BACKWARD speed={speed}")
        self.left_wheel.backward(speed)
        self.right_wheel.backward(speed)

    def look_left(self):
        print("[ROBOT] LOOK LEFT")
        self.head_pan.move_us(1200)

    def look_right(self):
        print("[ROBOT] LOOK RIGHT")
        self.head_pan.move_us(1800)

    def center_head(self):
        print("[ROBOT] CENTER HEAD")
        self.head_pan.center()
