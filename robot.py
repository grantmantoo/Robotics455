from motor import Motor
from servo import Servo

class Robot:
    SERVO_NEUTRALS = {
        "head_pan": 5800,
        "waist": 5000,
        "head_tilt": 5500,
        "right_shoulder_ud": 4000,
        "right_shoulder_yaw": 6000,
        "right_elbow_ud": 4500,
        "right_wrist_ud": 5800,
        "right_wrist_rot": 5900,
        "right_hand_pinch": 2000,
        "left_wrist_rot": 5600,
        "left_shoulder_ud": 8000,
        "left_shoulder_yaw": 6000,
        "left_elbow_ud": 4300,
        "left_wrist_ud": 6000,
        "left_hand_pinch": 2000,
    }

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
        self.head_pan  = Servo(maestro, 2, center_val=self.SERVO_NEUTRALS["head_pan"])
        self.waist     = Servo(maestro, 3, center_val=self.SERVO_NEUTRALS["waist"])
        self.head_tilt = Servo(maestro, 4, center_val=self.SERVO_NEUTRALS["head_tilt"])

        # Arms
        # Right arm
        self.right_shoulder_ud = Servo(maestro, 5, center_val=self.SERVO_NEUTRALS["right_shoulder_ud"])
        self.right_shoulder_yaw = Servo(maestro, 6, center_val=self.SERVO_NEUTRALS["right_shoulder_yaw"])
        self.right_elbow_ud = Servo(maestro, 7, center_val=self.SERVO_NEUTRALS["right_elbow_ud"])
        self.right_wrist_ud = Servo(maestro, 8, center_val=self.SERVO_NEUTRALS["right_wrist_ud"])
        self.right_wrist_rot = Servo(maestro, 9, center_val=self.SERVO_NEUTRALS["right_wrist_rot"])
        self.right_hand_pinch = Servo(maestro, 10, center_val=self.SERVO_NEUTRALS["right_hand_pinch"])

        # Left arm
        self.left_wrist_rot = Servo(maestro, 11, center_val=self.SERVO_NEUTRALS["left_wrist_rot"])
        self.left_shoulder_ud = Servo(maestro, 12, center_val=self.SERVO_NEUTRALS["left_shoulder_ud"])
        self.left_shoulder_yaw = Servo(maestro, 13, center_val=self.SERVO_NEUTRALS["left_shoulder_yaw"])
        self.left_elbow_ud = Servo(maestro, 14, center_val=self.SERVO_NEUTRALS["left_elbow_ud"])
        self.left_wrist_ud = Servo(maestro, 15, center_val=self.SERVO_NEUTRALS["left_wrist_ud"])
        self.left_hand_pinch = Servo(maestro, 16, center_val=self.SERVO_NEUTRALS["left_hand_pinch"])

        print("[INIT] Robot ready")

    def servo_neutral(self, attr_name):
        return self.SERVO_NEUTRALS[attr_name]

    def arm_servos(self):
        return [
            self.right_shoulder_ud,
            self.right_shoulder_yaw,
            self.right_elbow_ud,
            self.right_wrist_ud,
            self.right_wrist_rot,
            self.right_hand_pinch,
            self.left_wrist_rot,
            self.left_shoulder_ud,
            self.left_shoulder_yaw,
            self.left_elbow_ud,
            self.left_wrist_ud,
            self.left_hand_pinch,
        ]

    def set_arms_neutral(self):
        print("[ROBOT] ARMS NEUTRAL -> configured values")
        for attr_name in [
            "right_shoulder_ud",
            "right_shoulder_yaw",
            "right_elbow_ud",
            "right_wrist_ud",
            "right_wrist_rot",
            "right_hand_pinch",
            "left_wrist_rot",
            "left_shoulder_ud",
            "left_shoulder_yaw",
            "left_elbow_ud",
            "left_wrist_ud",
            "left_hand_pinch",
        ]:
            getattr(self, attr_name).move(self.servo_neutral(attr_name))

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
