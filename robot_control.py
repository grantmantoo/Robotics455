from maestro import Controller
from robot import Robot
import time

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

    def center_pose(self):
        self.stop()
        self.head_pan(self.robot.servo_neutral("head_pan"))
        self.head_tilt(self.robot.servo_neutral("head_tilt"))
        self.waist(self.robot.servo_neutral("waist"))

    # -------------------------
    # Arm joints
    # -------------------------
    def _arm_move(self, attr_name, label, value):
        value = int(clamp(value, 2000, 8000))
        servo = getattr(self.robot, attr_name, None)
        if servo is None:
            raise ValueError(f"{label} servo is not configured")
        print(f"[CTRL] {label} -> {value}")
        servo.move(value)

    def right_shoulder_ud(self, value):
        self._arm_move("right_shoulder_ud", "right_shoulder_ud", value)

    def right_shoulder_yaw(self, value):
        self._arm_move("right_shoulder_yaw", "right_shoulder_yaw", value)

    def right_elbow_ud(self, value):
        self._arm_move("right_elbow_ud", "right_elbow_ud", value)

    def right_wrist_ud(self, value):
        self._arm_move("right_wrist_ud", "right_wrist_ud", value)

    def right_wrist_rot(self, value):
        self._arm_move("right_wrist_rot", "right_wrist_rot", value)

    def right_hand_pinch(self, value):
        self._arm_move("right_hand_pinch", "right_hand_pinch", value)

    def left_wrist_rot(self, value):
        self._arm_move("left_wrist_rot", "left_wrist_rot", value)

    def left_shoulder_ud(self, value):
        self._arm_move("left_shoulder_ud", "left_shoulder_ud", value)

    def left_shoulder_yaw(self, value):
        self._arm_move("left_shoulder_yaw", "left_shoulder_yaw", value)

    def left_elbow_ud(self, value):
        self._arm_move("left_elbow_ud", "left_elbow_ud", value)

    def left_wrist_ud(self, value):
        self._arm_move("left_wrist_ud", "left_wrist_ud", value)

    def left_hand_pinch(self, value):
        self._arm_move("left_hand_pinch", "left_hand_pinch", value)

    # -------------------------
    # Project 2 action primitive
    # -------------------------
    def arm_raise_sequence(self, deadline=None, cancel_event=None):
        """
        Multi-joint arm pose for Project 2.
        Uses shoulders + elbows + wrists + hands for a more visible action.
        """
        def should_stop():
            if cancel_event is not None and cancel_event.is_set():
                return True
            if deadline is not None and time.time() > deadline:
                return True
            return False

        def move_if_exists(name, value):
            servo = getattr(self.robot, name, None)
            if servo is not None:
                servo.move(value)
                return True
            return False

        def neutral(name):
            return self.robot.servo_neutral(name)

        def with_delta(name, delta):
            return int(clamp(neutral(name) + delta, 2000, 8000))

        if should_stop():
            return

        moved_any = False
        # Phase 1: raise shoulders and elbows.
        # Shoulder U/D and shoulder yaw are mirrored left/right by opposite deltas.
        moved_any |= move_if_exists("right_shoulder_ud", with_delta("right_shoulder_ud", +1100))
        moved_any |= move_if_exists("left_shoulder_ud", with_delta("left_shoulder_ud", -1100))
        moved_any |= move_if_exists("right_elbow_ud", with_delta("right_elbow_ud", +900))
        moved_any |= move_if_exists("left_elbow_ud", with_delta("left_elbow_ud", +900))
        moved_any |= move_if_exists("right_shoulder_yaw", with_delta("right_shoulder_yaw", +600))
        moved_any |= move_if_exists("left_shoulder_yaw", with_delta("left_shoulder_yaw", -600))

        # Phase 2: wrist and hand flourish.
        moved_any |= move_if_exists("right_wrist_ud", with_delta("right_wrist_ud", -200))
        moved_any |= move_if_exists("left_wrist_ud", with_delta("left_wrist_ud", -200))
        moved_any |= move_if_exists("right_wrist_rot", with_delta("right_wrist_rot", +300))
        moved_any |= move_if_exists("left_wrist_rot", with_delta("left_wrist_rot", -300))
        moved_any |= move_if_exists("right_hand_pinch", with_delta("right_hand_pinch", +500))
        moved_any |= move_if_exists("left_hand_pinch", with_delta("left_hand_pinch", +500))

        if moved_any:
            time.sleep(0.55)
            if should_stop():
                return

            # Return to neutral.
            move_if_exists("right_shoulder_ud", neutral("right_shoulder_ud"))
            move_if_exists("left_shoulder_ud", neutral("left_shoulder_ud"))
            move_if_exists("right_elbow_ud", neutral("right_elbow_ud"))
            move_if_exists("left_elbow_ud", neutral("left_elbow_ud"))
            move_if_exists("right_shoulder_yaw", neutral("right_shoulder_yaw"))
            move_if_exists("left_shoulder_yaw", neutral("left_shoulder_yaw"))
            move_if_exists("right_wrist_ud", neutral("right_wrist_ud"))
            move_if_exists("left_wrist_ud", neutral("left_wrist_ud"))
            move_if_exists("right_wrist_rot", neutral("right_wrist_rot"))
            move_if_exists("left_wrist_rot", neutral("left_wrist_rot"))
            move_if_exists("right_hand_pinch", neutral("right_hand_pinch"))
            move_if_exists("left_hand_pinch", neutral("left_hand_pinch"))
            time.sleep(0.25)
            return

        # Fallback pose so the action is still visible in demos.
        self.waist(6200)
        time.sleep(0.45)
        if should_stop():
            return
        self.waist(5000)
        time.sleep(0.2)

    def reset_arms_neutral(self):
        self.robot.set_arms_neutral()

    def test_arms_basic(self, hold_s=0.5):
        """
        Quick arm validation:
        neutral -> open pose -> neutral.
        """
        print("[CTRL] test_arms_basic start")
        self.reset_arms_neutral()
        time.sleep(hold_s)

        # Open visible pose.
        self.robot.right_shoulder_ud.move(int(clamp(self.robot.servo_neutral("right_shoulder_ud") + 1100, 2000, 8000)))
        self.robot.left_shoulder_ud.move(int(clamp(self.robot.servo_neutral("left_shoulder_ud") - 1100, 2000, 8000)))
        self.robot.right_shoulder_yaw.move(int(clamp(self.robot.servo_neutral("right_shoulder_yaw") + 600, 2000, 8000)))
        self.robot.left_shoulder_yaw.move(int(clamp(self.robot.servo_neutral("left_shoulder_yaw") - 600, 2000, 8000)))
        self.robot.right_elbow_ud.move(int(clamp(self.robot.servo_neutral("right_elbow_ud") + 900, 2000, 8000)))
        self.robot.left_elbow_ud.move(int(clamp(self.robot.servo_neutral("left_elbow_ud") + 900, 2000, 8000)))
        self.robot.right_wrist_ud.move(int(clamp(self.robot.servo_neutral("right_wrist_ud") - 200, 2000, 8000)))
        self.robot.left_wrist_ud.move(int(clamp(self.robot.servo_neutral("left_wrist_ud") - 200, 2000, 8000)))
        self.robot.right_hand_pinch.move(int(clamp(self.robot.servo_neutral("right_hand_pinch") + 500, 2000, 8000)))
        self.robot.left_hand_pinch.move(int(clamp(self.robot.servo_neutral("left_hand_pinch") + 500, 2000, 8000)))
        time.sleep(hold_s)

        self.reset_arms_neutral()
        print("[CTRL] test_arms_basic done")

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
