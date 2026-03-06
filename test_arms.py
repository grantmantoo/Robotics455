import time
from robot_control import RobotControl


if __name__ == "__main__":
    ctrl = RobotControl(port="/dev/ttyACM0", device=0x0C)
    try:
        # 1) Hard reset all arm servos to neutral.
        ctrl.reset_arms_neutral()
        time.sleep(1.0)

        # 2) Run a basic coordinated arm test.
        ctrl.test_arms_basic(hold_s=0.8)
        time.sleep(0.5)

        # 3) End in neutral again.
        ctrl.reset_arms_neutral()
    finally:
        ctrl.close()
