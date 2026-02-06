from motor import Motor
class Head():
    def __init__(self):
        self.motors = {
            "left_eye": Motor("Left Eye"),
            "right_eye": Motor("Right Eye"),
            "mouth": Motor("Mouth")
        }

    def set_motor_speed(self, motor_name, speed):
        if motor_name in self.motors:
            self.motors[motor_name].set_speed(speed)
        else:
            print(f"Motor {motor_name} not found in head.")