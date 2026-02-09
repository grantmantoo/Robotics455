from motor import Motor


class Head:
    """
    Head subsystem using your Motor class.

    set_motor_speed(motor_name, speed):
    speed > 0  => forward(speed)
    speed < 0  => backward(abs(speed))
    speed = 0  => stop
    """

    def __init__(self, maestro, channel_map=None):
        print("[INIT] Head subsystem initializing")

        # Default channel assignments (CHANGE if your wiring differs)
        default_map = {
            "left_eye": 17,
            "right_eye": 18,
            "mouth": 19,
        }
        self.channel_map = channel_map or default_map

        # Create Motor objects for each head motor
        self.motors = {}
        for name, ch in self.channel_map.items():
            print(f"[INIT] Head motor '{name}' on channel {ch}")
            self.motors[name] = Motor(maestro, ch)

        print("[INIT] Head subsystem ready")

    def set_motor_speed(self, motor_name, speed):
        """
        speed: int in [-400..400] (typical range); 0 stops.
        Your Motor.forward/backward uses speed as delta microseconds from 1500.
        Keep it reasonable (e.g., 0-200) unless you know your controller range.
        """
        if motor_name not in self.motors:
            print(f"[HEAD ERROR] Motor '{motor_name}' not found. Valid: {list(self.motors.keys())}")
            return

        m = self.motors[motor_name]

        # Clamp to a reasonable range to avoid crazy values
        if speed > 200:
            print(f"[HEAD WARN] speed {speed} too high, clamping to 200")
            speed = 200
        elif speed < -200:
            print(f"[HEAD WARN] speed {speed} too low, clamping to -200")
            speed = -200

        if speed == 0:
            print(f"[HEAD] {motor_name}: STOP")
            m.stop_motor()
        elif speed > 0:
            print(f"[HEAD] {motor_name}: FORWARD speed={speed}")
            m.forward(speed)
        else:
            print(f"[HEAD] {motor_name}: BACKWARD speed={abs(speed)}")
            m.backward(abs(speed))

    def stop_all(self):
        print("[HEAD] STOP ALL")
        for name, m in self.motors.items():
            print(f"[HEAD] stopping {name}")
            m.stop_motor()
