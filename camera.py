class Camera():
    def __init__(self, name):
        self.name = name
        self.angle = 0

    def set_angle(self, angle):
        self.angle = angle
        print(f"{self.name} camera angle set to {self.angle}")


    # -------------------------
    # To Be Implemented
    # -------------------------