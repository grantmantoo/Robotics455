
class Motor():
    def __init__(self, name):
        self.name = name
        self.speed = 0

    def set_speed(self, speed):
        self.speed = speed
        print(f"{self.name} motor speed set to {self.speed}")