class Sensor():
    def __init__(self, name):
        self.name = name
        self.value = 0

    def read_value(self):
        print(f"{self.name} sensor value: {self.value}")