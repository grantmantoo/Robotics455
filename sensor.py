class Sensor:
    def __init__(self, name):
        self.name = name
        self.value = 0
        print(f"[INIT] Sensor '{self.name}' initialized")

    def read_value(self):
        print(f"[SENSOR] {self.name} value = {self.value}")
        return self.value


# -------------------------
# To Be Implemented
# -------------------------