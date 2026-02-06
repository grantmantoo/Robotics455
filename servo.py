class Servo:
    def __init__(self, maestro, channel, min_us=1000, max_us=2000):
        self.maestro = maestro
        self.channel = channel
        self.min = min_us * 4
        self.max = max_us * 4
        maestro.setRange(channel, self.min, self.max)

    def move_us(self, us):
        self.maestro.setTarget(self.channel, us * 4)

    def center(self):
        self.move_us(1500)
