class Motor:
    def __init__(self, maestro, channel, stop=1500):
        self.maestro = maestro
        self.channel = channel
        self.stop = stop * 4

    def forward(self, speed=100):
        self.maestro.setTarget(self.channel, (1500 + speed) * 4)

    def backward(self, speed=100):
        self.maestro.setTarget(self.channel, (1500 - speed) * 4)

    def stop_motor(self):
        self.maestro.setTarget(self.channel, self.stop)
