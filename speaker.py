class Speaker():
    def __init__(self, name):
        self.name = name
        self.volume = 0

    def set_volume(self, volume):
        self.volume = volume
        print(f"{self.name} speaker volume set to {self.volume}")