class Speaker:
    def __init__(self, name):
        self.name = name
        self.volume = 0
        print(f"[INIT] Speaker '{self.name}' initialized")

    def set_volume(self, volume):
        self.volume = volume
        print(f"[SPEAKER] {self.name} volume set to {self.volume}")
