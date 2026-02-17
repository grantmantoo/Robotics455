from maestro import Controller
import time

PORT = "/dev/ttyACM0"
DEVICE = 0x0C
NEUTRAL = 6000

LEFT = 0
RIGHT = 1

m = Controller(PORT, device=DEVICE)

print("[FORCE STOP] Sending neutral repeatedly...")
t0 = time.time()
while time.time() - t0 < 3.0:
    m.setTarget(LEFT, NEUTRAL)
    m.setTarget(RIGHT, NEUTRAL)
    time.sleep(0.05)

print("[FORCE STOP] Done. Closing.")
m.close()
