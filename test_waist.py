from maestro import Controller
import time

# Connect to Maestro (USB serial)
maestro = Controller('/dev/ttyACM0')

WAIST = 3   # waist rotate servo

# Optional safety range (adjust if needed)
maestro.setRange(WAIST, 4000, 8000)

print("Center waist")
maestro.setTarget(WAIST, 6000)   # center
time.sleep(2)

print("Rotate left")
maestro.setTarget(WAIST, 7500)
time.sleep(2)

print("Rotate right")
maestro.setTarget(WAIST, 4500)
time.sleep(2)

print("Back to center")
maestro.setTarget(WAIST, 6000)
time.sleep(2)

maestro.close()
te