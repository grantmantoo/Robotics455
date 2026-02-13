from maestro import Controller
import time

PORT = "/dev/ttyACM0"
DEVICE = 0x0C

LEFT = 0
RIGHT = 1

# start with your known "init" value
INIT = 6000

m = Controller(PORT, device=DEVICE)

def send(ch, val):
    print(f"[CMD] ch{ch} <- {val}")
    m.setTarget(ch, val)

try:
    print("[STEP 1] INIT (arming) both wheels at 5000 for 2 seconds")
    send(LEFT, INIT)
    send(RIGHT, INIT)
    time.sleep(2)

    print("\n[STEP 2] Now testing candidate STOP values.")
    print("Watch the wheels and press Ctrl+C once you see a value that STOPS them.\n")

    # Test a set of common "stop" candidates.
    # Many motor drivers use 0, 6000, 4000, or "hold last" depending on mode.
    candidates = [
        0, 1, 1000, 2000, 3000, 4000, 4500,
        5000,
        5500, 6000, 6500, 7000, 7500, 8000
    ]

    for val in candidates:
        print(f"\n[TEST] STOP candidate = {val} (holding 1.5s)")
        send(LEFT, val)
        send(RIGHT, val)
        time.sleep(1.5)

    print("\n[STEP 3] Fine sweep near 5000 (4800..5200 step 20)")
    for val in range(4800, 5201, 20):
        print(f"\n[TEST] {val} (holding 1.0s)")
        send(LEFT, val)
        send(RIGHT, val)
        time.sleep(1.0)

finally:
    print("\n[FINAL] Sending INIT=5000 and closing")
    send(LEFT, INIT)
    send(RIGHT, INIT)
    time.sleep(1)
    m.close()
