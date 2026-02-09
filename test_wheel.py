from maestro import Controller
import time

LEFT = 0
RIGHT = 1
PORT = "/dev/ttyACM0"
DEVICE = 0x0C

NEUTRAL = 5000
DELTAS = [100, 200, 300, 400]  # start small
HOLD = 1.0


def cmd(maestro, ch, value, label=""):
    direction = "STOP" if value == NEUTRAL else ("FWD" if value > NEUTRAL else "REV")
    tag = f"{label} " if label else ""
    print(f"[CMD] {tag}ch{ch} {direction} value={value}")
    maestro.setTarget(ch, value)


def arm(maestro):
    print("[ARM] Sending NEUTRAL 5000 to both wheels...")
    cmd(maestro, LEFT, NEUTRAL, "LEFT")
    cmd(maestro, RIGHT, NEUTRAL, "RIGHT")
    time.sleep(0.5)


def stop(maestro):
    cmd(maestro, LEFT, NEUTRAL, "LEFT")
    cmd(maestro, RIGHT, NEUTRAL, "RIGHT")


def pulse(maestro, ch, value, seconds, label):
    cmd(maestro, ch, value, label)
    time.sleep(seconds)
    cmd(maestro, ch, NEUTRAL, label)


def main():
    maestro = Controller(PORT, device=DEVICE)
    try:
        arm(maestro)
        stop(maestro)
        time.sleep(0.5)

        print("\n--- LEFT wheel forward ---")
        for d in DELTAS:
            pulse(maestro, LEFT, NEUTRAL + d, HOLD, "LEFT")
            time.sleep(0.7)

        print("\n--- LEFT wheel reverse ---")
        for d in DELTAS:
            pulse(maestro, LEFT, NEUTRAL - d, HOLD, "LEFT")
            time.sleep(0.7)

        print("\n--- RIGHT wheel forward ---")
        for d in DELTAS:
            pulse(maestro, RIGHT, NEUTRAL + d, HOLD, "RIGHT")
            time.sleep(0.7)

        print("\n--- RIGHT wheel reverse ---")
        for d in DELTAS:
            pulse(maestro, RIGHT, NEUTRAL - d, HOLD, "RIGHT")
            time.sleep(0.7)

        print("\n--- BOTH wheels forward ---")
        for d in DELTAS:
            cmd(maestro, LEFT, NEUTRAL + d, "LEFT")
            cmd(maestro, RIGHT, NEUTRAL + d, "RIGHT")
            time.sleep(HOLD)
            stop(maestro)
            time.sleep(0.7)

        print("\n--- BOTH wheels reverse ---")
        for d in DELTAS:
            cmd(maestro, LEFT, NEUTRAL - d, "LEFT")
            cmd(maestro, RIGHT, NEUTRAL - d, "RIGHT")
            time.sleep(HOLD)
            stop(maestro)
            time.sleep(0.7)

        print("\n[DONE] wheels stopped.")

    except KeyboardInterrupt:
        print("\n[INTERRUPT] stopping.")
        stop(maestro)

    finally:
        stop(maestro)
        maestro.close()
        print("[CLOSED]")

if __name__ == "__main__":
    main()
