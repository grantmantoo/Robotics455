from maestro import Controller
from robot import Robot
import time

m = Controller("/dev/ttyACM0")
robot = Robot(m)

try:
    robot.drive_forward(800)
    time.sleep(2)

    robot.stop()
    time.sleep(1)

    robot.drive_backward(800)
    time.sleep(2)

    robot.stop()
    time.sleep(1)

    robot.turn_left(1000)
    time.sleep(1.5)

    robot.turn_right(1000)
    time.sleep(1.5)

    robot.stop()

finally:
    robot.stop()
    m.close()
    print("[CLOSED]")
