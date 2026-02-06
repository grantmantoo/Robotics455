from maestro import Controller
from robot import Robot
import time

maestro = Controller('/dev/ttyACM0')
robot = Robot(maestro)

try:
    robot.drive_forward(120)
    time.sleep(2)
    robot.stop()

    robot.look_left()
    time.sleep(1)
    robot.look_right()
    time.sleep(1)
    robot.center_head()

except KeyboardInterrupt:
    pass
finally:
    maestro.close()
