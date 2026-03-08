"""
This example explains how to use the ultrasonic sensor. The ultrasonic sensor is plugged in its slot. Two servos have
been plugged into servo slots 1 and 4. The one on slot 1 is the left wheel, and the one on slot 4 is the right wheel.
Odysseas moves forward as long as there is no obstacle closer than 30cm, in which case Odysseas stops. If however,
the obstacle is moving towards Odysseas (because it is a moving obstacle), and the distance becomes less than 10 cm,
Odysseas will move backwards. Press CTRL+F2 to exit the program. The servos will remain in the
state they were when press CTRL+F2 was pressed. If they are moving, the fastest way to stop them is to turn Odysseas off
"""

import time
from Odysseus import *
ody = odysseus('COM3')
while True:
    if ody.ultrasonic_distance() >=30:
        ody.move_servo(1, "counterclockwise", 5)  # move "servo-1", direction "counterclockwise", speed 5
        ody.move_servo(4, "clockwise", 5)  # move "servo-4", direction "clockwise", speed 5
    else:
        if ody.ultrasonic_distance() < 10:
            ody.move_servo(1, "clockwise", 5)  # move "servo-1", direction "clockwise", speed 5
            ody.move_servo(4, "counterclockwise", 5)  # move "servo-4", direction "counterclockwise", speed 5
        else:
            ody.move_servo(1, "stopped")  # stop "servo-1"
            ody.move_servo(4, "stopped")  # stop "servo-4"
    time.sleep(0.2)




exit()