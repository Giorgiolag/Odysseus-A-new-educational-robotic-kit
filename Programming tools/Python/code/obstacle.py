"""
This example shows the use of the IR Infrared Obstacle Avoidance Sensor. This sensor is also used as a follow line
sensor. Tho servos have been plugged in servo slots 1 (used as left wheel) and 4 (used as right wheel).  An
IR Infrared Obstacle Avoidance Sensor has been plugged into sensor slot 1 and faces to the front of the robot (car).
According to the code, the robot moves forward as long as no abstacle is detected, stops when an obstacle is
detected and remains stopped as long as the obstacle remains. Press CTRL+F2 to exit the program. The servos will
remain in the state they were when CTRL+F2 was pressed. If they are moving, the fastest way to stop them is to turn
Odysseus off.

Keep in mind that you need to change the com port to the one assigned by your operating system:
--> to Ody's bluetooth module (in case that the bluetooth switch is ON and you wish to connect wirelessly)
--> to the arduino UNO (in case that the bluetooth switch is OFF and you wish to connect through USB cable)

"""
from Odysseus import *
ody = odysseus('COM3')
while True:
    if not ody.obstacle("slot-1"):
        ody.move_servo(1, "counterclockwise", 5)  # move "servo-1", direction "clockwise", speed 5
        ody.move_servo(4, "clockwise", 5)  # move "servo-4", direction "counterclockwise", speed 5
    else:
        ody.move_servo(1, "stopped")  # stop "servo-1"
        ody.move_servo(4, "stopped")  # stop "servo-4"




exit()