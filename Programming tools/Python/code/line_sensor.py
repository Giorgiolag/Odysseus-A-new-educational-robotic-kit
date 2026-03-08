'''
This example shows the use of the IR Infrared Obstacle Avoidance Sensor for detecting a dark line on a light colored
floor. Tho servos have been plugged in servo slots 1 (used as left wheel) and 4 (used as right wheel).
Two sensors are used allowing Ody to follow such a line. One is plugged into slot 1 and the other is plugged into
slot 4. The one in slot 1 is placed on the right side of Ody and the other is placed on the left side. This example
will make Ody follow the line forever. Press CTRL+F2 to exit the program. When the program terminates, The servos will
remain in the state they were when CTRL+F2 was pressed. If they are moving, the fastest way to stop them is to turn
Odysseus off.

Keep in mind that you need to change the com port to the one assigned by your operating system:
--> to Ody's bluetooth module (in case that the bluetooth switch is ON and you wish to connect wirelessly)
--> to the arduino UNO (in case that the bluetooth switch is OFF and you wish to connect through USB cable)


'''
from Odysseus import *
ody = odysseus('COM3')
while True:
    if (not ody.line("slot-1")) and (not ody.line("slot-4")):  # must move forward
        ody.move_servo(1, "counterclockwise", 5)  # move "servo-1", direction "clockwise", speed 5
        ody.move_servo(4, "clockwise", 5)  # move "servo-4", direction "counterclockwise", speed 5
    elif (not ody.line("slot-1")) and ody.line("slot-4"):   # must turn left
        ody.move_servo(1, "clockwise", 5)  # move "servo-1", direction "clockwise", speed 5
        ody.move_servo(4, "clockwise", 5)  # move "servo-4", direction "counterclockwise", speed 5
    else:   # must turn right
        ody.move_servo(1, "counterclockwise", 5)  # move "servo-1", direction "clockwise", speed 5
        ody.move_servo(4, "counterclockwise", 5)  # move "servo-4", direction "counterclockwise", speed 5
