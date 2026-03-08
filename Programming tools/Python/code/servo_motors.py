"""
This example moves the servo motors "plugged" into servo-slots 1 and 4. All the alternatives for passing the servo slot
 are used (i.e. s1 or S1 or 1 or "s1" or "s1")

 Keep in mind that you need to change the com port to the one assigned by your operating system:
--> to Ody's bluetooth module (in case that the bluetooth switch is ON and you wish to connect wirelessly)
--> to the arduino UNO (in case that the bluetooth switch is OFF and you wish to connect through USB cable)
"""

from Odysseus import *
ody = odysseus('COM3')
ody.move_servo(1, "clockwise", 7)  #move "servo-1", direction "clockwise", speed 7
time.sleep(1)
ody.move_servo(s1, "counterclockwise", 7) #move "servo-1", direction "counterclockwise", speed 7
time.sleep(1)
ody.move_servo("s1", "clockwise", 7, "upwards", 25)  #move "servo-1", direction "clockwise", speed 7.5
time.sleep(1)
ody.move_servo("S1", "counterclockwise", 8, "downwards", 25 ) #move "servo-1", direction "counterclockwise", speed 7.5
time.sleep(1)
ody.move_servo(S1, "stopped") #stop "servo-1"
time.sleep(1)
ody.move_servo(4, "clockwise", 7)  #move "servo-1", direction "clockwise", speed 7
time.sleep(1)
ody.move_servo(s4, "counterclockwise", 7) #move "servo-1", direction "counterclockwise", speed 7
time.sleep(1)
ody.move_servo("s4", "clockwise", 7, "upwards", 25)  #move "servo-1", direction "clockwise", speed 7.5
time.sleep(1)
ody.move_servo("S4", "counterclockwise", 8, "downwards", 25 ) #move "servo-1", direction "counterclockwise", speed 7.5
time.sleep(1)
ody.move_servo(S4, "stopped") #stop "servo-1"
exit()