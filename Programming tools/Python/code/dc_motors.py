"""
This example moves the DC motor "plugged" into DC-motor-slot 1. in the same way we can move DC-motors
 plugged into DC-motor-slot 2
"""

from Odysseus import *
ody = odysseus('COM3')
ody.move_DC_motor(1, "clockwise", 7)  #move "DC-motor-1", direction "clockwise", speed 7
time.sleep(1)
ody.move_DC_motor(dc1, "counterclockwise", 7) #move "DC-motor-1", direction "counterclockwise", speed 7
time.sleep(1)
ody.move_DC_motor(DC1, "clockwise", 7, "upwards", 20)  #move "DC-motor-1", direction "clockwise", speed 7.5
time.sleep(1)
ody.move_DC_motor("dc1", "counterclockwise", 8, "downwards", 20 ) #move "DC-motor-1", direction "counterclockwise", speed 7.5
time.sleep(1)
ody.move_DC_motor("DC1", "stopped") #stop "servo-1"
time.sleep(1)
ody.move_DC_motor(2, "clockwise", 7)  #move "DC-motor-1", direction "clockwise", speed 7
time.sleep(1)
ody.move_DC_motor(dc2, "counterclockwise", 7) #move "DC-motor-1", direction "counterclockwise", speed 7
time.sleep(1)
ody.move_DC_motor(DC2, "clockwise", 7, "upwards", 20)  #move "DC-motor-1", direction "clockwise", speed 7.5
time.sleep(1)
ody.move_DC_motor("dc2", "counterclockwise", 8, "downwards", 20 ) #move "DC-motor-1", direction "counterclockwise", speed 7.5
time.sleep(1)
ody.move_DC_motor("DC2", "stopped") #stop "servo-1"
exit()