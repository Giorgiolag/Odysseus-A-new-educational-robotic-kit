from Odysseus import *
import time
ody = odysseus('COM3')
print("starting...")
while 1:
    if ody.ultrasonic_distance() >=30:
            ody.print("forward")
    else:
            ody.print("left")

    time.sleep(0.2)
