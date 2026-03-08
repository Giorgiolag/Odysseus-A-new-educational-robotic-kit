from Odysseus import *
import time
ody = odysseus('COM3')
print("starting...")
katastasi = 1
while 1:
    if ody.ultrasonic_distance() >=30:
        if katastasi == 2:
            ody.print("forward")
            katastasi = 1
    else:
        if katastasi == 1:
            ody.print("left")
            katastasi = 2
    time.sleep(0.2)

