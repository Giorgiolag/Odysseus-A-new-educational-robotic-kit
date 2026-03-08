from Odysseus import *
import time
ody = odysseus('COM3')
print("starting...")
katastasi = 1
ob_time=0
ob_dur = 0
while ob_dur<3:
    distance = ody.ultrasonic_distance()
    if ody.ultrasonic_distance() >=30:
        if katastasi == 2:
            ody.print("forward")
            katastasi = 1
            print("Τελική διάρκεια εμποδίου : ", ob_dur)
            ob_dur = 0
    else:
        if katastasi == 1:
            ody.print("left")
            katastasi = 2
            ob_start = time.time()
        else:
            ob_dur = time.time()-ob_start
    step = time.time()
    while time.time()-step <0.2:
        pass
#   print("διάρκεια εμποδίου : ",ob_dur, "Κατάσταση : ", katastasi, "Απόσταση : ", distance)
    print
print("end of program")
exit()