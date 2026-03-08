from Odysseus import *
"""
This example prints all the supported images once, one after another, for one second each. By printing "flipped", 
the "odysseus" image is printed in the display, but upside down. After printing "flipped"", everything will be printed 
upside down, until "flipped" is printed again. 

Keep in mind that you need to change the com port to the one assigned by your operating system: 
--> to Ody's bluetooth module (in case that the bluetooth switch is ON and you wish to connect wirelessly) 
--> to the arduino UNO (in case that the bluetooth switch is OFF and you wish to connect through USB cable) 
"""

images_list = ["odysseus","angry", "smiling", "in love", "winking", "forward",
               "backward", "left", "right", "object detected", "color detected", "flipped"]

ody = odysseus('COM3')
for image in images_list:
    ody.print(image)
    time.sleep(1)

exit()