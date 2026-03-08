"""
This part initializes the color sensor. It needs some time for the initialization, and we have set this time to 2
seconds. The initialization is performed once and the color sensor is available afterwards.
"""

from Odysseus import *
ody = odysseus('COM3')
ody.init_color_sensor()
time.sleep(2)

"""
This part prints the activated colors. Initially the list of activated colors is empty and if not, we activate all 
colors. Then we activate green, blue and yellow and print again the activated colors. The activated colors can now be 
fixed i.e. we can now store RGB values for each of the activated colors. All the supported colors not given as 
parameters  are deactivated, if they were  activated before. Thus, to deactivate all colors we have to run 
"ody.activate_colors()"
"""
print("activated colors : ")
print(ody.active_colors())
if len(ody.active_colors()) >0:
    print("deactivating all colors...")
    ody.activate_colors()
    print("activated colors mow: ")
    print(ody.active_colors())


print("activating green, blue and yellow")
ody.activate_colors("green", "blue", "yellow")
print(ody.active_colors())

"""
This part stores (in the EEPROM of the arduino) the RGB values for the activated colors. Each time we store the RGB 
values for a color, we say that we "fix" this color. Each color is fixed separately. to fix a color, we place a piece
of paper (or object) having this color under the color sensor, write (in the terminal) the color to be fixed and press
"enter".  
"""

dec = input("you want to fix a color? (yes/no) ")
while dec  == "yes":
    print("you can fix red, green, blue, cyan, yellow, magenta, floor, black, white")
    print("place the object having the color under the color sensor")
    color = input("give the color of the object : ")
    ody.fix_color(color)
    dec = input("you want to fix a color? (yes/no) : ")

"""
This part prints the stored (in the EEPROM) RGB values for the fixed colors.
"""

active_color_list = ody.active_colors()
for color in ody.active_colors():
    r, g, b = ody.fixed_color(color)
    print("fixed color : ", color, "RGB values : ", r, g, b)

"""
This part prints the RGB values for the color placed under the color sensor. No EEPROM involved here. Keep in mind that
small variations in the RGB value may occur if we place the the same color under the color sensor and check the RGB 
values detected by the color sensor. This is due to small variations in the light of distance between thr color sensor
and the object, or limitations in the sensor's accuracy. Thus using the color sensor this way, special care must be
taken by the programmer in order to translate the RGB values to an actual color. Observe also the the RGB values are not 
between 0 and 255. the higher the light, the higher the values.
"""
dec = input(" do you want to see the RGB values detected by the color sensor? (yes/no) : ")
while dec  == "yes":
    print("place the color you want to detect under the color sensor")
    color = input("Press enter")
    r,g,b = ody.color_sensor_value()
    print("RGB values detected by the color sensor : ", r, g, b)
    dec = input("do you want to see the RGB values detected by the color sensor? (yes/no) : ")

"""
The next part shows how to match the RGB values detected by the color sensor, to one of the colors stored in th EEPROM.
It works only for the colors already activated and fixed. This means that if the color placed under the color sensor 
is not one of the activated and fixed colors, the answer will most likely be "unknown color". If the detected color 
is very close to an already activated and fixed color, we can successfully match the detected color by setting an
appropriate similarity frame. Setting the from to 5 (i.e. 5%) worked fine during the tests.
"""

dec = input(" do you want to match the color detected by the color sensor to one of the activated and fixed colors? (yes/no) : ")
while dec  == "yes":
    print("place the color you want to detect under the color sensor")
    color = input("Press enter")
    print("Matched color : ", ody.matched_fixed_color(5))
    dec = input(" do you want to match the color detected by the color sensor to one of the activated and fixed colors? (yes/no) : ")