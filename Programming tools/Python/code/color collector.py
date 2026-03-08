from Odysseus import *
ody = odysseus('COM3')
ody.init_color_sensor()
time.sleep(2)
color_list = []
ody.move_servo(1, "counterclockwise", 3)
ody.move_servo(4, "clockwise", 3)
while len(color_list)<3:
    color =  ody.matched_fixed_color(20)
    if color != "unknown":
        if not(color in color_list):
            color_list.append(color)
            print(color_list)
ody.move_servo(1, "stopped")
ody.move_servo(4, "stopped")
