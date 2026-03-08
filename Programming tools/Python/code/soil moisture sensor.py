"""
This example show the use of the soil moisture sensor plugged into "slot-2". The value returned is between 0 and 1
"""


from Odysseus import *
ody = odysseus('COM3')
print(ody.soil_moisture("slot-2"))