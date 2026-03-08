"""
This example show the use of the DHT sensors for detecting temperature and air humidity. We use a DHT11 sensor plugged
into "slot-2". Upon connection with Odysseas, 2 seconds are needed for the sensor to work. In fact, the sensor needs 2
seconds before it is used, anytime we use it. If, after its fists use we use it without waiting 2 seconds, se get the
previous measurements i.e. the sensor is not used at all. In the example that follows, the sensor is actually used only
once (the first time) and the next 2 times we get the values detected by the first use of the sensor. A total of 2
senconds must pass until after the first use, until the sensor is used again. The (supported by the library) temperature
range is [-127, 127] degrees Celsius
""" 

from Odysseus import *
ody = odysseus('COM3')
time.sleep(2)
print(ody.dht(11, "slot-2"))
time.sleep(1)
print(ody.dht(11, "slot-2"))
time.sleep(0.5)
print(ody.dht(11, "slot-2"))