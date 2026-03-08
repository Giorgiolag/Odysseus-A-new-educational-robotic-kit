# Odysseus Robotics Kit - Firmware and Software

This archive contains the source code, libraries, and firmware required to operate the Odysseus programmable robotics unit. The system relies on two separate microcontrollers: an ATtiny85 and an Arduino UNO.

## 1. ATtiny85 Firmware
This firmware handles specific peripheral tasks on the custom shield. It is intended for a single, permanent deployment.

### Flashing Instructions
1. **Setup:** Use an Arduino UNO as an In-System Programmer (ISP) and the Arduino IDE for compiling and uploading the code. 
2. **Libraries:** All necessary libraries for compiling the source code are included in this archive.
3. **Hardware Placement:** Once flashed, place the ATtiny85 into the custom PCB socket, ensuring you match the orientation markings on the silkscreen. 

 

The following Arduino IDE libraries need to be installed

* [TinywireM](https://github.com/adafruit/TinyWireM/)
* [TinyWireS](https://github.com/nadavmatalon/TinyWireS)
* [Tiny4kOLED-master](https://github.com/datacute/Tiny4kOLED)

To achieve full hardware functionality, Port 8 (which connects to the RESET pin of the ATtiny85) must be reconfigured to act as a general-purpose digital output. If this is not done, the LED plugged into Port 8 will not work.
* **The Modification:** You must modify the fuse settings of the ATtiny85 to reconfigure the pin.
* **The Warning:** Performing this modification **disables standard In-System Programming (ISP)**, preventing any further firmware updates via the Arduino IDE. 
* **Reverting:** In the unlikely event that reprogramming is required later, the configuration can only be reverted by applying 12V to the RESET pin using a High Voltage Serial Programmer (HVSP).

---

## 2. Odysseus Firmata Deployment (Arduino UNO)
The Arduino UNO acts as the main bridge between the host computer and the Odysseus hardware. 

Because Odysseus is designed for users of varying technical expertise, advanced users may occasionally flash their own custom sketches directly to the Arduino. Doing so will overwrite the Odysseus Firmata, rendering the system temporarily incompatible with the host software. 

To restore Odysseus to its standard functional state, you can redeploy the Firmata using one of two methods:

### Method A: Manual Pathway (Arduino IDE)
For users familiar with standard microcontroller programming:
1. Open the provided Firmata sketch in the Arduino IDE.
2. Ensure the required libraries (included in this archive) are installed.
3. Compile and upload the sketch to the Arduino UNO via USB.

The following Arduino IDE libraries need to be installed (All but the first library are used in educational scenarios involving IoT)
 - Adafruit_TCS34725
 - DHTLib – (by Rob Tillaart)
 - Grove_Sunlight_Sensor
 - BH1750
 - Adafruit_BusIO 


### Method B: No-Code Pathway (Node.js)
For a rapid, beginner-friendly restoration:
1. Utilize the provided Node.js-based server utility.
2. Follow the on-screen prompts to upload the standard firmware with a single click, instantly restoring communication with the host software.

---
