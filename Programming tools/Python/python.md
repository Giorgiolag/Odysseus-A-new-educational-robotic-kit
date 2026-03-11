# Odysseus Robot - Python Control API

[![Python Version](https://img.shields.io/badge/python-3.9-blue.svg)](https://www.python.org/downloads/release/python-390/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
 This repository contains the Python control library and example scripts for **Odysseus**, an open-source educational robot. The library interfaces with the robot's onboard microcontroller (Arduino UNO) via the Firmata protocol, allowing users to control motors, read sensor data, and interact with the robot's display and sound modules using high-level Python commands.

This software is part of the open-source hardware project published in the *HardwareX* journal.

---

## 📋 Prerequisites & Dependencies

To use this library, you need:
* **Python 3.9** (Recommended)
* **pyFirmata**: The core dependency for Python-Arduino communication.

You can install the required dependency via pip:

```bash
pip install pyfirmata
```

## 🚀 Quick Start

Here is a simple example of how to make Odysseus move forward, while avoiding obstacles using its ultrasonic sensor. 

Ensure Odysseus is connected (via Bluetooth or USB) and find the assigned COM port (e.g., `COM3`).

```python
import time
from Odysseus import odysseus

# Initialize connection (Change 'COM3' to your actual port)
ody = odysseus('COM3')

try:
    while True:
        # Read distance from the ultrasonic sensor
        dist = ody.ultrasonic_distance()

        if dist >= 30:
            # Move forward: Left wheel (Servo 1) CCW, Right wheel (Servo 4) CW
            ody.move_servo(1, "counterclockwise", 5)  
            ody.move_servo(4, "clockwise", 5)         
        elif dist < 10:
            # Turn right/avoid: Left wheel CW, Right wheel CCW
            ody.move_servo(1, "clockwise", 5)         
            ody.move_servo(4, "counterclockwise", 5)  
        else:
            # Stop both motors
            ody.move_servo(1, "stopped")              
            ody.move_servo(4, "stopped")              
            
        time.sleep(0.1) # Brief delay to prevent buffer overflow

except KeyboardInterrupt:
    # Safely stop the motors upon manual exit
    ody.move_servo(1, "stopped")
    ody.move_servo(4, "stopped")
    print("Odysseus stopped.")
```

---

## 🛠️ API Reference (`Odysseus.py`)

The `odysseus` class abstracts the underlying hardware complexities. Below is a summary of the core capabilities.

### 1. Movement & Motors
Odysseus supports both continuous rotation Servos and DC Motors, offering precise speed control and "refinement" for decimal-level speed adjustments.
* `move_servo(servo_number, direction, speed, refinement, ref_value)`: Controls servos in slots 1 to 4.
* `move_DC_motor(DC_motor_number, direction, speed, refinement, ref_value)`: Controls DC motors in slots 1 or 2.

### 2. Sensors
The API provides straightforward methods to read various environmental and navigation sensors:
* **Distance & Navigation:**
  * `ultrasonic_distance()`: Returns distance in cm.
  * `sharp_ir_value(sensor_slot)`: Returns Sharp IR sensor output in Volts.
  * `obstacle(sensor_slot)`: Returns `True` if an obstacle is detected.
  * `line(sensor_slot)`: Returns `True` if a dark line is detected on a light floor.
* **Environmental:**
  * `dht(sensor_model, sensor_slot)`: Returns a list `[temperature, humidity]`.
  * `soil_moisture(sensor_slot)`: Returns raw analog moisture readings.
* **Color Detection (I2C):**
  * Includes a full suite for color recognition: `init_color_sensor()`, `activate_colors()`, `fix_color(color)`, `fixed_color(color)`, and `matched_fixed_color(percentage)`.

### 3. Display, Sound & Lights
* `print(image)`: Displays pre-loaded emotions/symbols on the LED matrix (e.g., `"odysseus"`, `"smiling"`, `"flipped"` to invert orientation).
* `sing(song)`: Plays built-in melodies.
* `play_tone(freq, dur)` / `beep()`: Low-level buzzer control.
* `lights(side, status)`: Controls left/right LEDs (`"on"`, `"off"`, `"blink"`).

---

## 📁 Basic Examples

The repository includes several fundamental scripts demonstrating individual features of the Odysseus robot. These are perfect for testing your hardware setup:

* **Movement Control:**
  * `servo_motors.py`: Demonstrates controlling continuous rotation servos, including speed refinement and syntax variations for slot addressing.
  * `dc_motors.py`: Shows how to independently control DC motors, utilizing decimal-level speed refinements.
* **Navigation Sensors:**
  * `line_sensor.py`: Implements a basic line-following algorithm using two IR sensors pointing downwards.
  * `obstacle.py`: Demonstrates simple obstacle avoidance using a front-facing IR sensor.
* **Color Recognition:**
  * `color_sensor.py`: A comprehensive, interactive script for activating, training (fixing to EEPROM), and utilizing the color sensor to match objects with RGB profiles.
  * `color collector.py`: An automated routine where Odysseus spins in place to detect and collect three distinct colors into an array.
* **Environmental & Audio-Visual:**
  * `DHT sensor.py`: Reads and prints temperature and humidity data, demonstrating proper timing intervals.
  * `display.py`: Cycles through all supported LED matrix images, faces, and directional arrows.
  * `beep.py` & `play_tone.py`: Simple scripts to test the onboard buzzer.

---

## 🚀 Comprehensive Project Examples

To demonstrate how the individual modules can be combined into fully functional robotics applications, we provide the following sequential projects. They are designed as a pedagogical progression, guiding the user from basic reactive programming to advanced state and time management.

### Project 1: Basic Reactive Display (`project1.py`)
Demonstrates a simple, continuous polling loop. Odysseus continuously reads the ultrasonic sensor and directly updates its LED matrix to display a "forward" arrow if the path is clear (>= 30 cm) or a "left" arrow if an obstacle is near. It relies on a standard `time.sleep()` delay to pace the loop.

### Project 2: State Machine Optimization (`project2.py`)
Builds upon the first project by introducing a basic state machine (`katastasi` variable). Instead of continuously sending display commands over the serial connection, it only transmits a command when the robot's state actually changes (i.e., transitioning from clear to blocked, or vice versa). This significantly minimizes serial traffic and improves overall responsiveness.

### Project 3: Duration Tracking and Timeout (`project3.py`)
An advanced implementation that introduces time management. It tracks the exact duration an obstacle remains in front of the robot (`ob_dur`). If the path remains blocked for more than 3 continuous seconds, the program actively breaks the loop and safely terminates. Additionally, it demonstrates the use of a custom non-blocking timing loop instead of a standard sleep function.

---

