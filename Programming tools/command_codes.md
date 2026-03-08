# Custom Firmata Extensions for Odysseas Robot

This repository contains a specialized version of the **Firmata protocol**, extended to support the unique hardware configuration of the **Odysseas Robot**. These extensions allow for high-level control of motors, sensors (Ultrasonic, Color, DHT, Light), and grid-based navigation via `SYSEX` messages.

## 🚀 Overview of Added Commands

The following commands have been implemented within the `switch` logic of the Firmata parser. They use custom Hex IDs to trigger specific robotic functions.

### 1. Movement & Actuators
| Command | Name | Description |
| :--- | :--- | :--- |
| `0x0C` | **DC Motors** | Controls two DC motors with speed, direction, and "refinement" (fine-tuning) parameters. |
| `0x0F` | **Servo Move** | Precision control for continuous rotation servos with speed offset logic. |
| `0x34` | **Level 1 Move** | Simplified movement (forward, backward, turns) designed for entry-level block programming. |
| `0x35` | **Stop** | Immediate halt of all movement based on the current motion type. |
| `0x3C/3D` | **Actuators** | High-level commands for "Continuous" or "Angle" actuators, including timed return-to-origin functions. |

### 2. Grid Navigation & Calibration
The robot supports a "Grid Mode" where it performs calibrated steps and turns based on timing values stored in EEPROM.
* **`0x0E` (Fix Step):** Starts/Ends the calibration process for a grid step. Calculates time per step and saves it to EEPROM.
* **`0x1B` (Execute Step):** Executes a precise forward/backward step or 90° turn using the stored calibration data.

### 3. Sensing & Perception
| Command | Sensor | Functionality |
| :--- | :--- | :--- |
| `0xCA/CB` | **Ultrasonic** | Returns distance in cm. `0xCB` is an optimized, parameterless version for quick pings. |
| `0x21` | **Color Init** | Initializes the TCS34725 RGB sensor and plays a success/fail melody. |
| `0x27/0x32` | **Color Match** | Matches current readings against saved EEPROM color profiles using two different algorithms (Absolute Distance vs. Percentage Tolerance). |
| `0x3F` | **DHT** | Returns Temperature and Humidity. Includes a 2-second non-blocking guard for sensor stability. |
| `0x52` | **BH1750** | Returns ambient light intensity in Lux via I2C. |
| `0x51/0x41` | **SI1151** | Returns Visible and Infrared (IR) light levels. |

### 4. System & Feedback
* **`0x0B/0x0D` (Audio):** Functions for playing frequencies (`tone`) or triggering pre-defined melodies stored in the robot's memory.
* **`0x31` (I2C Display):** Sends image indices to an external display module.
* **`0x40` (LEDs):** Controls the status of left/right integrated LEDs via I2C.

---

## 🛠 Technical Implementation Details

### Bit Packing & Unpacking
Since Firmata SysEx messages use 7-bit data bytes, 16-bit and 32-bit values are reconstructed using bitwise operations. 

**Example: Reconstructing `time_of_step` from EEPROM:**
```cpp
time_reformed_step = byte1 | (byte2 << 7) | (byte3 << 14) | (byte4 << 21) | (byte5 << 28);