# Odysseus Robot: Block Command Reference

This documentation describes the specialized block commands for the **Odysseus** robot, bridging the gap between low-level hardware and high-level educational applications. Information on how to program Ody using Snap4Arduino and the specialized commands can also be found in [1] and in the YouTube channel [2] that contains the programming lessons (bibliography is included at the bottom of this page). In [2] there is a video dedicated to the connection procedure, i.e. to the actions that must be performed by the user in order to connect Snap4Arduino with Ody. 

The command blocks that follow are included in "Odysseus blocks.xml", contained in the current folder. To import the blocks, just click "Import..." after clicking on the leftmost menu icon at the top left part of the Snap4Arduino window.

## 🔦 Infrared (IR) Binary Sensors
These blocks use standard IR sensors to interact with the environment. Users must specify the **sensor slot** where the hardware is connected. Such sensors can be used either for detecting obstacles or for detecting lines. The images of the blocks appear in Figure 1.

| Block Name | Block Image | Application |
| :--- | :---: | :--- |
| **Obstacle detected at sensor [slot]** | 1 | It returns True (if an obstacle is in front of the sensor) or False (otherwise). The detection distance can be adjusted with a screwdriver but it is generally very small (practically less than 5 cm). |
| **Line detected at sensor [slot]** | 2 | It returns True (if the surface it faces is dark) or False (otherwise). The distance from the surface must be very small. |

<p align="center">
  <img src="block images/ir.png" width="400"><br>
  <em>Figure 1: Visualization of the IR sensor blocks for obstacle and line detection.</em>
</p>

---

## 🎨 Color Sensing Framework

### The Concept of Basic Colors in Odysseus

To bridge the gap between abstract sensor data and tangible programming, Odysseus utilizes a specialized framework of **Basic Colors** (Red, Green, Blue, Cyan, Magenta, Yellow, Floor, Black, and White). Instead of requiring users to manipulate volatile RGB numbers, the system allows them to identify elements in a list as distinct colors. This process of **"materializing" list elements** is a crucial pedagogical tool for teaching data structures to children.

### Activation and "Fixing"
The framework operates through two primary stages of configuration to ensure reliability in real-world environments:
* **Fixing**: Because sensors are highly sensitive to lighting variations, users must "fix" colors to ensure accuracy. By placing the color sensor over a physical colored object and executing a **"FIX"** command, the robot stores the current RGB values into the **EEPROM** as the permanent reference for that basic color.
* **Activation**: The user can easily activate or deactivate specific basic colors to suit the current educational scenario. By deactivating irrelevant colors, the system reduces the risk of false matches.

### Similarity and Matching
Once colors are fixed and activated, Odysseus matches detected objects to the closest basic color. To account for the slight variations that occur during detection due to lighting or distance, users can define a **Similarity Frame** (a sensitivity parameter). This defines "close enough" boundaries, ensuring that variations in the same color are not identified as "unknown" or as a different color entirely.

### The blocks
These blocks exploit the TCS 34725 I2C color sensor. The images of the blocks appear in Figure 2. All but the last two belong to the "Sensors" category but the last two belong to the "Configuration" category.

### ⚙️ Command Overview
| Block Name | Block Image | Description |
| :--- | :---: | :--- |
| **Activated / inactivated colors** | 1 | Prints the activated basic colors. |
| **Matched fixed color within similarity frame (0-100%)** | 2 | Matches detection to a fixed color using a percentage-based sensitivity. |
| **RGB values detected by color sensor** | 3 | Returns raw Red, Green, and Blue numerical data. |
| **RGB values of fixed [Color]** | 4 | Returns the values for Red, Green, and Blue stored in the EEPROM of the given basic color. |
| **Initialize color sensor** | 5 | **Mandatory.** Must be executed before any other color command. |
| **Sensors: FIX [Color]** | 6 | Stores current RGB values into the **EEPROM** as the color reference for the given basic color. |
| **Sensors: activate / inactivate colors** | 7 | Toggles which Basic Colors are relevant to the current task. |

<p align="center">
  <img src="block images/color_sensor.png" width="600"><br>
  <em>Figure 2: Exploiting the TCS 34725 color sensor.</em>
</p>

---

## 📏 Distance, Environment & Light Sensors
Numerical readings for spatial awareness and environmental monitoring. The images of the blocks appear in Figure 3.

| Sensor Type | Block image | Description |
| :--- | :--- | :--- |
| **Ultrasonic** | 1 | Measures distance in cm via ultrasound. |
| **Sharp IR sensor output in volts at sensor [slot]** | 2 | Raw voltage output of the distance sensor. |
| **Sharp IR sensor in product [range] output (cm) at sensor [slot]** | 3 | Distance of obstacle in cm. The user must also specify the minimum and maximum range of the sensor as there exist more than one version of this sensor and each version is characterized by the minimum and maximum detection range. |
| **Soil Moisture sensor output at sensor [slot]** | 4 | It returns the raw output of the soil moisture sensor. |
| **Soil Moisture (0 to 100) sensor output at sensor [slot] on dry soil [value] and water [value]** | 5 | It returns the soil moisture in the form of a percentage (0 to 100). In order for this to be achieved the user must insert the raw output of the sensor when the soil is completely dry (just hold it in the air) and the raw output of the sensor when the soil has maximum moisture (just put it in water). |
| **Temperature / Humidity DHT [11] at sensor [slot]** | 6 | It returns both temperature and humidity detected by the DHT sensor plugged into the given slot. |
| **Temperature DHT [11] at sensor [slot]** | 7 | It returns the temperature detected by the DHT sensor plugged into the given slot. |
| **Humidity DHT [11] at sensor [slot]** | 8 | It returns the humidity (0 to 100) detected by the DHT sensor plugged into the given slot. |
| **(SI1151) Visible Light Measurement** | 9 | Measures visible light intensity. |
| **(SI1151) IR Light Measurement** | 10 | Measures infrared light intensity. |

<p align="center">
  <img src="block images/distance and environmental sensors.png" width="600"><br>
  <em>Figure 3: Distance and environmental sensors.</em>
</p>

---

## 🎮 Youngsters & Game-Based Logic
Designed for very young learners, these blocks abstract away technical details like loops or servo speed settings, presenting only "game-ready" actions. Before children play, an educator uses configuration blocks to store parameters in the robot's permanent memory. 

### 🛠 Configuring Ody for easy moving
By moving, we mean moving forward, backward, turning left and turning right. The blocks through which easy moving is achieved appear in Figure 4. The first block in Figure 4 is a configuration block while the rest belong to the "Youngsters" category.

| Configuration Block | Block Image | Purpose |
| :--- | :---: | :--- |
| **STORE movement parameters** | 1 | It is a configuration block supposed to be executed by an educator. It predefines wheel slots and speed settings for basic move (forward, backward, turn left, turn right). After this configuration step is performed for a basic move, this move can be achieved with the corresponding abstract block. The abstract blocks follow. |
| **Move forward** | 2 | Ody starts moving forward. |
| **Stop** | 3 | Ody stops moving. |
| **Move backward** | 4 | Ody starts moving backward. |
| **Turn left** | 5 | Ody starts turning left. |
| **Turn right** | 6 | Ody starts turning right. |

<p align="center">
  <img src="block images/config step1.png" width="600"><br>
  <em>Figure 4: Easy moving.</em>
</p>


### 🛠 Configuring Ody for grid games
Grid games involve moving on a grid composed of rectangular "areas". The robot should be able to move in regard to this grid. To achieve this, Ody navigates by transitioning between these areas and making precise 90-degree turns. Before using these grid commands, the system requires a brief calibration process so the Arduino can learn the exact timing for these movements. Once configured, youngsters use simple, intuitive blocks to "drive" the robot or solve puzzles on maps and grids. The images of the blocks relative to grid games appear in Figure 5. The first 3 blocks of Figure 5 are the configuration blocks, while the rest belong to the "Youngsters" category.

| Block Name | Block Image | Function |
| :--- | :---: | :--- |
| **Start fixing [movement] step for [steps]** | 1 | It is used to start the configuration task for steps forward or backward. The number of steps is given by the user. For example, if 10 steps are to be performed, the time to perform a single step will be divided by 10 when the fixing step ends. |
| **End fixing step or turn** | 2 | When executed, it ends the step or turn fixing. For example, if the user has set the number of steps to 10 (when the configuration started), the user must end the fixing step when Ody reaches its destination (i.e. has made 10 steps). Then, the exact time for performing a step or a 90 degree turn is calculated and stored in the EEPROM. |
| **Start fixing [movement] turn and 360 degree [turns]** | 3 | It is used to start the configuration task for turning 90 degrees left or right. However, the exact time for such a turn is derived through 360 degree turns. The number of 360 degrees turns to be performed is given by the user. For example, if 10 360 degree turns are to be performed, the time to perform a 90 degree turn will be divided by 40 when the fixing step ends. |
| **Step backward** | 4 | Ody moves backward and stops when it has reached the neighboring rectangular area. |
| **Step forward** | 5 | Ody moves forward and stops when it has reached the neighboring rectangular area. |
| **Turn 90 degrees to the right** | 6 | Ody turns 90 degrees to the right. |
| **Turn 90 degrees to the left** | 7 | Ody turns 90 degrees to the left. |

<p align="center">
  <img src="block images/config step 2.png" width="600"><br>
  <em>Figure 5: Configuration and moves for grid games.</em>
</p>

---
### 🛠 Configuring Ody for map games
Map games are introduced through lines that Ody must follow, via two obstacle sensors. To configure Ody for map games, the educator needs to define the slots where these sensors are plugged in. Once this is done, (assuming that Ody is already configured for easy moves) abstract blocks for moving on a map become available to users. In advance, if an ultrasonic sensor is used, a separate configuration step allows for abstractly detecting obstacles. Using these blocks, Ody can follow a path to a specific endpoint, even navigating crossroads along the way. The images of the relevant blocks appear in Figure 6. The first 2 blocks in Figure 6 belong to the "Configuration" category, while the rest belong to the "Youngsters" category.

| Block Name | Block Image | Function |
| :--- | :---: | :--- |
| **Store slots of line_trackers. Set [left_tracker]. Set [right_tracker]** | 1 | It stores in the EEPROM the slot where the obstacle sensor detecting a line on the left and the obstacle sensor detecting a line on the right are plugged (The line is followed if it is always located between the two trackers). |
| **Store minimum distance from obstacle** | 2 | It is a configuration block. It stores in the EEPROM the detection distance i.e. an obstacle will be detected if the distance reported by the ultrasonic sensor is not bigger than the distance given by the user. |
| **Follow track** | 3 | Follow the line located between the two trackers. Stop when an obstacle or when a crossroad is detected. |
| **Turn left until the line is again between the trackers, but Ody faces the opposite direction** | 4 | It is used to prepare Ody to follow line between the trackers but to the opposite direction. |
| **Turn right until the line is again between the trackers, but Ody faces the opposite direction** | 5 | It is used to prepare Ody to follow line between the trackers but to the opposite direction (the difference from the previous block is the direction of the turn). |
| **Choose left track** | 6 | When Ody has reached a 2-way crossroad, this block makes Ody place the line to the left between the trackers. |
| **Choose right track** | 7 | When Ody has reached a 2-way crossroad, this block makes Ody place the line to the right between the trackers. |

<p align="center">
  <img src="block images/config step 3.png" width="600"><br>
  <em>Figure 6: Configuration and moves for map games.</em>
</p>


### 🛠 Configuring an actuator
Users can abstractly move an actuator after some configuration. The ability to abstractly move an actuator is introduced to enhance the engagement and excitement of educational scenarios, helping to capture and sustain users’ attention and enthusiasm. The relevant blocks appear in Figure 7. The last block in Figure 7 belongs to the "Youngsters" category whereas the rest of the blocks belong to the "Configuration" category. 

| Block Name | Block Image | Function |
| :--- | :---: | :--- |
| **Set actuator type as [type]** | 1 | The user can define the type of actuator to be used which can be a continuous servo motor or an angle servo motor. |
| **Set actuator [slot] to move [direction] for [seconds]. Reverse afterwards [yes/no]** | 2 | In case that the actuator is configured to be a continuous servo motor, this block can be used to define the exact move. The user can define the servo slot, the direction of the move and the duration of the move. Then the move of the actuator either ends (if reverse = no) or the actuator completes the move by moving to the opposite direction for the same amount of time (reverse = yes). |
| **Set actuator [slot] initial position [angle] target position [angle] return to initial position after [seconds]** | 3 | In case that the actuator is configured to be an angle servo motor, this block can be used to define the exact move. The user can define the servo slot, the initial angle of the servo motor and the final angle. Then the user sets the time during which the servo will remain in its final angle. When this time passes, the servo returns to its initial position. |
| **Move actuator** | 4 | After the configuration steps in regard to the actuator, the users can perform the already defined move by using this abstract block. |

<p align="center">
  <img src="block images/config step 4.png" width="600"><br>
  <em>Figure 7: Configuration and abstract move of an actuator.</em>
</p>


## Blocks regarding typical outputs
Typical outputs are the motors, the OLED display, the LEDs and the buzzer. The blocks handling the motors are placed in the same category and can be seen in Figure 8.

### Handling motors
The relevant blocks are included in the "Motors" category and appear in Figure 8. To handle a motor the user must first define the slot corresponding to the motor and then set direction and speed. For servo motors, fine tuning of the speed can be achieved by accordingly setting the refine and the value parameter. Lesson 1 of the YouTube course explains how to do this. 

| Block Name | Block Image | Function |
| :--- | :---: | :--- |
| **Set DC motor [slot] to [direction] at [speed] [refine] [value]** | 1 | Sets the direction (clockwise, counterclockwise or stop) and speed (0 to 10) of the DC motor plugged in the defined slot. The speed can be refined upwards or downwards to achieve values equal to, say, 7.5 or 8.6. |
| **Set servo motor [slot] to [direction] at [speed] [refine] [value]** | 2 | Sets the direction (clockwise, counterclockwise or stop) and speed (0 to 10) of the servo motor plugged in the defined slot. The speed can be refined upwards or downwards to achieve values equal to, say, 7.5 or 8.6. |
| **Set servo motor [slot] to [direction] at [speed]** | 3 | Sets the direction (clockwise, counterclockwise or stop) and speed (0 to 10) of the servo motor plugged in the defined slot. |
| **Set DC motor [slot] to [direction] at [speed]** | 4 | Sets the direction (clockwise, counterclockwise or stop) and speed (0 to 10) of the DC motor plugged in the defined slot. |

<p align="center">
  <img src="block images/motors.png" width="600"><br>
  <em>Figure 8: Handling motors.</em>
</p>

### OLED display, LEDs and sounds
The relevant blocks are included in the "Images, sounds, LEDs" category and appear in Figure 9. The blocks are straightforward and their description is omitted. However they are extensively explained in the YouTube course. The images of the blocks appear in Figure 9.

| Block Name | Block Image | Function |
| :--- | :---: | :--- |
| **Play tone [frequency] for [milliseconds]** | 1 | The buzzer produces a sound of the given frequency that lasts for the given number of milliseconds. |
| **Print [image]** | 2 | Displays the given image. |
| **[Number] LED [function]** | 3 | The function can be ON, OFF or blink. The user also gives the LED slot to apply the function. |
| **Play [song]** | 4 | A number of small songs are stored and can be played. |
| **Beep** | 5 | Produces a tone of standard frequency for a standard time period. |
| **Start tone [frequency]** | 6 | The buzzer starts producing a sound of the given frequency. The sound will continue until an "end tone" command is executed. |
| **End tone** | 7 | Stops the tone that started with the "start tone" block. |

<p align="center">
  <img src="block images/oled_led_sounds.png" width="600"><br>
  <em>Figure 9: Handling OLED display, LEDs and sounds.</em>
</p>

## Bibliography
1.  G. Lagogiannis, Empowering Computational Thinking through Personalized Robotics: The Odysseus Model, Int. J. Interact. Mob. Technol. 19 (2025) 21.
2.  Programming course, Odysseus Robotics Programming Lessons [video channel], YouTube, 2026. https://www.youtube.com/@Programmingcourse-i9j