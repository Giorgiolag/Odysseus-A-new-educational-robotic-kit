from pyfirmata import Arduino, SERVO, util
import time
import math
S1 = 1
s1 = 1
S2 = 2
s2 = 2
S3 = 3
s3 = 3
S4 = 4
s4 = 4
dc1 = 1
DC1 = 1
dc2 = 2
DC2 = 2
class odysseus:
    def __init__(self, port):
        board = Arduino(port)
        self.board = board
        self.board.add_cmd_handler(0xCB, self._handle_report_us_response)
        self.board.add_cmd_handler(0x20, self._handle_active_colors_response)
        self.board.add_cmd_handler(0x24, self._handle_red_response)
        self.board.add_cmd_handler(0x25, self._handle_green_response)
        self.board.add_cmd_handler(0x26, self._handle_blue_response)
        self.board.add_cmd_handler(0x32, self._handle_fixed_color_similarity_response)
        self.board.add_cmd_handler(0x28, self._handle_fixed_color_red_response)
        self.board.add_cmd_handler(0x29, self._handle_fixed_color_green_response)
        self.board.add_cmd_handler(0x30, self._handle_fixed_color_blue_response)
        self.board.add_cmd_handler(0x3F, self._handle_dht_data)
        self.board.fixed_color_similarity_value = None
        self.board.fixed_color_similarity_status = "not ready"
        self.board.fixed_color_red = None
        self.board.fixed_color_green = None
        self.board.fixed_color_blue = None
        self.board.fixed_color_status = "not ready"
        self.board.red_value = None
        self.board.green_value = None
        self.board.blue_value = None
        self.board.rgb_status = "not ready"
        self.board.us_value = None
        self.board.us_status = "not ready"
        self.board.sharp_ir_status = "not ready"
        self.board.sharp_ir_value = None
        self.board.active_colors_byte1 = None
        self.board.active_colors_byte2 = None
        self.board.active_colors_status = "not ready"
        self.board.servos_data = [[2, 1500],[4, 1500],[7, 1500],[8, 1500]]
        #self.board.DC_data = [[-1,-1,-1,-1], [-1,-1,-1,-1]]
        it = util.Iterator(self.board)
        self.board.dht_data = [-200,-200]
        self.board.dht_data_status = "not ready"
        it.start()

    def soil_moisture(self, sensor_slot):
        """
        a soil moisture sensor must be plugged into <sensor slot> in oder to use this function
        :param sensor_slot: it may be "slot-1", "slot-2", "slot-3", or "slot-4" numbering from left to right
        :return: the value returned by the soil moisture sensor
        """
        slots = {
            "slot-1": 0,
            "slot-2": 1,
            "slot-3": 2,
            "slot-4": 3,
        }
        self.board.analog[slots[sensor_slot]].enable_reporting()
        soil_mois = self.board.analog[slots[sensor_slot]].read()
        while soil_mois is None:
            soil_mois = self.board.analog[slots[sensor_slot]].read()
        return soil_mois


    def dht(self, sensor_model, sensor_slot):
        """

        :param sensor_slot: it may be "slot-1", "slot-2", "slot-3", or "slot-4" numbering from left to right
        :return: it is a list of two elements, the first of which is temperature and the second is air humidity. Both
                 list elements are percentages
        """
        slots = {
            "slot-1": 1,
            "slot-2": 2,
            "slot-3": 3,
            "slot-4": 4,
        }
        self.board.dht_data_status = "not ready"
        msg = bytearray([slots[sensor_slot]])
        self.board.send_sysex(0x3F, [int(sensor_model), slots[sensor_slot]])
        while self.board.dht_data_status == "not ready":
            pass
        return self.board.dht_data

    def _handle_dht_data(self, byte1, byte2, byte3):
        if byte2 == 1:
            self.board.dht_data[0] = byte3
        else:
            self.board.dht_data[0] = -int(byte3)
        self.board.dht_data[1] = byte1
        self.board.dht_data_status = "ready"


    def sharp_ir_value(self, sensor_slot):
        """

        :param sensor_slot : it may be "slot-1", "slot-2", "slot-3", or "slot-4" numbering from left to right
        :return: the output of the sharp IR distance sensor in volts. It is up to the user to transfotm this v
                 value to centimeters, according to the specifications of the product.
        """
        slots = {
            "slot-1": 0,
            "slot-2": 1,
            "slot-3": 2,
            "slot-4": 3,
        }
        self.board.analog[slots[sensor_slot]].enable_reporting()
        ir_distance = self.board.analog[slots[sensor_slot]].read()
        while ir_distance is None:
            ir_distance = self.board.analog[slots[sensor_slot]].read()
        return ir_distance*5/1.024

    def activate_colors(self, c1="", c2 = "", c3 = "", c4="", c5="", c6="", c7="", c8="", c9=""):
        """

        :param c1: can be "red", "green", blue", "yellow", "magenta", "cyan", "floor", "black" or "white"
        :param c2: can be "red", "green", blue", "yellow", "magenta", "cyan", "floor", "black" or "white"
        :param c2: can be "red", "green", blue", "yellow", "magenta", "cyan", "floor", "black" or "white"
        :param c4: can be "red", "green", blue", "yellow", "magenta", "cyan", "floor", "black" or "white"
        :param c5: can be "red", "green", blue", "yellow", "magenta", "cyan", "floor", "black" or "white"
        :param c6: can be "red", "green", blue", "yellow", "magenta", "cyan", "floor", "black" or "white"
        :param c7: can be "red", "green", blue", "yellow", "magenta", "cyan", "floor", "black" or "white"
        :param c8: can be "red", "green", blue", "yellow", "magenta", "cyan", "floor", "black" or "white"
        :param c9: can be "red", "green", blue", "yellow", "magenta", "cyan", "floor", "black" or "white"
        """
        colors = 0
        colors2 = 0
        color_list = ["red","green", "blue", "cyan", "yellow", "magenta", "floor"]
        if c1 in color_list:
            colors = colors+math.pow(2, color_list.index(c1))
        if c2 in color_list:
            colors = colors + math.pow(2, color_list.index(c2))
        if c3 in color_list:
            colors = colors + math.pow(2, color_list.index(c3))
        if c4 in color_list:
            colors = colors + math.pow(2, color_list.index(c4))
        if c5 in color_list:
            colors = colors + math.pow(2, color_list.index(c5))
        if c6 in color_list:
            colors = colors + math.pow(2, color_list.index(c6))
        if c7 in color_list:
            colors = colors + math.pow(2, color_list.index(c7))
        if c8 in color_list:
            colors = colors + math.pow(2, color_list.index(c8))
        if c9 in color_list:
            colors = colors + math.pow(2, color_list.index(c9))
        color_list2 = ["black", "white"]
        if c1 in color_list2:
            colors2 = colors2+math.pow(2, color_list2.index(c1))
        if c2 in color_list2:
            colors2 = colors2 + math.pow(2, color_list2.index(c2))
        if c3 in color_list2:
            colors2 = colors2 + math.pow(2, color_list2.index(c3))
        if c4 in color_list2:
            colors2 = colors2 + math.pow(2, color_list2.index(c4))
        if c5 in color_list2:
            colors2 = colors2 + math.pow(2, color_list2.index(c5))
        if c6 in color_list2:
            colors2 = colors2 + math.pow(2, color_list2.index(c6))
        if c7 in color_list2:
            colors2 = colors2 + math.pow(2, color_list2.index(c7))
        if c8 in color_list2:
            colors2 = colors2 + math.pow(2, color_list2.index(c8))
        if c9 in color_list2:
            colors2 = colors2 + math.pow(2, color_list2.index(c9))
        msg = bytearray([int(colors), int(colors2)])
        self.board.send_sysex(0x1F, msg)
        active_color_list = []
        for i in range(0, 8):
            if colors % 2 == 1:
                active_color_list.append(color_list[i])
            colors = int(colors/2)
        for i in range(0, 2):
            if colors2 % 2 == 1:
                active_color_list.append(color_list2[i])
            colors2 = int(colors2/2)
        return active_color_list

    def active_colors(self):
        """

        :return: the list of activated colors. Each element of this list is an element in the list
                 ["red", "green", "blue", "cyan", "yellow", "magenta", "floor", "black", "white"]
        """
        self.board.active_colors_status = "not ready"
        self.board.send_sysex(0x20, [])
        while self.board.active_colors_status == "not ready":
            pass
        while self.board.active_colors_byte1 == None or self.board.active_colors_byte2 == None:
            pass
        color_list = ["red", "green", "blue", "cyan", "yellow", "magenta", "floor"]
        color_list2 = ["black", "white"]
        active_color_list = []
        colors = self.board.active_colors_byte1
        colors2 = self.board.active_colors_byte2
        for i in range(0, 8):
            if colors % 2 == 1:
                active_color_list.append(color_list[i])
            colors = int(colors / 2)
        for i in range(0, 2):
            if colors2 % 2 == 1:
                active_color_list.append(color_list2[i])
            colors2 = int(colors2 / 2)
        return active_color_list

    def _handle_active_colors_response(self, res1, res2):
        self.board.active_colors_byte2 = res1
        self.board.active_colors_byte1 = res2
        self.board.active_colors_status = "ready"

    def init_color_sensor(self):
        """

        :return: nothing, this function initializes the color sensor it must be executed for the color sensor to
                 work properly. Executing it once is enough. Some time is needed for the initialization to take place.
                 We use 1 second for this purpose.
        """
        self.board.send_sysex(0x21, [])
        time.sleep(1)

    def color_sensor_value(self):
        """

        :return: the value for red, green and blue for the color detected by the color sensor
        """
        self.board.rgb_status = "not ready"
        self.board.send_sysex(0x23, [])
        self.board.send_sysex(0x24, [])
        self.board.send_sysex(0x25, [])
        self.board.send_sysex(0x26, [])
        while self.board.rgb_status == "not ready":
            pass
        while self.board.red_value == None:
            pass
        while self.board.green_value == None:
            pass
        while self.board.blue_value == None:
            pass
        return self.board.red_value, self.board.green_value, self.board.blue_value

    def fixed_color(self, color):
        """

        :param color: it is an element of the list
                      ["red", "green", "blue", "cyan", "yellow", "magenta", "floor", "black", "white"]
        :return: the red, green, and blue value for the color stored in EEPROM as parameter <<color>> if
                 parameter <<color>> belongs to the color_list, otherwise it returns -1, -1, -1
        """
        color_list = ["red", "green", "blue", "cyan", "yellow", "magenta", "floor", "black", "white"]
        self.board.fixed_color_status = "not ready"
        if color in color_list:
            color_number = color_list.index(color)+1
            msg = bytearray([color_number])
            self.board.send_sysex(0x28, msg)
            self.board.send_sysex(0x29, msg)
            self.board.send_sysex(0x30, msg)
            while self.board.fixed_color_status == "not ready":
                pass
            while self.board.fixed_color_red == None:
                pass
            while self.board.fixed_color_green == None:
                pass
            while self.board.fixed_color_blue == None:
                pass
            return self.board.fixed_color_red, self.board.fixed_color_green, self.board.fixed_color_blue
        else:
            return -1, -1, -1

    def _handle_fixed_color_red_response(self, res1, res2):
        self.board.fixed_color_red = int((res1 | (res2 << 7)))

    def _handle_fixed_color_green_response(self, res1, res2):
        self.board.fixed_color_green = int((res1 | (res2 << 7)))

    def _handle_fixed_color_blue_response(self, res1, res2):
        self.board.fixed_color_blue = int((res1 | (res2 << 7)))
        self.board.fixed_color_status = "ready"

    def _handle_fixed_color_similarity_response(self, res):
        print(res)
        color_list = ["red", "green", "blue", "cyan", "yellow", "magenta", "floor", "black", "white"]
        if res <=8:
            self.board.fixed_color_similarity_value = color_list[res]
        else:
            self.board.fixed_color_similarity_value = "unknown"
        self.board.fixed_color_similarity_status = "ready"


    def matched_fixed_color(self, percentage):
        """

        :param percentage: similarity frame, it can be between 0 and 100. if it is too small, slight variations
                           in the sensor's output mat cause failure of color recognition. If it is too big then
                           the detected color may be found to be close to more than one colors which means that again,
                           we have a failure of color recognition. A value around 5 is logical, but the user should
                           experiment to find the optimal value given the  (light, "closeness" of supported colors etc)
        :return: an element of the list
                           ["red", "green", "blue", "cyan", "yellow", "magenta", "floor", "black", "white", "unknown"]
                           The value "unknown" is return if a failure of color recognition has occured  i.e.
                           -->  if there is no (stored in EEPROM) color "close enough" the one detected (in which case
                                we have to increase parameter <<percentage>> or
                           -->  if there are more than one (stored in EEPROM) colors "close enough" the one detected
                                in which case we have to decrese parameter <<percentage>> or

        """
        self.board.fixed_color_similarity_status = "not ready"
        msg = bytearray([percentage])
        self.board.send_sysex(0x32, msg)

        while self.board.fixed_color_similarity_status == "not ready":
            pass
        while self.board.fixed_color_similarity_value == None:
            pass
        return self.board.fixed_color_similarity_value



    def _handle_red_response(self, res1, res2):
        self.board.red_value = int((res1 | (res2 << 7)))

    def _handle_green_response(self, res1, res2):
        self.board.green_value = int((res1 | (res2 << 7)))

    def _handle_blue_response(self, res1, res2):
        self.board.blue_value = int((res1 | (res2 << 7)))
        self.board.rgb_status = "ready"

    def fix_color(self, color):
        color_list = ["red", "green", "blue", "cyan", "yellow", "magenta", "floor", "black", "white"]
        if color in color_list:
            msg = bytearray([color_list.index(color)+1])
            self.board.send_sysex(0x1E, msg)
        else:
            print("wrong color in fix_color")

    def _handle_report_us_response(self, res1, res2):
        self.board.us_value = int((res1 | (res2 << 7)))
        self.board.us_status = "ready"

    def ultrasonic_distance(self):
        """

        :return: the distance in centimeters between the ultrasonic sensor and the obstacle
        """
        self.board.us_status = "not Ready"
        self.board.send_sysex(0xCB, [])
        while self.board.us_status == "not ready":
            pass
        while self.board.us_value == None:
            pass
        return self.board.us_value





    def move_servo(self, servo_number, servo_direction, servo_speed = 0, refinement="none", ref_value = 0):
        """
            sets a servo motor in motion.
            :param servo_number: can be :
                     --> 1 (or S1, or s1,or "s1" or "S1") indicating servo 1,
                     --> 2 (or S2, or s2 or "s2" or "S2") indicating servo 2,
                     --> 3 (or S3, or s3 or "s3" or "S3") indicating servo 3,
                     --> 4 (or S4, or s4 or "s4" or "S4") indicating servo 4
                        Numbers correspond to servo slots from left to right. If we orient Odysseus so that the motors
                        side is towards us, Servo 1 is the leftmost servo 3-pin slot and servo 4 is the rightmost one
            :param servo_direction: can be "clockwise", "counterclockwise" or "stopped"
            :param servo_speed:  can be between 0 and 10
            :param refinement:   can be "upwards", "downwards" or "none". It is used for achieving speeds between integer values
                         To achieve speed 7.5 one needs to set servo_speed to 7 and refine the servo_speed upwards.
                         The exact refinement value is explain in the next parameter, ref_value
            :param ref_value:    can be between 0 and 25. for example, setting speed = 7, refinement  = "upwards" we have:
                         if ref_value = 25, then speed = 7.5 (we can get the same speed if we set
                                                            servo_speed = 8 refinement = "downwards" and ref_value = 25)
                         if ref_ value = 10, then speed = 7.2
                         if ref_value = 0 then speed = 7
        """
        input_error = False
        final_speed = 0
        if servo_number == 1 or servo_number == "s1" or servo_number == "S1":
            new_servo_number = 2
            servo_data_position = 0
        elif servo_number == 2 or servo_number == "s2" or servo_number == "S2":
            new_servo_number = 4
            servo_data_position = 1
        elif servo_number == 3 or servo_number == "s3" or servo_number == "S3":
            new_servo_number = 7
            servo_data_position = 2
        elif servo_number == 4 or servo_number == "s4" or servo_number == "S4":
            new_servo_number = 8
            servo_data_position = 3
        else:
            input_error = True
        if servo_direction != "clockwise" and servo_direction != "counterclockwise" and servo_direction != "stopped":
            input_error = True
        if servo_speed <0 or servo_speed >10:
            input_error = True
        if refinement != "upwards" and refinement != "downwards" and refinement != "none":
            input_error = True
        if ref_value < 0 or ref_value >25:
            input_error = True
        if input_error == False:
            if servo_direction == "clockwise":
                final_speed = 1500+50*servo_speed
                if refinement == "upwards":
                    final_speed = final_speed+ref_value
                elif refinement == "downwards":
                    final_speed = final_speed - ref_value

            if servo_direction == "counterclockwise":
                final_speed = 1500-50*servo_speed
                if refinement == "upwards":
                    final_speed = final_speed-ref_value
                elif refinement == "downwards":
                    final_speed = final_speed + ref_value
            if servo_direction == "stopped":
                final_speed = 1500
            if self.board.servos_data[servo_data_position][1] != final_speed:
                self.board.digital[int(new_servo_number)].mode = SERVO
                self.board.digital[int(new_servo_number)].write(final_speed)
                self.board.servos_data[servo_data_position][1] = final_speed
        else:
            print("move_servo function: error in input parameters")


    def move_DC_motor(self, DC_motor_number, DC_motor_direction, DC_motor_speed = 0, refinement = "none", ref_value=0):
        """
        sets a DC motor in motion
        :param DC_motor_number: can be:
                         --> 1 (or, alternatively, DC1, dc1, "dc1", DC1"),
                         --> 2 (or, alternatively, DC2, dc2, "dc2", DC2")
                         Numbers correspond to DC-motor slots from left to right. if we orient Odysseus so that the
                         motors side is towards us, DC motor 1 is the leftmost 2-pin slot and DC motor 2 is the
                         rightmost one
        :param DC_motor_direction: :param servo_direction: can be: "clockwise", "counterclockwise" or "stopped"
        :param DC_motor_speed: can be between 0 and 10
        :param refinement: can be "upwards", "downwards" or "none". It is used for achieving speeds between integer values
                         To achieve speed 7.5 one needs to set servo_speed to 7 and refine the servo_speed upwards.
                         The exact refinement value is explain in the next parameter, ref_value
        :param ref_value: can be between 0 and 20. for example, setting speed = 7, refinement  = "upwards" we have:
                         if ref_value = 20, then speed = 7.5 (we can get the same speed if we set
                                                            servo_speed = 8 refinement = "downwards" and ref_value = 25)
                         if ref_ value = 4, then speed = 7.1
                         if ref_value = 0 then speed = 7
        """
        input_error = False
        if ( DC_motor_number != 1 and DC_motor_number != 2
             and DC_motor_number != dc1 and DC_motor_number != dc2 and
             DC_motor_number != "dc1" and DC_motor_number != "dc2" and
             DC_motor_number != "DC1" and DC_motor_number != "DC2"):
            input_error = True
        if DC_motor_direction == "clockwise":
            new_DC_motor_direction = 0
        elif DC_motor_direction == "counterclockwise":
             new_DC_motor_direction = 1
        elif DC_motor_direction == "stopped":
            new_DC_motor_direction = 2
        else:
            input_error = True
        if DC_motor_speed < 0 or DC_motor_speed > 10:
            input_error = True
        if refinement == "upwards":
            new_refinement = 1
        elif refinement == "downwards":
            new_refinement = 2
        elif refinement == "none":
            new_refinement = 0
        else:
            input_error = True
        if ref_value < 0 or ref_value > 25:
            input_error = True
        if input_error == False:
            if DC_motor_number == "dc1" or DC_motor_number == "DC1":
                DC_motor_number = 1
            if DC_motor_number == "dc2" or DC_motor_number == "DC2":
                DC_motor_number = 2
            msg = bytearray([DC_motor_number, new_DC_motor_direction, DC_motor_speed, new_refinement, ref_value])
            self.board.send_sysex(0x0C, msg)
        else:
            print("move_DC_motor function: error in input parameters")

    def print(self, image):
        """
        :param image: The supported images are: "odusseas" (face), "angry", "angry", "in love", "winking", "forward",
                      "backward", "left", "right", "object detected", "color detected", "flipped" (this is not a image,
                       but an instruction that changes the orientation of the display)

        """
        images = {
            "odysseus" : 3,
            "angry" : 4,
            "smiling" : 5,
            "in love" : 6,
            "winking" : 7,
            "forward" : 8,
            "backward": 9,
            "left" : 10,
            "right" : 11,
            "object detected" : 21,
            "color detected" : 22,
            "flipped": 27

        }
        if image in images:
            self.board.send_sysex(0x76, [1,0,images[image]])
        else:
            print("this image does not exist")

    def sing(self, song):
        """


        :param song: The supported songs are: "green Sleeves", "Mary had a little lamb", "happy birthday",
                     "star wars", "chariots of fire"

        """
        songs = {
            "green Sleeves" : 0,
            "Mary had a little lamb" : 1,
            "happy birthday" : 2,
            "star wars" : 3,
            "chariots of fire" : 4
        }
        msg = bytearray([songs[song]])
        self.board.send_sysex(0x0D, msg)

    def beep(self):
        msg = bytearray([0x09])
        self.board.send_sysex(0x0D, msg)

    def play_tone(self, freq, dur):
        dur = dur & 0xFFFFFFFF  # Clamping value to 32 bits

        data = [
            (dur >> 25) & 0x7F,
            (dur >> 18) & 0x7F,
            (dur >> 11) & 0x7F,
            (dur >> 4) & 0x7F,
            ((dur << 3) & int("01111000", 2)) | ((freq >> 13) & int("0111", 2)),
            (freq >> 6) & 0x7F,
            (freq >> 1) & 0x7F,
            freq & 0x7F
        ]
        msg = bytearray(data)
        self.board.send_sysex(0x0B, msg)

    def start_tone(self, freq):
        dur = 0
        dur = dur & 0xFFFF
        data = [
            (dur >> 25) & 0x7F,
            (dur >> 18) & 0x7F,
            (dur >> 11) & 0x7F,
            (dur >> 4) & 0x7F,
            ((dur << 3) & int("01111000", 2)) | ((freq >> 13) & int("0111", 2)),
            (freq >> 6) & 0x7F,
            (freq >> 1) & 0x7F,
            freq & 0x7F
        ]
        msg = bytearray(data)
        self.board.send_sysex(0x0B, msg)

    def end_tone(self):
        dur = 1000
        freq = 0
        dur = dur & 0xFFFF
        data = [
            (dur >> 25) & 0x7F,
            (dur >> 18) & 0x7F,
            (dur >> 11) & 0x7F,
            (dur >> 4) & 0x7F,
            ((dur << 3) & int("01111000", 2)) | ((freq >> 13) & int("0111", 2)),
            (freq >> 6) & 0x7F,
            (freq >> 1) & 0x7F,
            freq & 0x7F
        ]
        msg = bytearray(data)
        self.board.send_sysex(0x0B, msg)

    def lights(self, side, status):
        """
        :param side: it can be "left" (for the left led) or right (for the right led). The led slots are the 2-pin
                     slots between servo slot 1 and servo slot 4
        :param status: it can be "on", "off" or "blink"
        """
        if side == "left" and status == "on":
            to_send = 16
        if side == "left" and status == "off":
            to_send = 15
        if side == "left" and status == "blink":
            to_send = 17
        if side == "right" and status == "on":
            to_send = 18
        if side == "right" and status == "off":
            to_send = 19
        if side == "right" and status == "blink":
            to_send = 20
        msg = bytearray([to_send])
        self.board.send_sysex(0x76, [1, 0, msg])


    def obstacle(self, sensor_slot):
        """
        an IR obstacle avoidance sensor must be plugged into <sensor slot> in oder to use this function.
        :param sensor_slot: it may be "slot-1", "slot-2", "slot-3", or "slot-4" numbering from left to right
        :return: True if obstacle is detected, False otherwise
        """
        slots = {
            "slot-1": 0,
            "slot-2": 1,
            "slot-3": 2,
            "slot-4": 3,
        }
        self.board.analog[slots[sensor_slot]].enable_reporting()
        obs = self.board.analog[slots[sensor_slot]].read()
        while obs is None:
            obs = self.board.analog[slots[sensor_slot]].read()
        if obs > 0.5:
            return False
        else:
            return True

    def line(self, sensor_slot):
        """
                an IR obstacle avoidance sensor must be plugged into <sensor slot> in oder to use this function.
                To detect the line, the sensor must be facing down.
                :param sensor_slot: it may be "slot-1", "slot-2", "slot-3", or "slot-4" numbering from left to right
                :return: True if line is detected, False otherwise
                """
        slots = {
            "slot-1": 0,
            "slot-2": 1,
            "slot-3": 2,
            "slot-4": 3,
        }
        self.board.analog[slots[sensor_slot]].enable_reporting()
        obs = self.board.analog[slots[sensor_slot]].read()
        while obs is None:
            obs = self.board.analog[slots[sensor_slot]].read()
        if obs > 0.5:
            return True
        else:
            return False

    def line(self, sensor_slot):
        """
        an IR obstacle avoidance sensor must be plugged into <sensor slot> in oder to use this function
        :param sensor_slot: it may be "slot-1", "slot-2", "slot-3", or "slot-4" numbering from left to right
        :return: True if black line is detected on th floor, False otherwise

        :param sensor_slot:

        """
        return not self.obstacle(sensor_slot)


