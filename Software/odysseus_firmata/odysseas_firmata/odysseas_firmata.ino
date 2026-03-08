/*
   Odysseas firmata is a standard firmata version containing parts taken 
   from other versions and some new instructions created to ease the use 
   of Odysseas robot. Odysseas firmata has incorporated all the code from 
   the firmata library so the user does not have to install it and include it
   
  Firmata is a generic protocol for communicating with microcontrollers
  from software on a host computer. It is intended to work with
  any host computer software package.
  To download a host software package, please clink on the following link
  to open the list of Firmata client libraries your default browser.
  https://github.com/firmata/arduino#firmata-client-libraries
  Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2010-2011 Paul Stoffregen.  All rights reserved.
  Copyright (C) 2009 Shigeru Kobayashi.  All rights reserved.
  Copyright (C) 2009-2016 Jeff Hoefs.  All rights reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  See file LICENSE.txt for further informations on licensing terms.
  Last updated by Jeff Hoefs: January 10th, 2016
*/
#include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>
#include <dht.h>
#include "Si115X.h"
//#include <Adafruit_Sensor.h>    // https://github.com/adafruit/Adafruit_Sensor
//#include <Tiny_BME280.h>        // https://github.com/jasonacox/Tiny_BME280_Library
#include <BH1750.h>

#define SEALEVELPRESSURE_HPA (1013.25)

//Tiny_BME280 bme; 
dht DHT;


Si115X si1151;
BH1750 lightMeter;

//#define DECODE_NEC
//#include <IRremote.hpp>
#include "Adafruit_TCS34725.h"


/*
  Boards.h - Hardware Abstraction Layer for Firmata library
  Copyright (c) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2009-2017 Jeff Hoefs.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.

  Last updated April 15th, 2018
*/

#ifndef Firmata_Boards_h
#define Firmata_Boards_h

#include <inttypes.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"  // for digitalRead, digitalWrite, etc
#else
#include "WProgram.h"
#endif

// Normally Servo.h must be included before Firmata.h (which then includes
// this file).  If Servo.h wasn't included, this allows the code to still
// compile, but without support for any Servos.  Hopefully that's what the
// user intended by not including Servo.h
#ifndef MAX_SERVOS
#define MAX_SERVOS 0
#endif

/*
    Firmata Hardware Abstraction Layer

Firmata is built on top of the hardware abstraction functions of Arduino,
specifically digitalWrite, digitalRead, analogWrite, analogRead, and
pinMode.  While these functions offer simple integer pin numbers, Firmata
needs more information than is provided by Arduino.  This file provides
all other hardware specific details.  To make Firmata support a new board,
only this file should require editing.

The key concept is every "pin" implemented by Firmata may be mapped to
any pin as implemented by Arduino.  Usually a simple 1-to-1 mapping is
best, but such mapping should not be assumed.  This hardware abstraction
layer allows Firmata to implement any number of pins which map onto the
Arduino implemented pins in almost any arbitrary way.


General Constants:

These constants provide basic information Firmata requires.

TOTAL_PINS: The total number of pins Firmata implemented by Firmata.
    Usually this will match the number of pins the Arduino functions
    implement, including any pins pins capable of analog or digital.
    However, Firmata may implement any number of pins.  For example,
    on Arduino Mini with 8 analog inputs, 6 of these may be used
    for digital functions, and 2 are analog only.  On such boards,
    Firmata can implement more pins than Arduino's pinMode()
    function, in order to accommodate those special pins.  The
    Firmata protocol supports a maximum of 128 pins, so this
    constant must not exceed 128.

TOTAL_ANALOG_PINS: The total number of analog input pins implemented.
    The Firmata protocol allows up to 16 analog inputs, accessed
    using offsets 0 to 15.  Because Firmata presents the analog
    inputs using different offsets than the actual pin numbers
    (a legacy of Arduino's analogRead function, and the way the
    analog input capable pins are physically labeled on all
    Arduino boards), the total number of analog input signals
    must be specified.  16 is the maximum.

VERSION_BLINK_PIN: When Firmata starts up, it will blink the version
    number.  This constant is the Arduino pin number where a
    LED is connected.


Pin Mapping Macros:

These macros provide the mapping between pins as implemented by
Firmata protocol and the actual pin numbers used by the Arduino
functions.  Even though such mappings are often simple, pin
numbers received by Firmata protocol should always be used as
input to these macros, and the result of the macro should be
used with with any Arduino function.

When Firmata is extended to support a new pin mode or feature,
a pair of macros should be added and used for all hardware
access.  For simple 1:1 mapping, these macros add no actual
overhead, yet their consistent use allows source code which
uses them consistently to be easily adapted to all other boards
with different requirements.

IS_PIN_XXXX(pin): The IS_PIN macros resolve to true or non-zero
    if a pin as implemented by Firmata corresponds to a pin
    that actually implements the named feature.

PIN_TO_XXXX(pin): The PIN_TO macros translate pin numbers as
    implemented by Firmata to the pin numbers needed as inputs
    to the Arduino functions.  The corresponding IS_PIN macro
    should always be tested before using a PIN_TO macro, so
    these macros only need to handle valid Firmata pin
    numbers for the named feature.


Port Access Inline Funtions:

For efficiency, Firmata protocol provides access to digital
input and output pins grouped by 8 bit ports.  When these
groups of 8 correspond to actual 8 bit ports as implemented
by the hardware, these inline functions can provide high
speed direct port access.  Otherwise, a default implementation
using 8 calls to digitalWrite or digitalRead is used.

When porting Firmata to a new board, it is recommended to
use the default functions first and focus only on the constants
and macros above.  When those are working, if optimized port
access is desired, these inline functions may be extended.
The recommended approach defines a symbol indicating which
optimization to use, and then conditional complication is
used within these functions.

readPort(port, bitmask):  Read an 8 bit port, returning the value.
   port:    The port number, Firmata pins port*8 to port*8+7
   bitmask: The actual pins to read, indicated by 1 bits.

writePort(port, value, bitmask):  Write an 8 bit port.
   port:    The port number, Firmata pins port*8 to port*8+7
   value:   The 8 bit value to write
   bitmask: The actual pins to write, indicated by 1 bits.
*/

/*==============================================================================
 * Board Specific Configuration
 *============================================================================*/

#ifndef digitalPinHasPWM
#define digitalPinHasPWM(p)     IS_PIN_DIGITAL(p)
#endif

// Arduino Duemilanove, Diecimila, and NG
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
#if defined(NUM_ANALOG_INPUTS) && NUM_ANALOG_INPUTS == 6
#define TOTAL_ANALOG_PINS       6
#define TOTAL_PINS              20 // 14 digital + 6 analog
#else
#define TOTAL_ANALOG_PINS       8
#define TOTAL_PINS              22 // 14 digital + 8 analog
#endif
#define VERSION_BLINK_PIN       13
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) <= 19)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) < 14 + TOTAL_ANALOG_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) - 2 < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 18 || (p) == 19)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         ((p) - 2)
#define ARDUINO_PINOUT_OPTIMIZE 1


// Wiring (and board)
#elif defined(WIRING)
#define VERSION_BLINK_PIN       WLED
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= FIRST_ANALOG_PIN && (p) < (FIRST_ANALOG_PIN+TOTAL_ANALOG_PINS))
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == SDA || (p) == SCL)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - FIRST_ANALOG_PIN)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// old Arduinos
#elif defined(__AVR_ATmega8__)
#define TOTAL_ANALOG_PINS       6
#define TOTAL_PINS              20 // 14 digital + 6 analog
#define VERSION_BLINK_PIN       13
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) <= 19)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) <= 19)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) - 2 < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 18 || (p) == 19)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         ((p) - 2)
#define ARDUINO_PINOUT_OPTIMIZE 1


// Arduino Mega
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define TOTAL_ANALOG_PINS       16
#define TOTAL_PINS              70 // 54 digital + 16 analog
#define VERSION_BLINK_PIN       13
#define PIN_SERIAL1_RX          19
#define PIN_SERIAL1_TX          18
#define PIN_SERIAL2_RX          17
#define PIN_SERIAL2_TX          16
#define PIN_SERIAL3_RX          15
#define PIN_SERIAL3_TX          14
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= 54 && (p) < TOTAL_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 2 && (p) - 2 < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 20 || (p) == 21)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) > 13 && (p) < 20)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 54)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         ((p) - 2)


// Arduino DUE
#elif defined(__SAM3X8E__)
#define TOTAL_ANALOG_PINS       12
#define TOTAL_PINS              66 // 54 digital + 12 analog
#define VERSION_BLINK_PIN       13
#define PIN_SERIAL1_RX          19
#define PIN_SERIAL1_TX          18
#define PIN_SERIAL2_RX          17
#define PIN_SERIAL2_TX          16
#define PIN_SERIAL3_RX          15
#define PIN_SERIAL3_TX          14
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= 54 && (p) < TOTAL_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 2 && (p) - 2 < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 20 || (p) == 21) // 70 71
#define IS_PIN_SERIAL(p)        ((p) > 13 && (p) < 20)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 54)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         ((p) - 2)


// Arduino/Genuino MKR1000
#elif defined(ARDUINO_SAMD_MKR1000)
#define TOTAL_ANALOG_PINS       7
#define TOTAL_PINS              22 // 8 digital + 3 spi + 2 i2c + 2 uart + 7 analog
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) <= 21)
#define IS_PIN_ANALOG(p)        ((p) >= 15 && (p) < 15 + TOTAL_ANALOG_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS) // deprecated since v2.4
#define IS_PIN_I2C(p)           ((p) == 11 || (p) == 12) // SDA = 11, SCL = 12
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == PIN_SERIAL1_RX || (p) == PIN_SERIAL1_TX) //defined in variant.h  RX = 13, TX = 14
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 15)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p) // deprecated since v2.4


// Arduino MKRZero
#elif defined(ARDUINO_SAMD_MKRZERO)
#define TOTAL_ANALOG_PINS       7
#define TOTAL_PINS              34 // 8 digital + 3 spi + 2 i2c + 2 uart + 7 analog + 3 usb + 1 aref + 5 sd + 1 bottom pad + 1 led + 1 battery adc
#define IS_PIN_DIGITAL(p)       (((p) >= 0 && (p) <= 21) || (p) == 32)
#define IS_PIN_ANALOG(p)        (((p) >= 15 && (p) < 15 + TOTAL_ANALOG_PINS) || (p) == 33)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS) // deprecated since v2.4
#define IS_PIN_I2C(p)           ((p) == 11 || (p) == 12) // SDA = 11, SCL = 12
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == PIN_SERIAL1_RX || (p) == PIN_SERIAL1_TX) //defined in variant.h  RX = 13, TX = 14
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 15)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p) // deprecated since v2.4

// Arduino MKRFox1200
#elif defined(ARDUINO_SAMD_MKRFox1200)
#define TOTAL_ANALOG_PINS       7
#define TOTAL_PINS              33 // 8 digital + 3 spi + 2 i2c + 2 uart + 7 analog + 3 usb + 1 aref + 5 sd + 1 bottom pad + 1 battery adc
#define IS_PIN_DIGITAL(p)       (((p) >= 0 && (p) <= 21))
#define IS_PIN_ANALOG(p)        (((p) >= 15 && (p) < 15 + TOTAL_ANALOG_PINS) || (p) == 32)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS) // deprecated since v2.4
#define IS_PIN_I2C(p)           ((p) == 11 || (p) == 12) // SDA = 11, SCL = 12
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == PIN_SERIAL1_RX || (p) == PIN_SERIAL1_TX) //defined in variant.h  RX = 13, TX = 14
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 15)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p) // deprecated since v2.4

// Arduino MKR WAN 1300
#elif defined(ARDUINO_SAMD_MKRWAN1300)
#define TOTAL_ANALOG_PINS       7
#define TOTAL_PINS              33
#define IS_PIN_DIGITAL(p)       (((p) >= 0 && (p) <= 21))
#define IS_PIN_ANALOG(p)        (((p) >= 15 && (p) < 15 + TOTAL_ANALOG_PINS) || (p) == 32)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS) // deprecated since v2.4
#define IS_PIN_I2C(p)           ((p) == 11 || (p) == 12) // SDA = 11, SCL = 12
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == PIN_SERIAL1_RX || (p) == PIN_SERIAL1_TX) //defined in variant.h  RX = 13, TX = 14
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 15)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p) // deprecated since v2.4

// Arduino MKR GSM 1400
#elif defined(ARDUINO_SAMD_MKRGSM1400)
#define TOTAL_ANALOG_PINS       7
#define TOTAL_PINS              33
#define IS_PIN_DIGITAL(p)       (((p) >= 0 && (p) <= 21))
#define IS_PIN_ANALOG(p)        (((p) >= 15 && (p) < 15 + TOTAL_ANALOG_PINS) || (p) == 32)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS) // deprecated since v2.4
#define IS_PIN_I2C(p)           ((p) == 11 || (p) == 12) // SDA = 11, SCL = 12
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == PIN_SERIAL1_RX || (p) == PIN_SERIAL1_TX) //defined in variant.h  RX = 13, TX = 14
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 15)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p) // deprecated since v2.4

// Arduino Zero
// Note this will work with an Arduino Zero Pro, but not with an Arduino M0 Pro
// Arduino M0 Pro does not properly map pins to the board labeled pin numbers
#elif defined(_VARIANT_ARDUINO_ZERO_)
#define TOTAL_ANALOG_PINS       6
#define TOTAL_PINS              25 // 14 digital + 6 analog + 2 i2c + 3 spi
#define TOTAL_PORTS             3  // set when TOTAL_PINS > num digitial I/O pins
#define VERSION_BLINK_PIN       LED_BUILTIN
//#define PIN_SERIAL1_RX          0 // already defined in zero core variant.h
//#define PIN_SERIAL1_TX          1 // already defined in zero core variant.h
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) <= 19)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) < 14 + TOTAL_ANALOG_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS) // deprecated since v2.4
#define IS_PIN_I2C(p)           ((p) == 20 || (p) == 21) // SDA = 20, SCL = 21
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK) // SS = A2
#define IS_PIN_SERIAL(p)        ((p) == 0 || (p) == 1)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p) // deprecated since v2.4

// Arduino Primo
#elif defined(ARDUINO_PRIMO)
#define TOTAL_ANALOG_PINS       6
#define TOTAL_PINS              22 //14 digital + 6 analog + 2 i2c
#define VERSION_BLINK_PIN       LED_BUILTIN
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) < 20)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) < 20)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS+2)
#define IS_PIN_I2C(p)           ((p) == PIN_WIRE_SDA || (p) == PIN_WIRE_SCL) // SDA = 20, SCL = 21
#define IS_PIN_SPI(p)           ((p) == SS || (p)== MOSI || (p) == MISO || (p == SCK)) // 10, 11, 12, 13
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)

// Arduino 101
#elif defined(_VARIANT_ARDUINO_101_X_)
#define TOTAL_ANALOG_PINS       NUM_ANALOG_INPUTS
#define TOTAL_PINS              NUM_DIGITAL_PINS // 15 digital (including ATN pin) + 6 analog
#define VERSION_BLINK_PIN       LED_BUILTIN
#define PIN_SERIAL1_RX          0
#define PIN_SERIAL1_TX          1
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) <= 20)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) < 14 + TOTAL_ANALOG_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p) // 3, 5, 6, 9
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS) // deprecated since v2.4
#define IS_PIN_I2C(p)           ((p) == SDA || (p) == SCL) // SDA = 18, SCL = 19
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == 0 || (p) == 1)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p) // deprecated since v2.4


// Teensy 1.0
#elif defined(__AVR_AT90USB162__)
#define TOTAL_ANALOG_PINS       0
#define TOTAL_PINS              21 // 21 digital + no analog
#define VERSION_BLINK_PIN       6
#define PIN_SERIAL1_RX          2
#define PIN_SERIAL1_TX          3
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        (0)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           (0)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == 2 || (p) == 3)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (0)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// Teensy 2.0
#elif defined(__AVR_ATmega32U4__) && defined(CORE_TEENSY)
#define TOTAL_ANALOG_PINS       12
#define TOTAL_PINS              25 // 11 digital + 12 analog
#define VERSION_BLINK_PIN       11
#define PIN_SERIAL1_RX          7
#define PIN_SERIAL1_TX          8
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= 11 && (p) <= 22)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 5 || (p) == 6)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == 7 || (p) == 8)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (((p) < 22) ? 21 - (p) : 11)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// Teensy 3.5 and 3.6
// reference: https://github.com/PaulStoffregen/cores/blob/master/teensy3/pins_arduino.h
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define TOTAL_ANALOG_PINS       27 // 3.5 has 27 and 3.6 has 25
#define TOTAL_PINS              70 // 43 digital + 21 analog-digital + 6 analog (64-69)
#define VERSION_BLINK_PIN       13
#define PIN_SERIAL1_RX          0
#define PIN_SERIAL1_TX          1
#define PIN_SERIAL2_RX          9
#define PIN_SERIAL2_TX          10
#define PIN_SERIAL3_RX          7
#define PIN_SERIAL3_TX          8
#define PIN_SERIAL4_RX          31
#define PIN_SERIAL4_TX          32
#define PIN_SERIAL5_RX          34
#define PIN_SERIAL5_TX          33
#define PIN_SERIAL6_RX          47
#define PIN_SERIAL6_TX          48
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) <= 63)
#define IS_PIN_ANALOG(p)        (((p) >= 14 && (p) <= 23) || ((p) >= 31 && (p) <= 39) || ((p) >= 49 && (p) <= 50) || ((p) >= 64 && (p) <= 69))
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 18 || (p) == 19)
#define IS_PIN_SERIAL(p)        (((p) > 6 && (p) < 11) || ((p) == 0 || (p) == 1) || ((p) > 30 && (p) < 35) || ((p) == 47 || (p) == 48))
#define PIN_TO_DIGITAL(p)       (p)
// A0-A9 = D14-D23; A12-A20 = D31-D39; A23-A24 = D49-D50; A10-A11 = D64-D65; A21-A22 = D66-D67; A25-A26 = D68-D69
#define PIN_TO_ANALOG(p)        (((p) <= 23) ? (p) - 14 : (((p) <= 39) ? (p) - 19 : (((p) <= 50) ? (p) - 26 : (((p) <= 65) ? (p) - 55 : (((p) <= 67) ? (p) - 45 : (p) - 43)))))
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// Teensy 3.0, 3.1 and 3.2
#elif defined(__MK20DX128__) || defined(__MK20DX256__)
#define TOTAL_ANALOG_PINS       14
#define TOTAL_PINS              38 // 24 digital + 10 analog-digital + 4 analog
#define VERSION_BLINK_PIN       13
#define PIN_SERIAL1_RX          0
#define PIN_SERIAL1_TX          1
#define PIN_SERIAL2_RX          9
#define PIN_SERIAL2_TX          10
#define PIN_SERIAL3_RX          7
#define PIN_SERIAL3_TX          8
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) <= 33)
#define IS_PIN_ANALOG(p)        (((p) >= 14 && (p) <= 23) || ((p) >= 34 && (p) <= 38))
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 18 || (p) == 19)
#define IS_PIN_SERIAL(p)        (((p) > 6 && (p) < 11) || ((p) == 0 || (p) == 1))
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (((p) <= 23) ? (p) - 14 : (p) - 24)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// Teensy-LC
#elif defined(__MKL26Z64__)
#define TOTAL_ANALOG_PINS       13
#define TOTAL_PINS              27 // 27 digital + 13 analog-digital
#define VERSION_BLINK_PIN       13
#define PIN_SERIAL1_RX          0
#define PIN_SERIAL1_TX          1
#define PIN_SERIAL2_RX          9
#define PIN_SERIAL2_TX          10
#define PIN_SERIAL3_RX          7
#define PIN_SERIAL3_TX          8
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) <= 26)
#define IS_PIN_ANALOG(p)        ((p) >= 14)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 18 || (p) == 19)
#define IS_PIN_SERIAL(p)        (((p) > 6 && (p) < 11) || ((p) == 0 || (p) == 1))
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// Teensy++ 1.0 and 2.0
#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
#define TOTAL_ANALOG_PINS       8
#define TOTAL_PINS              46 // 38 digital + 8 analog
#define VERSION_BLINK_PIN       6
#define PIN_SERIAL1_RX          2
#define PIN_SERIAL1_TX          3
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= 38 && (p) < TOTAL_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 0 || (p) == 1)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == 2 || (p) == 3)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 38)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// Leonardo
#elif defined(__AVR_ATmega32U4__)
#define TOTAL_ANALOG_PINS       12
#define TOTAL_PINS              30 // 14 digital + 12 analog + 4 SPI (D14-D17 on ISP header)
#define VERSION_BLINK_PIN       13
#define PIN_SERIAL1_RX          0
#define PIN_SERIAL1_TX          1
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= 18 && (p) < TOTAL_PINS)
#define IS_PIN_PWM(p)           ((p) == 3 || (p) == 5 || (p) == 6 || (p) == 9 || (p) == 10 || (p) == 11 || (p) == 13)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 2 || (p) == 3)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == 0 || (p) == 1)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (p) - 18
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// Intel Galileo Board (gen 1 and 2) and Intel Edison
#elif defined(ARDUINO_LINUX)
#define TOTAL_ANALOG_PINS       6
#define TOTAL_PINS              20 // 14 digital + 6 analog
#define VERSION_BLINK_PIN       13
#define PIN_SERIAL1_RX          0
#define PIN_SERIAL1_TX          1
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) <= 19)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) <= 19)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) - 2 < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == SDA || (p) == SCL)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == 0 || (p) == 1)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         ((p) - 2)


// RedBearLab BLE Nano with factory switch settings (S1 - S10)
#elif defined(BLE_NANO)
#define TOTAL_ANALOG_PINS       6
#define TOTAL_PINS              15 // 9 digital + 3 analog
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) <= 14)
#define IS_PIN_ANALOG(p)        ((p) == 8 || (p) == 9 || (p) == 10 || (p) == 11 || (p) == 12 || (p) == 14) //A0~A5
#define IS_PIN_PWM(p)           ((p) == 3 || (p) == 5 || (p) == 6)
#define IS_PIN_SERVO(p)         ((p) >= 2 && (p) <= 7)
#define IS_PIN_I2C(p)           ((p) == SDA || (p) == SCL)
#define IS_PIN_SPI(p)           ((p) == CS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 8)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// Sanguino
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
#define TOTAL_ANALOG_PINS       8
#define TOTAL_PINS              32 // 24 digital + 8 analog
#define VERSION_BLINK_PIN       0
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= 24 && (p) < TOTAL_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 16 || (p) == 17)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 24)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         ((p) - 2)


// Illuminato
#elif defined(__AVR_ATmega645__)
#define TOTAL_ANALOG_PINS       6
#define TOTAL_PINS              42 // 36 digital + 6 analog
#define VERSION_BLINK_PIN       13
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= 36 && (p) < TOTAL_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 4 || (p) == 5)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 36)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         ((p) - 2)


// Pic32 chipKIT FubarinoSD
#elif defined(_BOARD_FUBARINO_SD_)
#define TOTAL_ANALOG_PINS       NUM_ANALOG_PINS  // 15
#define TOTAL_PINS              NUM_DIGITAL_PINS // 45, All pins can be digital
#define MAX_SERVOS              NUM_DIGITAL_PINS
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       1
#define IS_PIN_ANALOG(p)        ((p) >= 30 && (p) <= 44)
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 1 || (p) == 2)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (14 - (p - 30))
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)


// Pic32 chipKIT FubarinoMini
// Note, FubarinoMini analog pin 20 will not function in Firmata as analog input due to limitation in analog mapping
#elif defined(_BOARD_FUBARINO_MINI_)
#define TOTAL_ANALOG_PINS       14 // We have to fake this because of the poor analog pin mapping planning in FubarinoMini
#define TOTAL_PINS              NUM_DIGITAL_PINS // 33
#define MAX_SERVOS              NUM_DIGITAL_PINS
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       ((p) != 14 && (p) != 15 && (p) != 31 && (p) != 32)
#define IS_PIN_ANALOG(p)        ((p) == 0 || ((p) >= 3 && (p) <= 13))
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 25 || (p) == 26)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (p)
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)


// Pic32 chipKIT UNO32
#elif defined(_BOARD_UNO_) && defined(__PIC32)  // NOTE: no _BOARD_UNO32_ to use
#define TOTAL_ANALOG_PINS       NUM_ANALOG_PINS // 12
#define TOTAL_PINS              NUM_DIGITAL_PINS // 47 All pins can be digital
#define MAX_SERVOS              NUM_DIGITAL_PINS // All pins can be servo with SoftPWMservo
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       ((p) >= 2)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) <= 25)
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 45 || (p) == 46)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)


// Pic32 chipKIT DP32
#elif defined(_BOARD_DP32_)
#define TOTAL_ANALOG_PINS       15  // Really only has 9, but have to override because of mistake in variant file
#define TOTAL_PINS              NUM_DIGITAL_PINS // 19
#define MAX_SERVOS              NUM_DIGITAL_PINS // All pins can be servo with SoftPWMservo
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       (((p) != 1) && ((p) != 4) && ((p) != 5) && ((p) != 15) && ((p) != 16))
#define IS_PIN_ANALOG(p)        ((p) >= 6 && (p) <= 14)
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 2 || (p) == 3)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (p)
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)


// Pic32 chipKIT uC32
#elif defined(_BOARD_UC32_)
#define TOTAL_ANALOG_PINS       NUM_ANALOG_PINS  // 12
#define TOTAL_PINS              NUM_DIGITAL_PINS // 47 All pins can be digital
#define MAX_SERVOS              NUM_DIGITAL_PINS // All pins can be servo with SoftPWMservo
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       ((p) >= 2)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) <= 25)
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 45 || (p) == 46)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)


// Pic32 chipKIT WF32
#elif defined(_BOARD_WF32_)
#define TOTAL_ANALOG_PINS       NUM_ANALOG_PINS
#define TOTAL_PINS              NUM_DIGITAL_PINS
#define MAX_SERVOS              NUM_DIGITAL_PINS
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) <= 49)     // Accounts for SD and WiFi dedicated pins
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) <= 25)
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 34 || (p) == 35)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)


// Pic32 chipKIT WiFire
#elif defined(_BOARD_WIFIRE_)
#define TOTAL_ANALOG_PINS       NUM_ANALOG_PINS  // 14
#define TOTAL_PINS              NUM_DIGITAL_PINS // 71
#define MAX_SERVOS              NUM_DIGITAL_PINS
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) <= 47)     // Accounts for SD and WiFi dedicated pins
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) <= 25)
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 34 || (p) == 35)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) <= 25 ? ((p) - 14) : (p) - 36)
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)


// Pic32 chipKIT MAX32
#elif defined(_BOARD_MEGA_) && defined(__PIC32)  // NOTE: no _BOARD_MAX32_ to use
#define TOTAL_ANALOG_PINS       NUM_ANALOG_PINS  // 16
#define TOTAL_PINS              NUM_DIGITAL_PINS // 87
#define MAX_SERVOS              NUM_DIGITAL_PINS
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       ((p) >= 2)
#define IS_PIN_ANALOG(p)        ((p) >= 54 && (p) <= 69)
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 34 || (p) == 35)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 54)
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)


// Pic32 chipKIT Pi
#elif defined(_BOARD_CHIPKIT_PI_)
#define TOTAL_ANALOG_PINS       16
#define TOTAL_PINS              NUM_DIGITAL_PINS // 19
#define MAX_SERVOS              NUM_DIGITAL_PINS
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       (((p) >= 2) && ((p) <= 3) || (((p) >= 8) && ((p) <= 13)) || (((p) >= 14) && ((p) <= 17)))
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) <= 17)
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 16 || (p) == 17)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) <= 15 ? (p) - 14 : (p) - 12)
//#define PIN_TO_ANALOG(p)        (((p) <= 16) ? ((p) - 14) : ((p) - 16))
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)

// Pinoccio Scout
// Note: digital pins 9-16 are usable but not labeled on the board numerically.
// SS=9, MOSI=10, MISO=11, SCK=12, RX1=13, TX1=14, SCL=15, SDA=16
#elif defined(ARDUINO_PINOCCIO)
#define TOTAL_ANALOG_PINS       8
#define TOTAL_PINS              NUM_DIGITAL_PINS // 32
#define VERSION_BLINK_PIN       23
#define PIN_SERIAL1_RX          13
#define PIN_SERIAL1_TX          14
#define IS_PIN_DIGITAL(p)       (((p) >= 2) && ((p) <= 16)) || (((p) >= 24) && ((p) <= 31))
#define IS_PIN_ANALOG(p)        ((p) >= 24 && (p) <= 31)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == SCL || (p) == SDA)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == 13 || (p) == 14)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 24)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         ((p) - 2)

// ESP8266
// note: boot mode GPIOs 0, 2 and 15 can be used as outputs, GPIOs 6-11 are in use for flash IO
#elif defined(ESP8266)
#define TOTAL_ANALOG_PINS       NUM_ANALOG_INPUTS
#define TOTAL_PINS              A0 + NUM_ANALOG_INPUTS
#define PIN_SERIAL_RX           3
#define PIN_SERIAL_TX           1
#define IS_PIN_DIGITAL(p)       (((p) >= 0 && (p) <= 5) || ((p) >= 12 && (p) < A0))
#define IS_PIN_ANALOG(p)        ((p) >= A0 && (p) < A0 + NUM_ANALOG_INPUTS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == SDA || (p) == SCL)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_INTERRUPT(p)     (digitalPinToInterrupt(p) > NOT_AN_INTERRUPT)
#define IS_PIN_SERIAL(p)        ((p) == PIN_SERIAL_RX || (p) == PIN_SERIAL_TX)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - A0)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)
#define DEFAULT_PWM_RESOLUTION  10

// STM32 based boards
#elif defined(ARDUINO_ARCH_STM32)
#define TOTAL_ANALOG_PINS       NUM_ANALOG_INPUTS
#define TOTAL_PINS              NUM_DIGITAL_PINS
#define TOTAL_PORTS             MAX_NB_PORT
#define VERSION_BLINK_PIN       LED_BUILTIN
// PIN_SERIALY_RX/TX defined in the variant.h
#define IS_PIN_DIGITAL(p)       (digitalPinIsValid(p) && !pinIsSerial(p))
#define IS_PIN_ANALOG(p)        ((p >= A0) && (p < (A0 + TOTAL_ANALOG_PINS)) && !pinIsSerial(p))
#define IS_PIN_PWM(p)           (IS_PIN_DIGITAL(p) && digitalPinHasPWM(p))
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           (IS_PIN_DIGITAL(p) && digitalPinHasI2C(p))
#define IS_PIN_SPI(p)           (IS_PIN_DIGITAL(p) && digitalPinHasSPI(p))
#define IS_PIN_INTERRUPT(p)     (IS_PIN_DIGITAL(p) && (digitalPinToInterrupt(p) > NOT_AN_INTERRUPT)))
#define IS_PIN_SERIAL(p)        (digitalPinHasSerial(p) && !pinIsSerial(p))
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (p-A0)
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)
#define DEFAULT_PWM_RESOLUTION  PWM_RESOLUTION

// Adafruit Bluefruit nRF52 boards
#elif defined(ARDUINO_NRF52_ADAFRUIT)
#define TOTAL_ANALOG_PINS       NUM_ANALOG_INPUTS
#define TOTAL_PINS              32
#define VERSION_BLINK_PIN       LED_BUILTIN
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) == PIN_A0 || (p) == PIN_A1 || (p) == PIN_A2  || (p) == PIN_A3 || \
                                 (p) == PIN_A4 || (p) == PIN_A5 || (p) == PIN_A6  || (p) == PIN_A7)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == PIN_WIRE_SDA || (p) == PIN_WIRE_SCL)
#define IS_PIN_SPI(p)           ((p) == SS || (p)== MOSI || (p) == MISO || (p == SCK))
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ( ((p) == PIN_A0) ? 0 : ((p) == PIN_A1) ? 1 : ((p) == PIN_A2) ? 2 : ((p) == PIN_A3) ? 3 : \
                                  ((p) == PIN_A4) ? 4 : ((p) == PIN_A5) ? 5 : ((p) == PIN_A6) ? 6 : ((p) == PIN_A7) ? 7 : (127))
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)

// anything else
#else
#error "Please edit Boards.h with a hardware abstraction for this board"
#endif

// as long this is not defined for all boards:
#ifndef IS_PIN_SPI
#define IS_PIN_SPI(p)           0
#endif

#ifndef IS_PIN_SERIAL
#define IS_PIN_SERIAL(p)        0
#endif

#ifndef DEFAULT_PWM_RESOLUTION
#define DEFAULT_PWM_RESOLUTION  8
#endif

/*==============================================================================
 * readPort() - Read an 8 bit port
 *============================================================================*/

static inline unsigned char readPort(byte, byte) __attribute__((always_inline, unused));
static inline unsigned char readPort(byte port, byte bitmask)
{
#if defined(ARDUINO_PINOUT_OPTIMIZE)
  if (port == 0) return (PIND & 0xFC) & bitmask; // ignore Rx/Tx 0/1
  if (port == 1) return ((PINB & 0x3F) | ((PINC & 0x03) << 6)) & bitmask;
  if (port == 2) return ((PINC & 0x3C) >> 2) & bitmask;
  return 0;
#else
  unsigned char out = 0, pin = port * 8;
  if (IS_PIN_DIGITAL(pin + 0) && (bitmask & 0x01) && digitalRead(PIN_TO_DIGITAL(pin + 0))) out |= 0x01;
  if (IS_PIN_DIGITAL(pin + 1) && (bitmask & 0x02) && digitalRead(PIN_TO_DIGITAL(pin + 1))) out |= 0x02;
  if (IS_PIN_DIGITAL(pin + 2) && (bitmask & 0x04) && digitalRead(PIN_TO_DIGITAL(pin + 2))) out |= 0x04;
  if (IS_PIN_DIGITAL(pin + 3) && (bitmask & 0x08) && digitalRead(PIN_TO_DIGITAL(pin + 3))) out |= 0x08;
  if (IS_PIN_DIGITAL(pin + 4) && (bitmask & 0x10) && digitalRead(PIN_TO_DIGITAL(pin + 4))) out |= 0x10;
  if (IS_PIN_DIGITAL(pin + 5) && (bitmask & 0x20) && digitalRead(PIN_TO_DIGITAL(pin + 5))) out |= 0x20;
  if (IS_PIN_DIGITAL(pin + 6) && (bitmask & 0x40) && digitalRead(PIN_TO_DIGITAL(pin + 6))) out |= 0x40;
  if (IS_PIN_DIGITAL(pin + 7) && (bitmask & 0x80) && digitalRead(PIN_TO_DIGITAL(pin + 7))) out |= 0x80;
  return out;
#endif
}

/*==============================================================================
 * writePort() - Write an 8 bit port, only touch pins specified by a bitmask
 *============================================================================*/

static inline unsigned char writePort(byte, byte, byte) __attribute__((always_inline, unused));
static inline unsigned char writePort(byte port, byte value, byte bitmask)
{
#if defined(ARDUINO_PINOUT_OPTIMIZE)
  if (port == 0) {
    bitmask = bitmask & 0xFC;  // do not touch Tx & Rx pins
    byte valD = value & bitmask;
    byte maskD = ~bitmask;
    cli();
    PORTD = (PORTD & maskD) | valD;
    sei();
  } else if (port == 1) {
    byte valB = (value & bitmask) & 0x3F;
    byte valC = (value & bitmask) >> 6;
    byte maskB = ~(bitmask & 0x3F);
    byte maskC = ~((bitmask & 0xC0) >> 6);
    cli();
    PORTB = (PORTB & maskB) | valB;
    PORTC = (PORTC & maskC) | valC;
    sei();
  } else if (port == 2) {
    bitmask = bitmask & 0x0F;
    byte valC = (value & bitmask) << 2;
    byte maskC = ~(bitmask << 2);
    cli();
    PORTC = (PORTC & maskC) | valC;
    sei();
  }
  return 1;
#else
  byte pin = port * 8;
  if ((bitmask & 0x01)) digitalWrite(PIN_TO_DIGITAL(pin + 0), (value & 0x01));
  if ((bitmask & 0x02)) digitalWrite(PIN_TO_DIGITAL(pin + 1), (value & 0x02));
  if ((bitmask & 0x04)) digitalWrite(PIN_TO_DIGITAL(pin + 2), (value & 0x04));
  if ((bitmask & 0x08)) digitalWrite(PIN_TO_DIGITAL(pin + 3), (value & 0x08));
  if ((bitmask & 0x10)) digitalWrite(PIN_TO_DIGITAL(pin + 4), (value & 0x10));
  if ((bitmask & 0x20)) digitalWrite(PIN_TO_DIGITAL(pin + 5), (value & 0x20));
  if ((bitmask & 0x40)) digitalWrite(PIN_TO_DIGITAL(pin + 6), (value & 0x40));
  if ((bitmask & 0x80)) digitalWrite(PIN_TO_DIGITAL(pin + 7), (value & 0x80));
  return 1;
#endif
}




#ifndef TOTAL_PORTS
#define TOTAL_PORTS             ((TOTAL_PINS + 7) / 8)
#endif


#endif /* Firmata_Boards_h */


//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------

/*
  FirmataConstants.h
  Copyright (c) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2009-2017 Jeff Hoefs.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

#ifndef FirmataConstants_h
#define FirmataConstants_h

namespace firmata {
/* Version numbers for the Firmata library.
 * The firmware version will not always equal the protocol version going forward.
 * Query using the REPORT_FIRMWARE message.
 */
static const int FIRMWARE_MAJOR_VERSION =  2;
static const int FIRMWARE_MINOR_VERSION =  5;
static const int FIRMWARE_BUGFIX_VERSION = 7;

/* Version numbers for the protocol.  The protocol is still changing, so these
 * version numbers are important.
 * Query using the REPORT_VERSION message.
 */
static const int PROTOCOL_MAJOR_VERSION =  2; // for non-compatible changes
static const int PROTOCOL_MINOR_VERSION =  5; // for backwards compatible changes
static const int PROTOCOL_BUGFIX_VERSION = 1; // for bugfix releases

static const int MAX_DATA_BYTES =          64; // max number of data bytes in incoming messages

// message command bytes (128-255/0x80-0xFF)

static const int DIGITAL_MESSAGE =         0x90; // send data for a digital port (collection of 8 pins)
static const int ANALOG_MESSAGE =          0xE0; // send data for an analog pin (or PWM)
static const int REPORT_ANALOG =           0xC0; // enable analog input by pin #
static const int REPORT_DIGITAL =          0xD0; // enable digital input by port pair
//
static const int SET_PIN_MODE =            0xF4; // set a pin to INPUT/OUTPUT/PWM/etc
static const int SET_DIGITAL_PIN_VALUE =   0xF5; // set value of an individual digital pin
//
static const int REPORT_VERSION =          0xF9; // report protocol version
static const int SYSTEM_RESET =            0xFF; // reset from MIDI
//
static const int START_SYSEX =             0xF0; // start a MIDI Sysex message
static const int END_SYSEX =               0xF7; // end a MIDI Sysex message

// extended command set using sysex (0-127/0x00-0x7F)
/* 0x00-0x0F reserved for user-defined commands */

static const int SERIAL_DATA =             0x60; // communicate with serial devices, including other boards
static const int ENCODER_DATA =            0x61; // reply with encoders current positions
static const int SERVO_CONFIG =            0x70; // set max angle, minPulse, maxPulse, freq
static const int STRING_DATA =             0x71; // a string message with 14-bits per char
static const int STEPPER_DATA =            0x72; // control a stepper motor
static const int ONEWIRE_DATA =            0x73; // send an OneWire read/write/reset/select/skip/search request
static const int SHIFT_DATA =              0x75; // a bitstream to/from a shift register
static const int I2C_REQUEST =             0x76; // send an I2C read/write request
static const int I2C_REPLY =               0x77; // a reply to an I2C read request
static const int I2C_CONFIG =              0x78; // config I2C settings such as delay times and power pins
static const int REPORT_FIRMWARE =         0x79; // report name and version of the firmware
static const int EXTENDED_ANALOG =         0x6F; // analog write (PWM, Servo, etc) to any pin
static const int PIN_STATE_QUERY =         0x6D; // ask for a pin's current mode and value
static const int PIN_STATE_RESPONSE =      0x6E; // reply with pin's current mode and value
static const int CAPABILITY_QUERY =        0x6B; // ask for supported modes and resolution of all pins
static const int CAPABILITY_RESPONSE =     0x6C; // reply with supported modes and resolution
static const int ANALOG_MAPPING_QUERY =    0x69; // ask for mapping of analog to pin numbers
static const int ANALOG_MAPPING_RESPONSE = 0x6A; // reply with mapping info
static const int SAMPLING_INTERVAL =       0x7A; // set the poll rate of the main loop
static const int SCHEDULER_DATA =          0x7B; // send a createtask/deletetask/addtotask/schedule/querytasks/querytask request to the scheduler
static const int SYSEX_NON_REALTIME =      0x7E; // MIDI Reserved for non-realtime messages
static const int SYSEX_REALTIME =          0x7F; // MIDI Reserved for realtime messages

// pin modes
static const int PIN_MODE_INPUT =          0x00; // same as INPUT defined in Arduino.h
static const int PIN_MODE_OUTPUT =         0x01; // same as OUTPUT defined in Arduino.h
static const int PIN_MODE_ANALOG =         0x02; // analog pin in analogInput mode
static const int PIN_MODE_PWM =            0x03; // digital pin in PWM output mode
static const int PIN_MODE_SERVO =          0x04; // digital pin in Servo output mode
static const int PIN_MODE_SHIFT =          0x05; // shiftIn/shiftOut mode
static const int PIN_MODE_I2C =            0x06; // pin included in I2C setup
static const int PIN_MODE_ONEWIRE =        0x07; // pin configured for 1-wire
static const int PIN_MODE_STEPPER =        0x08; // pin configured for stepper motor
static const int PIN_MODE_ENCODER =        0x09; // pin configured for rotary encoders
static const int PIN_MODE_SERIAL =         0x0A; // pin configured for serial communication
static const int PIN_MODE_PULLUP =         0x0B; // enable internal pull-up resistor for pin
static const int PIN_MODE_IGNORE =         0x7F; // pin configured to be ignored by digitalWrite and capabilityResponse

static const int TOTAL_PIN_MODES =         13;

} // namespace firmata

#endif // FirmataConstants_h
//---------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------

/*
  FirmataParser.h
  Copyright (c) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2009-2016 Jeff Hoefs.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

#ifndef FirmataParser_h
#define FirmataParser_h

#if defined(__cplusplus) && !defined(ARDUINO)
  #include <cstddef>
  #include <cstdint>
#else
  #include <stddef.h>
  #include <stdint.h>
#endif

namespace firmata {

class FirmataParser
{
  public:
    /* callback function types */
    typedef void (*callbackFunction)(void * context, uint8_t command, uint16_t value);
    typedef void (*dataBufferOverflowCallbackFunction)(void * context);
    typedef void (*stringCallbackFunction)(void * context, const char * c_str);
    typedef void (*sysexCallbackFunction)(void * context, uint8_t command, size_t argc, uint8_t * argv);
    typedef void (*systemCallbackFunction)(void * context);
    typedef void (*versionCallbackFunction)(void * context, size_t sv_major, size_t sv_minor, const char * firmware);

    FirmataParser(uint8_t * dataBuffer = (uint8_t *)NULL, size_t dataBufferSize = 0);

    /* serial receive handling */
    void parse(uint8_t value);
    bool isParsingMessage(void) const;
    int setDataBufferOfSize(uint8_t * dataBuffer, size_t dataBufferSize);

    /* attach & detach callback functions to messages */
    void attach(uint8_t command, callbackFunction newFunction, void * context = NULL);
    void attach(dataBufferOverflowCallbackFunction newFunction, void * context = NULL);
    void attach(uint8_t command, stringCallbackFunction newFunction, void * context = NULL);
    void attach(uint8_t command, sysexCallbackFunction newFunction, void * context = NULL);
    void attach(uint8_t command, systemCallbackFunction newFunction, void * context = NULL);
    void attach(uint8_t command, versionCallbackFunction newFunction, void * context = NULL);
    void detach(uint8_t command);
    void detach(dataBufferOverflowCallbackFunction);

  private:
    /* input message handling */
    bool allowBufferUpdate;
    uint8_t * dataBuffer; // multi-byte data
    size_t dataBufferSize;
    uint8_t executeMultiByteCommand; // execute this after getting multi-byte data
    uint8_t multiByteChannel; // channel data for multiByteCommands
    size_t waitForData; // this flag says the next serial input will be data

    /* sysex */
    bool parsingSysex;
    size_t sysexBytesRead;

    /* callback context */
    void * currentAnalogCallbackContext;
    void * currentDigitalCallbackContext;
    void * currentReportAnalogCallbackContext;
    void * currentReportDigitalCallbackContext;
    void * currentPinModeCallbackContext;
    void * currentPinValueCallbackContext;
    void * currentReportFirmwareCallbackContext;
    void * currentReportVersionCallbackContext;
    void * currentDataBufferOverflowCallbackContext;
    void * currentStringCallbackContext;
    void * currentSysexCallbackContext;
    void * currentSystemResetCallbackContext;

    /* callback functions */
    callbackFunction currentAnalogCallback;
    callbackFunction currentDigitalCallback;
    callbackFunction currentReportAnalogCallback;
    callbackFunction currentReportDigitalCallback;
    callbackFunction currentPinModeCallback;
    callbackFunction currentPinValueCallback;
    dataBufferOverflowCallbackFunction currentDataBufferOverflowCallback;
    stringCallbackFunction currentStringCallback;
    sysexCallbackFunction currentSysexCallback;
    versionCallbackFunction currentReportFirmwareCallback;
    systemCallbackFunction currentReportVersionCallback;
    systemCallbackFunction currentSystemResetCallback;

    /* private methods ------------------------------ */
    bool bufferDataAtPosition(const uint8_t data, const size_t pos);
    size_t decodeByteStream(size_t bytec, uint8_t * bytev);
    void processSysexMessage(void);
    void systemReset(void);
};

} // firmata

#endif /* FirmataParser_h */
//---------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------

/*
  FirmataMarshaller.h
  Copyright (c) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2009-2016 Jeff Hoefs.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

#ifndef FirmataMarshaller_h
#define FirmataMarshaller_h



#include <Stream.h>

namespace firmata {

class FirmataMarshaller
{
    friend class FirmataClass;

  public:
    /* constructors */
    FirmataMarshaller();

    /* public methods */
    void begin(Stream &s);
    void end();

    /* serial send handling */
    void queryFirmwareVersion(void) const;
    void queryVersion(void) const;
    void reportAnalogDisable(uint8_t pin) const;
    void reportAnalogEnable(uint8_t pin) const;
    void reportDigitalPortDisable(uint8_t portNumber) const;
    void reportDigitalPortEnable(uint8_t portNumber) const;
    void sendAnalog(uint8_t pin, uint16_t value) const;
    void sendAnalogMappingQuery(void) const;
    void sendCapabilityQuery(void) const;
    void sendDigital(uint8_t pin, uint8_t value) const;
    void sendDigitalPort(uint8_t portNumber, uint16_t portData) const;
    void sendFirmwareVersion(uint8_t major, uint8_t minor, size_t bytec, uint8_t *bytev) const;
    void sendVersion(uint8_t major, uint8_t minor) const;
    void sendPinMode(uint8_t pin, uint8_t config) const;
    void sendPinStateQuery(uint8_t pin) const;
    void sendString(const char *string) const;
    void sendSysex(uint8_t command, size_t bytec, uint8_t *bytev) const;
    void setSamplingInterval(uint16_t interval_ms) const;
    void systemReset(void) const;

  private:
    /* utility methods */
    void reportAnalog(uint8_t pin, bool stream_enable) const;
    void reportDigitalPort(uint8_t portNumber, bool stream_enable) const;
    void sendExtendedAnalog(uint8_t pin, size_t bytec, uint8_t * bytev) const;
    void encodeByteStream (size_t bytec, uint8_t * bytev, size_t max_bytes = 0) const;

    Stream * FirmataStream;
};

} // namespace firmata

#endif /* FirmataMarshaller_h */


//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------


/*
  FirmataDefines.h
  Copyright (c) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2009-2016 Jeff Hoefs.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

#ifndef FirmataDefines_h
#define FirmataDefines_h


/* Version numbers for the Firmata library.
 * The firmware version will not always equal the protocol version going forward.
 * Query using the REPORT_FIRMWARE message.
 */
#define FIRMATA_FIRMWARE_MAJOR_VERSION  firmata::FIRMWARE_MAJOR_VERSION
#define FIRMATA_FIRMWARE_MINOR_VERSION  firmata::FIRMWARE_MINOR_VERSION
#define FIRMATA_FIRMWARE_BUGFIX_VERSION firmata::FIRMWARE_BUGFIX_VERSION

/* Version numbers for the protocol.  The protocol is still changing, so these
 * version numbers are important.
 * Query using the REPORT_VERSION message.
 */
#define FIRMATA_PROTOCOL_MAJOR_VERSION  firmata::PROTOCOL_MAJOR_VERSION // for non-compatible changes
#define FIRMATA_PROTOCOL_MINOR_VERSION  firmata::PROTOCOL_MINOR_VERSION // for backwards compatible changes
#define FIRMATA_PROTOCOL_BUGFIX_VERSION firmata::PROTOCOL_BUGFIX_VERSION // for bugfix releases

#ifdef MAX_DATA_BYTES
#undef MAX_DATA_BYTES
#endif
#define MAX_DATA_BYTES          firmata::MAX_DATA_BYTES // max number of data bytes in incoming messages

// message command bytes (128-255/0x80-0xFF)

#ifdef DIGITAL_MESSAGE
#undef DIGITAL_MESSAGE
#endif
#define DIGITAL_MESSAGE         firmata::DIGITAL_MESSAGE // send data for a digital port (collection of 8 pins)

#ifdef ANALOG_MESSAGE
#undef ANALOG_MESSAGE
#endif
#define ANALOG_MESSAGE          firmata::ANALOG_MESSAGE // send data for an analog pin (or PWM)

#ifdef REPORT_ANALOG
#undef REPORT_ANALOG
#endif
#define REPORT_ANALOG           firmata::REPORT_ANALOG // enable analog input by pin #

#ifdef REPORT_DIGITAL
#undef REPORT_DIGITAL
#endif
#define REPORT_DIGITAL          firmata::REPORT_DIGITAL // enable digital input by port pair

//

#ifdef SET_PIN_MODE
#undef SET_PIN_MODE
#endif
#define SET_PIN_MODE            firmata::SET_PIN_MODE // set a pin to INPUT/OUTPUT/PWM/etc

#ifdef SET_DIGITAL_PIN_VALUE
#undef SET_DIGITAL_PIN_VALUE
#endif
#define SET_DIGITAL_PIN_VALUE   firmata::SET_DIGITAL_PIN_VALUE // set value of an individual digital pin

//

#ifdef REPORT_VERSION
#undef REPORT_VERSION
#endif
#define REPORT_VERSION          firmata::REPORT_VERSION // report protocol version

#ifdef SYSTEM_RESET
#undef SYSTEM_RESET
#endif
#define SYSTEM_RESET            firmata::SYSTEM_RESET // reset from MIDI

//

#ifdef START_SYSEX
#undef START_SYSEX
#endif
#define START_SYSEX             firmata::START_SYSEX // start a MIDI Sysex message

#ifdef END_SYSEX
#undef END_SYSEX
#endif
#define END_SYSEX               firmata::END_SYSEX // end a MIDI Sysex message

// extended command set using sysex (0-127/0x00-0x7F)
/* 0x00-0x0F reserved for user-defined commands */

#ifdef SERIAL_MESSAGE
#undef SERIAL_MESSAGE
#endif
#define SERIAL_MESSAGE          firmata::SERIAL_DATA // communicate with serial devices, including other boards

#ifdef ENCODER_DATA
#undef ENCODER_DATA
#endif
#define ENCODER_DATA            firmata::ENCODER_DATA // reply with encoders current positions

#ifdef SERVO_CONFIG
#undef SERVO_CONFIG
#endif
#define SERVO_CONFIG            firmata::SERVO_CONFIG // set max angle, minPulse, maxPulse, freq

#ifdef STRING_DATA
#undef STRING_DATA
#endif
#define STRING_DATA             firmata::STRING_DATA // a string message with 14-bits per char

#ifdef STEPPER_DATA
#undef STEPPER_DATA
#endif
#define STEPPER_DATA            firmata::STEPPER_DATA // control a stepper motor

#ifdef ONEWIRE_DATA
#undef ONEWIRE_DATA
#endif
#define ONEWIRE_DATA            firmata::ONEWIRE_DATA // send an OneWire read/write/reset/select/skip/search request

#ifdef SHIFT_DATA
#undef SHIFT_DATA
#endif
#define SHIFT_DATA              firmata::SHIFT_DATA // a bitstream to/from a shift register

#ifdef I2C_REQUEST
#undef I2C_REQUEST
#endif
#define I2C_REQUEST             firmata::I2C_REQUEST // send an I2C read/write request

#ifdef I2C_REPLY
#undef I2C_REPLY
#endif
#define I2C_REPLY               firmata::I2C_REPLY // a reply to an I2C read request

#ifdef I2C_CONFIG
#undef I2C_CONFIG
#endif
#define I2C_CONFIG              firmata::I2C_CONFIG // config I2C settings such as delay times and power pins

#ifdef REPORT_FIRMWARE
#undef REPORT_FIRMWARE
#endif
#define REPORT_FIRMWARE         firmata::REPORT_FIRMWARE // report name and version of the firmware

#ifdef EXTENDED_ANALOG
#undef EXTENDED_ANALOG
#endif
#define EXTENDED_ANALOG         firmata::EXTENDED_ANALOG // analog write (PWM, Servo, etc) to any pin

#ifdef PIN_STATE_QUERY
#undef PIN_STATE_QUERY
#endif
#define PIN_STATE_QUERY         firmata::PIN_STATE_QUERY // ask for a pin's current mode and value

#ifdef PIN_STATE_RESPONSE
#undef PIN_STATE_RESPONSE
#endif
#define PIN_STATE_RESPONSE      firmata::PIN_STATE_RESPONSE // reply with pin's current mode and value

#ifdef CAPABILITY_QUERY
#undef CAPABILITY_QUERY
#endif
#define CAPABILITY_QUERY        firmata::CAPABILITY_QUERY // ask for supported modes and resolution of all pins

#ifdef CAPABILITY_RESPONSE
#undef CAPABILITY_RESPONSE
#endif
#define CAPABILITY_RESPONSE     firmata::CAPABILITY_RESPONSE // reply with supported modes and resolution

#ifdef ANALOG_MAPPING_QUERY
#undef ANALOG_MAPPING_QUERY
#endif
#define ANALOG_MAPPING_QUERY    firmata::ANALOG_MAPPING_QUERY // ask for mapping of analog to pin numbers

#ifdef ANALOG_MAPPING_RESPONSE
#undef ANALOG_MAPPING_RESPONSE
#endif
#define ANALOG_MAPPING_RESPONSE firmata::ANALOG_MAPPING_RESPONSE // reply with mapping info

#ifdef SAMPLING_INTERVAL
#undef SAMPLING_INTERVAL
#endif
#define SAMPLING_INTERVAL       firmata::SAMPLING_INTERVAL // set the poll rate of the main loop

#ifdef SCHEDULER_DATA
#undef SCHEDULER_DATA
#endif
#define SCHEDULER_DATA          firmata::SCHEDULER_DATA // send a createtask/deletetask/addtotask/schedule/querytasks/querytask request to the scheduler

#ifdef SYSEX_NON_REALTIME
#undef SYSEX_NON_REALTIME
#endif
#define SYSEX_NON_REALTIME      firmata::SYSEX_NON_REALTIME // MIDI Reserved for non-realtime messages

#ifdef SYSEX_REALTIME
#undef SYSEX_REALTIME
#endif
#define SYSEX_REALTIME          firmata::SYSEX_REALTIME // MIDI Reserved for realtime messages

// pin modes

#ifdef PIN_MODE_INPUT
#undef PIN_MODE_INPUT
#endif
#define PIN_MODE_INPUT          firmata::PIN_MODE_INPUT // same as INPUT defined in Arduino.h

#ifdef PIN_MODE_OUTPUT
#undef PIN_MODE_OUTPUT
#endif
#define PIN_MODE_OUTPUT         firmata::PIN_MODE_OUTPUT // same as OUTPUT defined in Arduino.h

#ifdef PIN_MODE_ANALOG
#undef PIN_MODE_ANALOG
#endif
#define PIN_MODE_ANALOG         firmata::PIN_MODE_ANALOG // analog pin in analogInput mode

#ifdef PIN_MODE_PWM
#undef PIN_MODE_PWM
#endif
#define PIN_MODE_PWM            firmata::PIN_MODE_PWM // digital pin in PWM output mode

#ifdef PIN_MODE_SERVO
#undef PIN_MODE_SERVO
#endif
#define PIN_MODE_SERVO          firmata::PIN_MODE_SERVO // digital pin in Servo output mode

#ifdef PIN_MODE_SHIFT
#undef PIN_MODE_SHIFT
#endif
#define PIN_MODE_SHIFT          firmata::PIN_MODE_SHIFT // shiftIn/shiftOut mode

#ifdef PIN_MODE_I2C
#undef PIN_MODE_I2C
#endif
#define PIN_MODE_I2C            firmata::PIN_MODE_I2C // pin included in I2C setup

#ifdef PIN_MODE_ONEWIRE
#undef PIN_MODE_ONEWIRE
#endif
#define PIN_MODE_ONEWIRE        firmata::PIN_MODE_ONEWIRE // pin configured for 1-wire

#ifdef PIN_MODE_STEPPER
#undef PIN_MODE_STEPPER
#endif
#define PIN_MODE_STEPPER        firmata::PIN_MODE_STEPPER // pin configured for stepper motor

#ifdef PIN_MODE_ENCODER
#undef PIN_MODE_ENCODER
#endif
#define PIN_MODE_ENCODER        firmata::PIN_MODE_ENCODER // pin configured for rotary encoders

#ifdef PIN_MODE_SERIAL
#undef PIN_MODE_SERIAL
#endif
#define PIN_MODE_SERIAL         firmata::PIN_MODE_SERIAL // pin configured for serial communication

#ifdef PIN_MODE_PULLUP
#undef PIN_MODE_PULLUP
#endif
#define PIN_MODE_PULLUP         firmata::PIN_MODE_PULLUP // enable internal pull-up resistor for pin

#ifdef PIN_MODE_IGNORE
#undef PIN_MODE_IGNORE
#endif
#define PIN_MODE_IGNORE         firmata::PIN_MODE_IGNORE // pin configured to be ignored by digitalWrite and capabilityResponse

#ifdef TOTAL_PIN_MODES
#undef TOTAL_PIN_MODES
#endif
#define TOTAL_PIN_MODES         firmata::TOTAL_PIN_MODES

#endif // FirmataConstants_h
//--------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------

/*
  Firmata.h - Firmata library v2.5.8 - 2018-04-15
  Copyright (c) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2009-2017 Jeff Hoefs.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

#ifndef Firmata_h
#define Firmata_h



/* DEPRECATED as of Firmata v2.5.1. As of 2.5.1 there are separate version numbers for
 * the protocol version and the firmware version.
 */
#define FIRMATA_MAJOR_VERSION           2 // same as FIRMATA_PROTOCOL_MAJOR_VERSION
#define FIRMATA_MINOR_VERSION           5 // same as FIRMATA_PROTOCOL_MINOR_VERSION
#define FIRMATA_BUGFIX_VERSION          1 // same as FIRMATA_PROTOCOL_BUGFIX_VERSION

// extended command set using sysex (0-127/0x00-0x7F)
/* 0x00-0x0F reserved for user-defined commands */
// these are DEPRECATED to make the naming more consistent
#define FIRMATA_STRING          0x71 // same as STRING_DATA
#define SYSEX_I2C_REQUEST       0x76 // same as I2C_REQUEST
#define SYSEX_I2C_REPLY         0x77 // same as I2C_REPLY
#define SYSEX_SAMPLING_INTERVAL 0x7A // same as SAMPLING_INTERVAL

// pin modes
//#define INPUT                 0x00 // defined in Arduino.h
//#define OUTPUT                0x01 // defined in Arduino.h
// DEPRECATED as of Firmata v2.5
#define ANALOG                  0x02 // same as PIN_MODE_ANALOG
#define PWM                     0x03 // same as PIN_MODE_PWM
#define SERVO                   0x04 // same as PIN_MODE_SERVO
#define SHIFT                   0x05 // same as PIN_MODE_SHIFT
#define I2C                     0x06 // same as PIN_MODE_I2C
#define ONEWIRE                 0x07 // same as PIN_MODE_ONEWIRE
#define STEPPER                 0x08 // same as PIN_MODE_STEPPER
#define ENCODER                 0x09 // same as PIN_MODE_ENCODER
#define IGNORE                  0x7F // same as PIN_MODE_IGNORE

namespace firmata {

// TODO make it a subclass of a generic Serial/Stream base class
class FirmataClass
{
  public:
    typedef void (*callbackFunction)(uint8_t, int);
    typedef void (*systemCallbackFunction)(void);
    typedef void (*stringCallbackFunction)(char *);
    typedef void (*sysexCallbackFunction)(uint8_t command, uint8_t argc, uint8_t *argv);

    FirmataClass();

    /* Arduino constructors */
    void begin();
    void begin(long);
    void begin(Stream &s);

    /* querying functions */
    void printVersion(void);
    void blinkVersion(void);
    void printFirmwareVersion(void);

    //void setFirmwareVersion(byte major, byte minor);  // see macro below
    void setFirmwareNameAndVersion(const char *name, byte major, byte minor);
    void disableBlinkVersion();

    /* serial receive handling */
    int available(void);
    void processInput(void);
    void parse(unsigned char value);
    boolean isParsingMessage(void);

    /* serial send handling */
    void sendAnalog(byte pin, int value);
    void sendDigital(byte pin, int value); // TODO implement this
    void sendDigitalPort(byte portNumber, int portData);
    void sendString(const char *string);
    void sendString(byte command, const char *string);
    void sendSysex(byte command, byte bytec, byte *bytev);
    void write(byte c);

    /* attach & detach callback functions to messages */
    void attach(uint8_t command, callbackFunction newFunction);
    void attach(uint8_t command, systemCallbackFunction newFunction);
    void attach(uint8_t command, stringCallbackFunction newFunction);
    void attach(uint8_t command, sysexCallbackFunction newFunction);
    void detach(uint8_t command);

    /* access pin state and config */
    byte getPinMode(byte pin);
    void setPinMode(byte pin, byte config);

    /* access pin state */
    int getPinState(byte pin);
    void setPinState(byte pin, int state);

    /* utility methods */
    void sendValueAsTwo7bitBytes(int value);
    void startSysex(void);
    void endSysex(void);

  private:
    uint8_t parserBuffer[MAX_DATA_BYTES];
    FirmataMarshaller marshaller;
    FirmataParser parser;
    Stream *FirmataStream;

    /* firmware name and version */
    byte firmwareVersionCount;
    byte *firmwareVersionVector;

    /* pin configuration */
    byte pinConfig[TOTAL_PINS];
    int pinState[TOTAL_PINS];

    boolean blinkVersionDisabled;

    /* private methods ------------------------------ */
    void strobeBlinkPin(byte pin, int count, int onInterval, int offInterval);
    friend void FirmataMarshaller::encodeByteStream (size_t bytec, uint8_t * bytev, size_t max_bytes = 0) const;

    /* callback functions */
    static callbackFunction currentAnalogCallback;
    static callbackFunction currentDigitalCallback;
    static callbackFunction currentPinModeCallback;
    static callbackFunction currentPinValueCallback;
    static callbackFunction currentReportAnalogCallback;
    static callbackFunction currentReportDigitalCallback;
    static stringCallbackFunction currentStringCallback;
    static sysexCallbackFunction currentSysexCallback;
    static systemCallbackFunction currentSystemResetCallback;

    /* static callbacks */
    inline static void staticAnalogCallback (void *, uint8_t command, uint16_t value) { if ( currentAnalogCallback ) { currentAnalogCallback(command,(int)value); } }
    inline static void staticDigitalCallback (void *, uint8_t command, uint16_t value) { if ( currentDigitalCallback ) { currentDigitalCallback(command, (int)value); } }
    inline static void staticPinModeCallback (void *, uint8_t command, uint16_t value) { if ( currentPinModeCallback ) { currentPinModeCallback(command, (int)value); } }
    inline static void staticPinValueCallback (void *, uint8_t command, uint16_t value) { if ( currentPinValueCallback ) { currentPinValueCallback(command, (int)value); } }
    inline static void staticReportAnalogCallback (void *, uint8_t command, uint16_t value) { if ( currentReportAnalogCallback ) { currentReportAnalogCallback(command, (int)value); } }
    inline static void staticReportDigitalCallback (void *, uint8_t command, uint16_t value) { if ( currentReportDigitalCallback ) { currentReportDigitalCallback(command, (int)value); } }
    inline static void staticStringCallback (void *, const char * c_str) { if ( currentStringCallback ) { currentStringCallback((char *)c_str); } }
    inline static void staticSysexCallback (void *, uint8_t command, size_t argc, uint8_t *argv) { if ( currentSysexCallback ) { currentSysexCallback(command, (uint8_t)argc, argv); } }
    inline static void staticReportFirmwareCallback (void * context, size_t, size_t, const char *) { if ( context ) { ((FirmataClass *)context)->printFirmwareVersion(); } }
    inline static void staticReportVersionCallback (void * context) { if ( context ) { ((FirmataClass *)context)->printVersion(); } }
    inline static void staticSystemResetCallback (void *) { if ( currentSystemResetCallback ) { currentSystemResetCallback(); } }
};

} // namespace firmata

extern "C" {
  // callback function types
  typedef firmata::FirmataClass::callbackFunction callbackFunction;
  typedef firmata::FirmataClass::systemCallbackFunction systemCallbackFunction;
  typedef firmata::FirmataClass::stringCallbackFunction stringCallbackFunction;
  typedef firmata::FirmataClass::sysexCallbackFunction sysexCallbackFunction;
}

extern firmata::FirmataClass Firmata;

/*==============================================================================
 * MACROS
 *============================================================================*/

/* shortcut for setFirmwareNameAndVersion() that uses __FILE__ to set the
 * firmware name.  It needs to be a macro so that __FILE__ is included in the
 * firmware source file rather than the library source file.
 */
#define setFirmwareVersion(x, y)   setFirmwareNameAndVersion(__FILE__, x, y)

#endif /* Firmata_h */

//#define DECODE_NEC


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_1X);
//-------------------------------
/*
  Boards.h - Hardware Abstraction Layer for Firmata library
  Copyright (c) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2009-2017 Jeff Hoefs.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.

  Last updated April 15th, 2018
*/

#ifndef Firmata_Boards_h
#define Firmata_Boards_h

#include <inttypes.h>

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"  // for digitalRead, digitalWrite, etc
#else
#include "WProgram.h"
#endif

// Normally Servo.h must be included before Firmata.h (which then includes
// this file).  If Servo.h wasn't included, this allows the code to still
// compile, but without support for any Servos.  Hopefully that's what the
// user intended by not including Servo.h
#ifndef MAX_SERVOS
#define MAX_SERVOS 0
#endif

/*
    Firmata Hardware Abstraction Layer

Firmata is built on top of the hardware abstraction functions of Arduino,
specifically digitalWrite, digitalRead, analogWrite, analogRead, and
pinMode.  While these functions offer simple integer pin numbers, Firmata
needs more information than is provided by Arduino.  This file provides
all other hardware specific details.  To make Firmata support a new board,
only this file should require editing.

The key concept is every "pin" implemented by Firmata may be mapped to
any pin as implemented by Arduino.  Usually a simple 1-to-1 mapping is
best, but such mapping should not be assumed.  This hardware abstraction
layer allows Firmata to implement any number of pins which map onto the
Arduino implemented pins in almost any arbitrary way.


General Constants:

These constants provide basic information Firmata requires.

TOTAL_PINS: The total number of pins Firmata implemented by Firmata.
    Usually this will match the number of pins the Arduino functions
    implement, including any pins pins capable of analog or digital.
    However, Firmata may implement any number of pins.  For example,
    on Arduino Mini with 8 analog inputs, 6 of these may be used
    for digital functions, and 2 are analog only.  On such boards,
    Firmata can implement more pins than Arduino's pinMode()
    function, in order to accommodate those special pins.  The
    Firmata protocol supports a maximum of 128 pins, so this
    constant must not exceed 128.

TOTAL_ANALOG_PINS: The total number of analog input pins implemented.
    The Firmata protocol allows up to 16 analog inputs, accessed
    using offsets 0 to 15.  Because Firmata presents the analog
    inputs using different offsets than the actual pin numbers
    (a legacy of Arduino's analogRead function, and the way the
    analog input capable pins are physically labeled on all
    Arduino boards), the total number of analog input signals
    must be specified.  16 is the maximum.

VERSION_BLINK_PIN: When Firmata starts up, it will blink the version
    number.  This constant is the Arduino pin number where a
    LED is connected.


Pin Mapping Macros:

These macros provide the mapping between pins as implemented by
Firmata protocol and the actual pin numbers used by the Arduino
functions.  Even though such mappings are often simple, pin
numbers received by Firmata protocol should always be used as
input to these macros, and the result of the macro should be
used with with any Arduino function.

When Firmata is extended to support a new pin mode or feature,
a pair of macros should be added and used for all hardware
access.  For simple 1:1 mapping, these macros add no actual
overhead, yet their consistent use allows source code which
uses them consistently to be easily adapted to all other boards
with different requirements.

IS_PIN_XXXX(pin): The IS_PIN macros resolve to true or non-zero
    if a pin as implemented by Firmata corresponds to a pin
    that actually implements the named feature.

PIN_TO_XXXX(pin): The PIN_TO macros translate pin numbers as
    implemented by Firmata to the pin numbers needed as inputs
    to the Arduino functions.  The corresponding IS_PIN macro
    should always be tested before using a PIN_TO macro, so
    these macros only need to handle valid Firmata pin
    numbers for the named feature.


Port Access Inline Funtions:

For efficiency, Firmata protocol provides access to digital
input and output pins grouped by 8 bit ports.  When these
groups of 8 correspond to actual 8 bit ports as implemented
by the hardware, these inline functions can provide high
speed direct port access.  Otherwise, a default implementation
using 8 calls to digitalWrite or digitalRead is used.

When porting Firmata to a new board, it is recommended to
use the default functions first and focus only on the constants
and macros above.  When those are working, if optimized port
access is desired, these inline functions may be extended.
The recommended approach defines a symbol indicating which
optimization to use, and then conditional complication is
used within these functions.

readPort(port, bitmask):  Read an 8 bit port, returning the value.
   port:    The port number, Firmata pins port*8 to port*8+7
   bitmask: The actual pins to read, indicated by 1 bits.

writePort(port, value, bitmask):  Write an 8 bit port.
   port:    The port number, Firmata pins port*8 to port*8+7
   value:   The 8 bit value to write
   bitmask: The actual pins to write, indicated by 1 bits.
*/

/*==============================================================================
 * Board Specific Configuration
 *============================================================================*/

#ifndef digitalPinHasPWM
#define digitalPinHasPWM(p)     IS_PIN_DIGITAL(p)
#endif

// Arduino Duemilanove, Diecimila, and NG
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
#if defined(NUM_ANALOG_INPUTS) && NUM_ANALOG_INPUTS == 6
#define TOTAL_ANALOG_PINS       6
#define TOTAL_PINS              20 // 14 digital + 6 analog
#else
#define TOTAL_ANALOG_PINS       8
#define TOTAL_PINS              22 // 14 digital + 8 analog
#endif
#define VERSION_BLINK_PIN       13
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) <= 19)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) < 14 + TOTAL_ANALOG_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) - 2 < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 18 || (p) == 19)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         ((p) - 2)
#define ARDUINO_PINOUT_OPTIMIZE 1


// Wiring (and board)
#elif defined(WIRING)
#define VERSION_BLINK_PIN       WLED
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= FIRST_ANALOG_PIN && (p) < (FIRST_ANALOG_PIN+TOTAL_ANALOG_PINS))
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == SDA || (p) == SCL)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - FIRST_ANALOG_PIN)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// old Arduinos
#elif defined(__AVR_ATmega8__)
#define TOTAL_ANALOG_PINS       6
#define TOTAL_PINS              20 // 14 digital + 6 analog
#define VERSION_BLINK_PIN       13
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) <= 19)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) <= 19)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) - 2 < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 18 || (p) == 19)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         ((p) - 2)
#define ARDUINO_PINOUT_OPTIMIZE 1


// Arduino Mega
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define TOTAL_ANALOG_PINS       16
#define TOTAL_PINS              70 // 54 digital + 16 analog
#define VERSION_BLINK_PIN       13
#define PIN_SERIAL1_RX          19
#define PIN_SERIAL1_TX          18
#define PIN_SERIAL2_RX          17
#define PIN_SERIAL2_TX          16
#define PIN_SERIAL3_RX          15
#define PIN_SERIAL3_TX          14
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= 54 && (p) < TOTAL_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 2 && (p) - 2 < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 20 || (p) == 21)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) > 13 && (p) < 20)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 54)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         ((p) - 2)


// Arduino DUE
#elif defined(__SAM3X8E__)
#define TOTAL_ANALOG_PINS       12
#define TOTAL_PINS              66 // 54 digital + 12 analog
#define VERSION_BLINK_PIN       13
#define PIN_SERIAL1_RX          19
#define PIN_SERIAL1_TX          18
#define PIN_SERIAL2_RX          17
#define PIN_SERIAL2_TX          16
#define PIN_SERIAL3_RX          15
#define PIN_SERIAL3_TX          14
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= 54 && (p) < TOTAL_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 2 && (p) - 2 < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 20 || (p) == 21) // 70 71
#define IS_PIN_SERIAL(p)        ((p) > 13 && (p) < 20)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 54)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         ((p) - 2)


// Arduino/Genuino MKR1000
#elif defined(ARDUINO_SAMD_MKR1000)
#define TOTAL_ANALOG_PINS       7
#define TOTAL_PINS              22 // 8 digital + 3 spi + 2 i2c + 2 uart + 7 analog
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) <= 21)
#define IS_PIN_ANALOG(p)        ((p) >= 15 && (p) < 15 + TOTAL_ANALOG_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS) // deprecated since v2.4
#define IS_PIN_I2C(p)           ((p) == 11 || (p) == 12) // SDA = 11, SCL = 12
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == PIN_SERIAL1_RX || (p) == PIN_SERIAL1_TX) //defined in variant.h  RX = 13, TX = 14
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 15)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p) // deprecated since v2.4


// Arduino MKRZero
#elif defined(ARDUINO_SAMD_MKRZERO)
#define TOTAL_ANALOG_PINS       7
#define TOTAL_PINS              34 // 8 digital + 3 spi + 2 i2c + 2 uart + 7 analog + 3 usb + 1 aref + 5 sd + 1 bottom pad + 1 led + 1 battery adc
#define IS_PIN_DIGITAL(p)       (((p) >= 0 && (p) <= 21) || (p) == 32)
#define IS_PIN_ANALOG(p)        (((p) >= 15 && (p) < 15 + TOTAL_ANALOG_PINS) || (p) == 33)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS) // deprecated since v2.4
#define IS_PIN_I2C(p)           ((p) == 11 || (p) == 12) // SDA = 11, SCL = 12
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == PIN_SERIAL1_RX || (p) == PIN_SERIAL1_TX) //defined in variant.h  RX = 13, TX = 14
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 15)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p) // deprecated since v2.4

// Arduino MKRFox1200
#elif defined(ARDUINO_SAMD_MKRFox1200)
#define TOTAL_ANALOG_PINS       7
#define TOTAL_PINS              33 // 8 digital + 3 spi + 2 i2c + 2 uart + 7 analog + 3 usb + 1 aref + 5 sd + 1 bottom pad + 1 battery adc
#define IS_PIN_DIGITAL(p)       (((p) >= 0 && (p) <= 21))
#define IS_PIN_ANALOG(p)        (((p) >= 15 && (p) < 15 + TOTAL_ANALOG_PINS) || (p) == 32)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS) // deprecated since v2.4
#define IS_PIN_I2C(p)           ((p) == 11 || (p) == 12) // SDA = 11, SCL = 12
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == PIN_SERIAL1_RX || (p) == PIN_SERIAL1_TX) //defined in variant.h  RX = 13, TX = 14
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 15)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p) // deprecated since v2.4

// Arduino MKR WAN 1300
#elif defined(ARDUINO_SAMD_MKRWAN1300)
#define TOTAL_ANALOG_PINS       7
#define TOTAL_PINS              33
#define IS_PIN_DIGITAL(p)       (((p) >= 0 && (p) <= 21))
#define IS_PIN_ANALOG(p)        (((p) >= 15 && (p) < 15 + TOTAL_ANALOG_PINS) || (p) == 32)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS) // deprecated since v2.4
#define IS_PIN_I2C(p)           ((p) == 11 || (p) == 12) // SDA = 11, SCL = 12
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == PIN_SERIAL1_RX || (p) == PIN_SERIAL1_TX) //defined in variant.h  RX = 13, TX = 14
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 15)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p) // deprecated since v2.4

// Arduino MKR GSM 1400
#elif defined(ARDUINO_SAMD_MKRGSM1400)
#define TOTAL_ANALOG_PINS       7
#define TOTAL_PINS              33
#define IS_PIN_DIGITAL(p)       (((p) >= 0 && (p) <= 21))
#define IS_PIN_ANALOG(p)        (((p) >= 15 && (p) < 15 + TOTAL_ANALOG_PINS) || (p) == 32)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS) // deprecated since v2.4
#define IS_PIN_I2C(p)           ((p) == 11 || (p) == 12) // SDA = 11, SCL = 12
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == PIN_SERIAL1_RX || (p) == PIN_SERIAL1_TX) //defined in variant.h  RX = 13, TX = 14
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 15)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p) // deprecated since v2.4

// Arduino Zero
// Note this will work with an Arduino Zero Pro, but not with an Arduino M0 Pro
// Arduino M0 Pro does not properly map pins to the board labeled pin numbers
#elif defined(_VARIANT_ARDUINO_ZERO_)
#define TOTAL_ANALOG_PINS       6
#define TOTAL_PINS              25 // 14 digital + 6 analog + 2 i2c + 3 spi
#define TOTAL_PORTS             3  // set when TOTAL_PINS > num digitial I/O pins
#define VERSION_BLINK_PIN       LED_BUILTIN
//#define PIN_SERIAL1_RX          0 // already defined in zero core variant.h
//#define PIN_SERIAL1_TX          1 // already defined in zero core variant.h
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) <= 19)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) < 14 + TOTAL_ANALOG_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS) // deprecated since v2.4
#define IS_PIN_I2C(p)           ((p) == 20 || (p) == 21) // SDA = 20, SCL = 21
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK) // SS = A2
#define IS_PIN_SERIAL(p)        ((p) == 0 || (p) == 1)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p) // deprecated since v2.4

// Arduino Primo
#elif defined(ARDUINO_PRIMO)
#define TOTAL_ANALOG_PINS       6
#define TOTAL_PINS              22 //14 digital + 6 analog + 2 i2c
#define VERSION_BLINK_PIN       LED_BUILTIN
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) < 20)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) < 20)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS+2)
#define IS_PIN_I2C(p)           ((p) == PIN_WIRE_SDA || (p) == PIN_WIRE_SCL) // SDA = 20, SCL = 21
#define IS_PIN_SPI(p)           ((p) == SS || (p)== MOSI || (p) == MISO || (p == SCK)) // 10, 11, 12, 13
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)

// Arduino 101
#elif defined(_VARIANT_ARDUINO_101_X_)
#define TOTAL_ANALOG_PINS       NUM_ANALOG_INPUTS
#define TOTAL_PINS              NUM_DIGITAL_PINS // 15 digital (including ATN pin) + 6 analog
#define VERSION_BLINK_PIN       LED_BUILTIN
#define PIN_SERIAL1_RX          0
#define PIN_SERIAL1_TX          1
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) <= 20)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) < 14 + TOTAL_ANALOG_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p) // 3, 5, 6, 9
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS) // deprecated since v2.4
#define IS_PIN_I2C(p)           ((p) == SDA || (p) == SCL) // SDA = 18, SCL = 19
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == 0 || (p) == 1)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p) // deprecated since v2.4


// Teensy 1.0
#elif defined(__AVR_AT90USB162__)
#define TOTAL_ANALOG_PINS       0
#define TOTAL_PINS              21 // 21 digital + no analog
#define VERSION_BLINK_PIN       6
#define PIN_SERIAL1_RX          2
#define PIN_SERIAL1_TX          3
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        (0)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           (0)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == 2 || (p) == 3)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (0)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// Teensy 2.0
#elif defined(__AVR_ATmega32U4__) && defined(CORE_TEENSY)
#define TOTAL_ANALOG_PINS       12
#define TOTAL_PINS              25 // 11 digital + 12 analog
#define VERSION_BLINK_PIN       11
#define PIN_SERIAL1_RX          7
#define PIN_SERIAL1_TX          8
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= 11 && (p) <= 22)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 5 || (p) == 6)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == 7 || (p) == 8)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (((p) < 22) ? 21 - (p) : 11)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// Teensy 3.5 and 3.6
// reference: https://github.com/PaulStoffregen/cores/blob/master/teensy3/pins_arduino.h
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define TOTAL_ANALOG_PINS       27 // 3.5 has 27 and 3.6 has 25
#define TOTAL_PINS              70 // 43 digital + 21 analog-digital + 6 analog (64-69)
#define VERSION_BLINK_PIN       13
#define PIN_SERIAL1_RX          0
#define PIN_SERIAL1_TX          1
#define PIN_SERIAL2_RX          9
#define PIN_SERIAL2_TX          10
#define PIN_SERIAL3_RX          7
#define PIN_SERIAL3_TX          8
#define PIN_SERIAL4_RX          31
#define PIN_SERIAL4_TX          32
#define PIN_SERIAL5_RX          34
#define PIN_SERIAL5_TX          33
#define PIN_SERIAL6_RX          47
#define PIN_SERIAL6_TX          48
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) <= 63)
#define IS_PIN_ANALOG(p)        (((p) >= 14 && (p) <= 23) || ((p) >= 31 && (p) <= 39) || ((p) >= 49 && (p) <= 50) || ((p) >= 64 && (p) <= 69))
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 18 || (p) == 19)
#define IS_PIN_SERIAL(p)        (((p) > 6 && (p) < 11) || ((p) == 0 || (p) == 1) || ((p) > 30 && (p) < 35) || ((p) == 47 || (p) == 48))
#define PIN_TO_DIGITAL(p)       (p)
// A0-A9 = D14-D23; A12-A20 = D31-D39; A23-A24 = D49-D50; A10-A11 = D64-D65; A21-A22 = D66-D67; A25-A26 = D68-D69
#define PIN_TO_ANALOG(p)        (((p) <= 23) ? (p) - 14 : (((p) <= 39) ? (p) - 19 : (((p) <= 50) ? (p) - 26 : (((p) <= 65) ? (p) - 55 : (((p) <= 67) ? (p) - 45 : (p) - 43)))))
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// Teensy 3.0, 3.1 and 3.2
#elif defined(__MK20DX128__) || defined(__MK20DX256__)
#define TOTAL_ANALOG_PINS       14
#define TOTAL_PINS              38 // 24 digital + 10 analog-digital + 4 analog
#define VERSION_BLINK_PIN       13
#define PIN_SERIAL1_RX          0
#define PIN_SERIAL1_TX          1
#define PIN_SERIAL2_RX          9
#define PIN_SERIAL2_TX          10
#define PIN_SERIAL3_RX          7
#define PIN_SERIAL3_TX          8
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) <= 33)
#define IS_PIN_ANALOG(p)        (((p) >= 14 && (p) <= 23) || ((p) >= 34 && (p) <= 38))
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 18 || (p) == 19)
#define IS_PIN_SERIAL(p)        (((p) > 6 && (p) < 11) || ((p) == 0 || (p) == 1))
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (((p) <= 23) ? (p) - 14 : (p) - 24)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// Teensy-LC
#elif defined(__MKL26Z64__)
#define TOTAL_ANALOG_PINS       13
#define TOTAL_PINS              27 // 27 digital + 13 analog-digital
#define VERSION_BLINK_PIN       13
#define PIN_SERIAL1_RX          0
#define PIN_SERIAL1_TX          1
#define PIN_SERIAL2_RX          9
#define PIN_SERIAL2_TX          10
#define PIN_SERIAL3_RX          7
#define PIN_SERIAL3_TX          8
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) <= 26)
#define IS_PIN_ANALOG(p)        ((p) >= 14)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 18 || (p) == 19)
#define IS_PIN_SERIAL(p)        (((p) > 6 && (p) < 11) || ((p) == 0 || (p) == 1))
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// Teensy++ 1.0 and 2.0
#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
#define TOTAL_ANALOG_PINS       8
#define TOTAL_PINS              46 // 38 digital + 8 analog
#define VERSION_BLINK_PIN       6
#define PIN_SERIAL1_RX          2
#define PIN_SERIAL1_TX          3
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= 38 && (p) < TOTAL_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 0 || (p) == 1)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == 2 || (p) == 3)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 38)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// Leonardo
#elif defined(__AVR_ATmega32U4__)
#define TOTAL_ANALOG_PINS       12
#define TOTAL_PINS              30 // 14 digital + 12 analog + 4 SPI (D14-D17 on ISP header)
#define VERSION_BLINK_PIN       13
#define PIN_SERIAL1_RX          0
#define PIN_SERIAL1_TX          1
#define IS_PIN_DIGITAL(p)       ((p) >= 0 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= 18 && (p) < TOTAL_PINS)
#define IS_PIN_PWM(p)           ((p) == 3 || (p) == 5 || (p) == 6 || (p) == 9 || (p) == 10 || (p) == 11 || (p) == 13)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 2 || (p) == 3)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == 0 || (p) == 1)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (p) - 18
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// Intel Galileo Board (gen 1 and 2) and Intel Edison
#elif defined(ARDUINO_LINUX)
#define TOTAL_ANALOG_PINS       6
#define TOTAL_PINS              20 // 14 digital + 6 analog
#define VERSION_BLINK_PIN       13
#define PIN_SERIAL1_RX          0
#define PIN_SERIAL1_TX          1
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) <= 19)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) <= 19)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) - 2 < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == SDA || (p) == SCL)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == 0 || (p) == 1)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         ((p) - 2)


// RedBearLab BLE Nano with factory switch settings (S1 - S10)
#elif defined(BLE_NANO)
#define TOTAL_ANALOG_PINS       6
#define TOTAL_PINS              15 // 9 digital + 3 analog
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) <= 14)
#define IS_PIN_ANALOG(p)        ((p) == 8 || (p) == 9 || (p) == 10 || (p) == 11 || (p) == 12 || (p) == 14) //A0~A5
#define IS_PIN_PWM(p)           ((p) == 3 || (p) == 5 || (p) == 6)
#define IS_PIN_SERVO(p)         ((p) >= 2 && (p) <= 7)
#define IS_PIN_I2C(p)           ((p) == SDA || (p) == SCL)
#define IS_PIN_SPI(p)           ((p) == CS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 8)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)


// Sanguino
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
#define TOTAL_ANALOG_PINS       8
#define TOTAL_PINS              32 // 24 digital + 8 analog
#define VERSION_BLINK_PIN       0
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= 24 && (p) < TOTAL_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 16 || (p) == 17)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 24)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         ((p) - 2)


// Illuminato
#elif defined(__AVR_ATmega645__)
#define TOTAL_ANALOG_PINS       6
#define TOTAL_PINS              42 // 36 digital + 6 analog
#define VERSION_BLINK_PIN       13
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) >= 36 && (p) < TOTAL_PINS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         ((p) >= 0 && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == 4 || (p) == 5)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 36)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         ((p) - 2)


// Pic32 chipKIT FubarinoSD
#elif defined(_BOARD_FUBARINO_SD_)
#define TOTAL_ANALOG_PINS       NUM_ANALOG_PINS  // 15
#define TOTAL_PINS              NUM_DIGITAL_PINS // 45, All pins can be digital
#define MAX_SERVOS              NUM_DIGITAL_PINS
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       1
#define IS_PIN_ANALOG(p)        ((p) >= 30 && (p) <= 44)
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 1 || (p) == 2)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (14 - (p - 30))
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)


// Pic32 chipKIT FubarinoMini
// Note, FubarinoMini analog pin 20 will not function in Firmata as analog input due to limitation in analog mapping
#elif defined(_BOARD_FUBARINO_MINI_)
#define TOTAL_ANALOG_PINS       14 // We have to fake this because of the poor analog pin mapping planning in FubarinoMini
#define TOTAL_PINS              NUM_DIGITAL_PINS // 33
#define MAX_SERVOS              NUM_DIGITAL_PINS
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       ((p) != 14 && (p) != 15 && (p) != 31 && (p) != 32)
#define IS_PIN_ANALOG(p)        ((p) == 0 || ((p) >= 3 && (p) <= 13))
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 25 || (p) == 26)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (p)
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)


// Pic32 chipKIT UNO32
#elif defined(_BOARD_UNO_) && defined(__PIC32)  // NOTE: no _BOARD_UNO32_ to use
#define TOTAL_ANALOG_PINS       NUM_ANALOG_PINS // 12
#define TOTAL_PINS              NUM_DIGITAL_PINS // 47 All pins can be digital
#define MAX_SERVOS              NUM_DIGITAL_PINS // All pins can be servo with SoftPWMservo
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       ((p) >= 2)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) <= 25)
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 45 || (p) == 46)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)


// Pic32 chipKIT DP32
#elif defined(_BOARD_DP32_)
#define TOTAL_ANALOG_PINS       15  // Really only has 9, but have to override because of mistake in variant file
#define TOTAL_PINS              NUM_DIGITAL_PINS // 19
#define MAX_SERVOS              NUM_DIGITAL_PINS // All pins can be servo with SoftPWMservo
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       (((p) != 1) && ((p) != 4) && ((p) != 5) && ((p) != 15) && ((p) != 16))
#define IS_PIN_ANALOG(p)        ((p) >= 6 && (p) <= 14)
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 2 || (p) == 3)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (p)
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)


// Pic32 chipKIT uC32
#elif defined(_BOARD_UC32_)
#define TOTAL_ANALOG_PINS       NUM_ANALOG_PINS  // 12
#define TOTAL_PINS              NUM_DIGITAL_PINS // 47 All pins can be digital
#define MAX_SERVOS              NUM_DIGITAL_PINS // All pins can be servo with SoftPWMservo
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       ((p) >= 2)
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) <= 25)
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 45 || (p) == 46)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)


// Pic32 chipKIT WF32
#elif defined(_BOARD_WF32_)
#define TOTAL_ANALOG_PINS       NUM_ANALOG_PINS
#define TOTAL_PINS              NUM_DIGITAL_PINS
#define MAX_SERVOS              NUM_DIGITAL_PINS
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) <= 49)     // Accounts for SD and WiFi dedicated pins
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) <= 25)
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 34 || (p) == 35)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 14)
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)


// Pic32 chipKIT WiFire
#elif defined(_BOARD_WIFIRE_)
#define TOTAL_ANALOG_PINS       NUM_ANALOG_PINS  // 14
#define TOTAL_PINS              NUM_DIGITAL_PINS // 71
#define MAX_SERVOS              NUM_DIGITAL_PINS
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) <= 47)     // Accounts for SD and WiFi dedicated pins
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) <= 25)
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 34 || (p) == 35)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) <= 25 ? ((p) - 14) : (p) - 36)
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)


// Pic32 chipKIT MAX32
#elif defined(_BOARD_MEGA_) && defined(__PIC32)  // NOTE: no _BOARD_MAX32_ to use
#define TOTAL_ANALOG_PINS       NUM_ANALOG_PINS  // 16
#define TOTAL_PINS              NUM_DIGITAL_PINS // 87
#define MAX_SERVOS              NUM_DIGITAL_PINS
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       ((p) >= 2)
#define IS_PIN_ANALOG(p)        ((p) >= 54 && (p) <= 69)
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 34 || (p) == 35)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 54)
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)


// Pic32 chipKIT Pi
#elif defined(_BOARD_CHIPKIT_PI_)
#define TOTAL_ANALOG_PINS       16
#define TOTAL_PINS              NUM_DIGITAL_PINS // 19
#define MAX_SERVOS              NUM_DIGITAL_PINS
#define VERSION_BLINK_PIN       PIN_LED1
#define IS_PIN_DIGITAL(p)       (((p) >= 2) && ((p) <= 3) || (((p) >= 8) && ((p) <= 13)) || (((p) >= 14) && ((p) <= 17)))
#define IS_PIN_ANALOG(p)        ((p) >= 14 && (p) <= 17)
#define IS_PIN_PWM(p)           IS_PIN_DIGITAL(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == 16 || (p) == 17)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) <= 15 ? (p) - 14 : (p) - 12)
//#define PIN_TO_ANALOG(p)        (((p) <= 16) ? ((p) - 14) : ((p) - 16))
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)

// Pinoccio Scout
// Note: digital pins 9-16 are usable but not labeled on the board numerically.
// SS=9, MOSI=10, MISO=11, SCK=12, RX1=13, TX1=14, SCL=15, SDA=16
#elif defined(ARDUINO_PINOCCIO)
#define TOTAL_ANALOG_PINS       8
#define TOTAL_PINS              NUM_DIGITAL_PINS // 32
#define VERSION_BLINK_PIN       23
#define PIN_SERIAL1_RX          13
#define PIN_SERIAL1_TX          14
#define IS_PIN_DIGITAL(p)       (((p) >= 2) && ((p) <= 16)) || (((p) >= 24) && ((p) <= 31))
#define IS_PIN_ANALOG(p)        ((p) >= 24 && (p) <= 31)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == SCL || (p) == SDA)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_SERIAL(p)        ((p) == 13 || (p) == 14)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - 24)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         ((p) - 2)

// ESP8266
// note: boot mode GPIOs 0, 2 and 15 can be used as outputs, GPIOs 6-11 are in use for flash IO
#elif defined(ESP8266)
#define TOTAL_ANALOG_PINS       NUM_ANALOG_INPUTS
#define TOTAL_PINS              A0 + NUM_ANALOG_INPUTS
#define PIN_SERIAL_RX           3
#define PIN_SERIAL_TX           1
#define IS_PIN_DIGITAL(p)       (((p) >= 0 && (p) <= 5) || ((p) >= 12 && (p) < A0))
#define IS_PIN_ANALOG(p)        ((p) >= A0 && (p) < A0 + NUM_ANALOG_INPUTS)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         (IS_PIN_DIGITAL(p) && (p) < MAX_SERVOS)
#define IS_PIN_I2C(p)           ((p) == SDA || (p) == SCL)
#define IS_PIN_SPI(p)           ((p) == SS || (p) == MOSI || (p) == MISO || (p) == SCK)
#define IS_PIN_INTERRUPT(p)     (digitalPinToInterrupt(p) > NOT_AN_INTERRUPT)
#define IS_PIN_SERIAL(p)        ((p) == PIN_SERIAL_RX || (p) == PIN_SERIAL_TX)
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ((p) - A0)
#define PIN_TO_PWM(p)           PIN_TO_DIGITAL(p)
#define PIN_TO_SERVO(p)         (p)
#define DEFAULT_PWM_RESOLUTION  10

// STM32 based boards
#elif defined(ARDUINO_ARCH_STM32)
#define TOTAL_ANALOG_PINS       NUM_ANALOG_INPUTS
#define TOTAL_PINS              NUM_DIGITAL_PINS
#define TOTAL_PORTS             MAX_NB_PORT
#define VERSION_BLINK_PIN       LED_BUILTIN
// PIN_SERIALY_RX/TX defined in the variant.h
#define IS_PIN_DIGITAL(p)       (digitalPinIsValid(p) && !pinIsSerial(p))
#define IS_PIN_ANALOG(p)        ((p >= A0) && (p < (A0 + TOTAL_ANALOG_PINS)) && !pinIsSerial(p))
#define IS_PIN_PWM(p)           (IS_PIN_DIGITAL(p) && digitalPinHasPWM(p))
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           (IS_PIN_DIGITAL(p) && digitalPinHasI2C(p))
#define IS_PIN_SPI(p)           (IS_PIN_DIGITAL(p) && digitalPinHasSPI(p))
#define IS_PIN_INTERRUPT(p)     (IS_PIN_DIGITAL(p) && (digitalPinToInterrupt(p) > NOT_AN_INTERRUPT)))
#define IS_PIN_SERIAL(p)        (digitalPinHasSerial(p) && !pinIsSerial(p))
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        (p-A0)
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)
#define DEFAULT_PWM_RESOLUTION  PWM_RESOLUTION

// Adafruit Bluefruit nRF52 boards
#elif defined(ARDUINO_NRF52_ADAFRUIT)
#define TOTAL_ANALOG_PINS       NUM_ANALOG_INPUTS
#define TOTAL_PINS              32
#define VERSION_BLINK_PIN       LED_BUILTIN
#define IS_PIN_DIGITAL(p)       ((p) >= 2 && (p) < TOTAL_PINS)
#define IS_PIN_ANALOG(p)        ((p) == PIN_A0 || (p) == PIN_A1 || (p) == PIN_A2  || (p) == PIN_A3 || \
                                 (p) == PIN_A4 || (p) == PIN_A5 || (p) == PIN_A6  || (p) == PIN_A7)
#define IS_PIN_PWM(p)           digitalPinHasPWM(p)
#define IS_PIN_SERVO(p)         IS_PIN_DIGITAL(p)
#define IS_PIN_I2C(p)           ((p) == PIN_WIRE_SDA || (p) == PIN_WIRE_SCL)
#define IS_PIN_SPI(p)           ((p) == SS || (p)== MOSI || (p) == MISO || (p == SCK))
#define PIN_TO_DIGITAL(p)       (p)
#define PIN_TO_ANALOG(p)        ( ((p) == PIN_A0) ? 0 : ((p) == PIN_A1) ? 1 : ((p) == PIN_A2) ? 2 : ((p) == PIN_A3) ? 3 : \
                                  ((p) == PIN_A4) ? 4 : ((p) == PIN_A5) ? 5 : ((p) == PIN_A6) ? 6 : ((p) == PIN_A7) ? 7 : (127))
#define PIN_TO_PWM(p)           (p)
#define PIN_TO_SERVO(p)         (p)

// anything else
#else
#error "Please edit Boards.h with a hardware abstraction for this board"
#endif

// as long this is not defined for all boards:
#ifndef IS_PIN_SPI
#define IS_PIN_SPI(p)           0
#endif

#ifndef IS_PIN_SERIAL
#define IS_PIN_SERIAL(p)        0
#endif

#ifndef DEFAULT_PWM_RESOLUTION
#define DEFAULT_PWM_RESOLUTION  8
#endif

/*==============================================================================
 * readPort() - Read an 8 bit port
 *============================================================================*/

static inline unsigned char readPort(byte, byte) __attribute__((always_inline, unused));
static inline unsigned char readPort(byte port, byte bitmask)
{
#if defined(ARDUINO_PINOUT_OPTIMIZE)
  if (port == 0) return (PIND & 0xFC) & bitmask; // ignore Rx/Tx 0/1
  if (port == 1) return ((PINB & 0x3F) | ((PINC & 0x03) << 6)) & bitmask;
  if (port == 2) return ((PINC & 0x3C) >> 2) & bitmask;
  return 0;
#else
  unsigned char out = 0, pin = port * 8;
  if (IS_PIN_DIGITAL(pin + 0) && (bitmask & 0x01) && digitalRead(PIN_TO_DIGITAL(pin + 0))) out |= 0x01;
  if (IS_PIN_DIGITAL(pin + 1) && (bitmask & 0x02) && digitalRead(PIN_TO_DIGITAL(pin + 1))) out |= 0x02;
  if (IS_PIN_DIGITAL(pin + 2) && (bitmask & 0x04) && digitalRead(PIN_TO_DIGITAL(pin + 2))) out |= 0x04;
  if (IS_PIN_DIGITAL(pin + 3) && (bitmask & 0x08) && digitalRead(PIN_TO_DIGITAL(pin + 3))) out |= 0x08;
  if (IS_PIN_DIGITAL(pin + 4) && (bitmask & 0x10) && digitalRead(PIN_TO_DIGITAL(pin + 4))) out |= 0x10;
  if (IS_PIN_DIGITAL(pin + 5) && (bitmask & 0x20) && digitalRead(PIN_TO_DIGITAL(pin + 5))) out |= 0x20;
  if (IS_PIN_DIGITAL(pin + 6) && (bitmask & 0x40) && digitalRead(PIN_TO_DIGITAL(pin + 6))) out |= 0x40;
  if (IS_PIN_DIGITAL(pin + 7) && (bitmask & 0x80) && digitalRead(PIN_TO_DIGITAL(pin + 7))) out |= 0x80;
  return out;
#endif
}

/*==============================================================================
 * writePort() - Write an 8 bit port, only touch pins specified by a bitmask
 *============================================================================*/

static inline unsigned char writePort(byte, byte, byte) __attribute__((always_inline, unused));
static inline unsigned char writePort(byte port, byte value, byte bitmask)
{
#if defined(ARDUINO_PINOUT_OPTIMIZE)
  if (port == 0) {
    bitmask = bitmask & 0xFC;  // do not touch Tx & Rx pins
    byte valD = value & bitmask;
    byte maskD = ~bitmask;
    cli();
    PORTD = (PORTD & maskD) | valD;
    sei();
  } else if (port == 1) {
    byte valB = (value & bitmask) & 0x3F;
    byte valC = (value & bitmask) >> 6;
    byte maskB = ~(bitmask & 0x3F);
    byte maskC = ~((bitmask & 0xC0) >> 6);
    cli();
    PORTB = (PORTB & maskB) | valB;
    PORTC = (PORTC & maskC) | valC;
    sei();
  } else if (port == 2) {
    bitmask = bitmask & 0x0F;
    byte valC = (value & bitmask) << 2;
    byte maskC = ~(bitmask << 2);
    cli();
    PORTC = (PORTC & maskC) | valC;
    sei();
  }
  return 1;
#else
  byte pin = port * 8;
  if ((bitmask & 0x01)) digitalWrite(PIN_TO_DIGITAL(pin + 0), (value & 0x01));
  if ((bitmask & 0x02)) digitalWrite(PIN_TO_DIGITAL(pin + 1), (value & 0x02));
  if ((bitmask & 0x04)) digitalWrite(PIN_TO_DIGITAL(pin + 2), (value & 0x04));
  if ((bitmask & 0x08)) digitalWrite(PIN_TO_DIGITAL(pin + 3), (value & 0x08));
  if ((bitmask & 0x10)) digitalWrite(PIN_TO_DIGITAL(pin + 4), (value & 0x10));
  if ((bitmask & 0x20)) digitalWrite(PIN_TO_DIGITAL(pin + 5), (value & 0x20));
  if ((bitmask & 0x40)) digitalWrite(PIN_TO_DIGITAL(pin + 6), (value & 0x40));
  if ((bitmask & 0x80)) digitalWrite(PIN_TO_DIGITAL(pin + 7), (value & 0x80));
  return 1;
#endif
}




#ifndef TOTAL_PORTS
#define TOTAL_PORTS             ((TOTAL_PINS + 7) / 8)
#endif


#endif /* Firmata_Boards_h */

//------------------------------   end of Boards.h



int connection_state;
long state_2_timer, timer2 = 0, timer_last = 0, timer_5 = 0;
long time_of_connection =0;
long DHT_last;
int DHT_last_temp = -100;
int DHT_last_humid = -1;
boolean sent = false;
boolean in_ir_key_fix = false, in_ir_key_return = false;
int key_pressed;
byte byte1_stored, byte2_stored, byte3_stored, byte4_stored, display_current_image;
long ir_number_received, ir_number_received_stored;
byte left_servo_wheel = -1, right_servo_wheel = -1, left_dc_wheel_pin_1 = -1, left_dc_wheel_pin_2 = -1, right_dc_wheel_pin_1 = -1,right_dc_wheel_pin_2 = -1;


//=================================================================================================================================


boolean isMelodyToPlay = false;
boolean isUserMelody = false;
#define SPEAKER          12
#define TEXT_TO_SAY_BUFFER_LEN  40
#define MELODY_TO_PLAY_BUFFER_LEN 30
#define AUDIO_SAY     0
#define AUDIO_MELODY_BLTIN  1
#define AUDIO_MELODY_USR  2
#define AUDIO_TONE      3
#define AUDIO_MELODY_SPEED  4
float melodyToPlaySpeed = 1.0;
uint8_t  melodyToPlayLen = 0;
uint16_t melodyToPlayNoteBuffer[MELODY_TO_PLAY_BUFFER_LEN], melodyRecordStart = 0;
uint8_t melodyToPlayDurationBuffer[MELODY_TO_PLAY_BUFFER_LEN];
#define NOTE_RST 0
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define NOTE_P0  0000

#define AUDIO_MELODIES_BLTIN  9

const static uint16_t AUDIO_MELODIES_NOTES[] PROGMEM = {
  // Green Sleeves 
  19,
  NOTE_FS3,
  NOTE_A3, NOTE_B3,
  NOTE_CS4, NOTE_D4, NOTE_CS4,
  NOTE_B3, NOTE_GS3,
  NOTE_E3, NOTE_FS3, NOTE_GS3,
  NOTE_A3, NOTE_FS3,
  NOTE_FS3, NOTE_F3, NOTE_FS3,
  NOTE_GS3, NOTE_F3,
  NOTE_CS3,
  // Marry Had a Little Lamb
  26,
  NOTE_E3, NOTE_D3, NOTE_C3, NOTE_D3,
  NOTE_E3, NOTE_E3, NOTE_E3,
  NOTE_D3, NOTE_D3, NOTE_D3,
  NOTE_E3, NOTE_G3, NOTE_G3,
  NOTE_E3, NOTE_D3, NOTE_C3, NOTE_D3,
  NOTE_E3, NOTE_E3, NOTE_E3, NOTE_E3,
  NOTE_D3, NOTE_D3, NOTE_E3, NOTE_D3,
  NOTE_C3,
  // Happy Birthday
  25,
  NOTE_G3, NOTE_G3,
  NOTE_A3, NOTE_G3, NOTE_C4,
  NOTE_B3, NOTE_G3, NOTE_G3,
  NOTE_A3, NOTE_G3, NOTE_D4,
  NOTE_C4, NOTE_G3, NOTE_G3,
  NOTE_G4, NOTE_E4, NOTE_C4,
  NOTE_B3, NOTE_A3, NOTE_G4, NOTE_G4,
  NOTE_E4, NOTE_C4, NOTE_D4,
  NOTE_C4,
  // Star Wars
  40,
  NOTE_A3, NOTE_A3, NOTE_A3,
  NOTE_D4, NOTE_A4, NOTE_RST,
  NOTE_G4, NOTE_FS4, NOTE_E4, NOTE_D5, NOTE_A4,
  NOTE_G4, NOTE_FS4, NOTE_E4, NOTE_D5, NOTE_A4,
  NOTE_G4, NOTE_FS4, NOTE_G4, NOTE_E4, NOTE_RST, NOTE_A3, NOTE_A3, NOTE_A3,
  NOTE_D4, NOTE_A4,
  NOTE_G4, NOTE_FS4, NOTE_E4, NOTE_D5, NOTE_A4,
  NOTE_G4, NOTE_FS4, NOTE_E4, NOTE_D5, NOTE_A4,
  NOTE_G4, NOTE_FS4, NOTE_G4, NOTE_E4,
  // Chariots of Fire
  81,
  NOTE_C4, NOTE_F4, NOTE_G4, NOTE_A4,
  NOTE_G4, NOTE_E4, NOTE_RST, NOTE_C4, NOTE_F4, NOTE_G4, NOTE_A4,
  NOTE_G4, NOTE_RST, NOTE_C4, NOTE_F4, NOTE_G4, NOTE_A4,
  NOTE_G4, NOTE_E4, NOTE_RST, NOTE_E4, NOTE_F4, NOTE_E4, NOTE_C4,
  NOTE_C4, NOTE_RST, NOTE_C4, NOTE_F4, NOTE_G4, NOTE_A4,
  NOTE_G4, NOTE_E4, NOTE_RST, NOTE_C4, NOTE_F4, NOTE_G4, NOTE_A4,
  NOTE_G4, NOTE_RST, NOTE_C4, NOTE_F4, NOTE_G4, NOTE_A4,
  NOTE_G4, NOTE_E4, NOTE_RST, NOTE_E4, NOTE_F4, NOTE_E4, NOTE_C4,
  NOTE_C4, NOTE_RST, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_G4,
  NOTE_B4, NOTE_G4, NOTE_A4, NOTE_F4, NOTE_G4, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_G4,
  NOTE_B4, NOTE_RST, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_G4,
  NOTE_B4, NOTE_G4, NOTE_A4, NOTE_F4, NOTE_G4, NOTE_E4, NOTE_F4, NOTE_E4, NOTE_C4,
  NOTE_C4,
  //connected 
  3,
  NOTE_G3, NOTE_A3, NOTE_B3,
  
  // disconnected
  3,
  NOTE_B3, NOTE_A3, NOTE_G3,
  //OK
  2,
  NOTE_G3, NOTE_B3,
  // OOPS
  2,
  NOTE_B3, NOTE_G3,
  // BEEP
  1,
  NOTE_A4
};

const uint16_t AUDIO_MELODIES_NOTES_LEN = sizeof(AUDIO_MELODIES_NOTES) / sizeof(uint16_t);

const static uint8_t AUDIO_MELODIES_DURATIONS[] PROGMEM = {
  // Green Sleevs
  19,
  4,
  2, 4,
  3, 8, 4,
  2, 4,
  3, 8, 4,
  2, 4,
  3, 8, 4,
  2, 4,
  1,
  // Mary Had a Little Lamb
  26,
  4, 4, 4, 4,
  4, 4, 2,
  4, 4, 2,
  4, 4, 2,
  4, 4, 4, 4,
  4, 4, 4, 4,
  4, 4, 4, 4,
  1,
  // Happy Birthday
  25,
  8, 8,
  4, 4, 4,
  2, 8, 8,
  4, 4, 4,
  2, 8, 8,
  4, 4, 4,
  4, 4, 8, 8,
  4, 4, 4,
  2,
  // Star Wars
  40,
  12, 12, 12,
  2, 2, 8,
  12, 12, 12, 2, 4,
  12, 12, 12, 2, 4,
  12, 12, 12, 2, 8, 12, 12, 12,
  2, 2,
  12, 12, 12, 2, 4,
  12, 12, 12, 2, 4,
  12, 12, 12, 2,
  // Chariots of fire
  81,
  8, 12, 12, 12,
  4, 4, 8, 8, 12, 12, 12,
  2, 8, 8, 12, 12, 12,
  4, 4, 8, 8, 12, 12, 12,
  2, 8, 8, 12, 12, 12,
  4, 4, 8, 8, 12, 12, 12,
  2, 8, 8, 12, 12, 12,
  4, 4, 8, 8, 12, 12, 12,
  2, 8, 8, 12, 12, 12,
  4, 16, 4, 16, 4, 16, 12, 12, 12,
  2, 8, 8, 12, 12, 12,
  4, 16, 4, 16, 4, 16, 12, 12, 12,
  2,
  //connected 
  3,
  8, 8, 4,
  //disconnected
  3, 
  8, 8, 4,
   //OK 
  2,
  8, 4,
  //OOPS
  2, 
  8, 4,
  //beep
  1,
  2
};

const uint16_t AUDIO_MELODIES_DURATIONS_LEN = sizeof(AUDIO_MELODIES_DURATIONS) / sizeof(uint8_t);

/*const static uint16_t NOTES[] PROGMEM = { // NOTE_P0 included to force a nice hash function 
  NOTE_A1, NOTE_AS1, NOTE_B1, NOTE_P0, NOTE_C1, NOTE_CS1, NOTE_D1, NOTE_DS1, NOTE_E1, NOTE_P0, NOTE_F1, NOTE_FS1, NOTE_G1, NOTE_GS1,  // Scale C1
  NOTE_A2, NOTE_AS1, NOTE_B2, NOTE_P0, NOTE_C2, NOTE_CS2, NOTE_D2, NOTE_DS2, NOTE_E2, NOTE_P0, NOTE_F2, NOTE_FS2, NOTE_G2, NOTE_GS2,  // Scale C2 
  NOTE_A3, NOTE_AS1, NOTE_B3, NOTE_P0, NOTE_C3, NOTE_CS3, NOTE_D3, NOTE_DS3, NOTE_E3, NOTE_P0, NOTE_F3, NOTE_FS3, NOTE_G3, NOTE_GS3,  // Scale C3 
  NOTE_A4, NOTE_AS1, NOTE_B4, NOTE_P0, NOTE_C4, NOTE_CS4, NOTE_D4, NOTE_DS4, NOTE_E4, NOTE_P0, NOTE_F4, NOTE_FS4, NOTE_G4, NOTE_GS4,  // Scale C4 
  NOTE_A5, NOTE_AS1, NOTE_B5, NOTE_P0, NOTE_C5, NOTE_CS5, NOTE_D5, NOTE_DS5, NOTE_E5, NOTE_P0, NOTE_F5, NOTE_FS5, NOTE_G5, NOTE_GS5,  // Scale C5 
  NOTE_A6, NOTE_AS1, NOTE_B6, NOTE_P0, NOTE_C6, NOTE_CS6, NOTE_D6, NOTE_DS6, NOTE_E6, NOTE_P0, NOTE_F6, NOTE_FS6, NOTE_G6, NOTE_GS6,  // Scale C6 
  NOTE_A7, NOTE_AS1, NOTE_B7, NOTE_P0, NOTE_C7, NOTE_CS7, NOTE_D7, NOTE_DS7, NOTE_E7, NOTE_P0, NOTE_F7, NOTE_FS7, NOTE_G7, NOTE_GS7,  // Scale C7 
  NOTE_P0, NOTE_P0, NOTE_P0, NOTE_P0, NOTE_C8, NOTE_CS8, NOTE_D1, NOTE_DS8                              // Scale C8 
};

const uint8_t NOTES_LEN = sizeof(NOTES) / sizeof(uint16_t);

*/
void setMelodytoPlay(uint16_t melody) {
  melodyRecordStart = 0; // Reset the record start

  for (uint16_t i = 0; i < melody; i++) {  // Work out melodies location
    melodyRecordStart += pgm_read_word(&(AUDIO_MELODIES_NOTES[melodyRecordStart])) + 1; // melody record number_of_notes (+1);
  }

  melodyToPlayLen = pgm_read_word(&(AUDIO_MELODIES_NOTES[melodyRecordStart]));

  melodyRecordStart = melodyRecordStart + 1; // offset the melody record start for first note in record
}

void updateMelodyToPlay(boolean _isUserMelody) {
  /* timer variables */
  static uint16_t note = 0;
  static uint16_t noteFrequency;
  static boolean noteActive = false, notePause = false;
  static unsigned long l_currentMillis; // store the current value from millis()
  static unsigned long l_previousMillis;  // for comparison with currentMillis
  static unsigned long noteDuration;    // store current note duration
  static unsigned long notePauseDuration; // store current note duration

  l_currentMillis = millis();

  if (note >= melodyToPlayLen) { // All done, reset
    isMelodyToPlay = false;
    note = 0;
    melodyToPlayLen = 0;
  }
  else { // Still have a note to play, have we finished the note?
    if (!noteActive) { // If no active note, play the next one
      if (_isUserMelody) {
        noteDuration = ((1000.0 / melodyToPlaySpeed) / (float)melodyToPlayDurationBuffer[note]);
        noteFrequency = melodyToPlayNoteBuffer[note];
      }
      else {
        noteDuration = ((1000.0 / melodyToPlaySpeed) / (float)pgm_read_byte(&(AUDIO_MELODIES_DURATIONS[melodyRecordStart + note])));
        noteFrequency = pgm_read_word(&(AUDIO_MELODIES_NOTES[melodyRecordStart + note]));
      }
      tone(SPEAKER, noteFrequency, noteDuration);
      noteActive = true;
      l_previousMillis = l_currentMillis;
    }
    else { // update timmer and tone
      if (!notePause) {
        if ((l_currentMillis - l_previousMillis) > noteDuration) { // tone duration reached
          notePauseDuration = noteDuration * 0.30; // 30% seems to work well
          notePause = true;
          l_previousMillis = l_currentMillis;
        }
      }
      else {
        if ((l_currentMillis - l_previousMillis) > notePauseDuration) {
          noTone(SPEAKER);
          notePause = false;
          noteActive = false;
          note++; // next note
        }
      }
    }
  }
}
//==============================================================================================================================================================



//******************************************************************************
//* Includes
//******************************************************************************


#if defined(__cplusplus) && !defined(ARDUINO)
  #include <cstring>
#else
  #include <string.h>
#endif



using namespace firmata;

//******************************************************************************
//* Support Functions
//******************************************************************************

/**
 * Request or halt a stream of analog readings from the Firmata host application. The range of pins is
 * limited to [0..15] when using the REPORT_ANALOG. The maximum result of the REPORT_ANALOG is limited to 14 bits
 * (16384). To increase the pin range or value, see the documentation for the EXTENDED_ANALOG
 * message.
 * @param pin The analog pin for which to request the value (limited to pins 0 - 15).
 * @param stream_enable A zero value will disable the stream, a non-zero will enable the stream
 * @note The maximum resulting value is 14-bits (16384).
 */
void FirmataMarshaller::reportAnalog(uint8_t pin, bool stream_enable)
const
{
  if ( (Stream *)NULL == FirmataStream ) { return; }
  // pin can only be 0-15, so chop higher bits
  FirmataStream->write(REPORT_ANALOG | (pin & 0xF));
  FirmataStream->write(stream_enable);
}

/**
 * Request or halt an 8-bit port stream from the Firmata host application (protocol v2 and later).
 * Send 14-bits in a single digital message (protocol v1).
 * @param portNumber The port number for which to request the value. Note that this is not the same as a "port" on the
 * physical microcontroller. Ports are defined in order per every 8 pins in ascending order
 * of the Arduino digital pin numbering scheme. Port 0 = pins D0 - D7, port 1 = pins D8 - D15, etc.
 * @param stream_enable A zero value will disable the stream, a non-zero will enable the stream
 */
void FirmataMarshaller::reportDigitalPort(uint8_t portNumber, bool stream_enable)
const
{
  if ( (Stream *)NULL == FirmataStream ) { return; }
  FirmataStream->write(REPORT_DIGITAL | (portNumber & 0xF));
  FirmataStream->write(stream_enable);
}

/**
 * An alternative to the normal analog message, this extended version allows addressing beyond
 * pin 15 and supports sending analog values with any number of bits.
 * @param pin The analog pin to which the value is sent.
 * @param bytec The size of the storage for the analog value
 * @param bytev The pointer to the location of the analog value
 */
void FirmataMarshaller::sendExtendedAnalog(uint8_t pin, size_t bytec, uint8_t * bytev)
const
{
  if ( (Stream *)NULL == FirmataStream ) { return; }
  FirmataStream->write(START_SYSEX);
  FirmataStream->write(EXTENDED_ANALOG);
  FirmataStream->write(pin);
  encodeByteStream(bytec, bytev, bytec);
  FirmataStream->write(END_SYSEX);
}

/**
 * Transform 8-bit stream into 7-bit message
 * @param bytec The number of data bytes in the message.
 * @param bytev A pointer to the array of data bytes to send in the message.
 * @param max_bytes Force message to be n bytes, regardless of data bits.
 */
void FirmataMarshaller::encodeByteStream (size_t bytec, uint8_t * bytev, size_t max_bytes)
const
{
  static const size_t transmit_bits = 7;
  static const uint8_t transmit_mask = ((1 << transmit_bits) - 1);

  size_t bytes_sent = 0;
  size_t outstanding_bits = 0;
  uint8_t outstanding_bit_cache = *bytev;

  if ( !max_bytes ) { max_bytes = static_cast<size_t>(-1); }
  for (size_t i = 0 ; (i < bytec) && (bytes_sent < max_bytes) ; ++i) {
    uint8_t transmit_byte = (outstanding_bit_cache|(bytev[i] << outstanding_bits));
    FirmataStream->write(transmit_mask & transmit_byte);
    ++bytes_sent;
    outstanding_bit_cache = (bytev[i] >> (transmit_bits - outstanding_bits));
    outstanding_bits = (outstanding_bits + (8 - transmit_bits));
    for ( ; (outstanding_bits >= transmit_bits) && (bytes_sent < max_bytes) ; ) {
      transmit_byte = outstanding_bit_cache;
    FirmataStream->write(transmit_mask & transmit_byte);
      ++bytes_sent;
      outstanding_bit_cache >>= transmit_bits;
      outstanding_bits -= transmit_bits;
    }
  }
  if ( outstanding_bits && (bytes_sent < max_bytes) ) {
    FirmataStream->write(static_cast<uint8_t>((1 << outstanding_bits) - 1) & outstanding_bit_cache);
  }
}

//******************************************************************************
//* Constructors
//******************************************************************************

/**
 * The FirmataMarshaller class.
 */
FirmataMarshaller::FirmataMarshaller()
:
  FirmataStream((Stream *)NULL)
{
}

//******************************************************************************
//* Public Methods
//******************************************************************************

/**
 * Reassign the Firmata stream transport.
 * @param s A reference to the Stream transport object. This can be any type of
 * transport that implements the Stream interface. Some examples include Ethernet, WiFi
 * and other UARTs on the board (Serial1, Serial2, etc).
 */
void FirmataMarshaller::begin(Stream &s)
{
  FirmataStream = &s;
}

/**
 * Closes the FirmataMarshaller stream by setting its stream reference to `(Stream *)NULL`
 */
void FirmataMarshaller::end(void)
{
  FirmataStream = (Stream *)NULL;
}

//******************************************************************************
//* Output Stream Handling
//******************************************************************************

/**
 * Query the target's firmware name and version
 */
void FirmataMarshaller::queryFirmwareVersion(void)
const
{
  if ( (Stream *)NULL == FirmataStream ) { return; }
  FirmataStream->write(START_SYSEX);
  FirmataStream->write(REPORT_FIRMWARE);
  FirmataStream->write(END_SYSEX);
}

/**
 * Query the target's Firmata protocol version
 */
void FirmataMarshaller::queryVersion(void)
const
{
  if ( (Stream *)NULL == FirmataStream ) { return; }
  FirmataStream->write(REPORT_VERSION);
}

/**
 * Halt the stream of analog readings from the Firmata host application. The range of pins is
 * limited to [0..15] when using the REPORT_ANALOG. The maximum result of the REPORT_ANALOG is limited to 14 bits
 * (16384). To increase the pin range or value, see the documentation for the EXTENDED_ANALOG
 * message.
 * @param pin The analog pin for which to request the value (limited to pins 0 - 15).
 */
void FirmataMarshaller::reportAnalogDisable(uint8_t pin)
const
{
  reportAnalog(pin, false);
}

/**
 * Request a stream of analog readings from the Firmata host application. The range of pins is
 * limited to [0..15] when using the REPORT_ANALOG. The maximum result of the REPORT_ANALOG is limited to 14 bits
 * (16384). To increase the pin range or value, see the documentation for the EXTENDED_ANALOG
 * message.
 * @param pin The analog pin for which to request the value (limited to pins 0 - 15).
 */
void FirmataMarshaller::reportAnalogEnable(uint8_t pin)
const
{
  reportAnalog(pin, true);
}

/**
 * Halt an 8-bit port stream from the Firmata host application (protocol v2 and later).
 * Send 14-bits in a single digital message (protocol v1).
 * @param portNumber The port number for which to request the value. Note that this is not the same as a "port" on the
 * physical microcontroller. Ports are defined in order per every 8 pins in ascending order
 * of the Arduino digital pin numbering scheme. Port 0 = pins D0 - D7, port 1 = pins D8 - D15, etc.
 */
void FirmataMarshaller::reportDigitalPortDisable(uint8_t portNumber)
const
{
  reportDigitalPort(portNumber, false);
}

/**
 * Request an 8-bit port stream from the Firmata host application (protocol v2 and later).
 * Send 14-bits in a single digital message (protocol v1).
 * @param portNumber The port number for which to request the value. Note that this is not the same as a "port" on the
 * physical microcontroller. Ports are defined in order per every 8 pins in ascending order
 * of the Arduino digital pin numbering scheme. Port 0 = pins D0 - D7, port 1 = pins D8 - D15, etc.
 */
void FirmataMarshaller::reportDigitalPortEnable(uint8_t portNumber)
const
{
  reportDigitalPort(portNumber, true);
}

/**
 * Send an analog message to the Firmata host application. The range of pins is limited to [0..15]
 * when using the ANALOG_MESSAGE. The maximum value of the ANALOG_MESSAGE is limited to 14 bits
 * (16384). To increase the pin range or value, see the documentation for the EXTENDED_ANALOG
 * message.
 * @param pin The analog pin to which the value is sent.
 * @param value The value of the analog pin (0 - 1024 for 10-bit analog, 0 - 4096 for 12-bit, etc).
 * @note The maximum value is 14-bits (16384).
 */
void FirmataMarshaller::sendAnalog(uint8_t pin, uint16_t value)
const
{
  if ( (Stream *)NULL == FirmataStream ) { return; }
  if ( (0xF >= pin) && (0x3FFF >= value) ) {
    FirmataStream->write(ANALOG_MESSAGE|pin);
    encodeByteStream(sizeof(value), reinterpret_cast<uint8_t *>(&value), sizeof(value));
  } else {
    sendExtendedAnalog(pin, sizeof(value), reinterpret_cast<uint8_t *>(&value));
  }
}

/**
 * Send an analog mapping query to the Firmata host application. The resulting sysex message will
 * have an ANALOG_MAPPING_RESPONSE command byte, followed by a list of pins [0-n]; where each
 * pin will specify its corresponding analog pin number or 0x7F (127) if not applicable.
 */
void FirmataMarshaller::sendAnalogMappingQuery(void)
const
{
  sendSysex(ANALOG_MAPPING_QUERY, 0, NULL);
}

/**
 * Send a capability query to the Firmata host application. The resulting sysex message will have
 * a CAPABILITY_RESPONSE command byte, followed by a list of byte tuples (mode and mode resolution)
 * for each pin; where each pin list is terminated by 0x7F (127).
 */
void FirmataMarshaller::sendCapabilityQuery(void)
const
{
  sendSysex(CAPABILITY_QUERY, 0, NULL);
}

/**
 * Send a single digital pin value to the Firmata host application.
 * @param pin The digital pin to send the value of.
 * @param value The value of the pin.
 */
void FirmataMarshaller::sendDigital(uint8_t pin, uint8_t value)
const
{
  if ( (Stream *)NULL == FirmataStream ) { return; }
  FirmataStream->write(SET_DIGITAL_PIN_VALUE);
  FirmataStream->write(pin & 0x7F);
  FirmataStream->write(value != 0);
}


/**
 * Send an 8-bit port in a single digital message (protocol v2 and later).
 * Send 14-bits in a single digital message (protocol v1).
 * @param portNumber The port number to send. Note that this is not the same as a "port" on the
 * physical microcontroller. Ports are defined in order per every 8 pins in ascending order
 * of the Arduino digital pin numbering scheme. Port 0 = pins D0 - D7, port 1 = pins D8 - D15, etc.
 * @param portData The value of the port. The value of each pin in the port is represented by a bit.
 */
void FirmataMarshaller::sendDigitalPort(uint8_t portNumber, uint16_t portData)
const
{
  if ( (Stream *)NULL == FirmataStream ) { return; }
  FirmataStream->write(DIGITAL_MESSAGE | (portNumber & 0xF));
  // Tx bits  0-6 (protocol v1 and higher)
  // Tx bits 7-13 (bit 7 only for protocol v2 and higher)
  encodeByteStream(sizeof(portData), reinterpret_cast<uint8_t *>(&portData), sizeof(portData));
}

/**
 * Sends the firmware name and version to the Firmata host application.
 * @param major The major verison number
 * @param minor The minor version number
 * @param bytec The length of the firmware name
 * @param bytev The firmware name array
 */
void FirmataMarshaller::sendFirmwareVersion(uint8_t major, uint8_t minor, size_t bytec, uint8_t *bytev)
const
{
  if ( (Stream *)NULL == FirmataStream ) { return; }
  size_t i;
  FirmataStream->write(START_SYSEX);
  FirmataStream->write(REPORT_FIRMWARE);
  FirmataStream->write(major);
  FirmataStream->write(minor);
  for (i = 0; i < bytec; ++i) {
    encodeByteStream(sizeof(bytev[i]), reinterpret_cast<uint8_t *>(&bytev[i]));
  }
  FirmataStream->write(END_SYSEX);
}

/**
 * Send the Firmata protocol version to the Firmata host application.
 * @param major The major verison number
 * @param minor The minor version number
 */
void FirmataMarshaller::sendVersion(uint8_t major, uint8_t minor)
const
{
  if ( (Stream *)NULL == FirmataStream ) { return; }
  FirmataStream->write(REPORT_VERSION);
  FirmataStream->write(major);
  FirmataStream->write(minor);
}

/**
 * Send the pin mode/configuration. The pin configuration (or mode) in Firmata represents the
 * current function of the pin. Examples are digital input or output, analog input, pwm, i2c,
 * serial (uart), etc.
 * @param pin The pin to configure.
 * @param config The configuration value for the specified pin.
 */
void FirmataMarshaller::sendPinMode(uint8_t pin, uint8_t config)
const
{
  if ( (Stream *)NULL == FirmataStream ) { return; }
  FirmataStream->write(SET_PIN_MODE);
  FirmataStream->write(pin);
  FirmataStream->write(config);
}

/**
 * Send a pin state query to the Firmata host application. The resulting sysex message will have
 * a PIN_STATE_RESPONSE command byte, followed by the pin number, the pin mode and a stream of
 * bits to indicate any *data* written to the pin (pin state).
 * @param pin The pin to query
 * @note The pin state is any data written to the pin (i.e. pin state != pin value)
 */
void FirmataMarshaller::sendPinStateQuery(uint8_t pin)
const
{
  if ( (Stream *)NULL == FirmataStream ) { return; }
  FirmataStream->write(START_SYSEX);
  FirmataStream->write(PIN_STATE_QUERY);
  FirmataStream->write(pin);
  FirmataStream->write(END_SYSEX);
}

/**
 * Send a sysex message where all values after the command byte are packet as 2 7-bit bytes
 * (this is not always the case so this function is not always used to send sysex messages).
 * @param command The sysex command byte.
 * @param bytec The number of data bytes in the message (excludes start, command and end bytes).
 * @param bytev A pointer to the array of data bytes to send in the message.
 */
void FirmataMarshaller::sendSysex(uint8_t command, size_t bytec, uint8_t *bytev)
const
{
  if ( (Stream *)NULL == FirmataStream ) { return; }
  size_t i;
  FirmataStream->write(START_SYSEX);
  FirmataStream->write(command);
  for (i = 0; i < bytec; ++i) {
    encodeByteStream(sizeof(bytev[i]), reinterpret_cast<uint8_t *>(&bytev[i]));
  }
  FirmataStream->write(END_SYSEX);
}

/**
 * Send a string to the Firmata host application.
 * @param string A pointer to the char string
 */
void FirmataMarshaller::sendString(const char *string)
const
{
  sendSysex(STRING_DATA, strlen(string), reinterpret_cast<uint8_t *>(const_cast<char *>(string)));
}

/**
 * The sampling interval sets how often analog data and i2c data is reported to the client.
 * @param interval_ms The interval (in milliseconds) at which to sample
 * @note The default sampling interval is 19ms
 */
void FirmataMarshaller::setSamplingInterval(uint16_t interval_ms)
const
{
  sendSysex(SAMPLING_INTERVAL, sizeof(interval_ms), reinterpret_cast<uint8_t *>(&interval_ms));
}

/**
 * Perform a software reset on the target. For example, StandardFirmata.ino will initialize
 * everything to a known state and reset the parsing buffer.
 */
void FirmataMarshaller::systemReset(void)
const
{
  if ( (Stream *)NULL == FirmataStream ) { return; }
  FirmataStream->write(SYSTEM_RESET);
}
//=========================================================================================================================================
//=================================================================================================================================================


/*
  FirmataParser.cpp
  Copyright (c) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2009-2016 Jeff Hoefs.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

//******************************************************************************
//* Includes
//******************************************************************************


using namespace firmata;

//******************************************************************************
//* Constructors
//******************************************************************************

/**
 * The FirmataParser class.
 * @param dataBuffer A pointer to an external buffer used to store parsed data
 * @param dataBufferSize The size of the external buffer
 */
FirmataParser::FirmataParser(uint8_t * const dataBuffer, size_t dataBufferSize)
:
  dataBuffer(dataBuffer),
  dataBufferSize(dataBufferSize),
  executeMultiByteCommand(0),
  multiByteChannel(0),
  waitForData(0),
  parsingSysex(false),
  sysexBytesRead(0),
  currentAnalogCallbackContext((void *)NULL),
  currentDigitalCallbackContext((void *)NULL),
  currentReportAnalogCallbackContext((void *)NULL),
  currentReportDigitalCallbackContext((void *)NULL),
  currentPinModeCallbackContext((void *)NULL),
  currentPinValueCallbackContext((void *)NULL),
  currentReportFirmwareCallbackContext((void *)NULL),
  currentReportVersionCallbackContext((void *)NULL),
  currentDataBufferOverflowCallbackContext((void *)NULL),
  currentStringCallbackContext((void *)NULL),
  currentSysexCallbackContext((void *)NULL),
  currentSystemResetCallbackContext((void *)NULL),
  currentAnalogCallback((callbackFunction)NULL),
  currentDigitalCallback((callbackFunction)NULL),
  currentReportAnalogCallback((callbackFunction)NULL),
  currentReportDigitalCallback((callbackFunction)NULL),
  currentPinModeCallback((callbackFunction)NULL),
  currentPinValueCallback((callbackFunction)NULL),
  currentDataBufferOverflowCallback((dataBufferOverflowCallbackFunction)NULL),
  currentStringCallback((stringCallbackFunction)NULL),
  currentSysexCallback((sysexCallbackFunction)NULL),
  currentReportFirmwareCallback((versionCallbackFunction)NULL),
  currentReportVersionCallback((systemCallbackFunction)NULL),
  currentSystemResetCallback((systemCallbackFunction)NULL)
{
    allowBufferUpdate = ((uint8_t *)NULL == dataBuffer);
}

//******************************************************************************
//* Public Methods
//******************************************************************************

//------------------------------------------------------------------------------
// Serial Receive Handling

/**
 * Parse data from the input stream.
 * @param inputData A single byte to be added to the parser.
 */
void FirmataParser::parse(uint8_t inputData)
{
  uint8_t command;

  if (parsingSysex) {
    if (inputData == END_SYSEX) {
      //stop sysex byte
      parsingSysex = false;
      //fire off handler function
      processSysexMessage();
    } else {
      //normal data byte - add to buffer
      bufferDataAtPosition(inputData, sysexBytesRead);
      ++sysexBytesRead;
    }
  } else if ( (waitForData > 0) && (inputData < 128) ) {
    --waitForData;
    bufferDataAtPosition(inputData, waitForData);
    if ( (waitForData == 0) && executeMultiByteCommand ) { // got the whole message
      switch (executeMultiByteCommand) {
        case ANALOG_MESSAGE:
          if (currentAnalogCallback) {
            (*currentAnalogCallback)(currentAnalogCallbackContext,
                                     multiByteChannel,
                                     (dataBuffer[0] << 7)
                                     + dataBuffer[1]);
          }
          break;
        case DIGITAL_MESSAGE:
          if (currentDigitalCallback) {
            (*currentDigitalCallback)(currentDigitalCallbackContext,
                                      multiByteChannel,
                                      (dataBuffer[0] << 7)
                                      + dataBuffer[1]);
          }
          break;
        case SET_PIN_MODE:
          if (currentPinModeCallback)
            (*currentPinModeCallback)(currentPinModeCallbackContext, dataBuffer[1], dataBuffer[0]);
          break;
        case SET_DIGITAL_PIN_VALUE:
          if (currentPinValueCallback)
            (*currentPinValueCallback)(currentPinValueCallbackContext, dataBuffer[1], dataBuffer[0]);
          break;
        case REPORT_ANALOG:
          if (currentReportAnalogCallback)
            (*currentReportAnalogCallback)(currentReportAnalogCallbackContext, multiByteChannel, dataBuffer[0]);
          break;
        case REPORT_DIGITAL:
          if (currentReportDigitalCallback)
            (*currentReportDigitalCallback)(currentReportDigitalCallbackContext, multiByteChannel, dataBuffer[0]);
          break;
      }
      executeMultiByteCommand = 0;
    }
  } else {
    // remove channel info from command byte if less than 0xF0
    if (inputData < 0xF0) {
      command = inputData & 0xF0;
      multiByteChannel = inputData & 0x0F;
    } else {
      command = inputData;
      // commands in the 0xF* range don't use channel data
    }
    switch (command) {
      case ANALOG_MESSAGE:
      case DIGITAL_MESSAGE:
      case SET_PIN_MODE:
      case SET_DIGITAL_PIN_VALUE:
        waitForData = 2; // two data bytes needed
        executeMultiByteCommand = command;
        break;
      case REPORT_ANALOG:
      case REPORT_DIGITAL:
        waitForData = 1; // one data byte needed
        executeMultiByteCommand = command;
        break;
      case START_SYSEX:
        parsingSysex = true;
        sysexBytesRead = 0;
        break;
      case SYSTEM_RESET:
        systemReset();
        break;
      case REPORT_VERSION:
        if (currentReportVersionCallback)
        {
          (*currentReportVersionCallback)(currentReportVersionCallbackContext);
          if (connection_state == 0)
          {
            connection_state = 1;
            Wire.beginTransmission(1);
            #if ARDUINO >= 100
                Wire.write(29);
            #else
                Wire.send(29);
            #endif
            Wire.endTransmission();
            delayMicroseconds(70);
          }
          else
          { 
            if (connection_state == 1)
            {
              connection_state = 3;
              timer2 = millis();
           /*   setMelodytoPlay(5);
              isUserMelody = false;
              isMelodyToPlay = true;
              Wire.beginTransmission(1);
              #if ARDUINO >= 100
              Wire.write(24);
              #else
                Wire.send(24);
              #endif
              Wire.endTransmission();
              delayMicroseconds(70);
              time_of_connection = millis();*/
            }
         
          }        
        }
        break;
    }
  }
}

/**
 * @return Returns true if the parser is actively parsing data.
 */
bool FirmataParser::isParsingMessage(void)
const
{
  return (waitForData > 0 || parsingSysex);
}

/**
 * Provides a mechanism to either set or update the working buffer of the parser.
 * The method will be enabled when no buffer has been provided, or an overflow
 * condition exists.
 * @param dataBuffer A pointer to an external buffer used to store parsed data
 * @param dataBufferSize The size of the external buffer
 */
int FirmataParser::setDataBufferOfSize(uint8_t * dataBuffer, size_t dataBufferSize)
{
    int result;

    if ( !allowBufferUpdate ) {
      result = __LINE__;
    } else if ((uint8_t *)NULL == dataBuffer) {
      result = __LINE__;
    } else {
      this->dataBuffer = dataBuffer;
      this->dataBufferSize = dataBufferSize;
      allowBufferUpdate = false;
      result = 0;
    }

    return result;
}

/**
 * Attach a generic sysex callback function to a command (options are: ANALOG_MESSAGE,
 * DIGITAL_MESSAGE, REPORT_ANALOG, REPORT DIGITAL, SET_PIN_MODE and SET_DIGITAL_PIN_VALUE).
 * @param command The ID of the command to attach a callback function to.
 * @param newFunction A reference to the callback function to attach.
 * @param context An optional context to be provided to the callback function (NULL by default).
 * @note The context parameter is provided so you can pass a parameter, by reference, to
 *       your callback function.
 */
void FirmataParser::attach(uint8_t command, callbackFunction newFunction, void * context)
{
  switch (command) {
    case ANALOG_MESSAGE:
      currentAnalogCallback = newFunction;
      currentAnalogCallbackContext = context;
      break;
    case DIGITAL_MESSAGE:
      currentDigitalCallback = newFunction;
      currentDigitalCallbackContext = context;
      break;
    case REPORT_ANALOG:
      currentReportAnalogCallback = newFunction;
      currentReportAnalogCallbackContext = context;
      break;
    case REPORT_DIGITAL:
      currentReportDigitalCallback = newFunction;
      currentReportDigitalCallbackContext = context;
      break;
    case SET_PIN_MODE:
      currentPinModeCallback = newFunction;
      currentPinModeCallbackContext = context;
      break;
    case SET_DIGITAL_PIN_VALUE:
      currentPinValueCallback = newFunction;
      currentPinValueCallbackContext = context;
      break;
  }
}

/**
 * Attach a version callback function (supported option: REPORT_FIRMWARE).
 * @param command The ID of the command to attach a callback function to.
 * @param newFunction A reference to the callback function to attach.
 * @param context An optional context to be provided to the callback function (NULL by default).
 * @note The context parameter is provided so you can pass a parameter, by reference, to
 *       your callback function.
 */
void FirmataParser::attach(uint8_t command, versionCallbackFunction newFunction, void * context)
{
  switch (command) {
    case REPORT_FIRMWARE:
      currentReportFirmwareCallback = newFunction;
      currentReportFirmwareCallbackContext = context;
      break;
  }
}

/**
 * Attach a system callback function (supported options are: SYSTEM_RESET, REPORT_VERSION).
 * @param command The ID of the command to attach a callback function to.
 * @param newFunction A reference to the callback function to attach.
 * @param context An optional context to be provided to the callback function (NULL by default).
 * @note The context parameter is provided so you can pass a parameter, by reference, to
 *       your callback function.
 */
void FirmataParser::attach(uint8_t command, systemCallbackFunction newFunction, void * context)
{
  switch (command) {
    case REPORT_VERSION:
      currentReportVersionCallback = newFunction;
      currentReportVersionCallbackContext = context;
      break;
    case SYSTEM_RESET:
      currentSystemResetCallback = newFunction;
      currentSystemResetCallbackContext = context;
      break;
  }
}

/**
 * Attach a callback function for the STRING_DATA command.
 * @param command Must be set to STRING_DATA or it will be ignored.
 * @param newFunction A reference to the string callback function to attach.
 * @param context An optional context to be provided to the callback function (NULL by default).
 * @note The context parameter is provided so you can pass a parameter, by reference, to
 *       your callback function.
 */
void FirmataParser::attach(uint8_t command, stringCallbackFunction newFunction, void * context)
{
  switch (command) {
    case STRING_DATA:
      currentStringCallback = newFunction;
      currentStringCallbackContext = context;
      break;
  }
}

/**
 * Attach a generic sysex callback function to sysex command.
 * @param command The ID of the command to attach a callback function to.
 * @param newFunction A reference to the sysex callback function to attach.
 * @param context An optional context to be provided to the callback function (NULL by default).
 * @note The context parameter is provided so you can pass a parameter, by reference, to
 *       your callback function.
 */
void FirmataParser::attach(uint8_t command, sysexCallbackFunction newFunction, void * context)
{
  (void)command;
  currentSysexCallback = newFunction;
  currentSysexCallbackContext = context;
}

/**
 * Attach a buffer overflow callback
 * @param newFunction A reference to the buffer overflow callback function to attach.
 * @param context An optional context to be provided to the callback function (NULL by default).
 * @note The context parameter is provided so you can pass a parameter, by reference, to
 *       your callback function.
 */
void FirmataParser::attach(dataBufferOverflowCallbackFunction newFunction, void * context)
{
  currentDataBufferOverflowCallback = newFunction;
  currentDataBufferOverflowCallbackContext = context;
}

/**
 * Detach a callback function for a specified command (such as SYSTEM_RESET, STRING_DATA,
 * ANALOG_MESSAGE, DIGITAL_MESSAGE, etc).
 * @param command The ID of the command to detatch the callback function from.
 */
void FirmataParser::detach(uint8_t command)
{
  switch (command) {
    case REPORT_FIRMWARE:
      attach(command, (versionCallbackFunction)NULL, NULL);
      break;
    case REPORT_VERSION:
    case SYSTEM_RESET:
      attach(command, (systemCallbackFunction)NULL, NULL);
      break;
    case STRING_DATA:
      attach(command, (stringCallbackFunction)NULL, NULL);
      break;
    case START_SYSEX:
      attach(command, (sysexCallbackFunction)NULL, NULL);
      break;
    default:
      attach(command, (callbackFunction)NULL, NULL);
      break;
  }
}

/**
 * Detach the buffer overflow callback
 * @param <unused> Any pointer of type dataBufferOverflowCallbackFunction.
 */
void FirmataParser::detach(dataBufferOverflowCallbackFunction)
{
  currentDataBufferOverflowCallback = (dataBufferOverflowCallbackFunction)NULL;
  currentDataBufferOverflowCallbackContext = (void *)NULL;
}

//******************************************************************************
//* Private Methods
//******************************************************************************

/**
 * Buffer abstraction to prevent memory corruption
 * @param data The byte to put into the buffer
 * @param pos The position to insert the byte into the buffer
 * @return writeError A boolean to indicate if an error occured
 * @private
 */
bool FirmataParser::bufferDataAtPosition(const uint8_t data, const size_t pos)
{
  bool bufferOverflow = (pos >= dataBufferSize);

  // Notify of overflow condition
  if ( bufferOverflow
  && ((dataBufferOverflowCallbackFunction)NULL != currentDataBufferOverflowCallback) )
  {
    allowBufferUpdate = true;
    currentDataBufferOverflowCallback(currentDataBufferOverflowCallbackContext);
    // Check if overflow was resolved during callback
    bufferOverflow = (pos >= dataBufferSize);
  }

  // Write data to buffer if no overflow condition persist
  if ( !bufferOverflow )
  {
    dataBuffer[pos] = data;
  }

  return bufferOverflow;
}

/**
 * Transform 7-bit firmata message into 8-bit stream
 * @param bytec The encoded data byte length of the message (max: 16383).
 * @param bytev A pointer to the encoded array of data bytes.
 * @return The length of the decoded data.
 * @note The conversion will be done in place on the provided buffer.
 * @private
 */
size_t FirmataParser::decodeByteStream(size_t bytec, uint8_t * bytev) {
  size_t decoded_bytes, i;

  for ( i = 0, decoded_bytes = 0 ; i < bytec ; ++decoded_bytes, ++i ) {
    bytev[decoded_bytes] = bytev[i];
    bytev[decoded_bytes] |= (uint8_t)(bytev[++i] << 7);
  }

  return decoded_bytes;
}

/**
 * Process incoming sysex messages. Handles REPORT_FIRMWARE and STRING_DATA internally.
 * Calls callback function for STRING_DATA and all other sysex messages.
 * @private
 */
void FirmataParser::processSysexMessage(void)
{
  switch (dataBuffer[0]) { //first byte in buffer is command
    case REPORT_FIRMWARE:
      if (currentReportFirmwareCallback) {
        const size_t major_version_offset = 1;
        const size_t minor_version_offset = 2;
        const size_t string_offset = 3;
        // Test for malformed REPORT_FIRMWARE message (used to query firmware prior to Firmata v3.0.0)
        if ( 3 > sysexBytesRead ) {
          (*currentReportFirmwareCallback)(currentReportFirmwareCallbackContext, 0, 0, (const char *)NULL);
        } else {
          const size_t end_of_string = (string_offset + decodeByteStream((sysexBytesRead - string_offset), &dataBuffer[string_offset]));
          bufferDataAtPosition('\0', end_of_string); // NULL terminate the string
          (*currentReportFirmwareCallback)(currentReportFirmwareCallbackContext, (size_t)dataBuffer[major_version_offset], (size_t)dataBuffer[minor_version_offset], (const char *)&dataBuffer[string_offset]);
        }
      }
      break;
    case STRING_DATA:
      if (currentStringCallback) {
        const size_t string_offset = 1;
        const size_t end_of_string = (string_offset + decodeByteStream((sysexBytesRead - string_offset), &dataBuffer[string_offset]));
        bufferDataAtPosition('\0', end_of_string); // NULL terminate the string
        (*currentStringCallback)(currentStringCallbackContext, (const char *)&dataBuffer[string_offset]);
      }
      break;
    default:
      if (currentSysexCallback)
        (*currentSysexCallback)(currentSysexCallbackContext, dataBuffer[0], sysexBytesRead - 1, dataBuffer + 1);
  }
}

/**
 * Resets the system state upon a SYSTEM_RESET message from the host software.
 * @private
 */
void FirmataParser::systemReset(void)
{
  size_t i;

  waitForData = 0; // this flag says the next serial input will be data
  executeMultiByteCommand = 0; // execute this after getting multi-byte data
  multiByteChannel = 0; // channel data for multiByteCommands

  for (i = 0; i < dataBufferSize; ++i) {
    dataBuffer[i] = 0;
  }

  parsingSysex = false;
  sysexBytesRead = 0;

  if (currentSystemResetCallback)
    (*currentSystemResetCallback)(currentSystemResetCallbackContext);
}


//----------------------------------------------------------------------------------------------------------------------------------------------
/*
  Firmata.cpp - Firmata library v2.5.8 - 2018-04-15
  Copyright (c) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2009-2017 Jeff Hoefs.  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

//******************************************************************************
//* Includes
//******************************************************************************

#include "HardwareSerial.h"

#include <string.h>
#include <stdlib.h>

using namespace firmata;

//******************************************************************************
//* Static Members
//******************************************************************************
// make one instance for the user to use
FirmataClass Firmata;

/* callback functions */
callbackFunction FirmataClass::currentAnalogCallback = (callbackFunction)NULL;
callbackFunction FirmataClass::currentDigitalCallback = (callbackFunction)NULL;
callbackFunction FirmataClass::currentPinModeCallback = (callbackFunction)NULL;
callbackFunction FirmataClass::currentPinValueCallback = (callbackFunction)NULL;
callbackFunction FirmataClass::currentReportAnalogCallback = (callbackFunction)NULL;
callbackFunction FirmataClass::currentReportDigitalCallback = (callbackFunction)NULL;
stringCallbackFunction FirmataClass::currentStringCallback = (stringCallbackFunction)NULL;
sysexCallbackFunction FirmataClass::currentSysexCallback = (sysexCallbackFunction)NULL;
systemCallbackFunction FirmataClass::currentSystemResetCallback = (systemCallbackFunction)NULL;

//******************************************************************************
//* Support Functions
//******************************************************************************

/**
 * Split a 16-bit byte into two 7-bit values and write each value.
 * @param value The 16-bit value to be split and written separately.
 */
void FirmataClass::sendValueAsTwo7bitBytes(int value)
{
  marshaller.encodeByteStream(sizeof(value), reinterpret_cast<uint8_t *>(&value), sizeof(value));
}

/**
 * A helper method to write the beginning of a Sysex message transmission.
 */
void FirmataClass::startSysex(void)
{
  FirmataStream->write(START_SYSEX);
}

/**
 * A helper method to write the end of a Sysex message transmission.
 */
void FirmataClass::endSysex(void)
{
  FirmataStream->write(END_SYSEX);
}

//******************************************************************************
//* Constructors
//******************************************************************************

/**
 * The Firmata class.
 * An instance named "Firmata" is created automatically for the user.
 */
FirmataClass::FirmataClass()
:
  parser(FirmataParser(parserBuffer, MAX_DATA_BYTES))
{
  firmwareVersionCount = 0;
  firmwareVersionVector = 0;
  blinkVersionDisabled = false;

  // Establish callback translation to parser callbacks
  parser.attach(ANALOG_MESSAGE, (FirmataParser::callbackFunction)staticAnalogCallback, (void *)NULL);
  parser.attach(DIGITAL_MESSAGE, (FirmataParser::callbackFunction)staticDigitalCallback, (void *)NULL);
  parser.attach(REPORT_ANALOG, (FirmataParser::callbackFunction)staticReportAnalogCallback, (void *)NULL);
  parser.attach(REPORT_DIGITAL, (FirmataParser::callbackFunction)staticReportDigitalCallback, (void *)NULL);
  parser.attach(SET_PIN_MODE, (FirmataParser::callbackFunction)staticPinModeCallback, (void *)NULL);
  parser.attach(SET_DIGITAL_PIN_VALUE, (FirmataParser::callbackFunction)staticPinValueCallback, (void *)NULL);
  parser.attach(STRING_DATA, (FirmataParser::stringCallbackFunction)staticStringCallback, (void *)NULL);
  parser.attach(START_SYSEX, (FirmataParser::sysexCallbackFunction)staticSysexCallback, (void *)NULL);
  parser.attach(REPORT_FIRMWARE, (FirmataParser::versionCallbackFunction)staticReportFirmwareCallback, this);
  parser.attach(REPORT_VERSION, (FirmataParser::systemCallbackFunction)staticReportVersionCallback, this);
  parser.attach(SYSTEM_RESET, (FirmataParser::systemCallbackFunction)staticSystemResetCallback, (void *)NULL);
}

//******************************************************************************
//* Public Methods
//******************************************************************************

/**
 * Initialize the default Serial transport at the default baud of 57600.
 */
void FirmataClass::begin(void)
{
  begin(57600);
}

/**
 * Initialize the default Serial transport and override the default baud.
 * Sends the protocol version to the host application followed by the firmware version and name.
 * blinkVersion is also called. To skip the call to blinkVersion, call Firmata.disableBlinkVersion()
 * before calling Firmata.begin(baud).
 * @param speed The baud to use. 57600 baud is the default value.
 */
void FirmataClass::begin(long speed)
{
  Serial.begin(speed);
  blinkVersion();
  begin(Serial);
}

/**
 * Reassign the Firmata stream transport.
 * @param s A reference to the Stream transport object. This can be any type of
 * transport that implements the Stream interface. Some examples include Ethernet, WiFi
 * and other UARTs on the board (Serial1, Serial2, etc).
 */
void FirmataClass::begin(Stream &s)
{
  FirmataStream = &s;
  marshaller.begin(s);
  // do not call blinkVersion() here because some hardware such as the
  // Ethernet shield use pin 13
  printVersion();         // send the protocol version
  printFirmwareVersion(); // send the firmware name and version
}

/**
 * Send the Firmata protocol version to the Firmata host application.
 */
void FirmataClass::printVersion(void)
{
  marshaller.sendVersion(FIRMATA_PROTOCOL_MAJOR_VERSION, FIRMATA_PROTOCOL_MINOR_VERSION);
}

/**
 * Blink the Firmata protocol version to the onboard LEDs (if the board has an onboard LED).
 * If VERSION_BLINK_PIN is not defined in Boards.h for a particular board, then this method
 * does nothing.
 * The first series of flashes indicates the firmware major version (2 flashes = 2).
 * The second series of flashes indicates the firmware minor version (5 flashes = 5).
 */
void FirmataClass::blinkVersion(void)
{
#if defined(VERSION_BLINK_PIN)
  if (blinkVersionDisabled) return;
  // flash the pin with the protocol version
  pinMode(VERSION_BLINK_PIN, OUTPUT);
  strobeBlinkPin(VERSION_BLINK_PIN, FIRMATA_FIRMWARE_MAJOR_VERSION, 40, 210);
  delay(250);
  strobeBlinkPin(VERSION_BLINK_PIN, FIRMATA_FIRMWARE_MINOR_VERSION, 40, 210);
  delay(125);
#endif
}

/**
 * Provides a means to disable the version blink sequence on the onboard LED, trimming startup
 * time by a couple of seconds.
 * Call this before Firmata.begin(). It only applies when using the default Serial transport.
 */
void FirmataClass::disableBlinkVersion()
{
  blinkVersionDisabled = true;
}

/**
 * Sends the firmware name and version to the Firmata host application. The major and minor version
 * numbers are the first 2 bytes in the message. The following bytes are the characters of the
 * firmware name.
 */
void FirmataClass::printFirmwareVersion(void)
{
  if (firmwareVersionCount) { // make sure that the name has been set before reporting
    marshaller.sendFirmwareVersion(static_cast<uint8_t>(firmwareVersionVector[0]), static_cast<uint8_t>(firmwareVersionVector[1]), (firmwareVersionCount - 2), reinterpret_cast<uint8_t *>(&firmwareVersionVector[2]));
  }
  if (connection_state == 1)
  {
    connection_state = 2;
    state_2_timer = millis();
    Wire.beginTransmission(1);
    #if ARDUINO >= 100
       Wire.write(30);
    #else
       Wire.send(30);
    #endif
    Wire.endTransmission();
    delayMicroseconds(70);
  }
}

/**
 * Sets the name and version of the firmware. This is not the same version as the Firmata protocol
 * (although at times the firmware version and protocol version may be the same number).
 * @param name A pointer to the name char array
 * @param major The major version number
 * @param minor The minor version number
 */
void FirmataClass::setFirmwareNameAndVersion(const char *name, byte major, byte minor)
{
  const char *firmwareName;
  const char *extension;

  // parse out ".cpp" and "applet/" that comes from using __FILE__
  extension = strstr(name, ".cpp");
  firmwareName = strrchr(name, '/');

  if (!firmwareName) {
    // windows
    firmwareName = strrchr(name, '\\');
  }
  if (!firmwareName) {
    // user passed firmware name
    firmwareName = name;
  } else {
    firmwareName ++;
  }

  if (!extension) {
    firmwareVersionCount = strlen(firmwareName) + 2;
  } else {
    firmwareVersionCount = extension - firmwareName + 2;
  }

  // in case anyone calls setFirmwareNameAndVersion more than once
  free(firmwareVersionVector);

  firmwareVersionVector = (byte *) malloc(firmwareVersionCount + 1);
  firmwareVersionVector[firmwareVersionCount] = 0;
  firmwareVersionVector[0] = major;
  firmwareVersionVector[1] = minor;
  strncpy((char *)firmwareVersionVector + 2, firmwareName, firmwareVersionCount - 2);
}

//------------------------------------------------------------------------------
// Serial Receive Handling

/**
 * A wrapper for Stream::available()
 * @return The number of bytes remaining in the input stream buffer.
 */
int FirmataClass::available(void)
{
  return FirmataStream->available();
}

/**
 * Read a single int from the input stream. If the value is not = -1, pass it on to parse(byte)
 */
void FirmataClass::processInput(void)
{
  int inputData = FirmataStream->read(); // this is 'int' to handle -1 when no data
  if (inputData != -1) {
    parser.parse(inputData);
  }
}

/**
 * Parse data from the input stream.
 * @param inputData A single byte to be added to the parser.
 */
void FirmataClass::parse(byte inputData)
{
    parser.parse(inputData);
}

/**
 * @return Returns true if the parser is actively parsing data.
 */
boolean FirmataClass::isParsingMessage(void)
{
  return parser.isParsingMessage();
}

//------------------------------------------------------------------------------
// Output Stream Handling

/**
 * Send an analog message to the Firmata host application. The range of pins is limited to [0..15]
 * when using the ANALOG_MESSAGE. The maximum value of the ANALOG_MESSAGE is limited to 14 bits
 * (16384). To increase the pin range or value, see the documentation for the EXTENDED_ANALOG
 * message.
 * @param pin The analog pin to send the value of (limited to pins 0 - 15).
 * @param value The value of the analog pin (0 - 1024 for 10-bit analog, 0 - 4096 for 12-bit, etc).
 * The maximum value is 14-bits (16384).
 */
void FirmataClass::sendAnalog(byte pin, int value)
{
  marshaller.sendAnalog(pin, value);
}

/* (intentionally left out asterix here)
 * STUB - NOT IMPLEMENTED
 * Send a single digital pin value to the Firmata host application.
 * @param pin The digital pin to send the value of.
 * @param value The value of the pin.
 */
void FirmataClass::sendDigital(byte pin, int value)
{
  (void)pin;
  (void)value;
  /* TODO add single pin digital messages to the protocol, this needs to
   * track the last digital data sent so that it can be sure to change just
   * one bit in the packet.  This is complicated by the fact that the
   * numbering of the pins will probably differ on Arduino, Wiring, and
   * other boards.
   */

  // TODO: the digital message should not be sent on the serial port every
  // time sendDigital() is called.  Instead, it should add it to an int
  // which will be sent on a schedule.  If a pin changes more than once
  // before the digital message is sent on the serial port, it should send a
  // digital message for each change.

  //    if(value == 0)
  //        sendDigitalPortPair();
}


/**
 * Send an 8-bit port in a single digital message (protocol v2 and later).
 * Send 14-bits in a single digital message (protocol v1).
 * @param portNumber The port number to send. Note that this is not the same as a "port" on the
 * physical microcontroller. Ports are defined in order per every 8 pins in ascending order
 * of the Arduino digital pin numbering scheme. Port 0 = pins D0 - D7, port 1 = pins D8 - D15, etc.
 * @param portData The value of the port. The value of each pin in the port is represented by a bit.
 */
void FirmataClass::sendDigitalPort(byte portNumber, int portData)
{
  marshaller.sendDigitalPort(portNumber, portData);
}

/**
 * Send a sysex message where all values after the command byte are packet as 2 7-bit bytes
 * (this is not always the case so this function is not always used to send sysex messages).
 * @param command The sysex command byte.
 * @param bytec The number of data bytes in the message (excludes start, command and end bytes).
 * @param bytev A pointer to the array of data bytes to send in the message.
 */
void FirmataClass::sendSysex(byte command, byte bytec, byte *bytev)
{
  marshaller.sendSysex(command, bytec, bytev);
}

/**
 * Send a string to the Firmata host application.
 * @param command Must be STRING_DATA
 * @param string A pointer to the char string
 */
void FirmataClass::sendString(byte command, const char *string)
{
  if (command == STRING_DATA) {
    marshaller.sendString(string);
  }
}

/**
 * Send a string to the Firmata host application.
 * @param string A pointer to the char string
 */
void FirmataClass::sendString(const char *string)
{
  marshaller.sendString(string);
}

/**
 * A wrapper for Stream::available().
 * Write a single byte to the output stream.
 * @param c The byte to be written.
 */
void FirmataClass::write(byte c)
{
  FirmataStream->write(c);
}

/**
 * Attach a generic sysex callback function to a command (options are: ANALOG_MESSAGE,
 * DIGITAL_MESSAGE, REPORT_ANALOG, REPORT DIGITAL, SET_PIN_MODE and SET_DIGITAL_PIN_VALUE).
 * @param command The ID of the command to attach a callback function to.
 * @param newFunction A reference to the callback function to attach.
 */
void FirmataClass::attach(uint8_t command, ::callbackFunction newFunction)
{
  switch (command) {
    case ANALOG_MESSAGE:
      currentAnalogCallback = newFunction;
      break;
    case DIGITAL_MESSAGE:
      currentDigitalCallback = newFunction;
      break;
    case REPORT_ANALOG:
      currentReportAnalogCallback = newFunction;
      break;
    case REPORT_DIGITAL:
      currentReportDigitalCallback = newFunction;
      break;
    case SET_PIN_MODE:
      currentPinModeCallback = newFunction;
      break;
    case SET_DIGITAL_PIN_VALUE:
      currentPinValueCallback = newFunction;
      break;
  }
}

/**
 * Attach a callback function for the SYSTEM_RESET command.
 * @param command Must be set to SYSTEM_RESET or it will be ignored.
 * @param newFunction A reference to the system reset callback function to attach.
 */
void FirmataClass::attach(uint8_t command, systemCallbackFunction newFunction)
{
  switch (command) {
    case SYSTEM_RESET:
      currentSystemResetCallback = newFunction;
      break;
  }
}

/**
 * Attach a callback function for the STRING_DATA command.
 * @param command Must be set to STRING_DATA or it will be ignored.
 * @param newFunction A reference to the string callback function to attach.
 */
void FirmataClass::attach(uint8_t command, stringCallbackFunction newFunction)
{
  switch (command) {
    case STRING_DATA:
      currentStringCallback = newFunction;
      break;
  }
}

/**
 * Attach a generic sysex callback function to sysex command.
 * @param command The ID of the command to attach a callback function to.
 * @param newFunction A reference to the sysex callback function to attach.
 */
void FirmataClass::attach(uint8_t command, sysexCallbackFunction newFunction)
{
  (void)command;
  currentSysexCallback = newFunction;
}

/**
 * Detach a callback function for a specified command (such as SYSTEM_RESET, STRING_DATA,
 * ANALOG_MESSAGE, DIGITAL_MESSAGE, etc).
 * @param command The ID of the command to detatch the callback function from.
 */
void FirmataClass::detach(uint8_t command)
{
  switch (command) {
    case SYSTEM_RESET:
      attach(command, (systemCallbackFunction)NULL);
      break;
    case STRING_DATA:
      attach(command, (stringCallbackFunction)NULL);
      break;
    case START_SYSEX:
      attach(command, (sysexCallbackFunction)NULL);
      break;
    default:
      attach(command, (callbackFunction)NULL);
      break;
  }
}

/**
 * @param pin The pin to get the configuration of.
 * @return The configuration of the specified pin.
 */
byte FirmataClass::getPinMode(byte pin)
{
  return pinConfig[pin];
}

/**
 * Set the pin mode/configuration. The pin configuration (or mode) in Firmata represents the
 * current function of the pin. Examples are digital input or output, analog input, pwm, i2c,
 * serial (uart), etc.
 * @param pin The pin to configure.
 * @param config The configuration value for the specified pin.
 */
void FirmataClass::setPinMode(byte pin, byte config)
{
  if (pinConfig[pin] == PIN_MODE_IGNORE)
    return;

  pinConfig[pin] = config;
}

/**
 * @param pin The pin to get the state of.
 * @return The state of the specified pin.
 */
int FirmataClass::getPinState(byte pin)
{
  return pinState[pin];
}

/**
 * Set the pin state. The pin state of an output pin is the pin value. The state of an
 * input pin is 0, unless the pin has it's internal pull up resistor enabled, then the value is 1.
 * @param pin The pin to set the state of
 * @param state Set the state of the specified pin
 */
void FirmataClass::setPinState(byte pin, int state)
{
  pinState[pin] = state;
}

// sysex callbacks
/*
 * this is too complicated for analogReceive, but maybe for Sysex?
 void FirmataClass::attachSysex(sysexFunction newFunction)
 {
 byte i;
 byte tmpCount = analogReceiveFunctionCount;
 analogReceiveFunction* tmpArray = analogReceiveFunctionArray;
 analogReceiveFunctionCount++;
 analogReceiveFunctionArray = (analogReceiveFunction*) calloc(analogReceiveFunctionCount, sizeof(analogReceiveFunction));
 for(i = 0; i < tmpCount; i++) {
 analogReceiveFunctionArray[i] = tmpArray[i];
 }
 analogReceiveFunctionArray[tmpCount] = newFunction;
 free(tmpArray);
 }
*/

//******************************************************************************
//* Private Methods
//******************************************************************************

/**
 * Flashing the pin for the version number
 * @private
 * @param pin The pin the LED is attached to.
 * @param count The number of times to flash the LED.
 * @param onInterval The number of milliseconds for the LED to be ON during each interval.
 * @param offInterval The number of milliseconds for the LED to be OFF during each interval.
 */
void FirmataClass::strobeBlinkPin(byte pin, int count, int onInterval, int offInterval)
{
  byte i;
  for (i = 0; i < count; i++) {
    delay(offInterval);
    digitalWrite(pin, HIGH);
    delay(onInterval);
    digitalWrite(pin, LOW);
  }
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------

// move the following defines to Firmata.h?
#define I2C_WRITE B00000000
#define I2C_READ B00001000
#define I2C_READ_CONTINUOUSLY B00010000
#define I2C_STOP_READING B00011000
#define I2C_READ_WRITE_MODE_MASK B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000

#define I2C_END_TX_MASK             B01000000
#define I2C_STOP_TX                 1
#define I2C_RESTART_TX              0
#define MAX_QUERIES 8
#define MINIMUM_SAMPLING_INTERVAL 1

#define REGISTER_NOT_SPECIFIED -1
#define MELODY_TO_PLAY_BUFFER_LEN 30

/*==============================================================================
   GLOBAL VARIABLES
  ============================================================================*/


/* Odysseas additions  */

byte motorAspeed = 0;
byte motorAdirection = -1;
unsigned long motorAmillis;
byte set_A_direction, motorAfinished = 1;
byte motorArefinementdirection, motorArefinementvalue;
byte motorBspeed = 0;
byte motorBdirection = -1;
unsigned long motorBmillis;
byte set_B_direction, motorBfinished = 1;
byte motorBrefinementdirection, motorBrefinementvalue, type_of_move;

long start_of_set_step;
long time_of_step;
byte number_of_steps;
long start_of_step;
boolean in_step = false;
long time_reformed_step;
boolean in_set_forward_step= false, in_set_backward_step= false, in_set_left_turn_step= false, in_set_right_turn_step= false;
long start_of_set_turn;
long time_of_turn;
byte speed_of_turn;
byte number_of_turns;
boolean in_set_turn;
long start_of_turn;
boolean in_turn = false;
long time_reformed_turn;
byte left_led_status = 15, right_led_status=19;

byte grid_number_under_set;
uint16_t r, g, b, c, colorTemp, lux;
boolean eeprom_read = false;
/* end of Odysseas additions */

#ifdef FIRMATA_SERIAL_FEATURE
SerialFirmata serialFeature;
#endif
/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte pinConfig[TOTAL_PINS];         // configuration of every pin
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else
int pinState[TOTAL_PINS];           // any value that has been written

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
/* my addition  */
unsigned long mycurrentMillis;

/* and of my addition  */
unsigned long previousMillis;       // for comparison with currentMillis
int samplingInterval = 19;          // how often to run the main loop (in ms)

/* i2c data */
struct i2c_device_info {
  byte addr;
  byte reg;
  byte bytes;
};
/* for i2c read continuous more */
i2c_device_info query[MAX_QUERIES];
byte i2cRxData[32];
boolean isI2CEnabled = false;
signed char queryIndex = -1;
unsigned int i2cReadDelayTime = 0;  // default delay time between i2c read request and Wire.requestFrom()
Servo servos[MAX_SERVOS];



/*==============================================================================
   FUNCTIONS
  ============================================================================*/

void readAndReportData(byte address, int theRegister, byte numBytes) {
  // allow I2C requests that don't require a register read
  // for example, some devices using an interrupt pin to signify new data available
  // do not always require the register read so upon interrupt you call Wire.requestFrom()
  if (theRegister != REGISTER_NOT_SPECIFIED) {
    Wire.beginTransmission(address);
#if ARDUINO >= 100
    Wire.write((byte)theRegister);
#else
    Wire.send((byte)theRegister);
#endif
    Wire.endTransmission();
    // do not set a value of 0
    if (i2cReadDelayTime > 0) {
      // delay is necessary for some devices such as WiiNunchuck
      delayMicroseconds(i2cReadDelayTime);
    }
  } else {
    theRegister = 0;  // fill the register with a dummy value
  }

  Wire.requestFrom(address, numBytes);  // all bytes are returned in requestFrom

  // check to be sure correct number of bytes were returned by slave
  if (numBytes == Wire.available()) {
    i2cRxData[0] = address;
    i2cRxData[1] = theRegister;
    for (int i = 0; i < numBytes; i++) {
#if ARDUINO >= 100
      i2cRxData[2 + i] = Wire.read();
#else
      i2cRxData[2 + i] = Wire.receive();
#endif
    }
  }
  else {
    if (numBytes > Wire.available()) {
      Firmata.sendString("I2C Read Error: Too many bytes received");
    } else {
      Firmata.sendString("I2C Read Error: Too few bytes received");
    }
  }

  // send slave address, register and received bytes
  Firmata.sendSysex(SYSEX_I2C_REPLY, numBytes + 2, i2cRxData);
}

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent
  if (forceSend || previousPINs[portNumber] != portValue) {
    Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
  }
}

/* -----------------------------------------------------------------------------
   check all the active digital inputs for change of state, then add any events
   to the Serial output queue using Serial.print() */
void checkDigitalInputs(void)
{
  /* Using non-looping code allows constants to be given to readPort().
     The compiler will apply substantial optimizations if the inputs
     to readPort() are compile-time constants. */
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), false);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), false);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), false);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), false);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), false);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), false);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), false);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), false);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), false);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), false);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), false);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), false);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), false);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), false);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), false);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), false);
}

// -----------------------------------------------------------------------------
/* sets the pin mode to the correct state and sets the relevant bits in the
   two bit-arrays that track Digital I/O and PWM status
*/
void setPinModeCallback(byte pin, int mode)
{
  if (pinConfig[pin] == I2C && isI2CEnabled && mode != I2C) {
    // disable i2c so pins can be used for other functions
    // the following if statements should reconfigure the pins properly
    disableI2CPins();
  }
  if (IS_PIN_SERVO(pin) && mode != SERVO && servos[PIN_TO_SERVO(pin)].attached()) {
    servos[PIN_TO_SERVO(pin)].detach();
  }
  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == ANALOG ? 1 : 0); // turn on/off reporting
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT) {
      portConfigInputs[pin / 8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
    }
  }
  pinState[pin] = 0;
  switch (mode) {
    case ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
          digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
        }
        pinConfig[pin] = ANALOG;
      }
      break;
    case INPUT:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
        pinConfig[pin] = INPUT;
      }
      break;
    case OUTPUT:
      if (IS_PIN_DIGITAL(pin)) {
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable PWM
        pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
        pinConfig[pin] = OUTPUT;
      }
      break;
    case PWM:
      if (IS_PIN_PWM(pin)) {
        pinMode(PIN_TO_PWM(pin), OUTPUT);
        analogWrite(PIN_TO_PWM(pin), 0);
        pinConfig[pin] = PWM;
      }
      break;
    case SERVO:
      if (IS_PIN_SERVO(pin)) {
        pinConfig[pin] = SERVO;
        if (!servos[PIN_TO_SERVO(pin)].attached()) {
          servos[PIN_TO_SERVO(pin)].attach(PIN_TO_DIGITAL(pin));
        }
      }
      break;
    case I2C:
      if (IS_PIN_I2C(pin)) {
        // mark the pin as i2c
        // the user must call I2C_CONFIG to enable I2C for a device
        pinConfig[pin] = I2C;
      }
      break;
    default:
      Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
  }
  // TODO: save status to EEPROM here, if changed
}

void analogWriteCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS) {
    switch (pinConfig[pin]) {
      case SERVO:
        if (IS_PIN_SERVO(pin))
          servos[PIN_TO_SERVO(pin)].write(value);
        pinState[pin] = value;
        break;
      case PWM:
        if (IS_PIN_PWM(pin))
          analogWrite(PIN_TO_PWM(pin), value);
        pinState[pin] = value;
        break;
    }
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, mask = 1, pinWriteMask = 0;

  if (port < TOTAL_PORTS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port * 8 + 8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin = port * 8; pin < lastPin; pin++) {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin)) {
        // only write to OUTPUT and INPUT (enables pullup)
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (pinConfig[pin] == OUTPUT || pinConfig[pin] == INPUT) {
          pinWriteMask |= mask;
          pinState[pin] = ((byte)value & mask) ? 1 : 0;
        }
      }
      mask = mask << 1;
    }
    writePort(port, (byte)value, pinWriteMask);
  }
}


// -----------------------------------------------------------------------------
/* sets bits in a bit array (int) to toggle the reporting of the analogIns
*/
//void FirmataClass::setAnalogPinReporting(byte pin, byte state) {
//}
void reportAnalogCallback(byte analogPin, int value)
{
  if (analogPin < TOTAL_ANALOG_PINS) {
    if (value == 0) {
      analogInputsToReport = analogInputsToReport & ~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
    }
  }
  // TODO: save status to EEPROM here, if changed
}

void reportDigitalCallback(byte port, int value)
{
  if (port < TOTAL_PORTS) {
    reportPINs[port] = (byte)value;
  }
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}

/*==============================================================================
   SYSEX-BASED commands
  ============================================================================*/

void sysexCallback(byte command, byte argc, byte *argv)
{
  byte mode;
  byte slaveAddress;
  byte slaveRegister;
  byte data;
  unsigned int delayTime;

  switch (command) {
    case I2C_REQUEST:
      mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
      if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK) {
        Firmata.sendString("10-bit addressing mode is not yet supported");
        return;
      }
      else {
        slaveAddress = argv[0];
      }

      switch (mode) {
        case I2C_WRITE:
          Wire.beginTransmission(slaveAddress);
          for (byte i = 2; i < argc; i += 2) {
            data = argv[i] + (argv[i + 1] << 7);
#if ARDUINO >= 100
            Wire.write(data);
#else
            Wire.send(data);
#endif
          }
          Wire.endTransmission();
          delayMicroseconds(70);
          break;
        case I2C_READ:
          if (argc == 6) {
            // a slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7);  // bytes to read
            readAndReportData(slaveAddress, (int)slaveRegister, data);
          }
          else {
            // a slave register is NOT specified
            data = argv[2] + (argv[3] << 7);  // bytes to read
            readAndReportData(slaveAddress, (int)REGISTER_NOT_SPECIFIED, data);
          }
          break;
        case I2C_READ_CONTINUOUSLY:
          if ((queryIndex + 1) >= MAX_QUERIES) {
            // too many queries, just ignore
            Firmata.sendString("too many queries");
            break;
          }
          queryIndex++;
          query[queryIndex].addr = slaveAddress;
          query[queryIndex].reg = argv[2] + (argv[3] << 7);
          query[queryIndex].bytes = argv[4] + (argv[5] << 7);
          break;
        case I2C_STOP_READING:
          byte queryIndexToSkip;
          // if read continuous mode is enabled for only 1 i2c device, disable
          // read continuous reporting for that device
          if (queryIndex <= 0) {
            queryIndex = -1;
          } else {
            // if read continuous mode is enabled for multiple devices,
            // determine which device to stop reading and remove it's data from
            // the array, shifiting other array data to fill the space
            for (byte i = 0; i < queryIndex + 1; i++) {
              if (query[i].addr = slaveAddress) {
                queryIndexToSkip = i;
                break;
              }
            }

            for (byte i = queryIndexToSkip; i < queryIndex + 1; i++) {
              if (i < MAX_QUERIES) {
                query[i].addr = query[i + 1].addr;
                query[i].reg = query[i + 1].addr;
                query[i].bytes = query[i + 1].bytes;
              }
            }
            queryIndex--;
          }
          break;
        default:
          break;
      }
      break;
    case I2C_CONFIG:
      delayTime = (argv[0] + (argv[1] << 7));

      if (delayTime > 0) {
        i2cReadDelayTime = delayTime;
      }

      if (!isI2CEnabled) {
        enableI2CPins();
      }

      break;
    case SERVO_CONFIG:
      if (argc > 4) {
        // these vars are here for clarity, they'll optimized away by the compiler
        byte pin = argv[0];
        int minPulse = argv[1] + (argv[2] << 7);
        int maxPulse = argv[3] + (argv[4] << 7);

        if (IS_PIN_SERVO(pin)) {
          if (servos[PIN_TO_SERVO(pin)].attached())
            servos[PIN_TO_SERVO(pin)].detach();
          servos[PIN_TO_SERVO(pin)].attach(PIN_TO_DIGITAL(pin), minPulse, maxPulse);
          setPinModeCallback(pin, SERVO);
        }
      }
      break;
    case SAMPLING_INTERVAL:
      if (argc > 1) {
        samplingInterval = argv[0] + (argv[1] << 7);
        if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
          samplingInterval = MINIMUM_SAMPLING_INTERVAL;
        }
      } else {
        //Firmata.sendString("Not enough data");
      }
      break;
    case EXTENDED_ANALOG:
      if (argc > 1) {
        int val = argv[1];
        if (argc > 2) val |= (argv[2] << 7);
        if (argc > 3) val |= (argv[3] << 14);
        analogWriteCallback(argv[0], val);
      }
      break;
    case CAPABILITY_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(CAPABILITY_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        if (IS_PIN_DIGITAL(pin)) {
          Firmata.write((byte)INPUT);
          Firmata.write(1);
          Firmata.write((byte)OUTPUT);
          Firmata.write(1);
        }
        if (IS_PIN_ANALOG(pin)) {
          Firmata.write(ANALOG);
          Firmata.write(10);
        }
        if (IS_PIN_PWM(pin)) {
          Firmata.write(PWM);
          Firmata.write(8);
        }
        if (IS_PIN_SERVO(pin)) {
          Firmata.write(SERVO);
          Firmata.write(14);
        }
        if (IS_PIN_I2C(pin)) {
          Firmata.write(I2C);
          Firmata.write(1);  // to do: determine appropriate value
        }
        Firmata.write(127);
      }
      Firmata.write(END_SYSEX);
      break;
    case PIN_STATE_QUERY:
      if (argc > 0) {
        byte pin = argv[0];
        Firmata.write(START_SYSEX);
        Firmata.write(PIN_STATE_RESPONSE);
        Firmata.write(pin);
        if (pin < TOTAL_PINS) {
          Firmata.write((byte)pinConfig[pin]);
          Firmata.write((byte)pinState[pin] & 0x7F);
          if (pinState[pin] & 0xFF80) Firmata.write((byte)(pinState[pin] >> 7) & 0x7F);
          if (pinState[pin] & 0xC000) Firmata.write((byte)(pinState[pin] >> 14) & 0x7F);
        }
        Firmata.write(END_SYSEX);
      }
      break;
    case ANALOG_MAPPING_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(ANALOG_MAPPING_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        Firmata.write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
      }
      Firmata.write(END_SYSEX);
      break;

    //************* Custom Commands ************//
    //******************************************//
    //microsecondsPulseOut
    //
    case 0x08:  //pulseOut command
      {
        unsigned int time1 = (unsigned int)argv[0] << 4 | (unsigned int)argv[1] >> 3;
        unsigned int time2 = (unsigned int)argv[1] << 8 | (unsigned int)argv[2] << 1 | (unsigned int)argv[3] >> 6;
        unsigned int time3 = (unsigned int)argv[3] << 5 | (unsigned int)argv[4] >> 2;
        byte value = ((byte)argv[4] & B010) >> 1;
        byte pin = ((byte)argv[4] & B01) << 7 | (byte)argv[5] & B01111111;
        pinMode(pin, OUTPUT);
        int state1;
        int state2;
        if (value == 1) {
          state1 = LOW;
          state2 = HIGH;
        } else {
          state1 = HIGH;
          state2 = LOW;
        }
        digitalWrite(pin, state1);
        delayMicroseconds(time1);
        digitalWrite(pin, state2);
        delayMicroseconds(time2);
        digitalWrite(pin, state1);
        delayMicroseconds(time3);
      }
      break;
    //
    case 0x09: //noTone function
      noTone(argv[0]);
      break;

    //PulseIn

    //
    case 0x0A:  //pulseIn
      {
        unsigned long timeout = (unsigned long)argv[0] << 25 | (unsigned long)argv[1] << 18 | (unsigned long)argv[2] << 11 | (unsigned long)argv[3] << 4 | (unsigned long)argv[4] >> 3;
        byte value = ((byte)argv[4] & B0100) >> 2;
        byte pin = ((byte)argv[4] & B011) << 6 | ((byte)argv[5] & B0111111);
        pinMode(pin, INPUT);
        unsigned long pulse;
        if (timeout == 0) {
          pulse = pulseIn(pin, value);
        } else {
          pulse = pulseIn(pin, value, timeout);
        }
        Serial.write(START_SYSEX);
        Serial.write(0xC8);
        Serial.write((pulse >> 25) & B01111111); // MSB
        Serial.write((pulse >> 18) & B01111111);
        Serial.write((pulse >> 11) & B01111111);
        Serial.write((pulse >> 4) & B01111111);
        Serial.write((pulse << 3) & B01111000 | (pin >> 5) & B0111); //LSB + pinMSB
        Serial.write(pin & B011111);
        Serial.write(END_SYSEX);
      }
      break;
    //

    case 0x0B:
      { //tone and noTone Commands
        unsigned long dur = (unsigned long)argv[0] << 25 | (unsigned long)argv[1] << 18 | (unsigned long)argv[2] << 11 | (unsigned long)argv[3] << 4 | (unsigned long)argv[4] >> 3;
        unsigned int freq = ((unsigned int)argv[4] & B0111) << 13 | (unsigned int)argv[5] << 6 | (unsigned int)argv[6] >> 1;
        //byte pin = ((byte)argv[6] & B01) << 7 | (byte)argv[7];
        //Code using Tone Commands - Not compatible with IRremote lib
        if (freq <32){
          noTone(SPEAKER);
        }else if (dur == 0){
          tone(SPEAKER,freq);
        }else {
          tone(SPEAKER,freq,dur);
        }
      break;
      }

    case 0x0C:  //DC motors command
      {
        byte refinement_value, motor_number, motor_speed, motor_direction, refinement_direction;
        motor_number = (byte)argv[0];
        motor_direction =  (byte)argv[1];
        motor_speed = (byte)argv[2];
        refinement_direction= (byte)argv[3];
        refinement_value = (byte)argv[4];
        if (motor_number == 1)
        {
          motorAdirection = motor_direction;
          motorAspeed = motor_speed;
          motorAmillis = millis();
          set_A_direction = 0;
          motorAfinished = 0;
          motorArefinementdirection = refinement_direction;
          motorArefinementvalue = refinement_value;
        }
        else
        {
          motorBdirection = motor_direction;
          motorBspeed = motor_speed;
          motorBmillis = millis();
          set_B_direction = 0;
          motorBfinished = 0;
          motorBrefinementdirection = refinement_direction;
          motorBrefinementvalue = refinement_value;
        }
        break;
      }
      case 0x0D:
    {

      // Locate the melody in the melody table 

      uint16_t melody = argv[0];
     
      setMelodytoPlay(melody);
      isUserMelody = false;
      isMelodyToPlay = true;
    }
    break;

     case 0x0E: //fix step on a grid using the servo configuration
    {

          // mode on grid and fix movements on grid 
          byte grid_instruction = argv[0];
          int left_servo_speed, right_servo_speed;
          byte left_servo_number, right_servo_number, offset;
      
     
                                   // start of fixing "step forward" or "step backward using the already stored servo configuration. 
                                   // Instruct Odysseas to make X steps. 
                                   // When odysseas reaches the end og the X-step jurney, the user must send an
                                   // end of fixing "step foreward" command. Then the the time
                                   // needed for prforming one step, is stored in EEPROM 
         if (grid_instruction != 2)
         {
          left_servo_number = EEPROM.read(grid_instruction);
          pinConfig[left_servo_number] = SERVO;
          if (!servos[PIN_TO_SERVO(left_servo_number)].attached()) 
          {
              servos[PIN_TO_SERVO(left_servo_number)].attach(PIN_TO_DIGITAL(left_servo_number));
          }
          right_servo_number = EEPROM.read(grid_instruction+5);
          pinConfig[right_servo_number] = SERVO;
          if (!servos[PIN_TO_SERVO(right_servo_number)].attached()) 
          {
            servos[PIN_TO_SERVO(right_servo_number)].attach(PIN_TO_DIGITAL(right_servo_number));
          }
          left_servo_speed = get_servo_speed(grid_instruction-1);
          right_servo_speed = get_servo_speed(grid_instruction+4);
          if ((grid_instruction == 1)||(grid_instruction == 11)||(grid_instruction == 31)||(grid_instruction == 41))
               number_of_steps = argv[1];
          //---------------------
          if (grid_instruction == 1)
              in_set_forward_step = true;
          else if (grid_instruction == 11)
              in_set_backward_step = true;
          else if (grid_instruction == 31)
              in_set_left_turn_step = true;
          else
              in_set_right_turn_step = true;    
           
          start_of_set_step = millis();
          servos[PIN_TO_SERVO(left_servo_number)].write(left_servo_speed);
          pinState[left_servo_number] = left_servo_speed;
          servos[PIN_TO_SERVO(right_servo_number)].write(right_servo_speed);
          pinState[right_servo_number] = right_servo_speed;
         }
        if (grid_instruction == 2)   // end of fixing "step". 
        {     
         
          time_of_step = (millis()-start_of_set_step)/number_of_steps;
          if  (in_set_forward_step == true)
          {
            left_servo_number = EEPROM.read(1);
            right_servo_number = EEPROM.read(6);
            offset = 1;
            in_set_forward_step = false;
          }
          else if (in_set_backward_step == true)
          {
            left_servo_number = EEPROM.read(11);
            right_servo_number = EEPROM.read(16);
            offset = 6;
            in_set_backward_step = false;
          }
          else if (in_set_left_turn_step == true)
          {
            left_servo_number = EEPROM.read(31);
            right_servo_number = EEPROM.read(36);
            offset = 31;
            in_set_left_turn_step = false;
            time_of_step = time_of_step/4;
          }
          else
          {
            left_servo_number = EEPROM.read(41);
            right_servo_number = EEPROM.read(46);
            offset = 36;
            in_set_right_turn_step = false;
            time_of_step = time_of_step/4;
          }
          servos[PIN_TO_SERVO(left_servo_number)].write(1500);
          pinState[left_servo_number] = 1500;
          servos[PIN_TO_SERVO(right_servo_number)].write(1500);
          pinState[right_servo_number] = 1500;
          EEPROM.write(20+offset,time_of_step & B01111111);
          EEPROM.write(21+offset,(time_of_step >> 7) & B01111111);
          EEPROM.write(22+offset,(time_of_step >> 14) & B01111111);
          EEPROM.write(23+offset,(time_of_step >> 21) & B01111111);
          EEPROM.write(24+offset,(time_of_step >> 28) & B00001111);
      }
    }
    break;

    case 0x0F: //move servo
    {
      byte servo_number;
      byte move_direction;
      int servo_speed;
      byte refinement;
      byte value;
      servo_number = argv[0];
      move_direction = argv[1];
      servo_speed = argv[2];
      refinement = argv[3];
      value = argv[4];
      if (move_direction==1)
      {
        if (refinement==1)
          servo_speed = 1500+50*servo_speed+value;
        else if (refinement==2)
          servo_speed = 1500+50*servo_speed-value;
        else
          servo_speed = 1500+50*servo_speed;
      }
      else
      {
        if (refinement==1)
          servo_speed = 1500-50*servo_speed-value;
        else if (refinement==2)
          servo_speed = 1500-50*servo_speed+value;
        else
          servo_speed = 1500-50*servo_speed;
      }
      pinConfig[servo_number] = SERVO;
      if (!servos[PIN_TO_SERVO(servo_number)].attached()) 
      {
         servos[PIN_TO_SERVO(servo_number)].attach(PIN_TO_DIGITAL(servo_number));
      }
      servos[PIN_TO_SERVO(servo_number)].write(servo_speed);              
   }
   break;
/*
     case 0x1A: //choose active grid
    {
        int new_grid;
        int current_grid;
        new_grid = argv[0];
        EEPROM.write(0, new_grid);         
   }
   break;
    */
     case 0x1B: //do a step forward or backward, or a 90 degrees turn, left or right
    {
       byte  speed_of_step_retrieved;
       byte time_of_step_byte1, time_of_step_byte2, time_of_step_byte3, time_of_step_byte4, time_of_step_byte5, left_servo_number, right_servo_number;
       byte grid_instruction, offset;
       int left_servo_speed, right_servo_speed;
       if (in_step == false)
       {
          in_step = true;
          grid_instruction = argv[0];
          type_of_move = grid_instruction;
          left_servo_number = EEPROM.read(grid_instruction);
          pinConfig[left_servo_number] = SERVO;
          if (!servos[PIN_TO_SERVO(left_servo_number)].attached()) 
          {
              servos[PIN_TO_SERVO(left_servo_number)].attach(PIN_TO_DIGITAL(left_servo_number));
          }
          right_servo_number = EEPROM.read(grid_instruction+5);
          pinConfig[right_servo_number] = SERVO;
          if (!servos[PIN_TO_SERVO(right_servo_number)].attached()) 
          {
            servos[PIN_TO_SERVO(right_servo_number)].attach(PIN_TO_DIGITAL(right_servo_number));
          }
          left_servo_speed = get_servo_speed(grid_instruction-1);
          right_servo_speed = get_servo_speed(grid_instruction+4);
          if (grid_instruction == 1)
            offset = 1;
          else if (grid_instruction == 11)
            offset = 6;
          else if (grid_instruction ==31)
            offset = 31;
          else
            offset = 36;
          time_of_step_byte1 = EEPROM.read(20+offset);
          time_of_step_byte2 = EEPROM.read(21+offset);
          time_of_step_byte3 = EEPROM.read(22+offset);
          time_of_step_byte4 = EEPROM.read(23+offset);
          time_of_step_byte5 = EEPROM.read(24+offset);
          servos[PIN_TO_SERVO(left_servo_number)].write(left_servo_speed);
          pinState[left_servo_number] = left_servo_speed;
          servos[PIN_TO_SERVO(right_servo_number)].write(right_servo_speed);
          pinState[right_servo_number] = right_servo_speed; 
          start_of_step = millis();
          Serial.write(START_SYSEX);
          Serial.write(0x1B);
          Serial.write(time_of_step_byte1); 
          Serial.write(time_of_step_byte2);
          Serial.write(time_of_step_byte3);
          Serial.write(time_of_step_byte4);
          Serial.write(time_of_step_byte5);
          Serial.write(END_SYSEX);
       }
       else
       { 
          time_reformed_step = time_of_step_byte1 | ( time_of_step_byte2 << 7) | (time_of_step_byte3 << 14)  | ( time_of_step_byte4 << 21) | (time_of_step_byte5 << 28);
          time_reformed_step = time_reformed_step - start_of_step;
          Serial.write(START_SYSEX);
          Serial.write(0x1B);
          Serial.write(time_reformed_step & B01111111); 
          Serial.write((time_reformed_step >> 7) & B01111111);
          Serial.write((time_reformed_step >> 14) & B01111111);
          Serial.write((time_reformed_step >> 21) & B01111111);
          Serial.write((time_reformed_step >> 28) & B01111111);
          Serial.write(END_SYSEX);
       }
       
    }
      break;
/*    case 0x1C: //read and return current grid
    {
        int current_grid;
        current_grid = EEPROM.read(2);
        Serial.write(START_SYSEX);
        Serial.write(0x1C);
        Serial.write(current_grid);
        Serial.write(END_SYSEX);             
   }
   break;*/
   
    case 0x1E: // fix a given color
    {
       int colour;
       uint16_t r1, g1, b1, c1;
       byte byte_r1, byte_r2, byte_g1, byte_g2, byte_b1, byte_b2;
       colour = argv[0];
       tcs.getRawData(&r1, &g1, &b1, &c1);
          // colorTemp = tcs.calculateColorTemperature(r, g, b);
         // colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
        //  lux = tcs.calculateLux(r, g, b);

       byte_r1 = r1 % 256;  
       EEPROM[60+(colour-1)*6+1] = byte_r1;
       byte_r2 = r1 / 256;
       EEPROM[60+(colour-1)*6+2] = byte_r2;
       byte_g1 = g1 % 256;
       EEPROM[60+(colour-1)*6+3] =  byte_g1;
       byte_g2 = g1 / 256;
       EEPROM[60+(colour-1)*6+4] = byte_g2;
       byte_b1 = b1 % 256;
       EEPROM[60+(colour-1)*6+5] = byte_b1;
       byte_b2 = b1 / 256;
       EEPROM[60+(colour-1)*6+6] = byte_b2;
    }
    
    break;


    case 0x1F: // set active/inactive colours
    {
      EEPROM.write(116, argv[0]);
      EEPROM.write(117, argv[1]);
      setMelodytoPlay(7);
      isUserMelody = false;
      isMelodyToPlay = true;
      
    }
     break;

    case 0xCA: //ultrasonic sensor v1
      {
        byte pinSen = (byte)argv[0] << 1 | (byte)argv[1] >> 6;
        byte pinRec = (byte)argv[2] << 1 | (byte)argv[3] >> 6;
        byte time1 = (byte)argv[1] & B011111;
        byte time2 = (byte)argv[3] & B011111;
        pinMode(pinSen, OUTPUT);
        digitalWrite(pinSen, LOW);
        delayMicroseconds(time1);
        digitalWrite(pinSen, HIGH);
        delayMicroseconds(time2);
        digitalWrite(pinSen, LOW);
        pinMode(pinRec, INPUT);
        unsigned int pulse = pulseIn(pinRec, HIGH, 65535); //timeout is maximum pulse value
        Serial.write(START_SYSEX);
        Serial.write(0xCA);
        Serial.write((pulse >> 9) & B01111111); // MSB pulse
        Serial.write((pulse >> 2) & B01111111);
        Serial.write((pulse << 5) & B01100000 | (pinRec >> 3) & B011111); //LSB pulse and MSB pinRec
        Serial.write(pinRec & B0111); // LSB pinRec
        Serial.write(END_SYSEX);
      }
      break;

       case 0xCB: //ultrasonic sensor v2 (simpler, without parameters on the client's side)
      {
        unsigned int us_distance;
        pinMode(5, OUTPUT);
        digitalWrite(5, LOW);
        delayMicroseconds(2);
        digitalWrite(5, HIGH);
        delayMicroseconds(10);
        digitalWrite(5, LOW);
        pinMode(11, INPUT);
        unsigned int pulse = pulseIn(11, HIGH, 65535); //timeout is maximum pulse value
        us_distance =  pulse * 0.034 / 2;
        Serial.write(START_SYSEX);
        Serial.write(0xCB);
        Serial.write(us_distance & B01111111); // MSB of distance
        Serial.write((us_distance >> 7) & B01111111);
        Serial.write(END_SYSEX);
      }
      break;

     case 0x20: //return activated / inactivates colors
    {
      byte col1, col2;
      col1 = EEPROM.read(116);
      col2 = EEPROM.read(117);
      Serial.write(START_SYSEX);
      Serial.write(0x20);
      Serial.write(col2); // MSB 
      Serial.write(col1); //LSB
      Serial.write(END_SYSEX);
    }
    break; 

    case 0x21: //initialize color sensor
    {
      if (tcs.begin())
      {
        setMelodytoPlay(7);
        isUserMelody = false;
        isMelodyToPlay = true;
             
      }
      else
      {
        setMelodytoPlay(8);
        isUserMelody = false;
        isMelodyToPlay = true; 
      }
    }
    break;


     case 0x23://Get color sensor readings
    {      
       tcs.getRawData(&r, &g, &b, &c);    
    }
     break;
     
      case 0x24: //return "red" component of the RGB value of the color sensor readings
    {
       Serial.write(START_SYSEX);
       Serial.write(0x24);
       Serial.write(r & B01111111);  
       Serial.write((r >> 7) & B01111111);  
       Serial.write(END_SYSEX); 
    }
     break;

   case 0x25: //return "green" component of the RGB value of the color sensor readings
    {
       Serial.write(START_SYSEX);
       Serial.write(0x25);
       Serial.write(g & B01111111);  
       Serial.write((g >> 7) & B01111111);  
       Serial.write(END_SYSEX); 
    }
     break;

     
     case 0x26: //return "blue" component of the RGB value of the color sensor readings
    {
       Serial.write(START_SYSEX);
       Serial.write(0x26);
       Serial.write(b & B01111111);  
       Serial.write((b >> 7) & B01111111);  
       Serial.write(END_SYSEX); 
    }
    break;
    
   case 0x27: // ("flavor 1") return matched fixed color that corresponds to the color sensor readings. The user must have fixed and activated the color before otherwise 
    {         //the command will return "unknown" or another active and fixed color that is close enougn to the detected one. 
              // We use two commands for this, each one using a different similarity measure. This is "flavor 1". "Flavor 2" is 0x32
      byte colors1, colors2, pos_min, frame, min_d_byte, i, accessed_r_byte1, accessed_r_byte2, accessed_g_byte1, accessed_g_byte2, accessed_b_byte1, accessed_b_byte2 ;
      int color[9], accessed_r, accessed_g, accessed_b, min_set = 0;
      float min_dist;
      uint16_t rn, gn, bn, cn;
      double distance_to_c, min_d;
      frame =  argv[0];
      tcs.getRawData(&rn, &gn, &bn, &cn);
      colors1 = EEPROM.read(116);
      colors2 = EEPROM.read(117);
      for (i = 0; i<=6; i++)
      {
        if (((colors1 >> i) & B00000001) == 1)
          color[i] = 1;
        else
          color[i] = 0;
      }
      if ((colors2  & B00000001) == 1)
        color[7] = 1;
      else
        color[7] = 0;

      if (((colors2 >> 1) & B00000001) == 1)
          color[8] = 1;
      else
          color[8] = 0;
      
      for (i=0; i<=8; i++)
      {
        if (color[i] == 1)
        {
          accessed_r_byte1 = EEPROM.read(60+i*6+1);
          accessed_r_byte2 = EEPROM.read(60+i*6+2);
          accessed_g_byte1 = EEPROM.read(60+i*6+3);
          accessed_g_byte2 = EEPROM.read(60+i*6+4);
          accessed_b_byte1 = EEPROM.read(60+i*6+5);
          accessed_b_byte2 = EEPROM.read(60+i*6+6);
          accessed_r = accessed_r_byte1 + accessed_r_byte2 * 256;
          accessed_g = accessed_g_byte1 + accessed_g_byte2 * 256;
          accessed_b = accessed_b_byte1 + accessed_b_byte2 * 256;
          distance_to_c = abs(accessed_r - rn) + abs(accessed_g - gn) + abs(accessed_b - bn);
          if (min_set == 0)
            {   
              min_d = distance_to_c;
              min_set = 1;
              pos_min = i;
            }
          else
          {
            if (distance_to_c < min_d)
            {
              min_d = distance_to_c;
              pos_min = i;
            }
          }
        }     
      }
      min_d_byte = min_d;
      if (min_d_byte > 2*frame)
          pos_min = 10;
      
      Serial.write(START_SYSEX);
      Serial.write(0x27);
      Serial.write(pos_min & B01111111);  
      Serial.write(END_SYSEX);  
     break;
    }
    case 0x28: // return the stored r value of a given color
    {
       byte r_byte1, r_byte2, colour;
       uint16_t r1;
       colour = argv[0];         
       r_byte1 = EEPROM.read(60+(colour-1)*6+1);
       r_byte2 = EEPROM.read(60+(colour-1)*6+2);
       r1 = r_byte2*256 + r_byte1;
       Serial.write(START_SYSEX);
       Serial.write(0x28);
       Serial.write(r1 & B01111111);  
       Serial.write((r1 >> 7) & B01111111);  
       Serial.write(END_SYSEX);   

       Serial.write(START_SYSEX);
       Serial.write(0x28);
       Serial.write(r1 & B01111111);  
       Serial.write((r1 >> 7) & B01111111);  
       Serial.write(END_SYSEX); 
    }
    break; 
        
    
    case 0x29: // return the stored g value of a given color
    {
       byte g_byte1, g_byte2, colour;
       uint16_t g1;
       colour = argv[0];         
       g_byte1 = EEPROM.read(60+(colour-1)*6+3);
       g_byte2 = EEPROM.read(60+(colour-1)*6+4);
       g1 = g_byte2 * 256 + g_byte1;
       Serial.write(START_SYSEX);
       Serial.write(0x29);
       Serial.write(g1 & B01111111);  
       Serial.write((g1 >> 7) & B01111111);  
       Serial.write(END_SYSEX);      
    }
    break; 

    case 0x30: // return the stored b value of a given color
    {
       byte b_byte1, b_byte2, colour;
       uint16_t b1;
       colour = argv[0];         
       b_byte1 = EEPROM.read(60+(colour-1)*6+5);
       b_byte2 = EEPROM.read(60+(colour-1)*6+6);
       b1 = b_byte2 * 256 + b_byte1;
       Serial.write(START_SYSEX);
       Serial.write(0x30);
       Serial.write(b1 & B01111111);  
       Serial.write((b1 >> 7) & B01111111);  
       Serial.write(END_SYSEX);       
    }
    break; 
    case 0x31: // command for printing something in the display
    {
       byte disp_image;
       disp_image = argv[0];
       if (disp_image != display_current_image)
       {
          Wire.beginTransmission(1);
          #if ARDUINO >= 100
            Wire.write(disp_image);
          #else
            Wire.send(disp_image);
          #endif
          Wire.endTransmission();
          delayMicroseconds(70);
          display_current_image = disp_image;
       }
       Serial.write(START_SYSEX);
       Serial.write(0x31);
       Serial.write(disp_image);   
       Serial.write(END_SYSEX);       
    }
    break;  

     case 0x32: // 
    {
     
      byte colors1, colors2, pos_min, frame, min_d_byte, i, accessed_r_byte1, accessed_r_byte2, accessed_g_byte1, accessed_g_byte2, accessed_b_byte1, accessed_b_byte2 ;
      int color[9], accessed_r, accessed_g, accessed_b, min_set = 0;
      float min_dist,  accessed_r_max,  accessed_r_min, accessed_g_max,  accessed_g_min,  accessed_b_max,  accessed_b_min;
      uint16_t rn, gn, bn, cn;
      double distance_to_c, min_d;
      frame =  argv[0];
      tcs.getRawData(&rn, &gn, &bn, &cn);
      colors1 = EEPROM.read(116);
      colors2 = EEPROM.read(117);
      for (i = 0; i<=6; i++)
      {
        if (((colors1 >> i) & B00000001) == 1)
          color[i] = 1;
        else
          color[i] = 0;
      }
      if ((colors2  & B00000001) == 1)
        color[7] = 1;
      else
        color[7] = 0;

      if (((colors2 >> 1) & B00000001) == 1)
          color[8] = 1;
      else
          color[8] = 0;
      
      for (i=0; i<=8; i++)
      {
        if (color[i] == 1)
        {
          accessed_r_byte1 = EEPROM.read(60+i*6+1);
          accessed_r_byte2 = EEPROM.read(60+i*6+2);
          accessed_g_byte1 = EEPROM.read(60+i*6+3);
          accessed_g_byte2 = EEPROM.read(60+i*6+4);
          accessed_b_byte1 = EEPROM.read(60+i*6+5);
          accessed_b_byte2 = EEPROM.read(60+i*6+6);
          accessed_r = accessed_r_byte1 + accessed_r_byte2 * 256;
          accessed_g = accessed_g_byte1 + accessed_g_byte2 * 256;
          accessed_b = accessed_b_byte1 + accessed_b_byte2 * 256;
          accessed_r_max =  accessed_r + accessed_r*frame/100;
          accessed_r_min =  accessed_r - accessed_r*frame/100;
          accessed_g_max =  accessed_g + accessed_g*frame/100;
          accessed_g_min =  accessed_g - accessed_g*frame/100;
          accessed_b_max =  accessed_b + accessed_b*frame/100;
          accessed_b_min =  accessed_b - accessed_b*frame/100;
          if ((rn >= accessed_r_min)&& (rn <= accessed_r_max) && (gn >= accessed_g_min)&& (gn <= accessed_g_max) && (bn >= accessed_b_min)&& (bn <= accessed_b_max))
          {
            if (min_set == 0)
            {
                min_set = 1;
                pos_min = i;
            }
            else
            {
              pos_min = 10;
            }
          }
        }     
      }
      if (min_set == 0)
      {
        pos_min = 11;
      }
      
      Serial.write(START_SYSEX);
      Serial.write(0x32);
      Serial.write(pos_min & B01111111);  
      Serial.write(END_SYSEX);  
    }
    break;
    case 0x33: // set movement constants for sevo grid
    {
      int i;
      byte par0;
      par0 = argv[0];
      for (i=1; i<= 10; i++)
      {
         EEPROM.write(10*(par0-1)+i, argv[i]);
      }
      setMelodytoPlay(7);
      isUserMelody = false;
      isMelodyToPlay = true;
                  
    }
    break; 
    case 0x34: // move forward, backward, turn left ot turn right for Level 1 blocks
    {
      byte left_servo_number, right_servo_number;
      int left_servo_speed, right_servo_speed, line_follow_mode;
      byte move_direction;
      if (in_step == false)
      {
          move_direction = argv[0];
          line_follow_mode = argv[1];
          //--------------------
          type_of_move = move_direction;
          left_servo_number = EEPROM.read(move_direction);
          pinConfig[left_servo_number] = SERVO;
          if (!servos[PIN_TO_SERVO(left_servo_number)].attached()) 
          {
            servos[PIN_TO_SERVO(left_servo_number)].attach(PIN_TO_DIGITAL(left_servo_number));
          }
          left_servo_speed = get_servo_speed(move_direction-1);
          //----------------------
          right_servo_number = EEPROM.read(move_direction+5);
          pinConfig[right_servo_number] = SERVO;
          if (!servos[PIN_TO_SERVO(right_servo_number)].attached()) 
          {
            servos[PIN_TO_SERVO(right_servo_number)].attach(PIN_TO_DIGITAL(right_servo_number));
          }
          right_servo_speed = get_servo_speed(move_direction+4);
          //---------------------    set values to servo wheels    ----------------------------------
          if (line_follow_mode == 0)
          {
            servos[PIN_TO_SERVO(left_servo_number)].write(left_servo_speed);
            servos[PIN_TO_SERVO(right_servo_number)].write(right_servo_speed);
          }
          else
          {
            if (line_follow_mode == 1) //this is for turning during line following. We need slow turns there
            {
              if (move_direction == 31)
              {
                servos[PIN_TO_SERVO(left_servo_number)].write(1580);
                servos[PIN_TO_SERVO(right_servo_number)].write(1650);
              }
              else
              {
                servos[PIN_TO_SERVO(left_servo_number)].write(1350);
                servos[PIN_TO_SERVO(right_servo_number)].write(1420);
              }
            }
          }
      }
    }
    break;
    case 0x35: // stop  movement using servo grid configuration
    { 
      byte left_servo_number, right_servo_number;
      byte par;
      par = argv[0];
      if (par == 2)
      {
        if ((type_of_move == 1) || (type_of_move == 11)|| (type_of_move == 31)|| (type_of_move == 41))
        {
          left_servo_number = EEPROM.read(type_of_move);
          right_servo_number = EEPROM.read(type_of_move+5);
          servos[PIN_TO_SERVO(left_servo_number)].write(1500);
          servos[PIN_TO_SERVO(right_servo_number)].write(1500);
          type_of_move = -1;
          in_step = false;
        }
      }
      else
      {
        if (in_step == false)
        {
          if ((type_of_move == 1) || (type_of_move == 11)|| (type_of_move == 31)|| (type_of_move == 41))
          {
            left_servo_number = EEPROM.read(type_of_move);
            right_servo_number = EEPROM.read(type_of_move+5);
            servos[PIN_TO_SERVO(left_servo_number)].write(1500);
            servos[PIN_TO_SERVO(right_servo_number)].write(1500);
            type_of_move = -1;
          }
        }
      }
    }
    break;

    case 0x36: //store slots for left and right tracker
    { 
      byte position_eeprom = argv[0], data_pos;
      data_pos = EEPROM.read(position_eeprom);
      Serial.write(START_SYSEX);
      Serial.write(0x36);
      Serial.write(data_pos);      
      Serial.write(END_SYSEX);
      
    }
    break;
    case 0x38: //store slots for left and right tracker
    { 
      EEPROM.write(118, argv[0]);
      EEPROM.write(119, argv[1]);
      setMelodytoPlay(7);
      isUserMelody = false;
      isMelodyToPlay = true;
    }
    break;
    
    case 0x39: //return slots for left and right tracker.
    { 
      byte left_tracker, right_tracker;
      left_tracker = EEPROM.read(118);
      right_tracker = EEPROM.read(119);
      Serial.write(START_SYSEX);
      Serial.write(0x39);
      Serial.write(left_tracker); 
      Serial.write(right_tracker);
      Serial.write(END_SYSEX);
    }
    break;
    
    case 0x3A: //store obstacle distance for ultrasonic sensor
    { 
      EEPROM.write(120, argv[0]);
      setMelodytoPlay(7);
      isUserMelody = false;
      isMelodyToPlay = true;
    }
    break;
    case 0x3B: //return obstacle distance for ultrasonic sensor
    { 
      byte obstacle_distance;
      obstacle_distance = EEPROM.read(120);
      Serial.write(START_SYSEX);
      Serial.write(0x3B);
      Serial.write(obstacle_distance); 
      Serial.write(END_SYSEX);
    }
    break;
    case 0x3C: //store servo number and parameters for actuator move for continuous actuator.
    { 
     
      EEPROM.write(122, argv[0]);   // servo number
      EEPROM.write(123, argv[1]);   // 1: clockwise, 2:counterclockwise
      EEPROM.write(124, argv[2]);   // duration of move in seconds  
      EEPROM.write(125, argv[3]);   // 1: perform opposite movement afterwards 2: do not perform opposite movement                       
      setMelodytoPlay(7);
      isUserMelody = false;
      isMelodyToPlay = true;
      setMelodytoPlay(7);
      isUserMelody = false;
      isMelodyToPlay = true;
    }
    break;

     case 0x3D: //store servo number and parameters for actuator move for angle actuator.
    { 
      EEPROM.write(126, argv[0]);   // servo number
      EEPROM.write(127, argv[1]);   // servo initial angle lsb
      EEPROM.write(128, argv[2]);   // servo initial angle msb
      EEPROM.write(129, argv[3]);   // target angle lsb (0-180)
      EEPROM.write(130, argv[4]);   // target angle msb (0-180)  
      EEPROM.write(131, argv[5]);   // time (in seconds) to return to initial position                
      setMelodytoPlay(7);
      isUserMelody = false;
      isMelodyToPlay = true;
      setMelodytoPlay(7);
      isUserMelody = false;
      isMelodyToPlay = true;
    }
    break;
    case 0x3E: //store type of actuator 1: continuous,    2: angle
    { 
      byte type_of_actuator;
      EEPROM.write(121, argv[0]);
      setMelodytoPlay(7);
      isUserMelody = false;
      isMelodyToPlay = true;
      setMelodytoPlay(7);
      isUserMelody = false;
      isMelodyToPlay = true;
    }
    break;
     case 0x3F: //DHT query
    { 
      byte message;
      
      
      if (millis() - DHT_last <=2000)
      {
        int time_to_next_call = millis() - DHT_last;
        Serial.write(START_SYSEX);
        Serial.write(0x3F);
        Serial.write(DHT_last_humid & B01111111);
      if (DHT_last_temp <0)
         Serial.write(0);
      else
         Serial.write(1);
      Serial.write(abs(DHT_last_temp) & B01111111);
        Serial.write(END_SYSEX);
      }
      else
      {
        int chk;
        byte DHTtype, DHTpin;
        DHTtype =argv[0];
        DHTpin = argv[1];
        if (DHTtype == 11)
            chk = DHT.read11(DHTpin+13);
        else
          if (DHTtype == 22)
            chk = DHT.read22(DHTpin+13);
          else
            if (DHTtype == 33)
                chk = DHT.read33(DHTpin+13);
            else
                chk = DHT.read44(DHTpin+13);
        
      Serial.write(START_SYSEX);
      Serial.write(0x3F);
      Serial.write(int(DHT.humidity) & B01111111);
      if (DHT.temperature <0)
         Serial.write(0);
      else
         Serial.write(1);
      Serial.write(abs(int(DHT.temperature)) & B01111111);
      Serial.write(END_SYSEX);
      DHT_last = millis();
      DHT_last_humid = DHT.humidity;
      DHT_last_temp = DHT.temperature;
    }
    }
    break;
    case 0x40: //set a led status
    {
      byte new_led_status, new_left_led_status, new_right_left_status;
       new_led_status = argv[0];
       if (new_led_status == 15 || new_led_status == 16 || new_led_status == 17)
       {
          if (new_led_status != left_led_status)
          {
              Wire.beginTransmission(1);
              #if ARDUINO >= 100
                Wire.write(new_led_status);
              #else
                Wire.send(new_led_status);
              #endif
              Wire.endTransmission();
              delayMicroseconds(70);
              left_led_status = new_led_status;
          }
       }
       else
       {
          if (new_led_status != right_led_status)
          {
              Wire.beginTransmission(1);
              #if ARDUINO >= 100
                Wire.write(new_led_status);
              #else
                Wire.send(new_led_status);
              #endif
              Wire.endTransmission();
              delayMicroseconds(70);
              right_led_status = new_led_status;
          }
       }
       
       Serial.write(START_SYSEX);
       Serial.write(0x40);
       Serial.write(new_led_status);   
       Serial.write(END_SYSEX);    
      
    }
    break;
/*    case 0x41:   //initialize seeed grove sunlight (SI1151) sensor   
    {       
      if (si1151.Begin()) {
          delay(100);   // small pause before first use
          setMelodytoPlay(7);
          isUserMelody = false;
          isMelodyToPlay = true;
      } else {
         setMelodytoPlay(8);
        isUserMelody = false;
        isMelodyToPlay = true; 
      }
    }
    break;*/
     case 0x51:   //SI 1151 visible light (SEEED GROVE sunlight sensor
    {
      uint16_t vlight;
      vlight = si1151.ReadVisible();
      Serial.write(START_SYSEX);
      Serial.write(0x51);
      Serial.write(vlight & 0x7F);
      Serial.write((vlight >> 7) & 0x7F);
      Serial.write(END_SYSEX);
    }
    break;
    case 0x41:   //SI 1151 IR light (SEEED GROVE sunlight sensor
    {
      uint16_t ir;
      ir = si1151.ReadIR();
      Serial.write(START_SYSEX);
      Serial.write(0x41);
      Serial.write(ir & 0x7F);
      Serial.write((ir >> 7) & 0x7F);
      Serial.write(END_SYSEX);
    }
    break;
     case 0x52:   //BH 1750 light sensor
    {
       uint16_t lux = round(lightMeter.readLightLevel());
       Serial.write(START_SYSEX);
       Serial.write(0x52);
       Serial.write(lux & 0x7F);
       Serial.write((lux >> 7) & 0x7F);
       Serial.write(END_SYSEX);
    }
    break;
/*     case 0x53:   //BME 280 sensor
    {
       byte sensor_mode;
       float value;
       sensor_mode = argv[0]; // a --> temperature, 2--> humidity, 3--> barometric preassure, 4-->altitude
       union {
                float f;
                uint32_t i;
       } data;
       switch (sensor_mode) {
          case 1: value = bme.readTemperature(); break;
          case 2: value = bme.readHumidity(); break;
          case 3: value = bme.readPressure(); break;
         
          default: value = 0.0; break;
      }


      data.f = value;
      Serial.write(START_SYSEX); // 0xF0
      Serial.write(0x53);     // your custom command, e.g. 0x53
      // Send 32-bit float in 5 chunks of 7-bit data
      Serial.write(data.i & 0x7F);
      Serial.write((data.i >> 7) & 0x7F);
      Serial.write((data.i >> 14) & 0x7F);
      Serial.write((data.i >> 21) & 0x7F);
      Serial.write((data.i >> 28) & 0x7F);
      Serial.write(END_SYSEX);   // 0xF7
    }
    break;
     
 */  
  }

}
int get_servo_speed(int base_position)
{
  int servo_speed;
  if (EEPROM.read(base_position+2)==1)
  {
        if (EEPROM.read(base_position+4)==1)
          servo_speed = 1500+50*EEPROM.read(base_position+3)+EEPROM.read(base_position+5);
        else if (EEPROM.read(base_position+4)==2)
          servo_speed = 1500+50*EEPROM.read(base_position+3)-EEPROM.read(base_position+5);
        else
          servo_speed = 1500+50*EEPROM.read(base_position+3);
  }
  else
  {
        if (EEPROM.read(base_position+4)==1)
          servo_speed = 1500-50*EEPROM.read(base_position+3)-EEPROM.read(base_position+5);
        else if (EEPROM.read(base_position+4)==2)
          servo_speed = 1500-50*EEPROM.read(base_position+3)+EEPROM.read(base_position+5);
        else
          servo_speed = 1500-50*EEPROM.read(base_position+3);
  }
  return servo_speed;
  
}
void enableI2CPins()
{
  byte i;
  // is there a faster way to do this? would probaby require importing
  // Arduino.h to get SCL and SDA pins
  for (i = 0; i < TOTAL_PINS; i++) {
    if (IS_PIN_I2C(i)) {
      // mark pins as i2c so they are ignore in non i2c data requests
      setPinModeCallback(i, I2C);
    }
  }

  isI2CEnabled = true;

  // is there enough time before the first I2C request to call this here?
  Wire.begin();
}

/* disable the i2c pins so they can be used for other functions */
void disableI2CPins() {
  isI2CEnabled = false;
  // disable read continuous mode for all devices
  queryIndex = -1;
  // uncomment the following if or when the end() method is added to Wire library
  // Wire.end();
}

/*==============================================================================
   SETUP()
  ============================================================================*/

void systemResetCallback()
{
  // initialize a defalt state
  // TODO: option to load config from EEPROM instead of default
  if (isI2CEnabled) {
    disableI2CPins();
  }
  for (byte i = 0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;      // by default, reporting off
    portConfigInputs[i] = 0;  // until activated
    previousPINs[i] = 0;
  }
  // pins with analog capability default to analog input
  // otherwise, pins default to digital output
  for (byte i = 0; i < TOTAL_PINS; i++) {
    if (IS_PIN_ANALOG(i)) {
      // turns off pullup, configures everything
      setPinModeCallback(i, ANALOG);
    } else {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(i, OUTPUT);
    }
  }
  // by default, do not report any analog inputs
  analogInputsToReport = 0;

  /* send digital inputs to set the initial state on the host computer,
     since once in the loop(), this firmware will only send on change */
  /*
    TODO: this can never execute, since no pins default to digital input
        but it will be needed when/if we support EEPROM stored config
    for (byte i=0; i < TOTAL_PORTS; i++) {
    outputPort(i, readPort(i, portConfigInputs[i]), true);
    }
  */
}




void dc_motorwork(byte &motor_finished,byte &motor_direction, byte motor_speed, unsigned long &motor_millis, byte &tdirection, int motor_pin1, int motor_pin2, byte refinement_direction, byte refinement_value)
{
 float time_unit = 100, border_line;
 
 if (refinement_direction == 1)
    border_line = motor_speed*(time_unit/10) + (time_unit/10)*(refinement_value/40);
 else if (refinement_direction == 2)
    border_line = motor_speed*(time_unit/10) - (time_unit/10)*(refinement_value/40);
 else
    border_line = motor_speed*(time_unit/10);
 if (motor_finished == 0)
 {
  if (motor_direction == 0)
  {
    if (millis()- motor_millis > time_unit)
    {
      motor_millis = millis();
      digitalWrite(motor_pin1, 255);
      digitalWrite(motor_pin2, 0);
    }
    else
    {
      if (millis()- motor_millis <= border_line)
      { 
        digitalWrite(motor_pin1, 255);
        digitalWrite(motor_pin2, 0);
      }
      else
      {
        digitalWrite(motor_pin1, 0);
        digitalWrite(motor_pin2, 0);
      }
    }   
  }
  if (motor_direction == 1)
  {
    if (millis()- motor_millis > time_unit)
    {
      motor_millis = millis();
      digitalWrite(motor_pin1, 0);
      digitalWrite(motor_pin2, 255);
    }
    else
    {
      if (millis()- motor_millis <= border_line)
      {
        digitalWrite(motor_pin1, 0);
        digitalWrite(motor_pin2, 255);
      }
      else
      { 
        digitalWrite(motor_pin1, 0);
        digitalWrite(motor_pin2, 0);
      }
    }   
  }
  if (motor_direction == 2)
  {
    digitalWrite(motor_pin1, 0);
    digitalWrite(motor_pin2, 0);
    motor_finished = 1; 
  }
  }
  return;
}

void setup()
{
  uint8_t number_of_attiny_bytes;
  uint8_t connection_state_received;
  
  delay(1000);
  Wire.begin(1);
  pinMode(13, INPUT);
  Wire.requestFrom(1, 1);                            
  number_of_attiny_bytes = Wire.available();                                   
  if (number_of_attiny_bytes == 1) 
  {                                  
     connection_state_received = Wire.read();                                                 
  } 
  else 
  {                                                           
     connection_state_received = -1;
  }
  Firmata.setFirmwareVersion(FIRMATA_MAJOR_VERSION, FIRMATA_MINOR_VERSION);
  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  Firmata.attach(START_SYSEX, sysexCallback);
  Firmata.attach(SYSTEM_RESET, systemResetCallback);


  Firmata.begin(57600);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for ATmega32u4-based boards and Arduino 101
  }
  systemResetCallback();  // reset to default config
 
  mycurrentMillis = 0;
  connection_state =  connection_state_received;
  DHT_last = millis();
  si1151.Begin();
  lightMeter.begin();
//  bme.begin();
//  IrReceiver.begin(A0, ENABLE_LED_FEEDBACK);
//     IrReceiver.enableIRIn();
}

/*==============================================================================
   LOOP()
  ============================================================================*/
void loop()
{
  byte pin, analogPin, byte1, byte2, byte3, byte4, i;

  static bool cwDirectionA = true; // assume initial direction(positive pwm) is clockwise
  static int pwmA = 150;
  static unsigned long lastMilliB = 0;
  static bool cwDirectionB = true; // assume initial direction(positive pwm) is clockwise
  static int pwmB = 1;
  /* DIGITALREAD - as fast as possible, check for changes and output them to the
     FTDI buffer using Serial.print()  */
  checkDigitalInputs();

  /* SERIALREAD - processing incoming messagse as soon as possible, while still
     checking digital inputs.  */
 
  while (Firmata.available())
  {
      Firmata.processInput();
      mycurrentMillis = millis();
  }
  /* SEND FTDI WRITE BUFFER - make sure that the FTDI buffer doesn't go over
     60 bytes. use a timer to sending an event character every 4 ms to
     trigger the buffer to dump. */

  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) 
  {
    previousMillis += samplingInterval;
    /* ANALOGREAD - do all analogReads() at the configured sampling interval */
    for (pin = 0; pin < TOTAL_PINS; pin++) {
      if (IS_PIN_ANALOG(pin) && pinConfig[pin] == ANALOG) {
        analogPin = PIN_TO_ANALOG(pin);
        if (analogInputsToReport & (1 << analogPin)) {
          Firmata.sendAnalog(analogPin, analogRead(analogPin));
        }
      }
    }
    // report i2c data for all device with read continuous mode enabled
    if (queryIndex > -1) {
      for (byte i = 0; i < queryIndex + 1; i++) {
        readAndReportData(query[i].addr, query[i].reg, query[i].bytes);
      }
    }
  }

  if (connection_state == 1 && (millis() - mycurrentMillis > 6000))
  {
      asm volatile (" jmp 0");
  }
  if (connection_state == 4)
  {
    if (millis() - mycurrentMillis > 6000)
    {
      setMelodytoPlay(6);
      isUserMelody = false;
      isMelodyToPlay = true;
      connection_state = 0;
      Wire.beginTransmission(1);
      #if ARDUINO >= 100
           Wire.write(23);
      #else
           Wire.send(23);
      #endif
      display_current_image = 23;
      Wire.endTransmission();
      delayMicroseconds(70);
      Wire.beginTransmission(1);
      connection_state = 5; 
      timer_5 = millis();    
    }
  } 
  if (connection_state == 5 && millis()-timer_5>1000)
  {
      asm volatile (" jmp 0"); 
  }
  if ((connection_state == 2 ) && (millis()- timer_last > 2000))
  {
     connection_state = 4;
      Wire.beginTransmission(1);
      #if ARDUINO >= 100
           Wire.write(3);
      #else
           Wire.send(3);
      #endif
      Wire.endTransmission();
      delayMicroseconds(70);
      display_current_image = 3;
      time_of_connection = millis();    
    }
    if ((connection_state == 3) && (millis()-timer2>1000))
    {
        connection_state = 2;
        setMelodytoPlay(5);
        isUserMelody = false;
        isMelodyToPlay = true;
        Wire.beginTransmission(1);
        #if ARDUINO >= 100
          Wire.write(24);
        #else
          Wire.send(24);
        #endif
        Wire.endTransmission();
        delayMicroseconds(70);
        display_current_image = 24;
        timer_last = millis();
    }
    //work on motor1
    dc_motorwork(motorAfinished, motorAdirection, motorAspeed, motorAmillis, set_A_direction, 3, 6, motorArefinementdirection, motorArefinementvalue);
    dc_motorwork(motorBfinished, motorBdirection, motorBspeed, motorBmillis, set_B_direction, 9, 10, motorArefinementdirection, motorArefinementvalue);

    if (isMelodyToPlay) updateMelodyToPlay(isUserMelody);
    if (millis()-start_of_step> time_reformed_step)
      in_step = false;
    if (millis()-start_of_turn> time_reformed_turn)
      in_turn = false;
    
}
