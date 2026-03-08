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
