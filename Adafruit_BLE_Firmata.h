/*
  Firmata.h - Firmata library
  Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.

  Modified for Adafruit_BLE_Uart by Limor Fried/Kevin Townsend for
  Adafruit Industries, 2014

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.
*/

#ifndef Adafruit_BLE_Firmata_h
#define Adafruit_BLE_Firmata_h

#include <Arduino.h>

//#include "Boards.h"  /* Hardware Abstraction Layer + Wiring/Arduino */

//#define BLE_DEBUG

// move the following defines to Firmata.h?
#define I2C_WRITE B00000000
#define I2C_READ B00001000
#define I2C_READ_CONTINUOUSLY B00010000
#define I2C_STOP_READING B00011000
#define I2C_READ_WRITE_MODE_MASK B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000

#define MAX_QUERIES 8
#define MINIMUM_SAMPLING_INTERVAL 10

#define REGISTER_NOT_SPECIFIED -1

/* Version numbers for the protocol.  The protocol is still changing, so these
 * version numbers are important.  This number can be queried so that host
 * software can test whether it will be compatible with the currently
 * installed firmware. */
#define FIRMATA_MAJOR_VERSION 2  // for non-compatible changes
#define FIRMATA_MINOR_VERSION 3  // for backwards compatible changes
#define FIRMATA_BUGFIX_VERSION 1 // for bugfix releases

#define MAX_DATA_BYTES 32 // max number of data bytes in non-Sysex messages

// message command bytes (128-255/0x80-0xFF)
#define DIGITAL_MESSAGE 0x90 // send data for a digital pin
#define ANALOG_MESSAGE 0xE0  // send data for an analog pin (or PWM)
#define REPORT_ANALOG 0xC0   // enable analog input by pin #
#define REPORT_DIGITAL 0xD0  // enable digital input by port pair
//
#define SET_PIN_MODE 0xF4 // set a pin to INPUT/OUTPUT/PWM/etc
//
#define REPORT_VERSION 0xF9 // report protocol version
#define SYSTEM_RESET 0xFF   // reset from MIDI
//
#define START_SYSEX 0xF0 // start a MIDI Sysex message
#define END_SYSEX 0xF7   // end a MIDI Sysex message

// extended command set using sysex (0-127/0x00-0x7F)
/* 0x00-0x0F reserved for user-defined commands */
#define SERVO_CONFIG 0x70 // set max angle, minPulse, maxPulse, freq
#define STRING_DATA 0x71  // a string message with 14-bits per char
#define SHIFT_DATA 0x75   // a bitstream to/from a shift register
#define I2C_REQUEST 0x76  // send an I2C read/write request
#define I2C_REPLY 0x77    // a reply to an I2C read request
#define I2C_CONFIG                                                             \
  0x78 // config I2C settings such as delay times and power pins
#define EXTENDED_ANALOG 0x6F    // analog write (PWM, Servo, etc) to any pin
#define PIN_STATE_QUERY 0x6D    // ask for a pin's current mode and value
#define PIN_STATE_RESPONSE 0x6E // reply with pin's current mode and value
#define CAPABILITY_QUERY                                                       \
  0x6B // ask for supported modes and resolution of all pins
#define CAPABILITY_RESPONSE 0x6C  // reply with supported modes and resolution
#define ANALOG_MAPPING_QUERY 0x69 // ask for mapping of analog to pin numbers
#define ANALOG_MAPPING_RESPONSE 0x6A // reply with mapping info
#define REPORT_FIRMWARE 0x79         // report name and version of the firmware
#define SAMPLING_INTERVAL 0x7A       // set the poll rate of the main loop
#define SYSEX_NON_REALTIME 0x7E      // MIDI Reserved for non-realtime messages
#define SYSEX_REALTIME 0x7F          // MIDI Reserved for realtime messages
// these are DEPRECATED to make the naming more consistent
#define FIRMATA_STRING 0x71          // same as STRING_DATA
#define SYSEX_I2C_REQUEST 0x76       // same as I2C_REQUEST
#define SYSEX_I2C_REPLY 0x77         // same as I2C_REPLY
#define SYSEX_SAMPLING_INTERVAL 0x7A // same as SAMPLING_INTERVAL

// pin modes
//#define INPUT                 0x00 // defined in wiring.h
//#define OUTPUT                0x01 // defined in wiring.h
#define ANALOG 0x02 // analog pin in analogInput mode
#define PWM 0x03    // digital pin in PWM output mode
#define SERVO 0x04  // digital pin in Servo output mode
#define SHIFT 0x05  // shiftIn/shiftOut mode
#define I2C 0x06    // pin included in I2C setup
#define TOTAL_PIN_MODES 7

extern "C" {
// callback function types
typedef void (*callbackFunction)(byte, int);
typedef void (*systemResetCallbackFunction)(void);
typedef void (*stringCallbackFunction)(char *);
typedef void (*sysexCallbackFunction)(byte command, byte argc, byte *argv);
}

// TODO make it a subclass of a generic Serial/Stream base class
/**
 * @brief Class to allow for using the Firmata API via BLE
 *
 */
class Adafruit_BLE_FirmataClass {
public:
  Adafruit_BLE_FirmataClass(Stream &s);

  /* Arduino constructors */
  void begin();
  void begin(Stream &s);
  /* querying functions */
  void printVersion(void);
  void blinkVersion(
      void); ///< **UNIMPLEMENTED** Blink the protocol version numbe on an LED
  void printFirmwareVersion(void);
  // void setFirmwareVersion(byte major, byte minor);  // see macro below
  void setFirmwareNameAndVersion(const char *name, byte major, byte minor);
  /* serial receive handling */
  int available(void);
  int processInput(void);
  /* serial send handling */
  void sendAnalog(byte pin, int value);
  void sendDigital(byte pin, int value); // TODO implement this
  void sendDigitalPort(byte portNumber, int portData);
  void sendString(const char *string);
  void sendString(byte command, const char *string);
  void sendSysex(byte command, byte bytec, byte *bytev);
  /* attach & detach callback functions to messages */
  void attach(byte command, callbackFunction newFunction);
  void attach(byte command, systemResetCallbackFunction newFunction);
  void attach(byte command, stringCallbackFunction newFunction);
  void attach(byte command, sysexCallbackFunction newFunction);
  void detach(byte command);

  /* board details */
  void setUsablePins(uint8_t *digitaliopins, uint8_t num_digitaliopins,
                     uint8_t *analogiopins, uint8_t num_analogiopins,
                     uint8_t *pwmpins, uint8_t num_pwmpins, uint8_t *servopins,
                     uint8_t num_servopins, uint8_t sdapin, uint8_t sclpin);
  /**
   * @brief Determine if the given pin is a digital pin
   *
   * @param p The pin
   * @return boolean true: the pin is a digital pin false: the pin is not a
   * digital pin
   */
  boolean IS_PIN_DIGITAL(uint8_t p) {
    return contains(_digitaliopins, _num_digitaliopins, p);
  }
  /**
   * @brief Change a pin to digital
   *
   * @param p The pin to change
   * @return uint8_t The pin
   */
  uint8_t PIN_TO_DIGITAL(uint8_t p) { return p; }
  /**
   * @brief Determine if the given pin is a analog pin
   *
   * @param p The pin
   * @return boolean true: the pin is a analog pin false: the pin is not a
   * analog pin
   */
  boolean IS_PIN_ANALOG(uint8_t p) {
    return contains(_analogiopins, _num_analogiopins, p);
  }
  /**
   * @brief Change the given pin to analog
   *
   * @param p The pin to change
   * @return uint8_t The pin
   */
  uint8_t PIN_TO_ANALOG(uint8_t p);
  /**
   * @brief Determine if the given pin is a PWM pin
   *
   * @param p The pin
   * @return boolean true: the pin is a PWM pin false: the pin is not a PWM pin
   */
  boolean IS_PIN_PWM(uint8_t p) { return contains(_pwmpins, _num_pwmpins, p); }
  /**
   * @brief Change the given pin to PWM
   *
   * @param p The pin to change
   * @return uint8_t The pin
   */
  uint8_t PIN_TO_PWM(uint8_t p) { return p; }
  /**
   * @brief Determine if the given pin is a Servo pin
   *
   * @param p The pin
   * @return boolean true: the pin is a Servo pin false: the pin is not a Servo
   * pin
   */
  boolean IS_PIN_SERVO(uint8_t p) {
    return contains(_servopins, _num_servopins, p);
  }
  /**
   * @brief Change the given pin to Servo
   *
   * @param p The pin to change
   * @return uint8_t The pin
   */
  uint8_t PIN_TO_SERVO(uint8_t p) { return p - 2; }
  /**
   * @brief Determine if the given pin is an I2C pin
   *
   * @param p The pin
   * @return boolean true: the pin is a I2C pin false: the pin is not a I2C pin
   */
  boolean IS_PIN_I2C(uint8_t p) { return (p == _sdapin) || (p == _sclpin); }

  unsigned char readPort(byte port, byte bitmask);
  unsigned char writePort(byte port, byte value, byte bitmask);

  uint8_t _num_analogiopins; ///< The number of available analog pins

private:
  Stream &FirmataSerial;

  uint8_t *_digitaliopins, _num_digitaliopins;
  uint8_t *_pwmpins, _num_pwmpins;
  uint8_t *_analogiopins;
  uint8_t *_servopins, _num_servopins;
  uint8_t _sdapin, _sclpin;

  boolean contains(uint8_t *set, uint8_t num, uint8_t test);
  uint8_t location(uint8_t *set, uint8_t num, uint8_t test);

  /* firmware name and version */
  byte firmwareVersionCount;
  byte *firmwareVersionVector;
  /* input message handling */
  byte waitForData; // this flag says the next serial input will be data
  byte executeMultiByteCommand; // execute this after getting multi-byte data
  byte multiByteChannel;        // channel data for multiByteCommands
  byte storedInputData[MAX_DATA_BYTES]; // multi-byte data
                                        /* sysex */
  boolean parsingSysex;
  int sysexBytesRead;
  /* callback functions */
  callbackFunction currentAnalogCallback;
  callbackFunction currentDigitalCallback;
  callbackFunction currentReportAnalogCallback;
  callbackFunction currentReportDigitalCallback;
  callbackFunction currentPinModeCallback;
  systemResetCallbackFunction currentSystemResetCallback;
  stringCallbackFunction currentStringCallback;
  sysexCallbackFunction currentSysexCallback;

  /* private methods ------------------------------ */
  void processSysexMessage(void);
  void systemReset(void);
  void pin13strobe(int count, int onInterval, int offInterval);
  void sendValueAsTwo7bitBytes(int value);
  void startSysex(void);
  void endSysex(void);
};

extern Adafruit_BLE_FirmataClass BLE_Firmata;

/*==============================================================================
 * MACROS
 *============================================================================*/

/* shortcut for setFirmwareNameAndVersion() that uses __FILE__ to set the
 * firmware name.  It needs to be a macro so that __FILE__ is included in the
 * firmware source file rather than the library source file.
 */
#define setFirmwareVersion(x, y) setFirmwareNameAndVersion(__FILE__, x, y)

#endif /* BLE_Firmata_h */