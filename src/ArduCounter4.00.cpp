/*
 * Sketch for counting impulses in a defined interval
 * e.g. for power meters with an s0 interface that can be 
 * connected to an input of an arduino or esp8266 board 
 *
 * the sketch uses pin change interrupts which can be anabled 
 * for any of the inputs on e.g. an arduino uno, jeenode, wemos d1 etc.
 *
 * the pin change Interrupt handling for arduinos used here 
 * is based on the arduino playground example on PCINT:
 * http://playground.arduino.cc/Main/PcInt which is outdated.
 *
 * see https://github.com/GreyGnome/EnableInterrupt for a newer library (not used here)
 * and also 
 * https://playground.arduino.cc/Main/PinChangeInterrupt
 * http://www.avrfreaks.net/forum/difference-between-signal-and-isr
 *
 * Refer to avr-gcc header files, arduino source and atmega datasheet.
 */

/* Arduino Uno / Nano Pin to interrupt map:
 * D0-D7 =           PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
 * D8-D13 =          PCINT 0-5 =   PCIR0 = PB = PCIE0 = pcmsk0
 * A0-A5 (D14-D19) = PCINT 8-13 =  PCIR1 = PC = PCIE1 = pcmsk1
 */

/* test cmds analog ESP 8266:
 *  20v               Verbose
 *  17,3,0,50a        A0, rising, no Pullup, MinLen 50
 *  15,25t            Level Diff Thresholds
 *  
 * for ESP8266 with D5 falling pullup 30
 *  5,2,1,30a
 *  20v
 *  10,20,1,1i
 * 
 * for ESP32 pin 23
 * 23,2,1,50a
 * 10,20,1,1i
 * 
 * for ESP32 with A0 = 36
 * 36,3,0,50a
 * 25v
 * Lilygo has right button at GPIO 35
 * 35,3,0,50a
 * 
 */

/*
    Changes:
        V1.2
        27.10.16 - use noInterrupts in report()
                 - avoid reporting very short timeDiff in case of very slow impulses after a report
                 - now reporting is delayed if impulses happened only within in intervalSml
                 - reporting is also delayed if less than countMin pulses counted
                 - extend command "int" for optional intervalSml and countMin
        29.10.16 - allow interval Min >= Max or Sml > Min 
                   which changes behavior to take fixed calculation interval instead of timeDiff between pulses
                   -> if intervalMin = intervalMax, counting will allways follow the reporting interval
        3.11.16  - more noInterrupt blocks when accessing the non uint8_t volatiles in report
        V1.3    
        4.11.16  - check min pulse width and add more output,
                 - prefix show output with M
        V1.4
        10.11.16 - restructure add Cmd
                 - change syntax for specifying minPulseLengh
             - res (reset) command
        V1.6
        13.12.16 - new startup message logic?, newline before first communication?
        18.12.16 - replace all code containing Strings, new communication syntax and parsing from Jeelink code
        V1.7
        2.1.17 - change message syntax again, report time as well, first and last impulse are reported 
                 relative to start of intervall not start of reporting intervall
        V1.8
        4.1.17 - fixed a missing break in the case statement for pin definition
        5.1.17 - cleanup debug logging
        14.10.17 - fix a bug where last port state was not initialized after interrupt attached but this is necessary there
        23.11.17 - beautify code, add comments, more debugging for users with problematic pulse creation devices
        28.12.17 - better reportung of first pulse (even if only one pulse and countdiff is 0 but realdiff is 1)
        30.12.17 - rewrite PCInt, new handling of min pulse length, pulse history ring
        1.1.18   - check len in add command, allow pin 8 and 13
        2.1.18   - add history per pin to report line, show negative starting times in show history
        3.1.18   - little reporting fix (start pos of history report)
        
        V2.0
        17.1.18  - rewrite many things - use pin number instead of pcIntPinNumber as index, split interrupt handler for easier porting to ESP8266, ...
        V2.23
        10.2.18  - new commands for check alive and quit, send setup message after reboot also over tcp
                    remove reporting time of first pulse (now we hava history)
                    remove pcIntMode (is always change now)
                    pulse min interval is now always checked and defaults to 2 if not set
        march 2018  many changes more to support ESP8266 
        7.3.18  - change pin config output, fix pullup (V2.26), store config in eeprom and read it back after boot
        22.4.18 - many changes, delay report if tcp mode and disconnected, verbose levels, ...
        13.5.18 - V2.36 Keepalive also on Arduino side
        9.12.18 - V3.0 start implementing analog input for old ferraris counters
        6.1.19  - V3.1 showIntervals in hello
        19.1.19 - V3.12 support for ESP with analog
        24.2.19 - V3.13 fix internal pin to GPIO mapping (must match ISR functions) when ESP8266 and analog support       
                - V3.14 added return of devVerbose upon startup
        27.6.19 - V3.20 replace timeNextReport with lastReportCall to avoid problem with data tyoes on ESP
                        fix a bug with analog counting on the ESP 
        20.7.19 -       nicer debug output for analog leves
        21.7.19 - V3.30 replace delay during analog read with millis() logic, optimize waiting times for analog read
        10.8.19 - V3.32 add ICACHE_RAM_ATTR for ISRs and remove remaining long casts (bug) when handling time
        12.8.19 - V3.33 fix handling of keepalive timeouts when millis wraps
                  V3.34 add RSSI output when devVerbose >= 5 in kealive responses
        16.8.19 - V3.35 fix a bug when analog support was disabled and a warning with an unused variable
        19.8.19 - V4.00 start porting to ESP32.
        21.12.19 - V4.10 Support for TTGO Lilygo Board (T-Display) with ST7789V 1,14 Zoll Display (240x135) see https://github.com/Xinyuan-LilyGO/TTGO-T-Display
                        or https://de.aliexpress.com/item/33048962331.html?spm=a2g0o.store_home.hotSpots_212315783.0
        30.12.19 - V4.20 started to make analog support user defineable at runtime
            reconnect when wifi is lost (see e.g. https://medium.com/diy-my-smart-home/esp-tipp-wifi-reconnect-einbauen-dc4a7397b741), was a bug,
            https://github.com/espressif/arduino-esp32/issues/2501
            https://github.com/jantenhove/GoodWeLogger/pull/20
            set autoReConnect: https://github.com/esp8266/Arduino/blob/master/doc/esp8266wifi/station-class.rst#setautoreconnect
            Doku: https://github.com/esp8266/Arduino/blob/master/doc/esp8266wifi/station-class.rst
            Source: https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/src/WiFi.h

                

    ToDo / Ideas:
        combine arrays (see below) - BitFields? https://en.cppreference.com/w/cpp/language/bit_field

        let analog pins be defined at runtime, save configuration for analog pins
        make analogInterval available in Fhem
        save analogInterval to Flash
        detect analog Threasholds automatically and adjust over time
        
        restructure ISRs // see https://github.com/arduino/Arduino/pull/4519, use attachInterrputArg on ESPs and AVR as soon as available
            https://github.com/arduino/ArduinoCore-avr/issues/85
            https://github.com/arduino/ArduinoCore-API/issues/95
            https://github.com/arduino/ArduinoCore-avr/pull/58
            https://www.bountysource.com/issues/30477489-support-extra-parameter-on-attachinterrupt


        OTA Update if wireless: https://github.com/espressif/arduino-esp32/tree/master/libraries/ArduinoOTA (via webserver)
        Better integrate TFT Display - https://github.com/Xinyuan-LilyGO/TTGO-T-Display

    
*/ 
#include <Arduino.h>

#if defined(TFT_DISPLAY)
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <Button2.h>
#include "esp_adc_cal.h"
/*#include "bmp.h"*/
TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
uint8_t lineCount;
#endif

#define analogIR 1  

/* allow printing of every pin change to Serial */
#define debugPins 1  

/* allow tracking of pulse lengths */
#define pulseHistory 1

#include "pins_arduino.h"
#include <EEPROM.h>

const char versionStr[] PROGMEM = "ArduCounter V4.20";

#define SERIAL_SPEED 38400      // todo: increase?
#define MAX_INPUT_NUM 8

#define digitalPinToGPIO(P) ( pgm_read_byte( digital_pin_to_gpio_PGM + (P) ) )
#define FF 255

#if defined(ESP8266) || defined(ESP32)  // Wifi stuff
#define WifiSupport 1
#if defined(ESP8266)
#include <ESP8266WiFi.h>          
#elif defined(ESP32)
#include <WiFi.h>          
#endif

#if defined(STATIC_WIFI)
#include "ArduCounterTestConfig.h"
#else
#include <WiFiManager.h>   
#endif

WiFiServer Server(80);              // For ESP WiFi connection
WiFiClient Client1;                 // active TCP connection
WiFiClient Client2;                 // secound TCP connection to send reject message

boolean Client1Connected;           // remember state of TCP connection
boolean Client2Connected;           // remember state of TCP connection

boolean tcpMode = false;
uint8_t delayedTcpReports = 0;      // how often did we already delay reporting because tcp disconnected
uint32_t lastDelayedTcpReports = 0; // last time we delayed

uint16_t keepAliveTimeout = 200;
uint32_t lastKeepAlive;

#endif


// function declaraions
void clearInput();
void restoreFromEEPROM();
void CmdSaveToEEPROM();
void CmdInterval(uint16_t *values, uint8_t size);
void CmdThreshold(uint16_t *values, uint8_t size);
void CmdAdd (uint16_t *values, uint8_t size);
void CmdRemove(uint16_t *values, uint8_t size);
void CmdShow();
void CmdHello();
void CmdWait(uint16_t *values, uint8_t size);
void CmdDevVerbose(uint16_t *values, uint8_t size);
void CmdKeepAlive(uint16_t *values, uint8_t size);
void CmdQuit();
void printConnection(Print *Out);



#if defined(ESP8266)                // ESP 8266 variables and definitions
                                    // ==================================

#define MAX_HIST 20                 // 20 history entries for ESP boards (can be increased)

#if defined (analogIR)              // code for ESP with analog pin and reflection light barrier support (test)

#define MAX_APIN 18
#define MAX_PIN 9

/* ESP8266 pins that are typically ok to use 
 * (some might be set to -1 (disallowed) because they are used 
 * as reset, serial, led or other things on most boards) 
 * maps printed pin numbers (aPin) to sketch internal index numbers */
 
 
// Todo: neues Array im Progmem (siehe https://atadiat.com/en/e-yield-function-printable-class-mapping-arrays-useful-arduino-core/)
// lesen über Makro wie in arduino.h digitalPinToPort
// -> digitalPinToGPIO oder forbidden
// zur Benutzung in add, remove, ...
// 

const uint8_t PROGMEM digital_pin_to_gpio_PGM[] = {
	D0, D1, D2, FF, 
    FF, D5, FF, FF,
    FF, FF, FF, FF,
    FF, FF, FF, FF,
    FF, A0
}; 

short allowedPins[MAX_APIN] =       // ESP 8266 with analog:
  { 0,  1,  2, -1,                  // printed pin numbers 0,1,2 are ok to be used
   -1,  5, -1, -1,                  // printed pin number 5 is ok to be used (6 and 7 are used for analog counting)
   -1, -1, -1, -1,                  // 8-11 not avaliable 
   -1, -1, -1, -1,                  // 12-15 not avaliable
   -1,  8 };                        // 16 not available, 17 is analog (at internal index 8)
     
/* Wemos / NodeMCU Pins 3,4 and 8 (GPIO 0,2 and 15) define boot mode and therefore
 * can not be used to connect to signal */

/* Map from sketch internal pin index to real chip IO pin number (not aPin, e.g. for ESP8266)
   Note that the internal numbers might be different from the printed 
   pin numbers (e.g. pin 0 is in index 0 but real chip pin number 16! */
   
short internalPins[MAX_PIN] = 
  { D0, D1, D2, D3,                 // map from internal pin Index to 
    D4, D5, D6, D7,                 // real GPIO pin numbers / defines
    A0 };                           // D0=16, D1=5, D2=4, D5=14, A0=17
                                    
                                    
uint8_t analogPins[MAX_PIN] = 
  { 0,0,0,0,0,0,0,0,1 };            // ESP pin A0 (pinIndex 8, internal 17) is analog 

const int analogInPin = A0;         // Analog input pin that the photo transistor is attached to (internally number 17)
const int irOutPin = D6;            // Digital output pin that the IR-LED is attached to
const int ledOutPin = D7;           // Signal LED output pin

#else                               // code for ESP8266 without analog pin and reflection light barrier support

#define MAX_APIN 8
#define MAX_PIN 8

/* ESP8266 pins that are typically ok to use 
 * (some might be set to -1 (disallowed) because they are used 
 * as reset, serial, led or other things on most boards) 
 * maps printed pin numbers to sketch internal index numbers */
short allowedPins[MAX_APIN] =       // ESP 8266 without analog: 
  { 0,  1,  2, -1,                  // printed pin numbers 0,1,2 are ok to be used, 3 not 
   -1,  5,  6,  7};                 // printed pin numbers 5-7 are ok to be used, 4 not, >8 not
                                    
/* Wemos / NodeMCU Pins 3,4 and 8 (GPIO 0,2 and 15) define boot mode and therefore
 * can not be used to connect to signal */

const uint8_t PROGMEM digital_pin_to_gpio_PGM[] = {
	D0, D1, D2, FF, 
    FF, D5, D6, D7
}; 


/* Map from sketch internal pin index to real chip IO pin number (not aPin, e.g. for ESP)
   Note that the internal numbers might be different from the printed 
   pin numbers (e.g. pin 0 is in index 0 but real chip pin number 16! */
   
short internalPins[MAX_PIN] = 
  { D0, D1, D2, D3,                 // printed pin numbers 0, 1, 2, 3   (3 should not be used and could be removed here)
    D5, D5, D6, D7};                // printed pin numbers 4, 5, 6, 7   (4 should not be used and could be removed here)
                                    // D0=16, D1=5, D2=4, D5=14, A0=17, ...
     
#endif                              // end of ESP8266 section without analog reading

#elif defined(ESP32)                // ESP32 variables and definitions
                                    // ==================================
#ifndef analogIR
#define analogIR 1                  // make sure analog is defined for now
#endif
const int irOutPin  = 27;           // Digital output pin that the IR-LED or Laser is attached to

#define MAX_HIST 20                 // 20 history entries for ESP boards (can be increased)
#define MAX_APIN 40
#define MAX_PIN 40

/* ESP32 pins that are typically ok to use 
 * (some might be set to -1 (disallowed) because they are used 
 * as reset, serial, led or other things on most boards) 
 * maps printed pin numbers (aPin) to sketch internal index numbers */
#if defined(TFT_DISPLAY)

const uint8_t PROGMEM digital_pin_to_gpio_PGM[] = {
    FF, FF, FF, FF,
    FF, FF, FF, FF,
    FF, FF, FF, FF,
    FF, FF, FF, FF,
    FF, 17, FF, FF,
    FF, 21, 22, FF,                  // 21-23 avaliable   
    FF, 25, 26, 27,                  // 25-26 avaliable, use 27 as irOut
    FF, FF, FF, FF,
    32, 33, 34, 35,                  // 32-35 avaliable (34/35 input only, 35 is right button)
    36, FF, FF, 39};                 // 36 is A0, is 39 avaliable but also input only


short allowedPins[MAX_APIN] =       // ESP32 Pins with TTGO Display
  {-1, -1, -1, -1,                  // printed pin numbers 0-3 are not ok (2 is LED)
   -1, -1, -1, -1,                  // printed pin number
   -1, -1, -1, -1,                  // 8-11 not avaliable
   -1, -1, -1, -1,                  // 12-15 for JTAG
   -1, 17, -1, -1,                  // 16-19 avaliable
   -1, 21, 22, -1,                  // 21-23 avaliable   
   -1, 25, 26, 27,                  // 25-26 avaliable, use 27 as irOut
   -1, -1, -1, -1,                  // 28-31 not avaliable
   32, 33, 34, 35,                  // 32-35 avaliable (34/35 input only, 35 is right button)
   36, -1, -1, 39};                 // 36 is A0, is 39 avaliable but also input only

const int ledOutPin = 0;            // no on board LED output pin
#else
short allowedPins[MAX_APIN] =       // ESP32 Pins
  {-1, -1, -1, -1,                  // printed pin numbers 0-3 are not ok (2 is LED)
   -1, -1, -1, -1,                  // printed pin number 4 is irOut
   -1, -1, -1, -1,                  // 8-11 not avaliable
   -1, -1, -1, -1,                  // 12-15 for JTAG
   16, 17, 18, 19,                  // 16-19 avaliable
   -1, 21, 22, 23,                  // 21-23 avaliable   
   -1, 25, 26, 27,                  // 25-27 avaliable   
   -1, -1, -1, -1,                  // 28-31 not avaliable
   32, 33, 34, 35,                  // 32-35 avaliable (34/35 input only)
   36, -1, -1, 39};                 // 36 is A0, is 39 avaliable but also input only
const int ledOutPin = 2;            // on board LED output pin 2
#endif

/* Map from sketch internal pin index to real chip IO pin number or GPIO numbers (same for ESP32) */
short internalPins[MAX_PIN] = 
  { 0, 1, 2, 3,                     // map from internal pin Index to 
    4, 5, 6, 7,                     // real GPIO pin numbers / defines
    8, 9, 10, 11,                   // real GPIO pin numbers / defines
    12, 13, 14, 15,                 // real GPIO pin numbers / defines
    16, 17, 18, 19,                 // real GPIO pin numbers / defines
    20, 21, 22, 23,                 // real GPIO pin numbers / defines
    24, 25, 26, 27,                 // real GPIO pin numbers / defines
    28, 29, 30, 31,                 // real GPIO pin numbers / defines
    32, 33, 34, 35,                 // real GPIO pin numbers / defines
    36, 37, 38, 39 };               // real GPIO pin numbers / defines
        
        
uint8_t analogPins[MAX_PIN] = 
  { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0 };            // ESP pin A0 (pinIndex 36, internal 36) is analog 
const int analogInPin = 36;         // Analog input pin that the photo transistor is attached to (internally number 36)
        
#elif defined(__AVR_ATmega328P__)
                                    // Arduino Uno or Nano variables and definitions
                                    // =============================================

#define MAX_HIST 20                 // 20 history entries for arduino boards

/* arduino pins that are typically ok to use 
 * (some might be set to -1 (disallowed) because they are used 
 * as reset, serial, led or other things on most boards) 
 * maps printed pin numbers to sketch internal index numbers */

#if defined(analogIR)

/* 2 is used for IR out, 12 for signal, A7 for In */
#define MAX_APIN 22
#define MAX_PIN 18
const int analogInPin = A7;         // Analog input pin that the photo transistor is attached to
const int irOutPin = 2;         // Digital output pin that the IR-LED is attached to


short allowedPins[MAX_APIN] = 
  {-1, -1,  -1, 0,      /* arduino pin 0 -  3 to internal index or -1 if pin is reserved */  
    1,  2,  3,  4,      /* arduino pin 4 -  7 to internal index */
    5,  6,  7,  8,      /* arduino pin 8 - 11 to internal index */
   -1,  9, 10, 11,      /* arduino pin 12, 13, A0, A1 / 14, 15 to internal index or -1 if pin is reserved*/
   12, 13, 14, 15,      /* arduino pin A2 - A5 / 16 - 19 to internal index */
   16, 17 };            /* arduino pin A6, A7 to internal index */

/* Map from sketch internal pin index to real chip IO pin number */
short internalPins[MAX_PIN] = 
  { 3,  4,  5,  6,      /* index  0 -  3 map to pins  3 -  6 */
    7,  8,  9, 10,      /* index  4 -  7 map to pins  7 - 10 */
   11, 12, 14, 15,      /* index  8 - 11 map to pins 11,13, A0 - A1 */
   16, 17, 18, 19,      /* index 12 - 15 map to pins A2 - A5 */
   20, 21 };            /* index 16 - 17 map to pin  A6, A7 */

uint8_t pinIndexMap[MAX_APIN];

// todo: internalPins can be byte / uint8_t instead of short
// and should be renamed gpioPins
// for ESP32 this is not needed, since printed pin numbers are same as gpio pin numbers
// however for ESP8266 we need this array.

// when the arrays with pinIndex becomes dynamic (pinIndex is assigned as pins are used)
// then we need another array to map to gpio pins too ...

// idea: new array that maps from printed pin numbers to gpio pins or "forbidden",
// another array of a struct containing used pins with their printed pin number, config and counters?
// or is this a conflict with volatile arrays?

uint8_t analogPins[MAX_PIN] = 
  { 0,0,0,0,            /* everything except Arduino A7 (pinIndex 17, internal 21) is digital by default */
    0,0,0,0, 
    0,0,0,0,
    0,0,0,0,
    0,1 };
    
// todo: merge this with some other state array so it can be defined at runtime




const int ledOutPin = 12;       // Signal LED output pin
// todo: these should also be defineable at runtime


   
#else
/* no analog IR support -> all Nano pins including analog available für digital counting */

#define MAX_APIN 22
#define MAX_PIN 20
short allowedPins[MAX_APIN] = 
  {-1, -1,  0,  1,      /* arduino pin 0 - 3 to internal Pin index or -1 if pin is reserved */  
    2,  3,  4,  5,      /* arduino pin 4 - 7 to internal Pin index or -1 if pin is reserved */
    6,  7,  8,  9,      /* arduino pin 8 - 11 to internal Pin index or -1 if pin is reserved */
   10, 11, 12, 13,      /* arduino pin 12, 13, A0, A1 to internal Pin index or -1 if pin is reserved */
   14, 15, 16, 17,      /* arduino pin A2 - A5 / 16 - 19 to internal Pin index or -1 if pin is reserved */
   18, 19 };            /* arduino pin A6, A7 to internal Pin index or -1 if pin is reserved */

// allowedPins is only used in PCint to map from port bits via firstPin array and allowedPins to our index,
// add and remove Cmds to validate input / map, hello cmd to print allowed pins 
// and analog read to get from analog in pin to internal index.

// PCint should be changed: fast way (RAM array) needed from GPIO to internal array index
// -> keep new array whe adding / removing from gpio to index.
// allowedpins can the go as Pin to GPIO into Progmem


/* Map from sketch internal pin index to real chip IO pin number */
short internalPins[MAX_PIN] = 
  { 2,  3,  4,  5,      /* index  0 -  3 map to pins  2 - 5 */
    6,  7,  8,  9,      /* index  4 -  7 map to pins  6 - 9 */
   10, 11, 12, 13,      /* index  8 - 11 map to pins 10 - 13 */
   14, 15, 16, 17,      /* index 12 - 15 map to pins A0 - A3 */
   18, 19, 20, 21 };    /* index 16 - 19 map to pins A4 - A7 */
   
uint8_t analogPins[MAX_PIN] = 
  { 0,0,0,0,            /* everything is digital by default */
    0,0,0,0,
    0,0,0,0,
    0,0,0,0,
    0,0,0,0 };
    
#endif
   
/* first and last pin at port PB, PC and PD for arduino uno/nano */
uint8_t firstPin[] = {8, 14, 0};    
// used in PCint to get from port bits -> aPin -> pinIndexMap[] -> pinIndex

uint8_t lastPin[]  = {13, 19, 7};
// not really needed. Instead PCint could check the bit position to be < 128 (or bit != 0 when bit is shifted out)

/* Pin change mask for each chip port on the arduino platform */
volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};

/* last PIN States at io port to detect individual pin changes in arduino ISR */
volatile static uint8_t PCintLast[3];

#endif                              // end of Nano / Uno specific stuff

Print *Output;                      // Pointer to output device (Serial / TCP connection with ESP8266)
uint32_t bootTime;                  
uint16_t bootWraps;                 // counter for millis wraps at last reset
uint16_t millisWraps;               // counter to track when millis counter wraps 
uint32_t lastMillis;                // milis at last main loop iteration
uint8_t devVerbose;                 // >=10 shows pin changes, >=5 shows pin history

#ifdef debugPins
uint8_t lastState[MAX_PIN];         // for debug output when a pin state changes
// rename to lastDebugLevel and merge with other array
#endif

uint32_t intervalMin = 30000;       // default 30 sec - report after this time if nothing else delays it
uint32_t intervalMax = 60000;       // default 60 sec - report after this time if it didin't happen before
uint32_t intervalSml =  2000;       // default 2 secs - continue count if timeDiff is less and intervalMax not over
uint16_t countMin    =     2;       // continue counting if count is less than this and intervalMax not over

uint32_t lastReportCall;

/* index to the following arrays is the internal pin index number  */

volatile boolean initialized[MAX_PIN];          // did we get first interrupt yet? 
// todo: merge with other state array (as activePin and analogPin)

short activePin[MAX_PIN];                       // printed arduino pin number for index if active - otherwise -1
// todo: replace this with a function call to convert pinIndex to aPin 
// and use other array to see if active

/*
new state array should contain:
- is pin active as input? (not needed if new array only contains active pin information)
- is pin initialized? (volatile)
- is pin analog?
- is pin unavailable because it is used as ir output, monitor out or else?

- is pin configured with pullup?
- configured pulse level
- last level (volatile)
- last level that was long (volatile)
- last pin debug level



or separate config values from volatile counting values:



allowedPins as the only const array maps from aPin to rPin or forbidden. 

Defined pins in use get into new arrays with pinIndex which is dynamically asssigned

for isr we need an array which contains rPin numbers to get the pin Level
and an array with pinIndex values to poin to (arg)

this could be a new array that contains only sequential pinIndex numbers for isr passing.

addPinChangeIntertupt would then be called with pinIndex.
attachInterrupt then passes pinIndex to isr.
this gets rPin via aPin and allowedPins? or faster with another array pinIndex to rPin...

config:
- pin index number itself for isr passing   (8 Bit)
- pin printed number of this entry          (8 Bit)
- real gpio pin number for reading          (8 Bit)
- pulse width min                           (8 Bit sollten ausreichen)

- corresponding analog output pin           (8 Bit)
- lower analog threshold                    (16 Bit)
- upper analog threshold                    (16 Bit)

- pullup flag                               (1 Bit / bool)
- configured pulse level                    (1 Bit)
- analog flag                               (1 Bit / bool)

counting:
- counter                                   (32 Bit)
- counter rejected                          (32 Bit)
- interval start time                       (32 Bit)
- interval end time                         (32 Bit)
- pulse width sum                           (32 Bit)

isr internal:
- last level change time (isr internal)     (32 Bit)
- last level (isr internal)                 (1 Bit)
- last long level (isr internal)            (1 Bit)

reporting:
- last count                                (32 Bit)
- last reject count                         (32 Bit)
- last reported time                        (32 Bit)
- report sequence                           (8 Bit)
- last debug level                          (1 Bit)

other array:
- pinUsage[aPin] -> c(ounting) a(nalog counting) i(r out) s(tatus out)  ??
or just check in above array if it is used as any of these ? (slower but less memory)

*/

#define PULLUP_MASK 1
#define ANALOG_MASK 2
#define LEVEL_MASK 4

// better just support one analog pin?? sum and count ...??
// dedicated struct for analog reading data (could become an array sometime later ...)



typedef struct pinData {
    // pin configuration data
    uint8_t ownIndex;                           // copy of pinIndex for passing to ISR
    uint8_t pinName;                            // printed pin Number for user input / output
    uint8_t gpioPinNumber;                      // for passing to pin i/o functions, attachinterrupt ...
    uint8_t pulseWidthMin;                      // minimal pulse length in millis for filtering
    uint8_t pulseLevel;                         // rising (1)/ falling (0)          // only one bit needed
    uint8_t pullupFlag;                         // 1 for pullup                     // only one bit needed
    uint8_t analogFlag;                         // 1 for analog                     // only one bit needed

    // counting data
    volatile uint32_t counter;                  // counter for pulses
    volatile uint32_t rejectCounter;            // counter for rejected pulses (width too small)
    volatile uint32_t intervalStart;            // time of first impulse in interval
    volatile uint32_t intervalEnd;              // time of last impulse in interval
    volatile uint32_t pulseWidthSum;            // sum of all pulse widthes during interval (for average)

    // isr internal states
    volatile uint32_t lastChange;               // millis at last level change (for measuring pulse length)
    volatile uint8_t lastLevel;                 // level of input at last interrupt                 // only one bit needed
    volatile uint8_t lastLongLevel;             // last level that was longer than pulseWidthMin    // only one bit needed

    // reporting data
    uint32_t lastCount;                         // counter at last report (to get the delta count)
    uint16_t lastRejCount;                      // reject counter at last report (to get the delta count)
    uint32_t lastReport;                        // millis at last report to find out when maxInterval is over
    uint8_t reportSequence;                     // sequence number for reports
    uint8_t lastState;                          // for debug output when a pin state changes
} pinData_t;

pinData_t pinData[MAX_PIN];
uint8_t maxPinIndex = 0;                        // the next available index (= number of indices used)

typedef struct analogData {
    uint8_t inPinName;                          // printed pin Number for user input / output (optinal here?)
    uint8_t outPinName;                         // printed pin number to use for ir / laser output (convert using our macro before use)
    uint16_t thresholdMin;                      // measurement for 0 level 
    uint16_t thresholdMax;                      // measurement for 1 
    uint8_t triggerState;                       // which level was it so far
    uint16_t sumOff = 0;                        // sum of measured values during light off
    uint16_t sumOn  = 0;                        // sum of measured values during light on
} analogData_t;

#define MAX_ANALOG 2
analogData_t analogData[MAX_ANALOG];
uint8_t maxAnalogIndex = 0;


#ifdef pulseHistory 
volatile uint8_t histIndex;                     // pointer to next entry in history ring
volatile uint16_t histNextSeq;                  // next seq number to use
volatile uint16_t histSeq[MAX_HIST];            // history sequence number
volatile uint8_t histPin[MAX_HIST];             // pin for this entry
volatile uint8_t histLevel[MAX_HIST];           // level for this entry
volatile uint32_t histTime[MAX_HIST];           // time for this entry
volatile uint32_t histLen[MAX_HIST];            // time that this level was held
volatile char histAct[MAX_HIST];                // action (count, reject, ...) as one char
#endif

uint16_t commandData[MAX_INPUT_NUM];            // input data over serial port or network
uint8_t  commandDataPointer = 0;                // index pointer to next input value
uint8_t  commandDataSize = 0;                   // number of input values specified in commandData array
char     commandLetter;                         // the actual command letter
uint16_t commandValue = 0;                      // the current value for input function

#ifdef analogIR
uint32_t lastAnalogRead;            // millis() at last analog read
uint16_t analogReadInterval = 50;   // interval at which to read analog values (miliseconds)
uint8_t analogReadState = 0;        // to keep track of switching LED on/off, measuring etc.
uint8_t analogReadAmp = 3;          // amplification for display
uint8_t analogReadNumOff = 4;       // samples to take with the light off - max 16 so sum can be an int 16
uint8_t analogReadNumOn = 4;        // samples to take with the light on

uint8_t analogReadCntOff = 0;       // counter for sampling
uint8_t analogReadCntOn = 0;        // counter for sampling

uint8_t triggerState;               // saved analog level, stays same if not exceeding a threshold

// idea: save analog measurement during same level as sum and count to get average and then put in history when doCount is called so that threshold can be auto adjusted or even self learning?

// but how do we do this before we can detect the levels?



#endif

void initPinVars(short pinIndex, uint32_t now) {
    uint8_t level = 0;
    pinData[pinIndex].pinName      = -1;          // inactive (-1)
    initialized[pinIndex]    = false;       // no pulse seen yet
    pinData[pinIndex].pulseWidthMin = 0;           // min pulse length
    counter[pinIndex]        = 0;           // counter to 0
    counterIgn[pinIndex]     = 0;    
    lastCount[pinIndex]      = 0;
    rejectCounter[pinIndex]  = 0;        
    lastRejCount[pinIndex]   = 0;
    intervalStart[pinIndex]  = now;         // time vars
    intervalEnd[pinIndex]    = now;
    lastChange[pinIndex]     = now;
    lastReport[pinIndex]     = now;
    reportSequence[pinIndex] = 0;
#ifdef analogIR 
    if (!analogPins[pinIndex]) {
            level = digitalRead(internalPins[pinIndex]);
      }
#else  
    level = digitalRead(internalPins[pinIndex]);
#endif  
    lastLevel[pinIndex]      = level;
    // todo: also initialize lastLongLevel
#ifdef debugPins      
    lastState[pinIndex]      = level;       // for debug output
#endif
    /* todo: add upper and lower thresholds for analog */
}


void initialize() {
    uint32_t now = millis();
    bootTime = now;             // with boot / reset time
    bootWraps = millisWraps;
    lastReportCall = now;       // time for first output after intervalMin from now
    devVerbose = 0;
    for (uint8_t pinIndex=0; pinIndex < MAX_PIN; pinIndex++)
        initPinVars(pinIndex, now);
    for (uint8_t aPin=0; aPin < MAX_APIN; aPin++) {
        pinIndexMap[aPin] = FF;
#ifdef analogIR    
    lastAnalogRead = now;
#endif
#ifdef WifiSupport  
    lastKeepAlive = now;
#endif      
    restoreFromEEPROM();
    clearInput();
}

 
uint8_t findPin (uint8_t aPin) {
    for (uint8_t pinIndex = 0; pinIndex < maxPinIndex; pinIndex++) {
        if (pinData[pinIndex].pinName == aPin) return pinIndex;
    }
    return FF;
}


bool checkPin (uint8_t aPin) {
    if (aPin >= MAX_APIN) return false;                     // pin number too big
    if (digital_pin_to_gpio(aPin) == FF) return false;      // pin is not allowed at all

    for (uint8_t pinIndex = 0; pinIndex < maxPinIndex; pinIndex++)
        if (pinData[pinIndex].analogFlag && pinData[pinIndex].analogOutPin == aPin) 
            return false;                                   // pin is used as analog out pin for this pinIndex
    if (ledOutPin && ledOutPin == aPin)
        return false;                                       // pin is used as led out pin
    return true;
}


void clearInput() {
        commandDataPointer = 0;
        commandDataSize = 0;
        commandValue = 0;
        for (uint8_t i=0; i < MAX_INPUT_NUM; i++)
            commandData[i] = 0;     
}


void PrintErrorMsg() {
    Output->print(F("Error: "));
    Output->print(F("command "));
    Output->println(commandLetter);
    Output->print(F(" "));
}

void PrintPinErrorMsg(uint8_t aPin) {
    PrintErrorMsg(); 
    Output->print(F("Illegal pin specification "));
    Output->println(aPin);
}    


void printVersionMsg() {  
    uint8_t len = strlen_P(versionStr);
    char myChar;
    for (unsigned char k = 0; k < len; k++) {
        myChar = pgm_read_byte_near(versionStr + k);
        Output->print(myChar);
    }
    Output->print(F(" on "));
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
#ifdef ARDUINO_AVR_NANO
    Output->print(F("NANO"));
#else 
    Output->print(F("UNO"));
#endif
#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
    Output->print(F("Leonardo"));
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    Output->print(F("Mega"));
#elif defined(ESP8266)
    Output->print(F("ESP8266"));
#else
    Output->print(F("UNKNOWN"));
#endif
#ifdef ARDUINO_BOARD
    Output->print(F(" "));
    Output->print(F(ARDUINO_BOARD));
#endif
    Output->print(F(" compiled "));
    Output->print(F(__DATE__ " " __TIME__));
#if defined(ESP8266)
#if defined (ARDUINO_ESP8266_RELEASE)
    Output->print(F(" with core version "));
    Output->print(F(ARDUINO_ESP8266_RELEASE));
#endif
#endif    
}


bool checkVal(uint8_t index, uint16_t min, uint16_t max, bool doErr = true) {
    if (index >= commandDataSize) {
        if (doErr) {
            PrintErrorMsg();
            Output->print(F("missing parameter number "));
            Output->println(index);
        }
        return false;
    }
    if (commandData[index] < min || commandData[index] > max) {
        PrintErrorMsg();
        Output->print(F("parameter number "));
        Output->print(index);
        Output->print(F(" (value "));
        Output->println(F(") is out of bounds"));
        return false;
    }
    return true;
}


/*
   do counting and set start / end time of interval.
   reporting is not triggered from here.
   
   only here counter[] is modified
   intervalEnd[] is set here and in report
   intervalStart[] is set in case a pin was not initialized yet and in report
*/
static void inline doCount(uint8_t pinIndex, uint8_t level, uint32_t now) {
    uint32_t len = now - lastChange[pinIndex];
    char act = ' ';
#ifdef pulseHistory 
    histIndex++;
    if (histIndex >= MAX_HIST) histIndex = 0;
    histSeq[histIndex]   = histNextSeq++;
    histPin[histIndex]   = pinIndex;
    histTime[histIndex]  = lastChange[pinIndex];
    histLen[histIndex]   = len;
    histLevel[histIndex] = lastLevel[pinIndex];
#endif    
    if (len < pinData[pinIndex].pulseWidthMin) {                    // pulse was too short
        lastChange[pinIndex] = now;
        if (lastLevel[pinIndex] == pinData[pinIndex].pulseLevel) {  // if change to gap level
            rejectCounter[pinIndex]++;                      // inc reject counter and set action to R (pulse too short)
            act = 'R';
        } else {
            act = 'X';                                      // set action to X (gap too short)
        }
    } else {
        if (lastLevel[pinIndex] != pinData[pinIndex].pulseLevel) {  // edge does fit defined pulse start, level is now pulse, before it was gap
            act = 'G';                                      // now the gap is confirmed (even if inbetween was a spike that we ignored)
        } else {                                            // edge is a change to gap, level is now gap
            if (lastLongLevel[pinIndex] != pinData[pinIndex].pulseLevel) { // last remembered valid level was also gap -> now we had valid new pulse -> count
                counter[pinIndex]++;                        // count
                intervalEnd[pinIndex] = now;                // remember time of in case pulse will be the last in the interval
                if (!initialized[pinIndex]) {
                    intervalStart[pinIndex] = now;          // if this is the very first impulse on this pin -> start interval now
                    initialized[pinIndex] = true;           // and start counting the next impulse (so far counter is 0)
                    counterIgn[pinIndex]++;                 // count has to be ignored for diff because it defines the start of the interval
                }
                pulseWidthSum[pinIndex] += len;             // for average calculation
                act = 'C';
            } else {                                        // last remembered valid level was a pulse -> now we had another valid pulse
                pulseWidthSum[pinIndex] += len;             // for average calculation
                act = 'P';                                  // pulse was already counted, only short drop inbetween
            }
        }       
        lastLongLevel[pinIndex] = lastLevel[pinIndex];      // remember this valid level as lastLongLevel
    }
#ifdef pulseHistory   
    histAct[histIndex]   = act;
#endif  
    lastChange[pinIndex] = now;
    lastLevel[pinIndex]  = level;
}


/* Interrupt handlers and their installation  */

#if defined(__AVR_ATmega328P__)

/* Add a pin to be handled (Arduino code) */
uint8_t AddPinChangeInterrupt(uint8_t pinIndex) {
    uint8_t rPin = internalPins[pinIndex];
    volatile uint8_t *pcmask;                       // pointer to PCMSK0 or 1 or 2 depending on the port corresponding to the pin
    uint8_t bitM = digitalPinToBitMask(rPin);       // mask to bit in PCMSK to enable pin change interrupt for this arduino pin 
    uint8_t port = digitalPinToPort(rPin);          // port that this arduno pin belongs to for enabling interrupts
    if (port == NOT_A_PORT) 
        return 0;
    port -= 2;                                      // from port (PB, PC, PD) to index in our array
    PCintLast[port] = *portInputRegister(port+2);   // save current inut state to detect changes in isr
    pcmask = port_to_pcmask[port];                  // point to PCMSK0 or 1 or 2 depending on the port corresponding to the pin
    *pcmask |= bitM;                                // set the pin change interrupt mask through a pointer to PCMSK0 or 1 or 2 
    PCICR |= 0x01 << port;                          // enable the interrupt
    return 1;
}


/* Remove a pin to be handled (Arduino code) */
uint8_t RemovePinChangeInterrupt(uint8_t pinIndex) {
    uint8_t rPin = internalPins[pinIndex];
    volatile uint8_t *pcmask;
    uint8_t bitM = digitalPinToBitMask(rPin);
    uint8_t port = digitalPinToPort(rPin);
    if (port == NOT_A_PORT)
        return 0;
    port -= 2;                                  // from port (PB, PC, PD) to index in our array
    pcmask = port_to_pcmask[port];          
    *pcmask &= ~bitM;                           // clear the bit in the mask.
    if (*pcmask == 0) {                         // if that's the last one, disable the interrupt.
        PCICR &= ~(0x01 << port);
    }
    return 1;
}


/* 
   common function for arduino pin change interrupt handlers. 
   "port" is the PCINT port index (0-2) as passed from above, 
   not PB, PC or PD which are mapped to 2-4
*/
static void PCint(uint8_t port) {
    uint8_t bit;
    uint8_t curr;
    uint8_t delta;
    short pinIndex;
    uint32_t now = millis();

    // get the pin states for the indicated port.
    curr  = *portInputRegister(port+2);                         // current pin states at port (add 2 to get from index to PB, PC or PD)
    delta = (curr ^ PCintLast[port]) & *port_to_pcmask[port];   // xor gets bits that are different and & screens out non pcint pins
    PCintLast[port] = curr;                                     // store new pin state for next interrupt

    if (delta == 0) return;                                     // no handled pin changed 
    
    // the printed pin numbers for the ports are sequential, starting with 8, 14 and 0 which we keep in an array
    bit = 0x01;                                                 // start mit rightmost (least significant) bit in a port
    for (uint8_t aPin = firstPin[port]; aPin <= lastPin[port]; aPin++) { // loop over each pin on the given port that changed - todo: until bit == 0 ??
        if (delta & bit) {                                      // did this pin change?
            pinIndex = pinIndexMap[aPin];
            if (pinIndex != FF)                                 // shound not be necessary but test anyway
                doCount (pinIndex, ((curr & bit) > 0), now);    // do the counting, history and so on
        }
        bit = bit << 1;                                         // shift mask to go to next bit
    } 
}


// now set the arduino interrupt service routines and call the common handler with the port index number
ISR(PCINT0_vect) {
    PCint(0);
}
ISR(PCINT1_vect) {
    PCint(1);
}
ISR(PCINT2_vect) {
    PCint(2);
}

#elif defined(ESP8266) || defined(ESP32)

void IRAM_ATTR ESPISR(void* arg) {                                          // common ISR for all Pins on ESP
    uint8_t pinIndex = *(uint8_t *)arg;
    doCount(pinIndex, digitalRead(internalPins[pinIndex]), millis());       // called with pinIndex, level, now
}

#endif


void showIntervals() {
    Output->print(F("I"));  Output->print(intervalMin / 1000);
    Output->print(F(" "));  Output->print(intervalMax / 1000);
    Output->print(F(" "));  Output->print(intervalSml / 1000);
    Output->print(F(" "));  Output->println(countMin);
}


#ifdef analogIR 
void showThresholds() {
    Output->print(F("T"));  Output->print(analogThresholdMin);
    Output->print(F(" "));  Output->println(analogThresholdMax);
}
#endif


void showpinData(short pinIndex) {
    Output->print(F("P"));  Output->print(pinData[pinIndex].pinName);
    switch (pinData[pinIndex].pulseLevel) {
        case 1:  Output->print(F(" rising")); break;
        case 0: Output->print(F(" falling")); break;
        default: Output->print(F(" -")); break;
    }        
    if (pinData[pinIndex].pullupFlag) 
        Output->print(F(" pullup"));
    Output->print(F(" min "));  Output->print(pinData[pinIndex].pulseWidthMin);
}

#ifdef pulseHistory 
void showPinHistory(short pinIndex, uint32_t now) {
    uint8_t hi;
    uint8_t start = (histIndex + 2) % MAX_HIST;
    uint8_t count = 0;
    uint32_t last = 0;
    boolean first = true;

    for (uint8_t i = 0; i < MAX_HIST; i++) {
        hi = (start + i) % MAX_HIST;
        if (histPin[hi] == pinIndex)
            if (first || (last <= histTime[hi]+histLen[hi])) count++;
    }
    if (!count) { 
      // Output->println (F("M No Pin History"));            
      return;
    }
    
    Output->print (F("H"));  Output->print (pinData[pinIndex].pinName);       // printed pin number
    Output->print (F(" "));
    for (uint8_t i = 0; i < MAX_HIST; i++) {
        hi = (start + i) % MAX_HIST;
        if (histPin[hi] == pinIndex) {
            if (first || (last <= histTime[hi]+histLen[hi])) {
                if (!first) Output->print (F(", "));
                Output->print (histSeq[hi]);            // sequence
                Output->print (F("s"));  Output->print ((long) (histTime[hi] - now)); // time when level started
                Output->print (F("/"));  Output->print (histLen[hi]);   // length 
                Output->print (F("@"));  Output->print (histLevel[hi]); // level (0/1)
                Output->print (histAct[hi]);                            // action
                first = false;
            }
            last = histTime[hi];
        }
    }        
    Output->println();    
}
#endif

/*
   lastCount[] is only modified here (count at time of last reporting)
   intervalEnd[]  is modified here and in ISR - disable interrupts in critcal moments to avoid garbage in var
   intervalStart[] is modified only here or for very first Interrupt in ISR
*/
void showPinCounter(short pinIndex, boolean showOnly, uint32_t now) {
    uint32_t count, countDiff, realDiff;
    uint32_t startT, endT, timeDiff, widthSum;
    uint16_t rejCount, rejDiff;
    uint8_t countIgn;
    
    noInterrupts();                                 // copy counters while they cant be changed in isr
    startT   = intervalStart[pinIndex];             // start of interval (typically first pulse)
    endT     = intervalEnd[pinIndex];               // end of interval (last unless not enough)
    count    = counter[pinIndex];                   // get current counter (counts all pulses
    rejCount = rejectCounter[pinIndex];
    countIgn = counterIgn[pinIndex];                // pulses that mark the beginning of an interval
    widthSum = pulseWidthSum[pinIndex];
    interrupts();
        
    timeDiff  = endT - startT;                      // time between first and last impulse
    realDiff  = count - lastCount[pinIndex];        // pulses during intervall
    countDiff = realDiff - countIgn;                // ignore forst pulse after device restart
    rejDiff   = rejCount - lastRejCount[pinIndex];
    
    if (!showOnly) {                                // real reporting sets the interval borders new
        if((now - lastReport[pinIndex]) > intervalMax) { 
            // intervalMax is over
            if ((countDiff >= countMin) && (timeDiff > intervalSml) && (intervalMin != intervalMax)) {
                // normal procedure
                noInterrupts();                     // vars could be modified in ISR as well
                intervalStart[pinIndex] = endT;     // time of last impulse becomes first in next
                interrupts();
            } else {
                // nothing counted or counts happened during a fraction of intervalMin only
                noInterrupts();                     // vars could be modified in ISR as well
                intervalStart[pinIndex] = now;      // start a new interval for next report now
                intervalEnd[pinIndex]   = now;      // no last impulse, use now instead
                interrupts();
                timeDiff  = now - startT;           // special handling - calculation ends now
            }        
        } else if (((now - lastReport[pinIndex]) > intervalMin)   
            && (countDiff >= countMin) && (timeDiff > intervalSml)) {
            // minInterval has elapsed and other conditions are ok
            noInterrupts();                         // vars could be modified in ISR as well
            intervalStart[pinIndex] = endT;         // time of last also time of first in next
            interrupts();
        } else {
          return;                                   // intervalMin and Max not over - dont report yet
        }
        noInterrupts(); 
        counterIgn[pinIndex]    = 0;
        pulseWidthSum[pinIndex] = 0;
        interrupts();
        lastCount[pinIndex]    = count;             // remember current count for next interval
        lastRejCount[pinIndex] = rejCount;
        lastReport[pinIndex]   = now;               // remember when we reported
#ifdef WifiSupport
        delayedTcpReports      = 0;
#endif
        reportSequence[pinIndex]++;
    }   
    Output->print(F("R"));  Output->print(pinData[pinIndex].pinName);
    Output->print(F(" C")); Output->print(count);
    Output->print(F(" D")); Output->print(countDiff);
    Output->print(F("/"));  Output->print(realDiff);
    Output->print(F(" T")); Output->print(timeDiff);  
    Output->print(F(" N")); Output->print(now);
    Output->print(F(","));  Output->print(millisWraps);    
    Output->print(F(" X")); Output->print(rejDiff);  
    
    if (!showOnly) {
        Output->print(F(" S")); Output->print(reportSequence[pinIndex]);  
    }
    if (countDiff > 0) {
        Output->print(F(" A")); Output->print(widthSum / countDiff);
    }
    Output->println();    
#ifdef WifiSupport
    if (tcpMode && !showOnly) {
        Serial.print(F("D reported pin "));  Serial.print(pinData[pinIndex].pinName);
        Serial.print(F(" sequence "));       Serial.print(reportSequence[pinIndex]);  
        Serial.println(F(" over tcp "));  
    }
#endif  
#if defined(TFT_DISPLAY)
    if (lineCount < 4) {
        tft.setCursor(0,32+16*lineCount);
        tft.print(F("R"));  tft.print(pinData[pinIndex].pinName);
        tft.print(F(" C")); tft.print(count);
        tft.print(F(" D")); tft.print(countDiff);
        tft.print(F("/"));  tft.print(realDiff);
        tft.print(F(" T")); tft.print(timeDiff);  
        tft.print(F(" X")); tft.print(rejDiff);  
        lineCount++;
    }
#endif
    
}


/* 
   report count and time for pins that are between min and max interval    
*/

boolean reportDue() {
    uint32_t now = millis();
    boolean doReport  = false;                          // check if report needs to be called
    if((now - lastReportCall) > intervalMin)            // works fine when millis wraps.
        doReport = true;                                // intervalMin is over 
    else 
        for (uint8_t pinIndex=0; pinIndex < MAX_PIN; pinIndex++)  
            if (pinData[pinIndex].pinName >= 0)
                if((now - lastReport[pinIndex]) > intervalMax)
                    doReport = true;                    // active pin has not been reported for langer than intervalMax
    return doReport;
}



void report() {
    uint32_t now = millis();    
#ifdef WifiSupport
    if (tcpMode && !Client1Connected && (delayedTcpReports < 3)) {
        if(delayedTcpReports == 0 || ((now - lastDelayedTcpReports) > 30000)) {
            Serial.print(F("D report called but tcp is disconnected - delaying ("));
            Serial.print(delayedTcpReports);
            Serial.print(F(")"));
            Serial.print(F(" now "));  Serial.print(now);
            Serial.print(F(" last ")); Serial.print(lastDelayedTcpReports);
            Serial.print(F(" diff ")); Serial.print(now - lastDelayedTcpReports);

            Serial.println();
            delayedTcpReports++;
            lastDelayedTcpReports = now;
            return;
        } else return;
    }
#endif    
#if defined(TFT_DISPLAY)
    tft.setCursor(0,64);
    lineCount = 0;
#endif
    for (uint8_t pinIndex=0; pinIndex < MAX_PIN; pinIndex++) {  // go through all observed pins as pinIndex
        if (pinData[pinIndex].pinName >= 0) {
            showPinCounter (pinIndex, false, now);              // report pin counters if necessary
#ifdef pulseHistory
            if (devVerbose >= 5) 
                showPinHistory(pinIndex, now);                  // show pin history if verbose >= 5
#endif          
        }
    }
    lastReportCall = now;                                       // check again after intervalMin or if intervalMax is over for a pin
}


void updateEEPROM(int &address, byte value) {
    if( EEPROM.read(address) != value){
        EEPROM.write(address, value);
    }
    address++;
}

   
void updateEEPROMSlot(int &address, char cmd, int v1, int v2, int v3, int v4) {
    updateEEPROM(address, cmd);         // I / A
    updateEEPROM(address, v1 & 0xff);       
    updateEEPROM(address, v1 >> 8);    
    updateEEPROM(address, v2 & 0xff);
    updateEEPROM(address, v2 >> 8);
    updateEEPROM(address, v3 & 0xff);
    updateEEPROM(address, v3 >> 8);
    updateEEPROM(address, v4 & 0xff);
    updateEEPROM(address, v4 >> 8);
}


void showEEPROM() {
    int address = 0;
    uint16_t v1, v2, v3, v4;
    char cmd;
    if (EEPROM.read(address) != 'C' || EEPROM.read(address+1) != 'f' || EEPROM.read(address+2) != 'g') {
        Output->println(F("M no config in EEPROM"));
        return;
    }
    address = 3;
    uint8_t slots = EEPROM.read(address++);
    if (slots > MAX_PIN + 2) {
        Output->println(F("M illegal config in EEPROM"));
        return;
    }
    Output->println();
    Output->print(F("M EEPROM Config: "));
    Output->print((char) EEPROM.read(0));
    Output->print((char) EEPROM.read(1));
    Output->print((char) EEPROM.read(2));
    Output->print(F(" Slots: "));
    Output->print((int) EEPROM.read(3));
    Output->println();
    for (uint8_t slot=0; slot < slots; slot++) {
        cmd = EEPROM.read(address);
        v1 = EEPROM.read(address+1) + (((uint16_t)EEPROM.read(address+2)) << 8);
        v2 = EEPROM.read(address+3) + (((uint16_t)EEPROM.read(address+4)) << 8);
        v3 = EEPROM.read(address+5) + (((uint16_t)EEPROM.read(address+6)) << 8);
        v4 = EEPROM.read(address+7) + (((uint16_t)EEPROM.read(address+8)) << 8);
        address = address + 9;
        Output->print(F("M Slot: "));
        Output->print(cmd); Output->print(F(" "));
        Output->print(v1);  Output->print(F(","));
        Output->print(v2);  Output->print(F(","));
        Output->print(v3);  Output->print(F(","));
        Output->print(v4);
        Output->println();
    }  
}


void restoreFromEEPROM() {
    int address = 0;  
    if (EEPROM.read(address) != 'C' || EEPROM.read(address+1) != 'f' || EEPROM.read(address+2) != 'g') {
        Serial.println(F("M no config in EEPROM"));
        return;
    }
    address = 3;
    uint8_t slots = EEPROM.read(address++);
    if (slots > MAX_PIN + 1 || slots < 1) {
        Serial.println(F("M illegal config in EEPROM"));
        return;
    }
    Serial.println(F("M restoring config from EEPROM"));
    char cmd;
    for (uint8_t slot=0; slot < slots; slot++) {
        cmd = EEPROM.read(address);
        commandData[0] = EEPROM.read(address+1) + (((uint16_t)EEPROM.read(address+2)) << 8);
        commandData[1] = EEPROM.read(address+3) + (((uint16_t)EEPROM.read(address+4)) << 8);
        commandData[2] = EEPROM.read(address+5) + (((uint16_t)EEPROM.read(address+6)) << 8);
        commandData[3] = EEPROM.read(address+7) + (((uint16_t)EEPROM.read(address+8)) << 8);
        address = address + 9;
        commandDataPointer = 4;
        if (cmd == 'I') CmdInterval(commandData, commandDataPointer);
#ifdef analogIR        
        if (cmd == 'T') CmdThreshold(commandData, commandDataPointer);
#endif        
        if (cmd == 'A') CmdAdd(commandData, commandDataPointer);
    }
    commandDataPointer = 0;
    commandValue = 0;
    for (uint8_t i=0; i < MAX_INPUT_NUM; i++)
        commandData[i] = 0;     

#if defined(TFT_DISPLAY) 
    tft.setCursor(0, 32);
    tft.print("Pin config loaded");
#endif


}

#if defined(WifiSupport)
void printConnection(Print *Out) {
    Out->print(F("M Connected to "));   Out->print(WiFi.SSID());    
    Out->print(F(" with IP "));         Out->print(WiFi.localIP());    
    Out->print(F(" RSSI "));            Out->print(WiFi.RSSI());
    Out->println();
#if defined(TFT_DISPLAY) 
    tft.setCursor(0, 0);
    tft.print(F("Conected to "));       tft.print(WiFi.SSID());
    tft.print(F("              "));
    tft.setCursor(0, 16);
    tft.print("IP ");       tft.print(WiFi.localIP());
    tft.print(F(" RSSI ")); tft.print(WiFi.RSSI());
#endif
}
#endif


/* todo: include analogPins as well as analog limits in save / restore */
void CmdSaveToEEPROM() {
    int address   = 0;
    uint8_t slots = 1;
    updateEEPROM(address, 'C');
    updateEEPROM(address, 'f');
    updateEEPROM(address, 'g');
    for (uint8_t pinIndex=0; pinIndex < MAX_PIN; pinIndex++)
        if (pinData[pinIndex].pinName >= 0) slots ++;
#ifdef analogIR     
    slots ++;
#endif  
    updateEEPROM(address, slots);                   // number of defined pins + intervall definition
    updateEEPROMSlot(address, 'I', (uint16_t)(intervalMin / 1000), (uint16_t)(intervalMax / 1000), 
                                  (uint16_t)(intervalSml / 1000), (uint16_t)countMin);
#ifdef analogIR     
    updateEEPROMSlot(address, 'T', (uint16_t)analogThresholdMin, (uint16_t)analogThresholdMax, 0, 0);
#endif
    for (uint8_t pinIndex=0; pinIndex < MAX_PIN; pinIndex++) 
        if (pinData[pinIndex].pinName >= 0)
            updateEEPROMSlot(address, 'A', (uint16_t)pinData[pinIndex].pinName, (uint16_t)(pinData[pinIndex].pulseLevel ? 3:2), 
                                                    (uint16_t)pinData[pinIndex].pullupFlag, (uint16_t)pinData[pinIndex].pulseWidthMin);
#if defined(ESP8266) || defined(ESP32)
    EEPROM.commit();               
#endif  
    Serial.print(F("config saved, "));
    Serial.print(slots);
    Serial.print(F(", "));
    Serial.println(address);
}



void CmdInterval(uint16_t *values, uint8_t size) {
    //Serial.print(F("D int ptr is ")); Serial.println(size);
    if (!checkVal(0,1,3600)) return;                // index 0 is interval min, min 1, max 3600 (1h), std error
    intervalMin = (uint32_t)values[0] * 1000;       // convert to miliseconds

    if (!checkVal(1,1,3600)) return;                // index 1 is interval max, min 1, max 3600 (1h), std error
    intervalMax = (uint32_t)values[1]* 1000;        // convert to miliseconds

    if (!checkVal(2,0,3600)) return;                // index 2 is interval small, max 3600 (1h), std error
    intervalSml = (uint32_t)values[2] * 1000;       // convert to miliseconds

    if (!checkVal(3,0,100)) return;                // index 3 is count min, max 100, std error
    countMin = values[3];

    Output->print(F("M intervals set to ")); Output->print(values[0]);
    Output->print(F(" ")); Output->print(values[1]);
    Output->print(F(" ")); Output->print(values[2]);
    Output->print(F(" ")); Output->println(values[3]);
}


#ifdef analogIR
void CmdThreshold(uint16_t *values, uint8_t size) {    
    if (!checkVal(0,1,1023)) return;               // index 0 is analog threshold min, min 1, max 1023, std error
    analogThresholdMin = (int)values[0];

    if (!checkVal(1,1,1023)) return;               // index 1 is analog threshold max, min 1, max 1023, std error
    analogThresholdMax = (int)values[1];

    Output->print(F("M analog thresholds set to ")); Output->print(values[0]);
    Output->print(F(" ")); Output->println(values[1]);
}
#endif

/*
    handle add command.
*/
void CmdAdd (uint16_t *values, uint8_t size) {
    uint32_t now = millis();
    uint8_t aPin = values[0];                   // values[0] is pin number
    uint8_t pinIndex;

    if (!checkPin (aPin)) {
        PrintPinErrorMsg(aPin);                 // illegal pin or already used for output
        return;
    }
    uint8_t rPin = digitalPinToGPIO(aPin)       // get gpio pin number for later use

    pinIndex = findPin(aPin);
    if (pinIndex == FF) {                       // not used so far
        pinIndex = maxPinIndex;
        initPinVars(pinIndex, now);
        pinData[pinIndex].pinName = aPin;             // save arduino pin number and flag this pin as active for reporting
    }

    pinData_t *config = &pinData[pinIndex]; // config entry to work with

    if (!checkVal(1,2,3)) return;               // index 1 is pulse level, min 2, max 3, std error
    config->pulseLevel = (values[1] == 3);      // 2 = falling -> pulseLevel 0, 3 = rising -> pulseLevel 1

    if (checkVal(2,0,1, false))                 // index 2 is pullup, optional, no error message if omitted
        config->pullupFlag = values[2];         // as defined
    else config->pullupFlag = 0;                // default to no pullup


    if (!checkVal(3,1,1000, false)) return;     // value 3 is min length, optional, no error message if omitted
    if (values[3] > 0)
        config->pulseWidthMin = values[3];
    else 
        config->pulseWidthMin = 2;                  
    
    /* todo: add upper and lower limits for analog pins as option here and in Fhem module */

    config->pinName = aPin;
    config->gpioPinNumber = rPin;
    config->ownIndex = pinIndex;
    maxPinIndex++;                              // increment used entries

#ifdef analogIR
    if (!analogPins[pinIndex]) {
#endif      
        if (config->pullupFlag)
            pinMode (rPin, INPUT_PULLUP);
        else 
            pinMode (rPin, INPUT); 
#if defined(ESP8266) || defined(ESP32)
        attachInterruptArg(digitalPinToInterrupt(rPin), ESPISR, &pinData[pinIndex].ownIndex, CHANGE);
#elif defined(__AVR_ATmega328P__)        
        AddPinChangeInterrupt(pinIndex);
        pinIndexMap[aPin] = pinIndex;
#endif        
#ifdef analogIR      
    } else {
        pinMode (rPin, INPUT);  
    }
#endif    
    Output->print(F("M defined ")); showPinConfig(pinIndex); Output->println();
}


/*
    handle rem command.
*/
void CmdRemove(uint16_t *values, uint8_t size) {
    uint8_t aPin = values[0];                   // values[0] is pin number
    uint8_t pinIndex;

    if (!checkPin (aPin)) {
        PrintPinErrorMsg(aPin);                 // illegal pin or already used for output
        return;
    }
    pinIndex = findPin(aPin);
    if (pinIndex == FF) return;                 // pin is unused
    pinData_t *config = &pinData[pinIndex];     // config entry to work with

    if (!config->analogFlag) {
#if defined(ESP8266) || defined(ESP32)
        detachInterrupt(digitalPinToInterrupt(digitalPinToGPIO(aPin););
#elif defined(__AVR_ATmega328P__)
        if (!RemovePinChangeInterrupt(pinIndex)) {      
            PrintErrorMsg(); 
            Output->println(F("RemInt"));
            return;
        }
        pinIndexMap[aPin] = FF;
#endif
    }
    initPinVars(pinIndex, 0);
    Output->print(F("M removed ")); Output->println(aPin);

    // clear entry and if pinIndex in the middle was removed, move the following entries up ...
}


/* give status report in between if requested over serial input */
void CmdShow() {
    uint32_t now = millis();  
    Output->print(F("M Status: "));
    printVersionMsg();
    Output->println();
    
#if defined(WifiSupport)
    printConnection(Output);
#endif    

    showIntervals();    
#ifdef analogIR 
    showThresholds();
#endif
    Output->print(F("V"));
    Output->println(devVerbose);
    
    for (uint8_t pinIndex=0; pinIndex < MAX_PIN; pinIndex++) {
        if (pinData[pinIndex].pinName >= 0) {
            showPinConfig(pinIndex);
            Output->print(F(", "));
            showPinCounter(pinIndex, true, now);
#ifdef pulseHistory             
            showPinHistory(pinIndex, now);
#endif          
        }
    }
    showEEPROM();
    Output->print(F("M Next report in "));
    Output->print(lastReportCall + intervalMin - millis());
    Output->println(F(" milliseconds"));
}


void CmdHello() {
    uint32_t now = millis();
    Output->println();
    printVersionMsg();
    Output->print(F(" Hello, pins "));
    boolean first = true;
    for (uint8_t aPin=0; aPin < MAX_APIN; aPin++) {
        if (digitalPinToGPIO(aPin) != FF) {
            if (!first) {
                Output->print(F(","));
            } else {
                first = false;
            }
            Output->print(aPin);
        }
    }
    Output->print(F(" available"));
    Output->print(F(" T")); Output->print(now);
    Output->print(F(","));  Output->print(millisWraps);
    Output->print(F(" B")); Output->print(bootTime);
    Output->print(F(","));  Output->print(bootWraps);
    
    Output->println();
    showIntervals();
#ifdef analogIR      
    showThresholds();
#endif    
    Output->print(F("V"));
    Output->println(devVerbose);
    
    for (uint8_t pinIndex=0; pinIndex < MAX_PIN; pinIndex++) { // go through all observed pins as pinIndex
        if (pinData[pinIndex].pinName >= 0) {
            showPinConfig(pinIndex);
            Output->println();
        }
    }
}


#ifdef analogIR  
void CmdWait(uint16_t *values, uint8_t size) {
    if (!checkVal(0,0,10000)) return;
    analogReadInterval = (int)commandData[0];

    if (!checkVal(1,1,100,false)) return;
    analogReadNumOff = (uint8_t)commandData[1];

    if (!checkVal(2,1,100,false)) return;
    analogReadNumOn = (uint8_t)commandData[2];
}
#endif


void CmdDevVerbose(uint16_t *values, uint8_t size) {    
    if (!checkVal(0,0,50)) return;                // index 0 is devVerbose level, max 50, std error
    devVerbose = values[0];
    Output->print(F("M devVerbose set to ")); Output->println(values[0]); 
}
            
            
void CmdKeepAlive(uint16_t *values, uint8_t size) {

    if (values[0] == 1 && size > 0) {
        Output->print(F("alive"));
#ifdef WifiSupport
        uint32_t now = millis();
        if (devVerbose >=5) {
            Output->print(F(" RSSI ")); Output->print(WiFi.RSSI());
        }
    
        if (values[0] == 1 && size > 0 && size < 3 && Client1.connected()) {
            tcpMode = true;
            if (size == 2) {
                keepAliveTimeout = values[1];   // timeout in seconds (on ESP side we use it times 3)
            } else {
                keepAliveTimeout = 200;         // *3*1000 gives 10 minutes if nothing sent (should not happen)
            }
        }  
        lastKeepAlive = now;
#endif
        Output->println();
    }
}


#ifdef WifiSupport
void CmdQuit() {
    if (Client1.connected()) {
        Client1.println(F("closing connection"));
        Client1.stop();
        tcpMode =  false;
        Serial.println(F("M TCP connection closed"));
    } else {
        Serial.println(F("M TCP not connected"));
    }
}
#endif


/* theoretical issue: when connected via Wifi and serial at the same time, input might overlap */
void handleInput(char c) {
    if (c == ',') {                       // Komma input, last value is finished
        if (commandDataPointer < (MAX_INPUT_NUM - 1)) {
            commandData[commandDataPointer++] = commandValue;
            commandValue = 0;
        }
    }
    else if ('0' <= c && c <= '9') {      // digit input
        commandValue = 10 * commandValue + c - '0';
        commandDataSize = commandDataPointer + 1;
    }
    else if ('a' <= c && c <= 'z') {      // letter input is command
        commandLetter = c;
        if (commandDataPointer < (MAX_INPUT_NUM - 1)) {
            commandData[commandDataPointer] = commandValue;    
        }
    
        if (devVerbose > 0) {
            Serial.print(F("D got "));
            for (short v = 0; v <= commandDataPointer; v++) {          
                if (v > 0) Serial.print(F(","));
                Serial.print(commandData[v]);
            }
            Serial.print(c);
            Serial.print(F(" size "));
            Serial.println(commandDataSize);
        }

        switch (c) {
        case 'a':                       // add a pin
            CmdAdd(commandData, commandDataSize); break;
        case 'd':                       // delete a pin
            CmdRemove(commandData, commandDataSize); break;
        case 'e':                       // save to EEPROM
            CmdSaveToEEPROM(); break; 
        case 'f':                       // flash ota
            break; 
        case 'h':                       // hello
            CmdHello(); break;
        case 'i':                       // interval
            CmdInterval(commandData, commandDataSize); break;
        case 'k':                       // keep alive
            CmdKeepAlive(commandData, commandDataSize); break;   
        // o should be ?
        // p should be pin config: analog led, analog input, monitor out led
        
        case 'r':                       // reset
            initialize(); break;
        case 's':                       // show
            CmdShow(); break;
        case 'v':                       // dev verbose
            CmdDevVerbose(commandData, commandDataSize); break;
#ifdef WifiSupport      
        case 'q':                       // quit
            CmdQuit(); break; 
#endif
#ifdef analogIR            
        case 't':                       // thresholds for analog pin - 
            CmdThreshold(commandData, commandDataSize); break;
        case 'w':                       // wait - delay between analog reads and other analog read parameters
            CmdWait(commandData, commandDataSize); break;
#endif            
        default:
            break;
        }
        clearInput();
        //Serial.println(F("D End of command"));
    }
}


#ifdef debugPins
void debugPinChanges() {
    for (uint8_t pinIndex=0; pinIndex < MAX_PIN; pinIndex++) {
        short aPin = pinData[pinIndex].pinName;
        if (aPin > 0) {
            uint8_t rPin = internalPins[pinIndex];
            uint8_t pinState = digitalRead(rPin);
                       
            if (pinState != lastState[pinIndex]) {
                lastState[pinIndex] = pinState;
                Output->print(F("M pin "));       Output->print(aPin);
                Output->print(F(" (internal "));  Output->print(rPin);
                Output->print(F(")")); 
                Output->print(F(" changed to ")); Output->print(pinState);
#ifdef pulseHistory                     
                Output->print(F(", histIdx "));   Output->print(histIndex);
#endif                  
                Output->print(F(", count "));     Output->print(counter[pinIndex]);
                Output->print(F(", reject "));    Output->print(rejectCounter[pinIndex]);
                Output->println();
            }
        }
    }
}
#endif


#ifdef WifiSupport    

void showWifiStatus() {
    Serial.print(F("M Status is "));
    switch (WiFi.status()) {
        case WL_CONNECT_FAILED: 
        Serial.println(F("Connect Failed")); break;
        case WL_CONNECTION_LOST: 
        Serial.println(F("Connection Lost")); break;
        case WL_DISCONNECTED: 
        Serial.println(F("Disconnected")); break;
        case WL_CONNECTED: 
        Serial.println(F("Connected")); break;
        default:
        Serial.println(WiFi.status());
    }
}


#if !defined(STATIC_WIFI)
void configModeCallback (WiFiManager *myWiFiManager) {
    Serial.println("Entered config mode");
    Serial.println(WiFi.softAPIP());
    //if you used auto generated SSID, print it
    Serial.println(myWiFiManager->getConfigPortalSSID());
#if defined(TFT_DISPLAY)    
    tft.setCursor(0,0);
    tft.print(F("Entered config mode      "));
    tft.setCursor(0, 16);
    tft.print(myWiFiManager->getConfigPortalSSID());
#endif
}
#endif


void connectWiFi() {
    Client1Connected = false;
    Client2Connected = false;
    WiFi.mode(WIFI_STA);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    delay (1000);    
    if (WiFi.status() != WL_CONNECTED) {
#if defined(STATIC_WIFI)
#if defined(TFT_DISPLAY) 
        tft.setCursor(0,0);
        tft.print(F("Conecting WiFi to ")); tft.print(ssid);
#endif
        uint8_t counter = 0;
        Serial.print(F("M Connecting WiFi to ")); Serial.println(ssid);
        WiFi.begin(ssid, password);                 // connect with compiled strings
        while (WiFi.status() != WL_CONNECTED) {
            showWifiStatus();
            delay(1000);
            counter++;
            if (counter > 2) {
#if defined(TFT_DISPLAY)    
                tft.setCursor(0,0);
                tft.print(F("Retry conecting WiFi to")); tft.print(ssid);
#endif
                Serial.println(F("M Retry connecting WiFi"));
                WiFi.begin(ssid, password);         // restart connecting
                delay (1000);
                counter = 0;                        // do forever until connected with retries
            }
        }    
#else                                               // connect using WifiManager if auto reconnect not successful
#if defined(TFT_DISPLAY) 
        tft.setCursor(0,0);
        tft.print(F("try reconecting WiFi"));
#endif
        Serial.print(F("M Try reconnecting WiFi"));
        WiFi.begin();             
        delay(1000);
        showWifiStatus();
        if (WiFi.status() != WL_CONNECTED) {
#if defined(TFT_DISPLAY)    
            tft.setCursor(0,0);
            tft.print(F("Retry reconecting WiFi"));
#endif
            Serial.println(F("M Retry reconnecting WiFi"));
            WiFi.begin();         
            delay (1000);
        }    
        WiFiManager wifiManager;
        //wifiManager.resetSettings();                // for testing

        wifiManager.setAPCallback(configModeCallback);  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
        wifiManager.autoConnect();
#endif
    }
    printConnection (&Serial);
    Server.begin();                     // Start the TCP server
    Serial.println(F("M Server started"));
}


void handleConnections() { 
    IPAddress remote;   
    uint32_t now = millis();
    
    if (Client1Connected) {
        if((now - lastKeepAlive) > (keepAliveTimeout * 3000)) {
            Serial.println(F("M no keepalive from Client - disconnecting"));
            Client1.stop();
        }
    }    
    if (Client1.available()) {
        handleInput(Client1.read());
        //Serial.println(F("M new Input over TCP"));
    }
    if (Client1.connected()) {
        Client2 = Server.available();
        if (Client2) {
            Client2.println(F("connection already busy"));
            remote = Client2.remoteIP();
            Client2.stop();
            Serial.print(F("M second connection from ")); Serial.print(remote);
            Serial.println(F(" rejected"));
        }
    } else {
        if (Client1Connected) {                                 // client used to be connected, now disconnected
            Client1Connected = false;
            Output = &Serial;
            Serial.println(F("M connection to client lost"));
        }
        Client1 = Server.available();
        if (Client1) {                                          // accepting new connection
            remote = Client1.remoteIP();
            Serial.print(F("M new connection from ")); Serial.print(remote);
            Serial.println(F(" accepted"));
            Client1Connected = true;
            Output = &Client1;
            lastKeepAlive = now;
            CmdHello();                                         // say hello to client
        }
    }
} 
#endif


void handleTime() {
    uint32_t now = millis();
    if (now < lastMillis) millisWraps++;
    lastMillis = now;
}


#ifdef analogIR 
void detectTrigger(analogData_t *ad, int val) {
    uint8_t nextState = ad->triggerState;       // set next trigger level to be the same as the old one 
    if (val > analogThresholdMax) {
        nextState = 1;                      // if above upper threshold then 1
    } else if (val < analogThresholdMin) {
        nextState = 0;                      // if below lower threshold then 0
    }                                       // otherwise it stays as old level
    ad->triggerState = nextState;               // save new level
    
    if (ledOutPin)
        digitalWrite(ledOutPin, triggerState);

    short pinIndex = allowedPins[analogInPin];  // ESP pin A0 (pinIndex 8, internal 17) or Arduino A7 (pinIndex 17, internal 21)
    uint32_t now = millis();
    doCount (pinIndex, triggerState, now);      // do the counting, history and so on
    
#ifdef debugPins
    if (devVerbose >= 10) {
        short rPin = internalPins[pinIndex];
        Output->print(F("M pin "));      Output->print(analogInPin);
        Output->print(F(" (index "));    Output->print(pinIndex);
        Output->print(F(", internal ")); Output->print(rPin);
        Output->print(F(" ) "));
        Output->print(F(" to "));        Output->print(nextState);
#ifdef pulseHistory                     
        Output->print(F("  histIdx "));  Output->print(histIndex);
#endif                  
        Output->print(F("  count "));    Output->print(counter[pinIndex]);
        Output->print(F("  reject "));   Output->print(rejectCounter[pinIndex]);
        Output->println();
    }
#endif
}


void readAnalog() {
    uint32_t now = millis();
    uint16_t interval2 = analogReadInterval + 1;            // last read + interval + 2ms -> read off value (state 2)
    uint16_t interval3 = analogReadInterval + 2;            // last read + interval + 4ms -> read on value (state 5)
    if ((now - lastAnalogRead) > analogReadInterval) {      // time for next analog read?
        switch (analogReadState) {
            case 0:                                                             // initial state
                for (uint8_t analogIndex = 0; analogIndex < maxAnalogIndex; analogIndex++) {
                    analogData_t ad* = &analogData[analogIndex];
                    digitalWrite(digitalPinToGPIO(ad->outPinName) , LOW);       // make sure IR LED is off for first measurement
                }
                readCnt = 0; sumOff = 0; sumOn = 0;
                analogReadState = 1;
                break;
            case 1:                                                             // wait before measuring
                if ((now - lastAnalogRead) < interval2)
                    return;
                analogReadState = 2;
                break;
            case 2:
                for (uint8_t analogIndex = 0; analogIndex < maxAnalogIndex; analogIndex++) {
                    analogData_t ad* = &analogData[analogIndex];
                    ad->sumOff += analogRead(digitalPinToGPIO(ad->inPinName));  // read the analog in value (off)
                }
                if (++readCnt < readNum)
                    break;
                for (uint8_t analogIndex = 0; analogIndex < maxAnalogIndex; analogIndex++) {
                    analogData_t ad* = &analogData[analogIndex];
                    digitalWrite(digitalPinToGPIO(ad->outPinName) , HIGH);      // turn IR LED on
                }
                readCnt = 0;
                analogReadState = 4;
                break;
            case 4:                                                             // wait again before measuring
                if ((now - lastAnalogRead) < interval3)
                    return;
                analogReadState = 5;
                break;
            case 5:
                int sensorDiff;
                for (uint8_t analogIndex = 0; analogIndex < maxAnalogIndex; analogIndex++) {
                    analogData_t ad* = &analogData[analogIndex];
                    ad->sumOn += analogRead(digitalPinToGPIO(ad->inPinName));   // read the analog in value (on)
                }
                if (++readCnt < readNum) 
                    break;
                for (uint8_t analogIndex = 0; analogIndex < maxAnalogIndex; analogIndex++) {
                    analogData_t ad* = &analogData[analogIndex];
                    digitalWrite(digitalPinToGPIO(ad->outPinName) , LOW);       // turn IR LED off again
                
                    sensorDiff = (ad->sumOn / readNum) - (ad->sumOff / readNum);
                    if (sensorDiff < 0) sensorDiff = 0;
                    if (sensorDiff > 4096) sensorDiff = 4096;
                    detectTrigger (ad, sensorDiff);                    // calculate level with triggers
                    if (devVerbose >= 25) {
                        char line[26];
                        sprintf(line, "L%2d: %4d, %4d -> % 4d", sensorSumOn / analogReadNumOn, 
                            sensorSumOff / analogReadNumOff, sensorDiff);
                        Output->println(line);
                    } else if (devVerbose >= 20) {
                        Output->print(F("L%2d: "));
                        Output->println(sensorDiff);
                    }                    
#if defined(TFT_DISPLAY)
                    int len = sensorDiff * analogReadAmp * TFT_HEIGHT / 4096;
                    tft.fillRect(0,TFT_WIDTH-10-(10*analogIndex),len,10, TFT_YELLOW);
                    tft.fillRect(len,TFT_WIDTH-10-(10*analogIndex),TFT_HEIGHT-len,10, TFT_BLACK);
#endif
                }
                analogReadState = 0;
                lastAnalogRead = now;
                break;
            default:
                analogReadState = 0;
                Output->println(F("D error: wrong analog read state"));
                break;
        }
    }
}
#endif


void setup() {
#ifdef TFT_DISPLAY    
    tft.init();
    tft.fillScreen(TFT_BLACK);
    tft.setRotation(1);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
#endif    

    Serial.begin(SERIAL_SPEED);             // initialize serial
#if defined(ESP8266) || defined (ESP32)
    EEPROM.begin(100);
#endif    
    delay (500);
    interrupts();    
    Serial.println();        
    Output = &Serial;    
    millisWraps = 0;
    lastMillis = millis();
    initialize();     
#ifdef analogIR
    pinMode(irOutPin, OUTPUT);
    if (ledOutPin)
      pinMode(ledOutPin, OUTPUT);
#endif  
    CmdHello();                             // started message to serial
#ifdef WifiSupport
    connectWiFi();
#endif
}


/*   Main Loop  */
void loop() {
    handleTime();                           // check if millis() wrapped (for reporting)
    if (Serial.available()) 
        handleInput(Serial.read());         // input over serial 
#ifdef WifiSupport    
    handleConnections();                    // new TCP connection or input over TCP
#endif
#ifdef analogIR
    readAnalog();                           // analog measurements
#endif
#ifdef debugPins
    if (devVerbose >= 10) {
        debugPinChanges();
    }
#endif
    if (reportDue()) {    
        report();                           // report counts
    }
}
