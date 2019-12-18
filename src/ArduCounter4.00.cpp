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
                

    ToDo / Ideas:
        make analogInterval available in Fhem
        save analogInterval to Flash
        detect analog Threasholds automatically and adjust over time
        make wifi ssid / secret configurable
        restructure ISRs // see https://github.com/arduino/Arduino/pull/4519

    
*/ 
#include <Arduino.h>

#define TTGO_DISPLAY 1
#ifdef TTGO_DISPLAY
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
#include <Button2.h>
#include "esp_adc_cal.h"
/*#include "bmp.h"*/

TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h
uint8_t lineCount;

#endif

/* Remove this before compiling */
/* todo: put this in ENV in PlatformIO as another target  */
#define TestConfig // include my SSID / secret


/* allow printing of every pin change to Serial */
#define debugPins 1  

/* allow tracking of pulse lengths */
#define pulseHistory 1

/* support analog input for ferraris counters with IR light hardware */
#define analogIR 1

#include "pins_arduino.h"
#include <EEPROM.h>

const char versionStr[] PROGMEM = "ArduCounter V4.00";

#define SERIAL_SPEED 38400
#define MAX_INPUT_NUM 8

#if defined(ESP8266) || defined(ESP32)  // Wifi stuff
#define WifiSupport 1
#if defined(ESP8266)
#include <ESP8266WiFi.h>          
#elif defined(ESP32)
#include <WiFi.h>          
#endif

#ifdef TestConfig
#include "ArduCounterTestConfig.h"
#else
const char* ssid = "MySSID";
const char* password = "secret";
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
short allowedPins[MAX_APIN] =       // ESP 8266 with analog:
  { 0,  1,  2, -1,                  // printed pin numbers 0,1,2 are ok to be used
   -1,  5, -1, -1,                  // printed pin number 5 is ok to be used (6 and 7 are used for analog counting)
   -1, -1, -1, -1,                  // 8-11 not avaliable 
   -1, -1, -1, -1,                  // 12-15 not avaliable
   -1,  8 };                        // 16 not available, 17 is analog (at internal index 8)
     
/* Wemos / NodeMCU Pins 3,4 and 8 (GPIO 0,2 and 15) define boot mode and therefore
 * can not be used to connect to signal */

/* Map from sketch internal pin index to real chip IO pin number (not aPin, e.g. for ESP)
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

#define MAX_HIST 20                 // 20 history entries for ESP boards (can be increased)
#define MAX_APIN 40
#define MAX_PIN 40

/* ESP32 pins that are typically ok to use 
 * (some might be set to -1 (disallowed) because they are used 
 * as reset, serial, led or other things on most boards) 
 * maps printed pin numbers (aPin) to sketch internal index numbers */
#if defined(TTGO_DISPLAY)
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
   -1, -1, -1, 39};                 // 36 is A0, is 39 avaliable but also input only

const int irOutPin  = 27;           // Digital output pin that the IR-LED is attached to

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
   -1, -1, -1, 39};                 // 36 is A0, is 39 avaliable but also input only
const int irOutPin  = 4;            // Digital output pin that the IR-LED is attached to
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
        
const int analogInPin = A0;         // Analog input pin that the photo transistor is attached to (internally number _)
const int ledOutPin = 2;            // on board LED output pin
        
        
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

uint8_t analogPins[MAX_PIN] = 
  { 0,0,0,0,            /* everything except Arduino A7 (pinIndex 17, internal 21) is digital by default */
    0,0,0,0,
    0,0,0,0,
    0,0,0,0,
    0,1 };

const int analogInPin = A7;     // Arduino analog input pin that the photo transistor is attached to (internal 21)
const int irOutPin = 2;         // Digital output pin that the IR-LED is attached to
const int ledOutPin = 12;       // Signal LED output pin
   
   
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
uint8_t firstPin[] = {8, 14, 0};    // aPin -> allowedPins[] -> pinIndex
uint8_t lastPin[]  = {13, 19, 7};

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
#endif

uint32_t intervalMin = 30000;       // default 30 sec - report after this time if nothing else delays it
uint32_t intervalMax = 60000;       // default 60 sec - report after this time if it didin't happen before
uint32_t intervalSml =  2000;       // default 2 secs - continue count if timeDiff is less and intervalMax not over
uint16_t countMin    =     2;       // continue counting if count is less than this and intervalMax not over

uint32_t lastReportCall;

/* index to the following arrays is the internal pin index number  */

volatile boolean initialized[MAX_PIN];          // did we get first interrupt yet? 
short activePin[MAX_PIN];                       // printed arduino pin number for index if active - otherwise -1
uint16_t pulseWidthMin[MAX_PIN];                // minimal pulse length in millis for filtering
uint8_t pulseLevel[MAX_PIN];                    // start of pulse for measuring length - 0 / 1 as defined for each pin
uint8_t pullup[MAX_PIN];                        // pullup configuration state
 
volatile uint32_t counter[MAX_PIN];             // real pulse counter
volatile uint8_t counterIgn[MAX_PIN];           // ignored first pulse after init
volatile uint16_t rejectCounter[MAX_PIN];       // counter for rejected pulses that are shorter than pulseWidthMin
uint32_t lastCount[MAX_PIN];                    // counter at last report (to get the delta count)
uint16_t lastRejCount[MAX_PIN];                 // reject counter at last report (to get the delta count)

volatile uint32_t lastChange[MAX_PIN];          // millis at last level change (for measuring pulse length)
volatile uint8_t lastLevel[MAX_PIN];            // level of input at last interrupt
volatile uint8_t lastLongLevel[MAX_PIN];        // last level that was longer than pulseWidthMin

volatile uint32_t pulseWidthSum[MAX_PIN];       // sum of pulse lengths for average calculation
uint8_t reportSequence[MAX_PIN];                // sequence number for reports


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

volatile uint32_t intervalStart[MAX_PIN];       // start of an interval - typically set by first / last pulse
volatile uint32_t intervalEnd[MAX_PIN];         // end of an interval - typically set by first / last pulse
uint32_t lastReport[MAX_PIN];                   // millis at last report to find out when maxInterval is over

uint16_t commandData[MAX_INPUT_NUM];            // input data over serial port or network
uint8_t  commandDataPointer = 0;                // index pointer to next input value
uint16_t value;                                 // the current value for input function

#ifdef analogIR
int sensorValueOff = 0;             // value read from the photo transistor when ir LED is off
int sensorValueOn  = 0;             // value read from the photo transistor when ir LED is on
int analogThresholdMin = 100;       // min value of analog input 
int analogThresholdMax = 110;       // max value of analog input
uint32_t lastAnalogRead;            // millis() at last analog read
uint16_t analogReadInterval = 20;   // interval at which to read analog values
uint8_t  analogReadState = 0;       // to keep track of switching LED on/off

uint8_t triggerState;               // todo: use existing arrays instead

// idea: save analog measurement during same level as sum and count to get average and then put in history when doCount is called
// but how do we do this before we can detect the levels?
#endif


void initPinVars(short pinIndex, uint32_t now) {
    uint8_t level = 0;
    activePin[pinIndex]      = -1;          // inactive (-1)
    initialized[pinIndex]    = false;       // no pulse seen yet
    pulseWidthMin[pinIndex]  = 0;           // min pulse length
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
#ifdef debugPins      
    lastState[pinIndex]      = level;      // for debug output
#endif
    /* todo: add upper and lower thresholds for analog */
}


void PrintErrorMsg() {
    Output->print(F("Error: "));
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
    if (len < pulseWidthMin[pinIndex]) {                    // pulse was too short
        lastChange[pinIndex] = now;
        if (lastLevel[pinIndex] == pulseLevel[pinIndex]) {  // if change to gap level
            rejectCounter[pinIndex]++;                      // inc reject counter and set action to R (pulse too short)
            act = 'R';
        } else {
            act = 'X';                                      // set action to X (gap too short)
        }
    } else {
        if (lastLevel[pinIndex] != pulseLevel[pinIndex]) {  // edge does fit defined pulse start, level is now pulse, before it was gap
            act = 'G';                                      // now the gap is confirmed (even if inbetween was a spike that we ignored)
        } else {                                            // edge is a change to gap, level is now gap
            if (lastLongLevel[pinIndex] != pulseLevel[pinIndex]) { // last remembered valid level was also gap -> now we had valid new pulse -> count
                counter[pinIndex]++;                        // count
                intervalEnd[pinIndex] = now;                // remember time of in case pulse will be the last in the interval
                if (!initialized[pinIndex]) {
                    intervalStart[pinIndex] = now;          // if this is the very first impulse on this pin -> start interval now
                    initialized[pinIndex] = true;           // and start counting the next impulse (so far counter is 0)
                    counterIgn[pinIndex]++;                 // count as to be ignored for diff because it defines the start of the interval
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


/* Interrupt handlers and their installation 
 *  on Arduino and ESP8266 platforms
 */

#if defined(__AVR_ATmega328P__)
/* Add a pin to be handled (Arduino code) */
uint8_t AddPinChangeInterrupt(uint8_t rPin) {
    volatile uint8_t *pcmask;                   // pointer to PCMSK0 or 1 or 2 depending on the port corresponding to the pin
    uint8_t bitM = digitalPinToBitMask(rPin);   // mask to bit in PCMSK to enable pin change interrupt for this arduino pin 
    uint8_t port = digitalPinToPort(rPin);      // port that this arduno pin belongs to for enabling interrupts
    if (port == NOT_A_PORT) 
        return 0;
    port -= 2;                                  // from port (PB, PC, PD) to index in our array
    pcmask = port_to_pcmask[port];              // point to PCMSK0 or 1 or 2 depending on the port corresponding to the pin
    *pcmask |= bitM;                            // set the pin change interrupt mask through a pointer to PCMSK0 or 1 or 2 
    PCICR |= 0x01 << port;                      // enable the interrupt
    return 1;
}


/* Remove a pin to be handled (Arduino code) */
uint8_t RemovePinChangeInterrupt(uint8_t rPin) {
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
   common function for arduino pin change interrupt handlers. "port" is the PCINT port index (0-2) as passed from above, not PB, PC or PD which are mapped to 2-4
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

    bit = 0x01;                                                 // start mit rightmost (least significant) bit in a port
    for (uint8_t aPin = firstPin[port]; aPin <= lastPin[port]; aPin++) { // loop over each pin on the given port that changed
        if (delta & bit) {                                      // did this pin change?
            pinIndex = allowedPins[aPin];
            if (pinIndex > 0) {                                 // shound not be necessary but test anyway
                doCount (pinIndex, ((curr & bit) > 0), now);    // do the counting, history and so on
            }
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

#elif defined(ESP8266)
/* Add a pin to be handled (ESP8266 code) */

/* attachInterrupt needs to be given an individual function for each interrrupt .
 *  since we cant pass the pin value into the ISR or we could use an 
 *  internal function __attachInnterruptArg ... but then we need a fixed reference for the pin numbers ...
*/

void ICACHE_RAM_ATTR ESPISR4() {    // ISR for real pin GPIO 4 / pinIndex 2
    doCount(2, digitalRead(4), millis());
    // called with pinIndex, level, now
}

void ICACHE_RAM_ATTR ESPISR5() {    // ISR for real pin GPIO 5 / pinIndex 1
    doCount(1, digitalRead(5), millis());
}

void ICACHE_RAM_ATTR ESPISR12() {   // ISR for real pin GPIO 12 / pinIndex 6
    doCount(6, digitalRead(12), millis());
}

void ICACHE_RAM_ATTR ESPISR13() {   // ISR for real pin GPIO 13 / pinIndex 7
    doCount(7, digitalRead(13), millis());
}

void ICACHE_RAM_ATTR ESPISR14() {// ISR for real pin GPIO 14 / pinIndex 5
    doCount(5, digitalRead(14), millis());
}

void ICACHE_RAM_ATTR ESPISR16() {   // ISR for real pin GPIO 16 / pinIndex 0
    doCount(0, digitalRead(16), millis());
}

uint8_t AddPinChangeInterrupt(uint8_t rPin) {
    switch(rPin) {
    case 4:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR4, CHANGE);
        break;
    case 5:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR5, CHANGE);
        break;
    case 12:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR12, CHANGE);
        break;
    case 13:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR13, CHANGE);
        break;
    case 14:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR14, CHANGE);
        break;
    case 16:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR16, CHANGE);
        break;
    default:
        PrintErrorMsg(); Output->println(F("illegal pin number in attachInterrupt"));
    }
    return 1;
}

#elif defined(ESP32)

void IRAM_ATTR ESPISR16() {   // ISR for real pin GPIO 16 / pinIndex 16
    doCount(16, digitalRead(16), millis());
    // called with pinIndex, level, now
}

void IRAM_ATTR ESPISR17() {   // ISR for real pin GPIO 17 / pinIndex 17
    doCount(17, digitalRead(17), millis());
    // called with pinIndex, level, now
}

void IRAM_ATTR ESPISR18() {   // ISR for real pin GPIO 18 / pinIndex 18
    doCount(18, digitalRead(18), millis());
    // called with pinIndex, level, now
}

void IRAM_ATTR ESPISR19() {   // ISR for real pin GPIO 19 / pinIndex 19
    doCount(19, digitalRead(19), millis());
    // called with pinIndex, level, now
}

void IRAM_ATTR ESPISR21() {   // ISR for real pin GPIO 21 / pinIndex 21
    doCount(21, digitalRead(21), millis());
    // called with pinIndex, level, now
}

void IRAM_ATTR ESPISR22() {   // ISR for real pin GPIO 22 / pinIndex 22
    doCount(22, digitalRead(22), millis());
    // called with pinIndex, level, now
}

void IRAM_ATTR ESPISR23() {   // ISR for real pin GPIO 23 / pinIndex 23
    doCount(23, digitalRead(23), millis());
    // called with pinIndex, level, now
}

void IRAM_ATTR ESPISR25() {   // ISR for real pin GPIO 25 / pinIndex 25
    doCount(25, digitalRead(25), millis());
    // called with pinIndex, level, now
}

void IRAM_ATTR ESPISR26() {   // ISR for real pin GPIO 26 / pinIndex 26
    doCount(26, digitalRead(26), millis());
    // called with pinIndex, level, now
}

void IRAM_ATTR ESPISR27() {   // ISR for real pin GPIO 27 / pinIndex 27
    doCount(27, digitalRead(27), millis());
    // called with pinIndex, level, now
}

void IRAM_ATTR ESPISR32() {   // ISR for real pin GPIO 32 / pinIndex 32
    doCount(32, digitalRead(32), millis());
    // called with pinIndex, level, now
}

void IRAM_ATTR ESPISR33() {   // ISR for real pin GPIO 33 / pinIndex 33
    doCount(33, digitalRead(33), millis());
    // called with pinIndex, level, now
}

void IRAM_ATTR ESPISR34() {   // ISR for real pin GPIO 34 / pinIndex 34
    doCount(34, digitalRead(34), millis());
    // called with pinIndex, level, now
}

void IRAM_ATTR ESPISR35() {   // ISR for real pin GPIO 35 / pinIndex 35
    doCount(35, digitalRead(35), millis());
    // called with pinIndex, level, now
}

void IRAM_ATTR ESPISR36() {   // ISR for real pin GPIO 36 / pinIndex 36
    doCount(36, digitalRead(36), millis());
    // called with pinIndex, level, now
}

void IRAM_ATTR ESPISR39() {   // ISR for real pin GPIO 39 / pinIndex 39
    doCount(39, digitalRead(39), millis());
    // called with pinIndex, level, now
}


uint8_t AddPinChangeInterrupt(uint8_t rPin) {
    switch(rPin) {
    case 16:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR16, CHANGE);
        break;
    case 17:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR17, CHANGE);
        break;
    case 18:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR18, CHANGE);
        break;
    case 19:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR19, CHANGE);
        break;
    case 21:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR21, CHANGE);
        break;
    case 22:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR22, CHANGE);
        break;
    case 23:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR23, CHANGE);
        break;
    case 25:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR25, CHANGE);
        break;
    case 26:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR26, CHANGE);
        break;
    case 27:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR27, CHANGE);
        break;
    case 32:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR32, CHANGE);
        break;
    case 33:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR33, CHANGE);
        break;
    case 34:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR34, CHANGE);
        break;
    case 35:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR35, CHANGE);
        break;
    case 36:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR36, CHANGE);
        break;
    case 39:
        attachInterrupt(digitalPinToInterrupt(rPin), ESPISR39, CHANGE);
        break;
    default:
        PrintErrorMsg(); Output->println(F("illegal pin number in attachInterrupt"));
    }
    return 1;
}
#endif


void showIntervals() {
    Output->print(F("I"));
    Output->print(intervalMin / 1000);
    Output->print(F(" "));
    Output->print(intervalMax / 1000);
    Output->print(F(" "));
    Output->print(intervalSml / 1000);
    Output->print(F(" "));
    Output->println(countMin);
}


#ifdef analogIR 
void showThresholds() {
    Output->print(F("T"));
    Output->print(analogThresholdMin);
    Output->print(F(" "));
    Output->println(analogThresholdMax);
}
#endif


void showPinConfig(short pinIndex) {
    Output->print(F("P"));
    Output->print(activePin[pinIndex]);
    switch (pulseLevel[pinIndex]) {
        case 1:  Output->print(F(" rising")); break;
        case 0: Output->print(F(" falling")); break;
        default: Output->print(F(" -")); break;
    }        
    if (pullup[pinIndex]) 
        Output->print(F(" pullup"));
    Output->print(F(" min "));
    Output->print(pulseWidthMin[pinIndex]);
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
    
    Output->print (F("H"));                     // start with H
    Output->print (activePin[pinIndex]);        // printed pin number
    Output->print (F(" "));
    for (uint8_t i = 0; i < MAX_HIST; i++) {
        hi = (start + i) % MAX_HIST;
        if (histPin[hi] == pinIndex) {
            if (first || (last <= histTime[hi]+histLen[hi])) {
                if (!first) Output->print (F(", "));
                        Output->print (histSeq[hi]);            // sequence
                        Output->print (F("s"));                         
                Output->print ((long) (histTime[hi] - now));    // time when level started
                Output->print (F("/"));                         
                Output->print (histLen[hi]);                    // length 
                Output->print (F("@"));                         
                Output->print (histLevel[hi]);                  // level (0/1)
                Output->print (histAct[hi]);                    // action
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
    Output->print(F("R"));                          // R Report
    Output->print(activePin[pinIndex]);
    Output->print(F(" C"));                         // C - Count
    Output->print(count);
    Output->print(F(" D"));                         // D - Count Diff (without pulse that marks the begin)
    Output->print(countDiff);
    Output->print(F("/"));                          // R - real Diff for long counter - includes first after restart
    Output->print(realDiff);
    Output->print(F(" T"));                         // T - Time
    Output->print(timeDiff);  
    Output->print(F(" N"));                         // N - now
    Output->print(now);
    Output->print(F(","));
    Output->print(millisWraps);    
    Output->print(F(" X"));                         // X Reject
    Output->print(rejDiff);  
    
    if (!showOnly) {
        Output->print(F(" S"));                     // S - Sequence number
        Output->print(reportSequence[pinIndex]);  
    }
    if (countDiff > 0) {
        Output->print(F(" A"));
        Output->print(widthSum / countDiff);
    }
    Output->println();    
#ifdef WifiSupport
    if (tcpMode && !showOnly) {
        Serial.print(F("D reported pin "));
        Serial.print(activePin[pinIndex]);
        Serial.print(F(" sequence "));
        Serial.print(reportSequence[pinIndex]);  
        Serial.println(F(" over tcp "));  
    }
#endif  
#ifdef TTGO_DISPLAY
    if (lineCount < 4) {
        tft.setCursor(0,32+8*lineCount);
        tft.print(F("R"));                          // R Report
        tft.print(activePin[pinIndex]);
        tft.print(F(" C"));                         // C - Count
        tft.print(count);
        tft.print(F(" D"));                         // D - Count Diff (without pulse that marks the begin)
        tft.print(countDiff);
        tft.print(F("/"));                          // R - real Diff for long counter - includes first after restart
        tft.print(realDiff);
        tft.print(F(" T"));                         // T - Time
        tft.print(timeDiff);  
        tft.print(F(" X"));                         // X Reject
        tft.print(rejDiff);  
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
            if (activePin[pinIndex] >= 0)
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
            Serial.print(F(" now "));
            Serial.print(now);
            Serial.print(F(" last "));
            Serial.print(lastDelayedTcpReports);
            Serial.print(F(" diff "));
            Serial.println(now - lastDelayedTcpReports);
            delayedTcpReports++;
            lastDelayedTcpReports = now;
            return;
        } else return;
    }
#endif    
#ifdef TTGO_DISPLAY
    tft.setCursor(0,48);
    lineCount = 0;
#endif
    for (uint8_t pinIndex=0; pinIndex < MAX_PIN; pinIndex++) {  // go through all observed pins as pinIndex
        if (activePin[pinIndex] >= 0) {
            showPinCounter (pinIndex, false, now);              // report pin counters if necessary
#ifdef pulseHistory
            if (devVerbose >= 5) 
                showPinHistory(pinIndex, now);                  // show pin history if verbose >= 5
#endif          
        }
    }
    lastReportCall = now;                                       // check again after intervalMin or if intervalMax is over for a pin
}


void intervalCmd(uint16_t *values, uint8_t size) {
    /*Serial.print(F("D int ptr is "));
    Serial.println(size);*/
    if (size < 4) {               // i command always gets 4 values: min, max, sml, cntMin
        PrintErrorMsg();
        Output->print(F("interval expects 4 values but got "));
        Output->println(size);
        return;
    }
    if (values[0] < 1 || values[0] > 3600) {
        PrintErrorMsg(); Output->println(values[0]);
        return;
    }
    intervalMin = (uint32_t)values[0] * 1000;

    if (values[1] < 1 || values[1] > 3600) {
        PrintErrorMsg(); Output->println(values[1]);
        return;
    }
    intervalMax = (uint32_t)values[1]* 1000;

    if (values[2] > 3600) {
        PrintErrorMsg(); Output->println(values[2]);
        return;
    }
    intervalSml = (uint32_t)values[2] * 1000;

    if (values[3] > 100) {
        PrintErrorMsg(); Output->println(values[3]);
        return;
    }
    countMin = values[3];

    Output->print(F("M intervals set to ")); 
    Output->print(values[0]);
    Output->print(F(" ")); 
    Output->print(values[1]);
    Output->print(F(" ")); 
    Output->print(values[2]);
    Output->print(F(" ")); 
    Output->println(values[3]);
}


#ifdef analogIR
void thresholdCmd(uint16_t *values, uint8_t size) {    
    if (size < 2) {               // t command gets 2 values: min, max
        PrintErrorMsg();
        Output->print(F("size"));
        Output->println();
        return;
    }
    if (values[0] < 1 || values[0] > 1023) {
        PrintErrorMsg(); Output->println(values[0]);
        return;
    }
    analogThresholdMin = (int)values[0];

    if (values[1] < 1 || values[1] > 1023) {
        PrintErrorMsg(); Output->println(values[1]);
        return;
    }
    analogThresholdMax = (int)values[1];

    Output->print(F("M analog thresholds set to ")); 
    Output->print(values[0]);
    Output->print(F(" ")); 
    Output->println(values[1]);
}


/*
    handle add command.
*/
void addCmd(uint16_t *values, uint8_t size) {
    uint16_t pulseWidth;
    uint32_t now = millis();
  
    uint8_t aPin = values[0];                   // values[0] is pin number
    if (aPin >= MAX_APIN  || allowedPins[aPin] < 0) {
        PrintErrorMsg(); 
        Output->print(F("Illegal pin specification "));
        Output->println(aPin);
        return;
    }; 
    uint8_t pinIndex = allowedPins[aPin];
    uint8_t rPin = internalPins[pinIndex];

    if (activePin[pinIndex] != aPin) {          // in case this pin is not already active counting
      #if defined(__AVR_ATmega328P__)
        uint8_t port = digitalPinToPort(rPin) - 2;
        PCintLast[port] = *portInputRegister(port+2);
      #endif    
        initPinVars(pinIndex, now);
        activePin[pinIndex] = aPin;             // save arduino pin number and flag this pin as active for reporting
    }

    if (values[1] < 2 || values[1] > 3) {       // values[1] is level (3->rising / 2->falling)
        PrintErrorMsg(); 
        Output->print(F("Illegal pulse level specification for pin "));
        Output->println(aPin);
    }
    pulseLevel[pinIndex] = (values[1] == 3);    // 2 = falling -> pulseLevel 0, 3 = rising -> pulseLevel 1
  
#ifdef analogIR    
    if (size > 2 && values[2] && !analogPins[pinIndex]) { 
#else
    if (size > 2 && values[2]) { 
#endif      
        pinMode (rPin, INPUT_PULLUP);           // values[2] is pullup flag
        pullup[pinIndex] = 1;
    } else {
        pinMode (rPin, INPUT);  
        pullup[pinIndex] = 0;
    }

    if (size > 3 && values[3] > 0) {            // value 3 is min length
        pulseWidth = values[3];
    } else {
        pulseWidth = 2;
    }  
    
    /* todo: add upper and lower limits for analog pins as option here and in Fhem module */
    
    pulseWidthMin[pinIndex] = pulseWidth;

#ifdef analogIR
    if (!analogPins[pinIndex]) {
#endif      
        if (!AddPinChangeInterrupt(rPin)) {     // add Pin Change Interrupt
            PrintErrorMsg(); 
            Output->println(F("AddInt"));
            return;
        }
#ifdef analogIR      
    }
#endif    
    Output->print(F("M defined ")); 
    showPinConfig(pinIndex);    
    Output->println();
}


/*
    handle rem command.
*/
void removeCmd(uint16_t *values, uint8_t size) {
    uint8_t aPin = values[0];
    if (size < 1 || aPin >= MAX_APIN || allowedPins[aPin] < 0) {
        PrintErrorMsg(); 
        Output->print(F("Illegal pin specification "));
        Output->println(aPin);
        return;
    };
    uint8_t pinIndex = allowedPins[aPin];

#ifdef analogIR
    if (!analogPins[pinIndex]) {
#endif      
#if defined(ESP8266) || defined(ESP32)
        detachInterrupt(digitalPinToInterrupt(internalPins[pinIndex]));
#elif defined(__AVR_ATmega328P__)
        if (!RemovePinChangeInterrupt(internalPins[pinIndex])) {      
            PrintErrorMsg(); 
            Output->println(F("RemInt"));
            return;
        }
#endif
#ifdef analogIR
    }
#endif    
    initPinVars(pinIndex, 0);
    Output->print(F("M removed "));
    Output->println(aPin);
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

/* todo: include analogPins as well as analog limits in save / restore */

void saveToEEPROMCmd() {
    int address   = 0;
    uint8_t slots = 1;
    updateEEPROM(address, 'C');
    updateEEPROM(address, 'f');
    updateEEPROM(address, 'g');
    for (uint8_t pinIndex=0; pinIndex < MAX_PIN; pinIndex++)
        if (activePin[pinIndex] >= 0) slots ++;
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
        if (activePin[pinIndex] >= 0)
            updateEEPROMSlot(address, 'A', (uint16_t)activePin[pinIndex], (uint16_t)(pulseLevel[pinIndex] ? 3:2), 
                                                    (uint16_t)pullup[pinIndex], (uint16_t)pulseWidthMin[pinIndex]);
#if defined(ESP8266) || defined(ESP32)
    EEPROM.commit();               
#endif  
    Serial.print(F("config saved, "));
    Serial.print(slots);
    Serial.print(F(", "));
    Serial.println(address);
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
        Output->print(cmd);
        Output->print(F(" "));
        Output->print(v1);
        Output->print(F(","));
        Output->print(v2);
        Output->print(F(","));
        Output->print(v3);
        Output->print(F(","));
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
        if (cmd == 'I') intervalCmd(commandData, commandDataPointer);
#ifdef analogIR        
        if (cmd == 'T') thresholdCmd(commandData, commandDataPointer);
#endif        
        if (cmd == 'A') addCmd(commandData, commandDataPointer);
    }
    commandDataPointer = 0;
    value = 0;
    for (uint8_t i=0; i < MAX_INPUT_NUM; i++)
        commandData[i] = 0;     
  
}

#if defined(WifiSupport)
void printConnection(Print *Out) {
    Out->print(F("M Connected to "));
    Out->print(WiFi.SSID());    
    Out->print(F(" with IP "));
    Out->print(WiFi.localIP());    
    Out->print(F(" RSSI "));
    Out->print(WiFi.RSSI());
    Out->println();
#ifdef TTGO_DISPLAY    
    tft.setCursor(0, 0);
    tft.print(F("Conected to "));
    tft.print(WiFi.SSID());
    tft.print(F("        "));
    tft.setCursor(0, 16);
    tft.print("IP ");
    tft.print(WiFi.localIP());
    tft.print(F(" RSSI "));
    tft.print(WiFi.RSSI());
#endif
}
#endif

/* give status report in between if requested over serial input */
void showCmd() {
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
        if (activePin[pinIndex] >= 0) {
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
    Output->print(F(" milliseconds"));
    Output->println();  
}


void initialize() {
    uint32_t now = millis();
    for (uint8_t pinIndex=0; pinIndex < MAX_PIN; pinIndex++) {
        initPinVars(pinIndex, now);
    }   
    bootTime        = now;          // with boot / reset time
    bootWraps       = millisWraps;
#ifdef analogIR    
    lastAnalogRead  = now;
#endif
    lastReportCall  = now;          // time for first output after intervalMin from now
    devVerbose = 0;
#if defined(__AVR_ATmega328P__)
    for (uint8_t port=0; port <= 2; port++) {
        PCintLast[port] = *portInputRegister(port+2); // current pin states at port for PCInt handler
    }
#endif
    restoreFromEEPROM();
#ifdef WifiSupport  
    lastKeepAlive = now;
#endif      
}



void helloCmd() {
    uint32_t now = millis();
    Output->println();
    printVersionMsg();
    Output->print(F(" Hello, pins "));
    boolean first = true;
    for (uint8_t aPin=0; aPin < MAX_APIN; aPin++) {
        if (allowedPins[aPin] >= 0) {
            if (!first) {
                Output->print(F(","));
            } else {
                first = false;
            }
            Output->print(aPin);
        }
    }
    Output->print(F(" available"));
    Output->print(F(" T"));
    Output->print(now);
    Output->print(F(","));
    Output->print(millisWraps);
    Output->print(F(" B"));
    Output->print(bootTime);
    Output->print(F(","));
    Output->print(bootWraps);
    
    Output->println();
    showIntervals();
#ifdef analogIR      
    showThresholds();
#endif    
    Output->print(F("V"));
    Output->println(devVerbose);
    
    for (uint8_t pinIndex=0; pinIndex < MAX_PIN; pinIndex++) { // go through all observed pins as pinIndex
        if (activePin[pinIndex] >= 0) {
            showPinConfig(pinIndex);
            Output->println();
        }
    }
}


void waitCmd(uint16_t *values, uint8_t size) {    
    if (size < 1) {               // w command gets 1 value: analogReadInterval
        PrintErrorMsg();
        Output->print(F("size"));
        Output->println();
        return;
    }
    if (values[0] < 1 || values[0] > 999) {
        PrintErrorMsg(); Output->println(values[0]);
        return;
    }
    analogReadInterval = (int)values[0];

    Output->print(F("M analog read delay set to ")); 
    Output->println(values[0]);
}
#endif


void devVerboseCmd(uint16_t *values, uint8_t size) {    
    if (size < 1) {               // v command gets 1 value: verbose level
        PrintErrorMsg();
        Output->print(F("size"));
        Output->println();
        return;
    }
    if (values[0] > 50) {
        PrintErrorMsg(); Output->println(values[0]);
        return;
    }
    devVerbose = values[0];
    Output->print(F("M devVerbose set to ")); 
    Output->println(values[0]); 
}
            
            
void keepAliveCmd(uint16_t *values, uint8_t size) {

    if (values[0] == 1 && size > 0) {
        Output->print(F("alive"));
#ifdef WifiSupport
        uint32_t now = millis();
        if (devVerbose >=5) {
            Output->print(F(" RSSI "));
            Output->print(WiFi.RSSI());
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
void quitCmd() {
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



void handleInput(char c) {
    if (c == ',') {                       // Komma input, last value is finished
        if (commandDataPointer < (MAX_INPUT_NUM - 1)) {
            commandData[commandDataPointer++] = value;
            value = 0;
        }
    }
    else if ('0' <= c && c <= '9') {      // digit input
        value = 10 * value + c - '0';
    }
    else if ('a' <= c && c <= 'z') {      // letter input is command
    
        if (devVerbose > 0) {
            commandData[commandDataPointer] = value;
            Serial.print(F("D got "));
            for (short v = 0; v <= commandDataPointer; v++) {          
                if (v > 0) Serial.print(F(","));
                Serial.print(commandData[v]);
            }
            Serial.print(c);
            Serial.print(F(" size "));
            Serial.print(commandDataPointer+1);
            Serial.println();
        }

        switch (c) {
        case 'a':                       // add a pin
            commandData[commandDataPointer] = value;
            addCmd(commandData, commandDataPointer+1);
            break;
        case 'd':                       // delete a pin
            commandData[commandDataPointer] = value;
            removeCmd(commandData, commandDataPointer+1);
            break;
        case 'e':                       // save to EEPROM
            saveToEEPROMCmd();
            break; 
        case 'f':                       // flash ota
            // OTA flash from HTTP Server
            break; 
        case 'h':                       // hello
            helloCmd();
            break;
        case 'i':                       // interval
            commandData[commandDataPointer] = value;
            intervalCmd(commandData, commandDataPointer+1);
            break;
        case 'k':                       // keep alive
            commandData[commandDataPointer] = value;
            keepAliveCmd(commandData, commandDataPointer+1);
            break;   
#ifdef WifiSupport      
        case 'q':                       // quit
            quitCmd();
            break; 
#endif
        case 'r':                       // reset
            initialize();
            break;
        case 's':                       // show
            showCmd();
            break;
#ifdef analogIR            
        case 't':                       // thresholds for analog pin
            commandData[commandDataPointer] = value;
            thresholdCmd(commandData, commandDataPointer+1);
            break;
#endif            
        case 'v':                       // dev verbose
            commandData[commandDataPointer] = value;
            devVerboseCmd(commandData, commandDataPointer+1);
            break;
#ifdef analogIR            
        case 'w':                       // wait - delay between analog reads
            commandData[commandDataPointer] = value;
            waitCmd(commandData, commandDataPointer+1);
            break;
#endif            
        default:
            break;
        }
        commandDataPointer = 0;
        value = 0;
        for (uint8_t i=0; i < MAX_INPUT_NUM; i++)
            commandData[i] = 0;     
        //Serial.println(F("D End of command"));
    }
}


#ifdef debugPins
void debugPinChanges() {
    for (uint8_t pinIndex=0; pinIndex < MAX_PIN; pinIndex++) {
        short aPin = activePin[pinIndex];
        if (aPin > 0) {
            uint8_t rPin = internalPins[pinIndex];
            uint8_t pinState = digitalRead(rPin);
                       
            if (pinState != lastState[pinIndex]) {
                lastState[pinIndex] = pinState;
                Output->print(F("M pin "));
                Output->print(aPin);
                Output->print(F(" (internal "));
                Output->print(rPin);
                Output->print(F(")"));
                Output->print(F(" changed to "));
                Output->print(pinState);
#ifdef pulseHistory                     
                Output->print(F(", histIdx "));
                Output->print(histIndex);
#endif                  
                Output->print(F(", count "));
                Output->print(counter[pinIndex]);
                Output->print(F(", reject "));
                Output->print(rejectCounter[pinIndex]);
                Output->println();
            }
        }
    }
}
#endif


#ifdef WifiSupport    
void connectWiFi() {
    Client1Connected = false;
    Client2Connected = false;
    int counter = 0;

    // Connect to WiFi network
    WiFi.mode(WIFI_STA);
    delay (1000);    
    if (WiFi.status() != WL_CONNECTED) {
#ifdef TTGO_DISPLAY 
        tft.setCursor(0,0);
        tft.print(F("Conecting WiFi to "));
        tft.print(ssid);
#endif
        Serial.print(F("M Connecting WiFi to "));
        Serial.println(ssid);
        WiFi.begin(ssid, password);                 // authenticate 
        while (WiFi.status() != WL_CONNECTED) {
            Serial.print(F("M Status is "));
            switch (WiFi.status()) {
              case WL_CONNECT_FAILED: 
                Serial.println(F("Connect Failed"));
                break;
              case WL_CONNECTION_LOST: 
                Serial.println(F("Connection Lost"));
                break;
              case WL_DISCONNECTED: 
                Serial.println(F("Disconnected"));
                break;
              case WL_CONNECTED: 
                Serial.println(F("Connected"));
                break;
              default:
                Serial.println(WiFi.status());
            }
            delay(1000);
            counter++;

            if (counter > 5) {
#ifdef TTGO_DISPLAY    
                tft.setCursor(0,0);
                tft.print(F("Retry conecting WiFi to"));
                tft.print(ssid);
#endif
                Serial.println(F("M Retry connecting WiFi"));
                WiFi.begin(ssid, password);         // authenticate again
                delay (1000);
                counter = 0;
            }
        }    
    }
    printConnection (&Serial);

    // Start the server
    Server.begin();
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
            Serial.print(F("M second connection from "));
            Serial.print(remote);
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
            Serial.print(F("M new connection from "));
            Serial.print(remote);
            Serial.println(F(" accepted"));
            Client1Connected = true;
            Output = &Client1;
            lastKeepAlive = now;
            helloCmd();                                         // say hello to client
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
void detectTrigger(int val) {
    uint8_t nextState = triggerState;
    if (val > analogThresholdMax) {
        nextState = 1;
    } else if (val < analogThresholdMin) {
        nextState = 0;
    }
    if (nextState != triggerState) {
        triggerState = nextState;
    if (ledOutPin)
        digitalWrite(ledOutPin, triggerState);

        short pinIndex = allowedPins[analogInPin];  // ESP pin A0 (pinIndex 8, internal 17) or Arduino A7 (pinIndex 17, internal 21)
        uint32_t now = millis();
        doCount (pinIndex, triggerState, now);      // do the counting, history and so on
        
#ifdef debugPins
        if (devVerbose >= 10) {
            short rPin = internalPins[pinIndex];
            Output->print(F("M pin "));
            Output->print(analogInPin);
            Output->print(F(" (index "));
            Output->print(pinIndex);
            Output->print(F(", internal "));
            Output->print(rPin);
            Output->print(F(" ) "));
            Output->print(F(" to "));
            Output->print(nextState);
#ifdef pulseHistory                     
            Output->print(F("  histIdx "));
            Output->print(histIndex);
#endif                  
            Output->print(F("  count "));
            Output->print(counter[pinIndex]);
            Output->print(F("  reject "));
            Output->print(rejectCounter[pinIndex]);
            Output->println();
        }
#endif
    }
}

void readAnalog() {
    short AIndex = allowedPins[analogInPin];
    if (AIndex >= 0 && activePin[AIndex] >= 0) {                // analog Pin active?
        uint32_t now = millis();
        uint16_t interval2 = analogReadInterval + 2;
        uint16_t interval3 = analogReadInterval + 4;
        if ((now - lastAnalogRead) > analogReadInterval) {      // time for next analog read?
            switch (analogReadState) {
                case 0:                                         // initial state
                    digitalWrite(irOutPin, LOW);                // switch IR LED is off
                    analogReadState = 1;
                    break;
                case 1:                                         // wait before measuring
                    if ((now - lastAnalogRead) < interval2)
                        return;
                    analogReadState = 2;
                    break;
                case 2:
                    sensorValueOff = analogRead(analogInPin);   // read the analog in value (off)
                    analogReadState = 3;
                    break;
                case 3:
                    digitalWrite(irOutPin, HIGH);               // turn IR LED on
                    analogReadState = 4;
                    break;
                case 4:                                         // wait again before measuring
                    if ((now - lastAnalogRead) < interval3)
                        return;
                    analogReadState = 5;
                    break;
                case 5:
                    int sensorDiff;
                    sensorValueOn = analogRead(analogInPin);    // read the analog in value (on)
                    sensorDiff = sensorValueOn - sensorValueOff;
                    analogReadState = 0;
                    lastAnalogRead = now;
                    detectTrigger (sensorDiff);            
                    if (devVerbose >= 25) {
                        char line[20];
                        sprintf(line, "L%4d, %4d -> % 4d", sensorValueOn, sensorValueOff, sensorDiff);
                        Output->println(line);
                    } else if (devVerbose >= 20) {
                        Output->print(F("L"));
                        Output->println(sensorDiff);
                    }                    
#ifdef TTGO_DISPLAY                    
                    tft.drawRect(0,300,sensorDiff,10);  // todo: set color, clear to right ...
#endif
                    break;
                default:
                    analogReadState = 0;
                    Output->println(F("D error: wrong analog read state"));
                    break;
            }
        }
    }
}
#endif


void setup() {
#ifdef TTGO_DISPLAY    
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
    helloCmd();                             // started message to serial
#ifdef WifiSupport
    connectWiFi();
#endif
}


/*   Main Loop  */
void loop() {
    handleTime();
    if (Serial.available()) handleInput(Serial.read());
#ifdef WifiSupport    
    handleConnections();
#endif
#ifdef analogIR
    readAnalog();
#endif
#ifdef debugPins
    if (devVerbose >= 10) {
        debugPinChanges();
    }
#endif
    if (reportDue()) {    
        report();
    }
}
