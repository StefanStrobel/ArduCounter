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
 * TTGO T-Display has right button at GPIO 35
 * 35,3,0,50a
 * 36,3,0,50,27a
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

        20.1.2020 - rewrite many things ...


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
TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h   todo: select TFT as -D in platformio.ini instead
uint8_t lineCount;
#endif

/* allow printing of every pin change to Serial */
#define debugPins 1  

/* allow tracking of pulse lengths */
#define pulseHistory 1

#include "pins_arduino.h"
#include <EEPROM.h>

const char versionStr[] PROGMEM = "ArduCounter V4.20";

#define SERIAL_SPEED 38400      // todo: increase?
#define MAX_INPUT_NUM 8

#define pin2GPIO(P) ( pgm_read_byte( digital_pin_to_gpio_PGM + (P) ) )
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
void CmdInterval();
void CmdThreshold();
void CmdAdd ();
void CmdRemove();
void CmdShow();
void CmdHello();
void CmdWait();
void CmdDevVerbose();
void CmdKeepAlive();
void CmdQuit();
void printConnection(Print *Out);


/* ESP8266 pins that are typically ok to use 
 * (some might be set to FF (disallowed) because they are used 
 * as reset, serial, led or other things on most boards) 
 * maps printed pin numbers (aPin) to gpio pin numbers (via their macros if available)
 
 * Wemos / NodeMCU Pins 3,4 and 8 (GPIO 0,2 and 15) define boot mode and therefore
 * can not be used to connect to signal */
 
#if defined(ESP8266)                // ESP 8266 variables and definitions
#define MAX_PIN 7                   // max number of pins that can be defined
#define MAX_HIST 20                 // 20 history entries for ESP boards (can be increased)
#define MAX_APIN 18
const uint8_t PROGMEM digital_pin_to_gpio_PGM[] = {
	D0, D1, D2, FF,                 // D0=16, D1=5, D2=4
    FF, D5, D6, D7,                 // D5=14, D6 is my preferred pin for IR out, D7 for LED out
    FF, FF, FF, FF,
    FF, FF, FF, FF,
    FF, A0                          // A0 is gpio pin 17
}; 


/* ESP32 pins that are typically ok to use 
 * (some might be set to -1 (disallowed) because they are used 
 * as reset, serial, led or other things on most boards) 
 * maps printed pin numbers (aPin) to sketch internal index numbers */
 
#elif defined(ESP32)                // ESP32 variables and definitions
#define MAX_PIN 12
#define MAX_HIST 20                 // 20 history entries for ESP boards (can be increased)
#define MAX_APIN 40
#if defined(TFT_DISPLAY)
const uint8_t PROGMEM digital_pin_to_gpio_PGM[] = {
    FF, FF, FF, FF,                 // pwm at boot, debug, LED, high at boot
    04, FF, FF, FF,                 // 4 is ok, pwm at boot, 
    FF, FF, FF, FF,                 // 6-11 is flash
    FF, FF, FF, FF,                 // 12-15 are used for JTAG
    FF, 17, FF, FF,                 // only 17 is free. 16,18 and 19 for display
    FF, 21, 22, FF,                 // 21-22 avaliable, 23 for display
    FF, 25, 26, 27,                 // 25-26 avaliable, use 27 as irOut
    FF, FF, FF, FF,
    32, 33, 34, 35,                 // 32-35 avaliable (34/35 input only, 35 is right button)
    36, FF, FF, 39};                // 36 is A0, is 39 avaliable but also input only
#else
const uint8_t PROGMEM digital_pin_to_gpio_PGM[] = {
    FF, FF, FF, FF,                 // pwm at boot, debug, LED, high at boot
    04, FF, FF, FF,                 // 4 is ok, pwm at boot, 
    FF, FF, FF, FF,                 // 6-11 is flash
    FF, FF, FF, FF,                 // 12 is used at boot, 12-15 used for JTAG, otherwise 13 is ok, 14/15 output pwm
    16, 17, 18, 19,
    FF, 21, 22, 23,                 // 21-23 avaliable   
    FF, 25, 26, 27,                 // 25-26 avaliable, use 27 as irOut
    FF, FF, FF, FF,
    32, 33, 34, 35,                 // 32-35 avaliable (34/35 input only)
    36, FF, FF, 39};                // 36 is A0, 39 is avaliable but also input only
#endif


#elif defined(__AVR_ATmega328P__)
#define MAX_HIST 20                 // 20 history entries for arduino boards
#define MAX_PIN 12                  // max 20 counting pins at the same time
#define MAX_APIN 22
const uint8_t PROGMEM digital_pin_to_gpio_PGM[] = {
    FF,  FF,  D2,  D3,              // 2 is typically ir out for analog,
    D4,  D5,  D6,  D7,
    D8,  D9,  D10, D11,
    D12, D13, A0,  A1,              // 12 often is led out
    A2,  A3,  A4,  A5,
    A6,  A7                         // A7 is typically analog in for ir
};
  
uint8_t pinIndexMap[MAX_APIN];      // map needed by 328p isr to map back from aPin to pinIndex in pinData
uint8_t firstPin[] = {8, 14, 0};    // first and last pin at port PB, PC and PD for arduino uno/nano
uint8_t lastPin[]  = {13, 19, 7};   // not really needed. Instead check bit position (or bit != 0)
uint8_t *port_to_pcmask[] = {&PCMSK0, &PCMSK1, &PCMSK2};    // Pin change mask for each port on arduino platform
volatile static uint8_t PCintLast[3];   // last PIN States at io port to detect pin changes in arduino ISR
#endif                              // end of Nano / Uno specific stuff


Print *Output;                      // Pointer to output device (Serial / TCP connection with ESP8266)
uint32_t bootTime;                  
uint16_t bootWraps;                 // counter for millis wraps at last reset
uint16_t millisWraps;               // counter to track when millis counter wraps 
uint32_t lastMillis;                // milis at last main loop iteration
uint8_t devVerbose;                 // >=10 shows pin changes, >=5 shows pin history

uint32_t intervalMin = 30000;       // default 30 sec - report after this time if nothing else delays it
uint32_t intervalMax = 60000;       // default 60 sec - report after this time if it didin't happen before
uint32_t intervalSml =  2000;       // default 2 secs - continue if timeDiff is less and intervalMax not over
uint16_t countMin    =     2;       // continue counting if count is less than this and intervalMax not over
uint8_t ledOutPin;
uint32_t lastReportCall;


typedef struct pinData {
    // pin configuration data
    uint8_t pinName;                            // printed pin Number for user input / output
                                                
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
    volatile uint8_t counterIgn;                // counts only first pulse that marks the begin of the very first interval

    // isr internal states
    volatile uint8_t initialized;               // set if first pulse has ben seen to start interval
    volatile uint32_t lastChange;               // millis at last level change (for measuring pulse length)
    volatile uint8_t lastLevel;                 // level of input at last interrupt     // only one bit needed
    volatile uint8_t lastLongLevel;             // last level that was longer than pulseWidthMin    // only bit

    // reporting data
    uint32_t lastCount;                         // counter at last report (to get the delta count)
    uint16_t lastRejCount;                      // reject counter at last report (to get the delta count)
    uint32_t lastReport;                        // millis at last report to find out when maxInterval is over
    uint8_t reportSequence;                     // sequence number for reports
    uint8_t lastDebugLevel;                          // for debug output when a pin state changes
} pinData_t;

pinData_t pinData[MAX_PIN];
uint8_t maxPinIndex = 0;                        // the next available index (= number of indices used)

typedef struct analogData {
    pinData_t *inPinData;                       // pointer to pinData structure for input pin (used e.g. to call doCount from analogRead)
    uint8_t inPinName;                          // printed pin Number for user input / output (optinal here?)
    uint8_t outPinName;                         // printed pin number to use for ir (convert using our macro)
    uint16_t thresholdMin;                      // measurement for 0 level                  todo: change all occurrences to this array
    uint16_t thresholdMax;                      // measurement for 1 
    uint8_t triggerState;                       // which level was it so far
    uint16_t sumOff = 0;                        // sum of measured values during light off
    uint16_t sumOn  = 0;                        // sum of measured values during light on
} analogData_t;

#define MAX_ANALOG 2
analogData_t analogData[MAX_ANALOG];
uint8_t maxAnalogIndex = 0;                     // the next available index

#ifdef pulseHistory 
typedef struct histData {
    volatile uint16_t seq;                      // history sequence number
    volatile uint8_t pin;                       // pin for this entry
    volatile uint8_t level;                     // level for this entry (was index and should by pinName)
    volatile uint32_t time;                     // time for this entry
    volatile uint32_t len;                      // time that this level was held
    volatile char act;                          // action (count, reject, ...) as one char
} histData_t;

histData_t histData[MAX_HIST];
volatile uint8_t histIndex;                     // pointer to next entry in history ring
volatile uint16_t histNextSeq;                  // next seq number to use
#endif

uint16_t commandData[MAX_INPUT_NUM];            // input data over serial port or network
uint8_t  commandDataPointer = 0;                // index pointer to next input value
uint8_t  commandDataSize = 0;                   // number of input values specified in commandData array
char     commandLetter;                         // the actual command letter
uint16_t commandValue = 0;                      // the current value for input function


uint32_t analogReadLast;            // millis() at last analog read
uint16_t analogReadInterval = 50;   // interval at which to read analog values (miliseconds)
uint8_t analogReadState = 0;        // to keep track of switching LED on/off, measuring etc.
uint8_t analogReadAmp = 3;          // amplification for display
uint8_t analogReadSamples = 4;      // samples to take with the light off - max 16 so sum can be an int 16
uint8_t analogReadCount = 0;        // counter for sampling


// idea: save analog measurement during same level as sum and count to get average and then put in history when doCount is called so that threshold can be auto adjusted or even self learning?


void initPinVars(pinData_t *pd, uint32_t now) {
    uint8_t level = 0;
    pd->pinName        = FF;        // inactive
    pd->initialized    = false;     // no pulse seen yet
    pd->pulseWidthMin  = 0;         // min pulse length
    pd->counter        = 0;         // counters to 0
    pd->counterIgn     = 0;    
    pd->lastCount      = 0;
    pd->rejectCounter  = 0;        
    pd->lastRejCount   = 0;
    pd->intervalStart  = now;       // time vars
    pd->intervalEnd    = now;
    pd->lastChange     = now;
    pd->lastReport     = now;
    pd->reportSequence = 0;
    if (!pd->analogFlag)
        level = digitalRead(pin2GPIO(pd->pinName));
    pd->lastLevel = level;
    pd->lastLongLevel = level;
#ifdef debugPins
    pd->lastDebugLevel = level;       // for debug output
#endif
}


void initialize() {
    uint32_t now = millis();
    bootTime = now;             // with boot / reset time
    bootWraps = millisWraps;    
    lastReportCall = now;       // time for first output after intervalMin from now
    analogReadLast = now;
    devVerbose = 0;
    for (uint8_t pinIndex=0; pinIndex < MAX_PIN; pinIndex++)
        initPinVars(&pinData[pinIndex], now);                   // todo: is this still needed now that there is maxPinIndex?
#if defined(__AVR_ATmega328P__)        
    for (uint8_t aPin=0; aPin < MAX_APIN; aPin++)
        pinIndexMap[aPin] = FF;
#endif        
#ifdef WifiSupport  
    lastKeepAlive = now;
#endif      
    restoreFromEEPROM();
    clearInput();
}

// search pinData for aPin and return the pinIndex
uint8_t findInPin (uint8_t aPin) {
    for (uint8_t pinIndex = 0; pinIndex < maxPinIndex; pinIndex++) {
        if (pinData[pinIndex].pinName == aPin) return pinIndex;
    }
    return FF;
}

// search analogData for oPin and return the analogIndex
uint8_t findOutPin (uint8_t oPin) {
    for (uint8_t analogIndex = 0; analogIndex < maxAnalogIndex; analogIndex++) {
        if (analogData[analogIndex].outPinName == oPin) return analogIndex;
    }
    return FF;
}

// check if pin is allowed
bool checkPin (uint8_t aPin) {
    if (aPin >= MAX_APIN) return false;                     // pin number too big
    if (pin2GPIO(aPin) == FF) return false;      // pin is not allowed at all
    return true;
}


// find analogData for given pinData
uint8_t findAnalogData(pinData_t *pd) {
    for (uint8_t aIdx = 0; aIdx < maxAnalogIndex; aIdx++) {
        if (analogData[aIdx].inPinData == pd) 
            return aIdx;
    }
    return FF;
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
            Output->print(F("missing parameter number ")); Output->println(index);
        }
        return false;
    }
    if (commandData[index] < min || commandData[index] > max) {
        PrintErrorMsg();
        Output->print(F("parameter number ")); Output->print(index);
        Output->print(F(" (value "));          Output->print(commandData[index]);
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
static void doCount(pinData_t *pd, uint8_t level) {
    uint32_t now = millis();
    uint32_t len = now - pd->lastChange;
    char act = ' ';
    if (len < pd->pulseWidthMin) {                      // len is too short
        if (pd->lastLevel == pd->pulseLevel) {          // if change to gap level (we just had a too short pulse)
            act = 'R';                                  // -> reject
            pd->rejectCounter++;                       
        } else {                                        // change to pulse level (we just had a too short gap)
            act = 'X';                                  // -> reject gap / set action to X (gap too short)
        }
    } else {    // len is big enough
        if (pd->lastLevel != pd->pulseLevel) {          // edge fits pulse start, level is pulse, before was gap
            act = 'G';                                  // -> gap (even if betw. was a spike that we ignored)
        } else {                                        // edge is a change to gap, level is now gap
            if (pd->lastLongLevel != pd->pulseLevel) {  // last valid level was gap, now pulse 
                act = 'C';                              // -> count
                pd->counter++;                          
                pd->intervalEnd = now;                  // remember time in case pulse is last in the interval
                if (!pd->initialized) {
                    pd->intervalStart = now;            // if first impulse on this pin -> start interval now
                    pd->initialized = true;             // and start counting the next impulse (counter is 0)
                    pd->counterIgn++;                   // count ignored for diff because defines start of intv
                }
                pd->pulseWidthSum += len;               // for average calculation
            } else {                                    // last valid level was pulse -> now another valid pulse
                act = 'P';                              // -> pulse was already counted, only short drop inbetween
                pd->pulseWidthSum += len;               // for average calculation
            }
        }       
        pd->lastLongLevel = pd->lastLevel;              // remember this valid level as lastLongLevel
    }
#ifdef pulseHistory   
    if (++histIndex >= MAX_HIST) histIndex = 0;
    histData_t *hd = &histData[histIndex];              // write pin history
    hd->seq   = histNextSeq++;
    hd->pin   = pd->pinName;
    hd->time  = pd->lastChange;
    hd->level = pd->lastLevel;
    hd->len   = len;
    hd->act   = act;
#endif  
    pd->lastChange = now;
    pd->lastLevel  = level;
}


/* Interrupt handlers and their installation  */

#if defined(__AVR_ATmega328P__)

/* Add a pin to be handled (Arduino code) */
uint8_t AddPinChangeInterrupt(uint8_t rPin) {
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
   common function for arduino pin change interrupt handlers. 
   "port" is the PCINT port index (0-2) as passed from above, 
   not PB, PC or PD which are mapped to 2-4
*/
static void PCint(uint8_t port) {
    uint8_t bit;
    uint8_t curr;
    uint8_t delta;
    short pinIndex;

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
                doCount (&pinData[pinIndex], ((curr & bit) > 0));    // do the counting, history and so on
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

void IRAM_ATTR ESPISR(void* arg) {                      // common ISR for all Pins on ESP
    // ESP32 now also defines IRAM_ATTR as ICACHE_RAM_ATTR
    pinData_t *pd = (pinData_t *)arg;
    doCount(pd, digitalRead(pin2GPIO(pd->pinName)));
}
#endif


void showIntervals() {
    Output->print(F("I"));  Output->print(intervalMin / 1000);
    Output->print(F(" "));  Output->print(intervalMax / 1000);
    Output->print(F(" "));  Output->print(intervalSml / 1000);
    Output->print(F(" "));  Output->println(countMin);
}


void showThresholds() {             // todo: remove this command and show thresholds as part of pin configuration
    Output->print(F("T"));  //Output->print(analogThresholdMin);
    Output->print(F(" "));  //Output->print(analogThresholdMax);
    Output->println();
}

// todo: add analog parameters here
void showPinConfig(pinData_t *pd) {
    Output->print(F("P"));  Output->print(pd->pinName);
    switch (pd->pulseLevel) {
        case 1:  Output->print(F(" rising")); break;
        case 0: Output->print(F(" falling")); break;
        default: Output->print(F(" -")); break;
    }        
    if (pd->pullupFlag) 
        Output->print(F(" pullup"));
    Output->print(F(" min "));  Output->print(pd->pulseWidthMin);
    if (pd->analogFlag) {
        uint8_t analogIndex = findAnalogData(pd);           // reuse or create analog data entry for aPin with oPin
        if (analogIndex != FF) {
            analogData_t *ad = &analogData[analogIndex];
            Output->print(F(" analog out-pin "));  Output->print(ad->outPinName);
            Output->print(F(" thresholds "));  Output->print(ad->thresholdMin);
            Output->print(F(" / "));  Output->print(ad->thresholdMax);
        }
    }
}


#ifdef pulseHistory 
void showPinHistory(pinData_t *pd, uint32_t now) {
    histData_t *hd;
    uint8_t start = (histIndex + 2) % MAX_HIST;
    uint8_t count = 0;
    uint32_t last = 0;
    boolean first = true;
    uint8_t pinName = pd->pinName;

    for (uint8_t i = 0; i < MAX_HIST; i++) {
        hd = &histData[(start + i) % MAX_HIST];
        if (hd->pin == pinName)                                                 // are there entries for this pin at all?
            if (first || (last <= hd->time + hd->len)) count++;
    }
    if (!count) { 
      // Output->println (F("M No Pin History"));            
      return;
    }
    
    Output->print (F("H"));  Output->print(pinName);                           // printed pin number
    Output->print (F(" "));
    for (uint8_t i = 0; i < MAX_HIST; i++) {
        hd = &histData[(start + i) % MAX_HIST];
        if (hd->pin == pinName) {
            if (first || (last <= hd->time + hd->len)) {
                if (!first) Output->print (F(", "));
                Output->print (hd->seq);                                        // sequence
                Output->print (F("s"));  Output->print ((long) (hd->time-now)); // time when level started
                Output->print (F("/"));  Output->print (hd->len);               // length 
                Output->print (F("@"));  Output->print (hd->level);             // level (0/1)
                Output->print (hd->act);                                        // action
                first = false;
            }
            last = hd->time;
        }
    }        
    Output->println();    
}
#endif

/*
   lastCount and lastRejCount are only modified here (counters at time of last reporting)
   intervalEnd is modified here and in ISR - disable interrupts in critcal moments to avoid garbage in var
   intervalStart is modified only here or for very first Interrupt in ISR
*/
void showPinCounter(pinData_t *pd, boolean showOnly, uint32_t now) {
    uint32_t count, countDiff, realDiff;
    uint32_t startT, endT, timeDiff, widthSum;
    uint16_t rejCount, rejDiff;
    uint8_t countIgn;
    
    noInterrupts();                             // copy counters while they cant be changed in isr
    startT   = pd->intervalStart;               // start of interval (typically first pulse)
    endT     = pd->intervalEnd;                 // end of interval (last unless not enough)
    count    = pd->counter;                     // get current counter (counts all pulses
    rejCount = pd->rejectCounter;
    countIgn = pd->counterIgn;                  // pulses that mark the beginning of an interval
    widthSum = pd->pulseWidthSum;
    interrupts();
        
    timeDiff  = endT - startT;                  // time between first and last impulse
    realDiff  = count - pd->lastCount;          // pulses during intervall
    countDiff = realDiff - countIgn;            // ignore forst pulse after device restart
    rejDiff   = rejCount - pd->lastRejCount;
    
    if (!showOnly) {                            // real reporting sets the interval borders new
        if((now - pd->lastReport) > intervalMax) { 
            // intervalMax is over
            if ((countDiff >= countMin) && (timeDiff > intervalSml) && (intervalMin != intervalMax)) {
                // normal procedure
                noInterrupts();                 // vars could be modified in ISR as well
                pd->intervalStart = endT;       // time of last impulse becomes first in next
                interrupts();
            } else {
                // nothing counted or counts happened during a fraction of intervalMin only
                noInterrupts();                 // vars could be modified in ISR as well
                pd->intervalStart = now;        // start a new interval for next report now
                pd->intervalEnd   = now;        // no last impulse, use now instead
                interrupts();
                timeDiff  = now - startT;       // special handling - calculation ends now
            }        
        } else if (((now - pd->lastReport) > intervalMin)   
                    && (countDiff >= countMin) && (timeDiff > intervalSml)) {
            // minInterval has elapsed and other conditions are ok
            noInterrupts();                     // vars could be modified in ISR as well
            pd->intervalStart = endT;           // time of last also time of first in next
            interrupts();
        } else {
          return;                               // intervalMin and Max not over - dont report yet
        }
        noInterrupts(); 
        pd->counterIgn    = 0;
        pd->pulseWidthSum = 0;
        interrupts();
        pd->lastCount    = count;               // remember current count for next interval
        pd->lastRejCount = rejCount;
        pd->lastReport   = now;                 // remember when we reported
#ifdef WifiSupport
        delayedTcpReports      = 0;
#endif
        pd->reportSequence++;
    }   
    Output->print(F("R"));  Output->print(pd->pinName);
    Output->print(F(" C")); Output->print(count);
    Output->print(F(" D")); Output->print(countDiff);
    Output->print(F("/"));  Output->print(realDiff);
    Output->print(F(" T")); Output->print(timeDiff);  
    Output->print(F(" N")); Output->print(now);
    Output->print(F(","));  Output->print(millisWraps);    
    Output->print(F(" X")); Output->print(rejDiff);  
    
    if (!showOnly) {
        Output->print(F(" S")); Output->print(pd->reportSequence);  
    }
    if (countDiff > 0) {
        Output->print(F(" A")); Output->print(widthSum / countDiff);
    }
    Output->println();    
#ifdef WifiSupport
    if (tcpMode && !showOnly) {
        Serial.print(F("D reported pin "));  Serial.print(pd->pinName);
        Serial.print(F(" sequence "));       Serial.print(pd->reportSequence);  
        Serial.println(F(" over tcp "));  
    }
#endif  
#if defined(TFT_DISPLAY)
    if (lineCount < 4) {
        tft.setCursor(0,32+16*lineCount);
        tft.print(F("R"));  tft.print(pd->pinName);
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
        for (uint8_t pinIndex=0; pinIndex < maxPinIndex; pinIndex++)  
            if((now - pinData[pinIndex].lastReport) > intervalMax)
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
    for (uint8_t pinIndex=0; pinIndex < maxPinIndex; pinIndex++) {  // go through all observed pins as pinIndex
        pinData_t *pd = &pinData[pinIndex];
        showPinCounter (pd, false, now);                            // report pin counters if necessary
#ifdef pulseHistory
        if (devVerbose >= 5) 
            showPinHistory(pd, now);                                // show pin history if verbose >= 5
#endif          
    }
    lastReportCall = now;                                           // check again after intervalMin or if intervalMax is over for a pin
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
        commandLetter = cmd;
        commandData[0] = EEPROM.read(address+1) + (((uint16_t)EEPROM.read(address+2)) << 8);
        commandData[1] = EEPROM.read(address+3) + (((uint16_t)EEPROM.read(address+4)) << 8);
        commandData[2] = EEPROM.read(address+5) + (((uint16_t)EEPROM.read(address+6)) << 8);
        commandData[3] = EEPROM.read(address+7) + (((uint16_t)EEPROM.read(address+8)) << 8);
        address = address + 9;
        commandDataPointer = 4;
        if (cmd == 'I') CmdInterval();
        if (cmd == 'T') CmdThreshold();   
        if (cmd == 'A') CmdAdd();
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


/* todo: include analog Pins as well as analog limits in save / restore */
// idea: store character string in eeprom such that it can be feeded to handleInput

void CmdSaveToEEPROM() {
    int address   = 0;
    uint8_t slots = maxPinIndex + 2;
    updateEEPROM(address, 'C');
    updateEEPROM(address, 'f');
    updateEEPROM(address, 'g');
    updateEEPROM(address, slots);                   // number of defined pins + intervall definition
    updateEEPROMSlot(address, 'I', (uint16_t)(intervalMin / 1000), (uint16_t)(intervalMax / 1000), 
                                  (uint16_t)(intervalSml / 1000), (uint16_t)countMin);
    // updateEEPROMSlot(address, 'T', (uint16_t)analogThresholdMin, (uint16_t)analogThresholdMax, 0, 0);
    for (uint8_t pinIndex=0; pinIndex < maxPinIndex; pinIndex++) 
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



void CmdInterval() {
    if (!checkVal(0,1,3600)) return;                    // index 0 is interval min, min 1, max 3600 (1h), std error
    intervalMin = (uint32_t)commandData[0] * 1000;      // convert to miliseconds

    if (!checkVal(1,1,3600)) return;                    // index 1 is interval max, min 1, max 3600 (1h), std error
    intervalMax = (uint32_t)commandData[1]* 1000;       // convert to miliseconds

    if (!checkVal(2,0,3600)) return;                    // index 2 is interval small, max 3600 (1h), std error
    intervalSml = (uint32_t)commandData[2] * 1000;      // convert to miliseconds

    if (!checkVal(3,0,100)) return;                     // index 3 is count min, max 100, std error
    countMin = commandData[3];

    Output->print(F("M intervals set to ")); Output->print(commandData[0]);
    Output->print(F(" ")); Output->print(commandData[1]);
    Output->print(F(" ")); Output->print(commandData[2]);
    Output->print(F(" ")); Output->println(commandData[3]);
}


// idea: remove t command and add thresholds to add command
// remove w command and add to interval command

void CmdThreshold() {    
    if (!checkVal(0,1,1023)) return;                            // index 0 is analog threshold min, min 1, max 1023, std error
    for (uint8_t aIdx = 0; aIdx < maxAnalogIndex; aIdx++)
        analogData[aIdx].thresholdMin = commandData[0];         // set all threasholds until new command is implemented and threshold is set in add

    if (!checkVal(1,1,1023)) return;                            // index 1 is analog threshold max, min 1, max 1023, std error
    for (uint8_t aIdx = 0; aIdx < maxAnalogIndex; aIdx++)
        analogData[aIdx].thresholdMax = commandData[1];         // set all threasholds until new command is implemented and threshold is set in add

    Output->print(F("M analog thresholds set to ")); Output->print(commandData[0]);
    Output->print(F(" ")); Output->println(commandData[1]);
}

/*
    handle add command.
*/
void CmdAdd () {
    uint8_t aPin = commandData[0];              // commandData[0] is pin number
    if (!checkPin(aPin)                         // is pin allowed?
         || (ledOutPin && aPin == ledOutPin)    // is pin used as led out?
         || findOutPin(aPin) != FF)  {          // is pin used as output so far?
        PrintPinErrorMsg(aPin);
        return;
    }
    uint8_t rPin = pin2GPIO(aPin);              // get gpio pin number for later use
    uint8_t pinIndex = findInPin(aPin);         // is pin already in use counting?
    if (pinIndex == FF) {                       // not used so far
        pinIndex = maxPinIndex;                 // use next available index
        initPinVars(&pinData[pinIndex], millis());
    }
    pinData_t *pd = &pinData[pinIndex];         // pinData entry to work with

    if (!checkVal(1,2,3)) return;               // index 1 is pulse level, min 2, max 3, std error
    pd->pulseLevel = (commandData[1] == 3);     // 2 = falling -> pulseLevel 0, 3 = rising -> pulseLevel 1
    pd->pinName = aPin;                         // save printed pin number for reporting

    if (checkVal(2,0,1, false))                 // index 2 is pullup, optional, no error message if omitted
        pd->pullupFlag = commandData[2];        // as defined
    else pd->pullupFlag = 0;                    // default to no pullup

    if (!checkVal(3,1,1000, false)) 
        commandData[3] = 2;                     // value 3 is min length, optional. Assume default 2 if invalid
    pd->pulseWidthMin = commandData[3];
    
    // todo: move this to t command, analog conf with max 4 parameters keeps save to eeprom?
    // t pin out tmin tmax?
    // or modify save functions, add thresholds to add but allow t command to modify thresholds?
    // or just have add command to modify thresholds ?
    
    if (checkVal(4,1,MAX_APIN, false)) {
        // 4 - analog out pin number given. This must not be used as input, other output / led
        uint8_t oPin = commandData[4];
        if (!checkPin(oPin) || (ledOutPin && oPin == ledOutPin) || findInPin(oPin) != FF)  {          
            PrintPinErrorMsg(oPin);             // pin alreday used as input or ledout (analog out would be ok)
            return;
        }

        uint8_t analogIndex = findAnalogData(pd);           // reuse or create analog data entry for aPin with oPin
        if (analogIndex == FF) {
            if (maxAnalogIndex < MAX_ANALOG) {
                analogIndex = maxAnalogIndex++;             // new entry -> initialize
                analogData_t *ad = &analogData[analogIndex];
                ad->thresholdMin = 0;
                ad->thresholdMax = 0;
                ad->inPinData = pd;
            } else {
                PrintErrorMsg();
                Output->println(F("too many analog pins"));
            }
        }
        pd->analogFlag = 1;
        analogData_t *ad = &analogData[analogIndex];
        ad->inPinName = aPin;
        ad->outPinName = oPin;
    }
        
    pd->pinName = aPin;
    maxPinIndex++;                              // increment used entries
    
    if (!pd->analogFlag) {
        if (pd->pullupFlag)
            pinMode (rPin, INPUT_PULLUP);
        else 
            pinMode (rPin, INPUT); 
#if defined(ESP8266) || defined(ESP32)
        attachInterruptArg(digitalPinToInterrupt(rPin), ESPISR, pd, CHANGE);
#elif defined(__AVR_ATmega328P__)        
        AddPinChangeInterrupt(rPin);
        pinIndexMap[aPin] = pinIndex;
#endif        
    } else {
        pinMode (rPin, INPUT);  
    }
    Output->print(F("M defined ")); showPinConfig(pd); Output->println();
}
// analog thresholds are defined with t command (new: also for individual pins)
// analog delay and sample number with command w   


/*
    handle rem command.
*/
void CmdRemove() {
    uint8_t aPin = commandData[0];              // commandData[0] is pin number
    uint8_t pinIndex = findInPin(aPin);
    if (pinIndex == FF) return;                 // pin is currently not used as input
    pinData_t *pd = &pinData[pinIndex];         // config entry to work with

    if (!pd->analogFlag) {                      
#if defined(ESP8266) || defined(ESP32)
        detachInterrupt(digitalPinToInterrupt(pin2GPIO(aPin)));
#elif defined(__AVR_ATmega328P__)
        RemovePinChangeInterrupt(rPin);
        pinIndexMap[aPin] = FF;
#endif
    } else {
        // find analog data entry
        uint8_t analogIndex = FF;
        for (uint8_t aIdx = 0; aIdx < maxAnalogIndex; aIdx++) {
            if (analogData[aIdx].inPinData == pd) {
                analogIndex = aIdx;
                break;
            }
        }
        if (analogIndex != FF) {
            // copy idx +1 and following up and clear the last
            // then dec max
            for (uint8_t rIdx = analogIndex; (rIdx + 1) < maxAnalogIndex; rIdx++)
                analogData[rIdx] = analogData[rIdx + 1];
            analogData_t *lastAd = &analogData[--maxAnalogIndex];
            lastAd->inPinData = 0;
            lastAd->inPinName = 0;
            lastAd->outPinName = 0;
        }
    }
    
    for (uint8_t rIdx = pinIndex; (rIdx + 1) < maxPinIndex; rIdx++)
        pinData[rIdx] = pinData[rIdx + 1];
    pinData_t *lastPd = &pinData[--maxPinIndex];
    initPinVars(lastPd, 0);
    
    Output->print(F("M removed ")); Output->println(aPin);

}


/* give status report in between if requested over serial input */
void CmdShow() {
    uint32_t now = millis();  
    Output->print(F("M Status: ")); printVersionMsg();
    Output->println();
    
#if defined(WifiSupport)
    printConnection(Output);
#endif    

    showIntervals();    
    showThresholds();

    Output->print(F("V")); Output->println(devVerbose);
    
    for (uint8_t pinIndex=0; pinIndex < maxPinIndex; pinIndex++) {
        pinData_t *pd = &pinData[pinIndex];
        showPinConfig(pd);
        Output->print(F(", "));
        showPinCounter(pd, true, now);
#ifdef pulseHistory             
        showPinHistory(pd, now);
#endif          
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
        if (pin2GPIO(aPin) != FF) {
            if (!first) {
                Output->print(F(","));
            } else {
                first = false;
            }
            Output->print(aPin);            // show available pins
        }
    }
    Output->print(F(" available"));
    Output->print(F(" T")); Output->print(now);
    Output->print(F(","));  Output->print(millisWraps);
    Output->print(F(" B")); Output->print(bootTime);
    Output->print(F(","));  Output->print(bootWraps);
    
    Output->println();
    showIntervals();
    showThresholds();
    Output->print(F("V"));
    Output->println(devVerbose);
    
    for (uint8_t pinIndex=0; pinIndex < maxPinIndex; pinIndex++) { // go through all observed pins as pinIndex
        showPinConfig(&pinData[pinIndex]);
        Output->println();
    }
}


void CmdLED() {
    // set monitor ouput LED and a max of 5 pins to monitor
    // 12,2,3l would set pin 12 as LED and pins 2 and 3 to create LED switches when their level changes
    // dont allow 0, check if already used as led, if not checkPin
    uint8_t aPin = commandData[0];                   // commandData[0] is lec pin number
    
    if (!aPin || !checkPin(aPin) || findOutPin(aPin) != FF || findInPin(aPin) != FF) {
        PrintPinErrorMsg(aPin);                 // illegal pin or already used for in/other output
        return;
    }
    
    // save pin and validate other params / pins to monitor
}


// interval to i command, samples to add command ?
// need new fhem module anyway so commands can be rearranged.
// new module will then require this new sketch

void CmdWait() {
    if (!checkVal(0,0,10000)) return;
    analogReadInterval = (int)commandData[0];

    if (!checkVal(1,1,100,false)) return;
    //analogReadSamples = (uint8_t)commandData[1];
}


// combine devverbose and debug and make it multi param for diffeent things ..


void CmdDevVerbose() {    
    if (!checkVal(0,0,50)) return;                // index 0 is devVerbose level, max 50, std error
    devVerbose = commandData[0];
    Output->print(F("M devVerbose set to ")); Output->println(commandData[0]); 
}
            
            
void CmdKeepAlive() {
    if (commandData[0] == 1 && commandDataSize > 0) {
        Output->print(F("alive"));
#ifdef WifiSupport
        uint32_t now = millis();
        if (devVerbose >=5) {
            Output->print(F(" RSSI ")); Output->print(WiFi.RSSI());
        }
        if (commandData[0] == 1 && commandDataSize > 0 && commandDataSize < 3 && Client1.connected()) {
            tcpMode = true;
            if (commandDataSize == 2) {
                keepAliveTimeout = commandData[1];  // timeout in seconds (on ESP side we use it times 3)
            } else {
                keepAliveTimeout = 200;             // *3*1000 gives 10 minutes if nothing sent (should not happen)
            }
        }  
        lastKeepAlive = now;
#endif
        Output->println();
    }
}


 
void CmdQuit() {
#ifdef WifiSupport
    if (Client1.connected()) {
        Client1.println(F("closing connection"));
        Client1.stop();
        tcpMode =  false;
        Serial.println(F("M TCP connection closed"));
        return;
    } 
#endif
    Serial.println(F("M TCP not connected"));
}


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
            Serial.print(F(" size ")); Serial.println(commandDataSize);
        }

        switch (c) {
        case 'a':                       // add a pin
            CmdAdd(); break;
        case 'd':                       // delete a pin
            CmdRemove(); break;
        case 'e':                       // save to EEPROM
            CmdSaveToEEPROM(); break; 
        case 'f':                       // flash ota
            break; 
        case 'h':                       // hello
            CmdHello(); break;
        case 'i':                       // interval
            CmdInterval(); break;
        case 'k':                       // keep alive
            CmdKeepAlive(); break;   
        case 'l'    :                       // keep alive
            CmdLED(); break;   
            
        // o / p should be ?
         
        case 'q':                       // quit
            CmdQuit(); break; 
        case 'r':                       // reset - todo: maybe do real device reset here?
            initialize(); break;
        case 's':                       // show
            CmdShow(); break;
        case 'v':                       // dev verbose
            CmdDevVerbose(); break;

        case 't':                       // thresholds for analog pins (one or all)
            CmdThreshold(); break;
        case 'w':                       // wait - delay between analog reads and other analog read parameters
            CmdWait(); break;
        default:
            break;
        }
        clearInput();
        //Serial.println(F("D End of command"));
    }
}


#ifdef debugPins
void debugPinChanges() {
    for (uint8_t pinIndex=0; pinIndex < maxPinIndex; pinIndex++) {
        pinData_t *pd = &pinData[pinIndex];        
        uint8_t pinState = digitalRead(pin2GPIO(pd->pinName));
                    
        if (pinState != pd->lastDebugLevel) {
            pd->lastDebugLevel = pinState;
            Output->print(F("M pin "));       Output->print(pd->pinName);
            Output->print(F(" changed to ")); Output->print(pinState);
#ifdef pulseHistory                     
            Output->print(F(", histIdx "));   Output->print(histIndex);
#endif                  
            Output->print(F(", count "));     Output->print(pd->counter);
            Output->print(F(", reject "));    Output->print(pd->rejectCounter);
            Output->println();
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
        Serial.println(F("M Try reconnecting WiFi"));
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
            remote = Client2.remoteIP();
            Client2.println(F("connection already busy"));
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


void detectTrigger(analogData_t *ad, int val) {
    uint8_t nextState = ad->triggerState;       // set next trigger level to be the same as the old one 
    if (val > ad->thresholdMax) {
        nextState = 1;                          // if above upper threshold then 1
    } else if (val < ad->thresholdMin) {
        nextState = 0;                          // if below lower threshold then 0
    }                                           // otherwise it stays as old level
    ad->triggerState = nextState;               // save new level
    
    if (ledOutPin)
        digitalWrite(ledOutPin, ad->triggerState);

    doCount (ad->inPinData, ad->triggerState);  // do the counting, history and so on
    
#ifdef debugPins
    if (devVerbose >= 10) {
        Output->print(F("M pin "));      Output->print(ad->inPinName);
        Output->print(F(" ) "));
        Output->print(F(" to "));        Output->print(nextState);
#ifdef pulseHistory                     
        Output->print(F("  histIdx "));  Output->print(histIndex);
#endif                  
        Output->print(F("  count "));    Output->print(ad->inPinData->counter);
        Output->print(F("  reject "));   Output->print(ad->inPinData->rejectCounter);
        Output->println();
    }
#endif
}


void readAnalog() {
    uint32_t now = millis();
    uint16_t interval2 = analogReadInterval + 2;            // last read + interval + 2ms -> read off value (state 2)
    uint16_t interval3 = analogReadInterval + 8;            // last read + interval + 4ms -> read on value (state 5)
    if ((now - analogReadLast) > analogReadInterval) {      // time for next analog read?
        Serial.print(F("AR")); Serial.print(analogReadState); Serial.print(F(" ")); 
        switch (analogReadState) {
            case 0:                                                 // initial state
                for (uint8_t aIdx = 0; aIdx < maxAnalogIndex; aIdx++) {                    
                    analogData_t *ad = &analogData[aIdx];
                    Serial.print(F("ap")); Serial.print(ad->outPinName); Serial.print(F(" ")); 
                    digitalWrite(pin2GPIO(ad->outPinName) , HIGH);   // make sure IR LED is off for first read
                    analogReadCount = 0;                            // initialize sums and counter
                    Serial.print(F("gpio ")); Serial.print(pin2GPIO(ad->outPinName)); Serial.print(F(" off ")); 
                    Serial.println();
                    ad->sumOff = 0; ad->sumOn = 0;
                }
                analogReadState = 1;
                break;
            case 1:                                                 // wait before measuring
                if ((now - analogReadLast) < interval2)
                    return;
                analogReadState = 2;
                break;
            case 2:
                for (uint8_t aIdx = 0; aIdx < maxAnalogIndex; aIdx++) {
                    analogData_t *ad = &analogData[aIdx];
                    ad->sumOff += analogRead(pin2GPIO(ad->inPinName));  // read the analog in value (off)
                }
                if (++analogReadCount < analogReadSamples)
                    break;
                for (uint8_t aIdx = 0; aIdx < maxAnalogIndex; aIdx++) {
                    analogData_t *ad = &analogData[aIdx];
                    digitalWrite(pin2GPIO(ad->outPinName), HIGH);      // turn IR LED on
                    Serial.print(F("gpio ")); Serial.print(pin2GPIO(ad->outPinName)); Serial.print(F(" on ")); 
                    Serial.println();
                }
                analogReadCount = 0;
                analogReadState = 4;
                break;
            case 4:                                                 // wait again before measuring
                if ((now - analogReadLast) < interval3)
                    return;
                analogReadState = 5;
                break;
            case 5:
                int sensorDiff;
                for (uint8_t aIdx = 0; aIdx < maxAnalogIndex; aIdx++) {
                    analogData_t *ad = &analogData[aIdx];
                    ad->sumOn += analogRead(pin2GPIO(ad->inPinName));   // read the analog in value (on)
                }
                if (++analogReadCount < analogReadSamples) 
                    break;
                for (uint8_t aIdx = 0; aIdx < maxAnalogIndex; aIdx++) {
                    analogData_t *ad = &analogData[aIdx];
                    digitalWrite(pin2GPIO(ad->outPinName) , LOW);       // turn IR LED off again
                
                    sensorDiff = (ad->sumOn / analogReadSamples) - (ad->sumOff / analogReadSamples);
                    if (sensorDiff < 0) sensorDiff = 0;
                    if (sensorDiff > 4096) sensorDiff = 4096;
                    detectTrigger (ad, sensorDiff);                    // calculate level with triggers
                    if (devVerbose >= 25) {
                        char line[26];
                        sprintf(line, "L%2d: %4d, %4d -> % 4d", ad->inPinName, 
                            ad->sumOn / analogReadSamples, ad->sumOff / analogReadSamples, sensorDiff);
                        Output->println(line);
                    } else if (devVerbose >= 20) {
                        Output->print(F("L%2d: "));
                        Output->println(sensorDiff);
                    }                    
#if defined(TFT_DISPLAY)
                    int len = sensorDiff * analogReadAmp * TFT_HEIGHT / 4096;
                    tft.fillRect(0,TFT_WIDTH-10-(10*aIdx),len,10, TFT_YELLOW);
                    tft.fillRect(len,TFT_WIDTH-10-(10*aIdx),TFT_HEIGHT-len,10, TFT_BLACK);
#endif
                }
                analogReadState = 0;
                analogReadLast = now;
                break;
            default:
                analogReadState = 0;
                Output->println(F("D error: wrong analog read state"));
                break;
        }
    }
}


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
    readAnalog();                           // analog measurements
#ifdef debugPins
    if (devVerbose >= 10)
        debugPinChanges();
#endif
    if (reportDue())
        report();                           // report counts
}
