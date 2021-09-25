# ArduCounter
pulse counter / energy or water consumption calculator for Arduino, ESP8266 or ESP32 to be used with smart home platforms like Fhem

This firmware implements a counter for pulses on any input pin of an Arduino Uno, Nano, Jeenode, NodeMCU, Wemos D1, TTGO T-Display or similar device.
It was designed to be used with Fhem but could also be used in other environments.
The device connects to Fhem either through USB / serial or via Wifi / TCP if an ESP board is used. 
ArduCounter does not only count pulses but also measure pulse lenghts and the time between pulses so it can filter noise / bounces and 
gives better power/flow (Watts or liters/second) readings than systems that just count in fixed time intervals. 
The typical use case is an S0-Interface on an energy meter or water meter, but also reflection light barriers to monitor old ferraris counters 
or analog water meters are supported. On the Fhem side counters are configured with attributes that define which GPIO pins should count pulses and in which intervals 
the board should report the current counts. The firmware uses pin change interrupts so it can efficiently count pulses on all available input pins. 
It has been tested with 14 inputs of an Arduino Uno counting in parallel and pulses as short as 3 milliseconds. 
The corresponding Fhem side creates readings for pulse counts, consumption and optionally also a pin history with pulse lengths and gaps of the last pulses. 
If an ESP8266 or ESP32 is used, the device can be flashed and configured over Wifi (it opens its own temporary Hotspot / SSID for configuration 
so you can set which existing SSID to connect to and which password to use). 
For TTGO T-Display boards (ESP32 with TFT display) the local display on the device itself can also display Wifi status and current consumption.
