#pragma once

#define VERSION "Steamy V1.0.1"

#define DEBUG true // Comment out this line to disable debug print statements

#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print( x ); // Debug print
    #define DEBUG_PRINTLN(x) Serial.println( x ); // Debug print
#else
    #define DEBUG_PRINT(x) // Do nothing
    #define DEBUG_PRINTLN(x) // Do nothing
#endif


#define MAX_BUFFER_SIZE 512  // Maximum data size for MQTT messages (the default is a measly 128 bytes, yikes!)

#define SS_SWITCH_SELECT 1
#define SS_SWITCH_UP     3
#define SS_SWITCH_LEFT   4
#define SS_SWITCH_DOWN   5
#define SS_SWITCH_RIGHT  2

// Adafruit HUZZAH32 GPIO pin assignments for board version 1.0, see comment for FLOW_PIN
#define ANO_INT_PIN 34 // ANO seesaw INT pin, this is an input only pin and ISR compatable when WiFi is running
#define HEATER_PIN 33 // Drives the 20A SSR powering the Gaggia boiler
#define PUMP_PIN 27 // Drives the 1A SSR powering the UKLA vibratory pump
#define PIEZO_PIN 12 // Drives the piezo speaker
#define ADC_PIN 21 // 
#define SD_DET_PIN 26 // Micro SD card detect (not available on Adafruit Adalogger shield)
#define SD_CS_PIN 25 // Micro SD CS
// #define SD_CS_PIN 33 // Micro SD CS for the Adafruit Adalogger shield
#define RTC_INT_PIN 36 // RTC INT - don't program as an ISR pin because it gets false ISR triggers when WiFi is running
#define FLOW_PIN 16 // Output of the flow sensor - use 16 for V1.0 board and the top PCB adder, the original 39 doesn't work as an ISR with WiFi active
#define TC1_PIN 4 // /CS pin on the MAX31855 TC sensor for the main boiler thermocouple
#define TC2_PIN 15 // /CS pin on the MAX31855 TC sensor for the outlet thermocouple
#define TC3_PIN 14 // /CS pin on the MAX31855 TC sensor for the oven thermocouple
#define CS_XTRA 32 // / XTRA SPI connector S pin
#define TX_PIN 17 // Not needed but included
// For debugging only:
//#define FSS_PIN 16 // Normally RX. Debugging pin to drive the input of the flow sensor -- GPIO 39

#include <Arduino.h>

#include <Heater Functions.h>
// Note: 'heater' is declared here and defined in main.cpp
extern HeaterControl heater;

#include <Pump Functions.h>
// Note: 'pump' is declared here and defined in main.cpp
extern PumpControl pump;

#include <RTC Functions.h>
// Note: 'rtc' is declared here and defined in RTC Functions.cpp
extern RTC_DS3231 rtc;  // Used for the custom board (V1.0)
// extern RTC_PCF8523 rtc;  // RTC on the Adafruit Adalogger
extern DateTime now;

#include "Pressure Functions.h"
extern PressureControl pressure;

#include "ANO Input.h"
#include "Operation Functions.h"
#include <Heater Functions.h>
#include <Pump Functions.h>
#include "LCD Functions.h"
#include "Data Functions.h"
#include "RTC Functions.h"

#include "WiFi MQTT.h"

#define debug
