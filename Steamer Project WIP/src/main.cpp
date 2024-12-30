// #include <Arduino.h>

#include "shared.h"

uint8_t success;

// Instantiate the heater, pump and pressure sensor objects
HeaterControl heater( HEATER_PIN, TC1_PIN, TC2_PIN, TC3_PIN );
PumpControl pump( PUMP_PIN, FLOW_PIN );
PressureControl pressure( ADC_PIN, 3.3 );

void setup() {

    // Save the bootup time
    bootTime = millis();

    // Make a bit of sound here to announce the startup has begun
    tone( PIEZO_PIN, 400, 200 );
    
    // Turn on the onboard LED
    pinMode( LED_BUILTIN, OUTPUT ); digitalWrite( LED_BUILTIN, HIGH );

    // Initialize the LCD
    success = InitLCD();

    // Initialize the USB serial communication
    Serial.begin(115200);
    // If we are in DEBUG mode wait 5 seconds to give the user time to start the serial interface on their end
    if(DEBUG) {
        for (int i=5; i>0; i-- ) { 
            Serial.printf( " %d", i );
            TextAt(1,0,""); Serial1.printf( "Serial: %d...", i );
            delay( 1000 );
            tone( PIEZO_PIN, 500, 100 );
        }
    }

    // Initalize the ANO input device
    initMsg( 1, "InitANO()" );
    passFail( InitANO() );
    
    // Initlaize the RTC device
    initMsg( 2, "initRTC()" );
    passFail( initRTC() );

    // Initlaize the SD card device
    initMsg( 3, "initSDcard()" );
    passFail( initSDcard() );  // NOTE! Must run InitRTC() before running this function!!
    
    // Initlaize the heater control routines
    initMsg( 1, "heater.Init()" );
    passFail( heater.Init() );
    
    // Initialize the flud pump control routines
    initMsg( 2, "pump.Init()" );
    passFail( pump.Init() );
    
    // Initialize the pressure sensor routines
    initMsg( 3, "pressure.Init()" );
    passFail( pressure.Init() );    

    // Set the heater and pump parameters for water
    setFluid( "WATER" );

    // Initialize the WiFi connection
    initMsg( 1, "initWiFi()" );
    passFail( initWiFi( 5 ) );

    // Initialize the MQTT client connection and subscriptions  //boolean rc = mqttClient.publish("test", "This is a test message");
    initMsg( 2, "initMQTT()" );
    passFail( initMQTT() );

    // If we were successful in connecting with the MQTT broker then send the "startup" event message and request a timesync
    if( mqttActive ) {
        LogEvent( "SYSTEM STARTUP" );
        // Request the current datetime from the MQTT server
        success = timeSync( "datetime?" );
        if ( success ) {
            Serial.printf( "\nMQTT time is: %s\n RTC time is: %s", timeSyncData, getTimestamp() );
            mqttNewMsg = false;
        }
    }

    delay(1000);
    // Update the LCD
    UpdateLCD( false );

    // Clear any ANO input (INT) activity
    checkANO();

    // Turn off the onboard LED
    digitalWrite( LED_BUILTIN, LOW );

    // Reset the SD card and MQTT data update times
    OpResetLogTimes();    

    // Start in the idle mode
    IdleMode( "START" );

    // Serial.printf( "End of setup() Flow counter: %u", pump.GetFlowCounts() );

    // Announce the end of setup()
    Serial.print( "Exiting setup()\n" );

    loopStartTime = millis(); // This is used to determine the loop() speed

}

void loop() {
    
    if ( millis()-loopStartTime >= 1000 ) {
        loopHz = loopCounter;
        loopStartTime += 1000;
        loopCounter = 0;
    }
    
    heater.Update();  // Update the heater activities
    pump.Update();  // Update the pump activities
    pressure.Update(); // Update the pressure sensor    
    
    OpUpdate();  // Update the current operation mode functions
    OpLogData();  // Log the latest data is using the SD card, MQTT and USB serial
    OpCheckSerial();  // Check for incoming commands USB serial
    OpCheckMQTT();  // Check for incoming commands via MQTT

    UpdateLCD( false );  // Update the LCD

    loopCounter += 1;

}