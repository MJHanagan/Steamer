// #include <Arduino.h>
#include "RTC Functions.h"

char timestamp[50];

RTC_DS3231 rtc;  // RTC in use on the V1.0 board
// RTC_PCF8523 rtc; // RTC in use on the Adafruit Adalogger board
DateTime now;

bool initRTC() {

    if ( !rtc.begin() ) {
        Serial.println( "DS3231 RTC not found on I2C bus." );
        return false;
    }

    // rtc.adjust( DateTime( F(__DATE__), F(__TIME__) ) );

    getNow();
    // Serial.printf( "RTC initialized: %d run status: %d startup time: %s\n", rtc.initialized(), rtc.isrunning(), getTimestamp() );
    Serial.printf( "RTC startup time: %s\n", getTimestamp() );
    
    if ( rtc.lostPower() ) {
        Serial.println("RTC has no time setting.  Setting to this file date and time.");
        // When time needs to be set on a new device, or after a power loss, the
        // following line sets the RTC to the date & time this sketch was compiled
        rtc.adjust( DateTime( F(__DATE__), F(__TIME__) ) );
        // rtc.start();
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust( DateTime(2024, 9, 28, 17, 1, 29) );
    }

    

    // rtc.adjust( DateTime(2024, 9, 28, 23, 5, 33) );

    // rtc.start();

    return true;

}

DateTime getNow() {
    now = rtc.now();
    return now;
}

char* getYYYYMMDD() {
    sprintf( timestamp, "%04d%02d%02d", now.year(), now.month(), now.day() );
    return timestamp;
}
char* getTimestamp() {
    sprintf( timestamp, "%04d-%02d-%02dT%02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second() );
    return timestamp;
}
char* getHHMMSS() {
    sprintf( timestamp, "%02d:%02d:%02d", now.hour(), now.minute(), now.second() );
    return timestamp;
}
char* dirYYYYMMDD() {
    sprintf( timestamp, "%04d%02d%02d", now.year(), now.month(), now.day() );
    return timestamp;
}
char* fileHHMMSS() {
    sprintf( timestamp, "%02d_%02d_%02d", now.hour(), now.minute(), now.second() );
    return timestamp;
}
uint8_t setRTCtime( uint8_t hh, uint8_t mm, uint8_t ss ) {

    getNow();
    rtc.adjust( DateTime( now.year(), now.month(), now.day(), hh, mm, ss) );

    return true;

}