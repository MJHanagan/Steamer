// #include <Arduino.h>
#include "Data Functions.h"
// #include "RTC Functions.h"
// #include <SdFat.h>  // <-- this is needed to get the definitions of "FAT_TIME" and "FAT_DATE" used in setSDdatetime()

char dirName[10];
char fileName[12]; //= "Steamy";
char fullFilename[50];
bool sdCard;

File sdFile;

bool initSDcard() {
    

    #ifdef SD_DET_PIN
        pinMode( SD_DET_PIN, INPUT_PULLUP );
        if( digitalRead( SD_DET_PIN ) ) {
            Serial.print( "SD card not present.\n" );
            sdCard = false;
            return false;
        }
        Serial.print( "SD card detected.\n" );
    #endif 
    
    Serial.print( "Starting SPI for SD...");
    sdCard = SD.begin( SD_CS_PIN );

    if ( !sdCard ) {
        Serial.print( " SD.begin() FAILED.\n");
        return false;
    }

    Serial.print( " Started.\n");
    // Get date from RTC
    getNow();

    // Check if 'today' as a folder exists
    sprintf( dirName, "/%s", dirYYYYMMDD() );
    Serial.printf( "Checking for directory: %s", dirName );

    if( !SD.exists( dirName ) ) {
        Serial.print( " Creating..." );
        if ( SD.mkdir( dirName ) ) {
            Serial.print( " exits.\n" );
        } else {
            Serial.print( " ** FAILED **.\n" );
            return false;
        }
    }

    // Create the full file name
    sprintf( fullFilename, "%s/%s.dat", dirName, fileHHMMSS() );
    
    // Open this file for write/append
    sdFile = SD.open( fullFilename, FILE_APPEND );

    if( !sdFile ) {
        Serial.printf( "Error opening file '%s'\n", fullFilename );
        sdCard = false;
        return false;
    }

    Serial.printf( " File created: %s\n", fullFilename );

    // Get current time from DS3231 RTC
    getNow();

    sdFile.print( "TS=" );
    sdFile.print( getTimestamp() );
    sdFile.print( ",EVENT=Startup,millis=" );
    sdFile.print( millis() );
    sdFile.print( "\n" );
    
    sdFile.close();

    return true;
}

bool writeToSDcard( const char * charBuffer ) {

    if( !sdCard ) {
        Serial.print( "No sdCard object.  " );
        return false;
    }

    #ifdef SD_DET_PIN
        if( digitalRead( SD_DET_PIN ) ) { Serial.print( "\n** SD card not present.\n" ); return false; }
    #endif

    digitalWrite( LED_BUILTIN, HIGH );

    sdFile = SD.open( fullFilename, FILE_APPEND );

    if( !sdFile ) {
        Serial.printf( "\n** Failed to open %s\n", fullFilename );
        return false;
    }

    sdFile.print( charBuffer );

    // Write a \n to the file if needed, but do not duplicate.
    if ( charBuffer[strlen(charBuffer)] != '\n' ) sdFile.print( '\n' );

    sdFile.close();

    digitalWrite( LED_BUILTIN, LOW );

    // Serial.print( "\nSD write: "); Serial.print( charBuffer ); Serial.print( "\n" );
    
    return true;
}

// bool publishDataToMQTT( const char *charBuffer ) {

//     digitalWrite( LED_BUILTIN, HIGH );

//     // Serial.print( "@publishDataToMQTT()\n");

//     digitalWrite( LED_BUILTIN, LOW );

//     return true;
// }