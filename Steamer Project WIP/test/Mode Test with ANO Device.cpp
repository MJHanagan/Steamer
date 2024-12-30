#include <Arduino.h>

/*
 * This example shows how to read from a seesaw encoder module.
 * The available encoder API is:
 *      int32_t getEncoderPosition();
        int32_t getEncoderDelta();
        void enableEncoderInterrupt();
        void disableEncoderInterrupt();
 */
#include "Adafruit_seesaw.h"

String SwitchNames[] = { "NONE", "SELECT", "UP", "LEFT", "DOWN", "RIGHT" };
#define SS_SWITCH_SELECT 1
#define SS_SWITCH_UP     2
#define SS_SWITCH_LEFT   3
#define SS_SWITCH_DOWN   4
#define SS_SWITCH_RIGHT  5

#define ANO_INT_PIN    14

#define SEESAW_ADDR      0x49

#define iModes 3
#define jModes 3
#define pModes 5
#define nLevels 5

Adafruit_seesaw ss;
int32_t encoder_position;
int8_t iANO;


String opMode;
String myarray[2][3] = {
  { "hello", "jack", "dawson" }, 
  { "hello", "hello", "hello" }
};

String mainMode, mainModes[] = { "OFF", "RUN", "SETUP" };
uint8_t mainModeIndex;

String runMode, runModes[] = { "PREHEAT", "READY", "STEAM" };
uint8_t runModeIndex;

String setupParameter, setupParameters[] = { "MM", "SS", "SP", "PULSE", "TIME", "SAVE" };
uint8_t setupParameterIndex;

String settingName, settingNames[] = { "MM", "SS", "SP", "PULSE", "TIME" };
uint8_t settingIndex;

volatile uint8_t intFlag;
volatile uint32_t intTime, intCount;

uint8_t activeFlag, activeSwitches, switchStates, rotaryFlag;
uint32_t startTime, pressedTime;

uint8_t iMode, jMode, pMode;

void IRAM_ATTR INTevent() {
  intFlag = true;
  intTime = millis();
  ++intCount;
  return;
}

int8_t checkANO() {

  if (!intFlag) {  // There is no INT event to process so check for a few other special conditions before returning

    // Return if switch press has exceeded 5000 mSec
    if ( activeFlag and millis()-startTime > 5000 ) return -1;

    // Return with 0 if the INT pin is not low
    if ( digitalRead( ANO_INT_PIN ) ) return 0;

    // The INT pin has been low (active) low for <500 mSec return now with a 0
    if ( millis()-startTime < 500 ) return 0;
    
    // Read the state of one of the switches, this should clear the INT condition
    ss.digitalRead( SS_SWITCH_SELECT );
    delay( 5 );

    // Return a 0 if the INT condition cleared.
    if ( digitalRead( ANO_INT_PIN ) ) return 0;  

    // If the read didn't clear it then return with a -2
    return -2;

  }

  // We have an INT event to process, so read the state of the 5 input switches
  switchStates = ss.digitalReadBulk( 0b111110 );
  // Shift the fisrt bit out leaving just the 5 input switches and invert the bits (switch inputs are active low)
  switchStates = ~( switchStates >>1 ) & 0b11111;
  // Clear the intFlag
  intFlag = false;


  if ( activeFlag ) {
    // We have an active switch press event in play, so test if this is the switch release event
    if ( switchStates==0 ) {
      // All switches are inactive so the press event is over, calculate the pressed time
      pressedTime = intTime - startTime;
      // Clear the activeFlag
      activeFlag = false;
      // Return with the active switch value
      return activeSwitches;
    }
  } else {
    // There is no current switch press so this is a new switch active event
    if ( switchStates ) {
      // Save the switch condition
      activeSwitches = switchStates;
      // Save the starting time
      startTime = intTime;
      // Set the activeFlag
      activeFlag = true;
      // Return with a 0
      return 0;
    } else {
      // We have an INT even but no switches show activity, therefore this must be a rotary encode event, so return with a -3
      rotaryFlag = true;
      return -3;
    }

  }

  // We should never get here, so return with this doozy value!
  return -4;

}

void TurnOff() { Serial.println("@TurnOff()"); return; }
void SetupMode() { Serial.println("@SetupMode()"); return; }
void RunMode() { Serial.println("@RunMode()"); return; }
void StartSteamCycle() { Serial.println("@StartSteamCycle()"); return; }
void StopSteamCycle() { Serial.println("@StopSteamCycle()"); return; }
void ExitSetupMode() { Serial.println("@ExitSetupMode()"); return; }


void setup() {

    int i;
    Serial.begin(115200);

    for (i=10; i>0; i-- ) { Serial.printf( " %d", i ); delay( 1000 ); } Serial.println( i );


    Serial.println("Looking for seesaw!");

    if (! ss.begin(SEESAW_ADDR)) {
    Serial.println("Couldn't find seesaw on default address");
    while(1) delay(10);
    }
    Serial.println("seesaw started");
    uint32_t version = ((ss.getVersion() >> 16) & 0xFFFF);
    if (version  != 5740){
    Serial.print("Wrong firmware loaded? ");
    Serial.println(version);
    while(1) delay(10);
    }
    Serial.println("Found Product 5740");

    ss.pinMode(SS_SWITCH_UP, INPUT_PULLUP);
    ss.pinMode(SS_SWITCH_DOWN, INPUT_PULLUP);
    ss.pinMode(SS_SWITCH_LEFT, INPUT_PULLUP);
    ss.pinMode(SS_SWITCH_RIGHT, INPUT_PULLUP);
    ss.pinMode(SS_SWITCH_SELECT, INPUT_PULLUP);

    // get starting position
    encoder_position = ss.getEncoderPosition();

    Serial.println("Turning on interrupts");
    ss.enableEncoderInterrupt();

    ss.setGPIOInterrupts( 0b111110, 1 );
    // // Feather ANO_INT_PIN is connected to seesaw INT pin
    pinMode( ANO_INT_PIN, INPUT );
    attachInterrupt( ANO_INT_PIN, INTevent, FALLING );

    opMode = "OFF";

    mainMode = mainModes[mainModeIndex];

}

void loop() {

  iANO = checkANO();

  if (iANO) {

    Serial.printf("* INT Event intCount=%u iANO= %d States: %u opMode: %s", intCount, iANO, switchStates, opMode );

    if ( iANO>0 ) {
        Serial.printf(" Switch: %d Name: %s Time: %u mSec", activeSwitches, SwitchNames[iANO], pressedTime );
    }

    if ( rotaryFlag ) {
        Serial.printf(" Rotary: %d", ss.getEncoderPosition() );
        rotaryFlag = false;
    }

    Serial.print("\n");

    switch ( iANO ) {
        case -1: // Long center button press - reboot
            Serial.print( "\n\n**** USER INITATED RESART ****" );
            delay( 1000 );
            ESP.restart();
        case -2: // Stuck INT pin
            Serial.println( "ANO INT pin stuck and did not clear." );
            break;
        case -3: // Rotary encoder activity
            if ( iMode==1 ) {
                Serial.println( "rotary activity... " );
            }
            break;
        case 1:  // SS_SWITCH_SELECT 1
          if ( opMode == "OFF" ) {
            // Respond to left right, or down/center presses
            TurnOff();
          } else if ( opMode == "SETUP" ) {
            SetupMode();
          } else if ( opMode == "RUN" ) {
            RunMode();
          } else if ( opMode == "READY" ) {
            StartSteamCycle();
          } else if ( opMode == "STEAM" ) {
            StopSteamCycle();
          } else {
            Serial.printf( "No option with opMode=%s key press: %s", opMode, SwitchNames[iANO] );
          }
            break;
        case 2:  // SS_SWITCH_UP     2
          if ( opMode == "SETUP" ) {
            ExitSetupMode();
          } else if ( opMode == "READY" ) {
            opMode = "OFF";
          } else if ( opMode == "STEAM" ) {
            StopSteamCycle();
            opMode = "READY";
          } else {
            Serial.printf( "No option with opMode=%s key press: %s", opMode, SwitchNames[iANO] );
          }
          break;
        case 4:  // SS_SWITCH_LEFT   3
            if ( opMode == "OFF" ) {
              // Respond to left right, or down/center presses
              opMode = "SETUP";
            } else if ( opMode == "SETUP" ) {
              opMode = "RUN";
            } else if ( opMode == "RUN" ) {
              runMode = "OFF";
            } else {
              Serial.printf( "No option with opMode=%s key press: %s", opMode, SwitchNames[iANO] );
            }
            break;
        case 8:  // SS_SWITCH_DOWN   4
          // iLevel = (iLevel + 1) % nLevels;
          break;
        case 16: // SS_SWITCH_RIGHT  5
            if ( opMode == "OFF" ) {
              opMode = "RUN";
            } else if ( opMode == "SETUP" ) {
              opMode = "OFF";
            } else if ( opMode == "RUN" ) {
              runMode = "SETUP";
            } else {
              Serial.printf( "No option with opMode=%s key press: %s", opMode, SwitchNames[iANO] );
            }

            break;
        default:
          Serial.printf( "Mystery ANO reply: %d", iANO );
        }

    } else {
    // Yield to the USB serial port for a moment
    delay(10);
    }
}


