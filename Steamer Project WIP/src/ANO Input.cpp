// #include <Arduino.h>

#include "ANO Input.h"
// #include "shared.h"
// #include "Adafruit_seesaw.h"

#define SEESAW_ADDR      0x49

Adafruit_seesaw ss;

const String SwitchNames[] = { "NONE", "SELECT", "UP", "LEFT", "DOWN", "RIGHT" };
String keyPress;

int32_t encoder_position, rotaryDelta;
int8_t iANO;

volatile uint8_t intFlag;
volatile uint32_t intTime, intCount;

uint8_t activeFlag, activeSwitches, switchStates, rotaryFlag;
uint32_t pressStartTime, pressedTime;

uint8_t InitANO() {

    DEBUG_PRINT("Looking for seesaw...");

    if (! ss.begin(SEESAW_ADDR)) {
    DEBUG_PRINT(" Couldn't find seesaw on default address!");
    return false;
    }
    DEBUG_PRINT(" found, seesaw started");
    uint32_t version = ((ss.getVersion() >> 16) & 0xFFFF);
    if (version  != 5740){
    DEBUG_PRINT(" Wrong firmware loaded? ");
    DEBUG_PRINT(version);
    return false;
    }

    DEBUG_PRINT(" Found Product 5740");
    // Set pinMode for each input
    ss.pinMode( SS_SWITCH_UP, INPUT_PULLUP );
    ss.pinMode( SS_SWITCH_DOWN, INPUT_PULLUP );
    ss.pinMode( SS_SWITCH_LEFT, INPUT_PULLUP );
    ss.pinMode( SS_SWITCH_RIGHT, INPUT_PULLUP );
    ss.pinMode( SS_SWITCH_SELECT, INPUT_PULLUP );

    // get starting position
    encoder_position = ss.getEncoderPosition();

    DEBUG_PRINTLN(" Turning on interrupts");
    ss.enableEncoderInterrupt();

    ss.setGPIOInterrupts( 0b111110, 1 );
    // // Feather ANO_INT_PIN is connected to seesaw INT pin
    pinMode( ANO_INT_PIN, INPUT );
    attachInterrupt( ANO_INT_PIN, INTevent, FALLING );

    // Clear out any residual activity due to initilaization
    while ( checkANO() != 0 ) {}

    return true;
}

void IRAM_ATTR INTevent() {
  intFlag = true;
  intTime = millis();
  ++intCount;
  return;
}

int8_t checkANO() {

  // Returns:
  //   0 = no ANO device activity to report
  //  +n = 5-bit integer, one bit for each input switch: center, up, down, left, right.  Return value when switch is RELEASED
  //  -1 = a switch has been pressed longer than 5 seconds
  //  -2 = the input INT pin is still low, even after a reset attempt (something wrong??)
  //  -3 = rotary encoder event
  //  -4 = unknown error (should never return this)

  if (!intFlag) {  // There is no ISR INT event to process so check for a few other special conditions before returning

    // Return if switch press has exceeded 5000 mSec
    if ( activeFlag and millis()-pressStartTime > 5000 ) return -1;

    // Return with 0 if the INT pin is not low
    if ( digitalRead( ANO_INT_PIN ) ) return 0;

    // The INT pin has been low (active) low for <500 mSec return now with a 0
    if ( millis()-pressStartTime < 500 ) return 0;
    
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
  // Shift the unused first bit out leaving just the 5 input switches, use '~' to invert the bits (inputs are active LOW), 
  // then finally mask just the lowest 5 bits.
  switchStates = ~( switchStates >>1 ) & 0b11111;
  // Clear the intFlag
  intFlag = false;

  // Test for an already inplay switch event.
  if ( activeFlag ) {
    // We have an active switch press event, so test if this is the switch release event
    if ( switchStates==0 ) {
      // All switches are inactive so the press event is over, calculate the pressed time
      pressedTime = intTime - pressStartTime;
      // Clear the activeFlag
      activeFlag = false;
      // Return with the active switch value
      // #define SS_SWITCH_SELECT 1
      // #define SS_SWITCH_UP     3
      // #define SS_SWITCH_LEFT   4
      // #define SS_SWITCH_DOWN   5
      // #define SS_SWITCH_RIGHT  2
      if ( activeSwitches & 1<<(SS_SWITCH_SELECT-1) ) {
        keyPress = "CENTER";
      } else if ( activeSwitches & 1<<(SS_SWITCH_UP-1) ) {
        keyPress = "UP";
      } else if ( activeSwitches & 1<<(SS_SWITCH_LEFT-1) ) {
        keyPress = "LEFT";
      } else if ( activeSwitches & 1<<(SS_SWITCH_DOWN-1) ) {
        keyPress = "DOWN";
      } else if ( activeSwitches & 1<<(SS_SWITCH_RIGHT-1) ) {
        keyPress = "RIGHT";
      }
      // Serial.printf( "Keypress: %u Name: %s\n", activeSwitches, keyPress );
      return activeSwitches;
    }
  } else {
    // There is no current switch press so this is a new switch active event
    if ( switchStates ) {
      // Save the switch condition
      activeSwitches = switchStates;
      // Save the starting time
      pressStartTime = intTime;
      // Set the activeFlag
      activeFlag = true;
      // Return with a 0
      return 0;
    } else {
      // We have an INT even but no switches show activity, therefore this must be a rotary encode event, so return with a -3
      rotaryFlag = true;
      rotaryDelta = ss.getEncoderDelta();
      return -3;
    }

  }

  // We should never get here, so return with this doozy value!
  return -4;

}

