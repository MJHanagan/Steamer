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

#define SEESAW_ADDR      0x49

Adafruit_seesaw ss;
int32_t encoder_position;
int8_t iANO;

volatile uint8_t intFlag;
volatile uint32_t intTime, intCount;

uint8_t activeFlag, activeSwitches, switchStates, rotaryFlag;
uint32_t startTime, pressedTime;

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

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

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


mainModeIndex = runModeIndex = setupParameterIndex = settingIndex = 0;




}

void loop() {

  iANO = checkANO();

  if (iANO) {

    Serial.printf("* INT Event intCount=%u iANO= %d States: %u", intCount, iANO, switchStates );

    if ( iANO>0 ) {
      Serial.printf(" Switch: %d Time: %u mSec", activeSwitches, pressedTime );
    }
    
    if ( rotaryFlag ) {
      Serial.printf(" Rotary: %d", ss.getEncoderPosition() );
      rotaryFlag = false;
    }

    Serial.print("\n");
    
  } else {
  // Yield to the USB serial port for a moment
    delay(10);
  }
}

