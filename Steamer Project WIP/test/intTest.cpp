#include <Arduino.h>

#include "Adafruit_seesaw.h"

#define SS_SWITCH_SELECT 1
#define SS_SWITCH_UP     2
#define SS_SWITCH_LEFT   3
#define SS_SWITCH_DOWN   4
#define SS_SWITCH_RIGHT  5

#define ANO_INT_PIN    14

#define SEESAW_ADDR      0x49

Adafruit_seesaw ss;
int32_t encoder_position;

volatile uint8_t state, flag;

void IRAM_ATTR detectInterrupt() {

    state = !ss.digitalRead( SS_SWITCH_SELECT );
    flag = true;

}

void setup() {
  Serial.begin(115200);

  delay(2000);

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

    // Set the seesaw GPIO pins
    ss.pinMode(SS_SWITCH_UP, INPUT_PULLUP);
    ss.pinMode(SS_SWITCH_DOWN, INPUT_PULLUP);
    ss.pinMode(SS_SWITCH_LEFT, INPUT_PULLUP);
    ss.pinMode(SS_SWITCH_RIGHT, INPUT_PULLUP);
    ss.pinMode(SS_SWITCH_SELECT, INPUT_PULLUP);

  // get starting position
  encoder_position = ss.getEncoderPosition();

  Serial.println("Turning on interrupts");
  
  // Enable encoder interrupts
  ss.enableEncoderInterrupt();

  // Enable GPIO interrupts for buttons
  uint32_t gpio_interrupts = (1 << SS_SWITCH_UP) | (1 << SS_SWITCH_DOWN) | 
                             (1 << SS_SWITCH_LEFT) | (1 << SS_SWITCH_RIGHT) | 
                             (1 << SS_SWITCH_SELECT);
  ss.setGPIOInterrupts(gpio_interrupts, 1);

  // Feather ANO_INT_PIN is connected to seesaw INT pin
  pinMode( ANO_INT_PIN, INPUT_PULLUP );
  attachInterrupt(digitalPinToInterrupt(ANO_INT_PIN), detectInterrupt, FALLING);
}

void loop() {
  // Main loop can be empty as interrupts will handle the events
  delay(10);
  if ( flag ) {
    Serial.printf( "Interupt! state=%u", state );
    flag = false;
  }

}
