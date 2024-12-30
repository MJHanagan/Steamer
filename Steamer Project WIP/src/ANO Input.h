/*
 * This example shows how to read from a seesaw encoder module.
 * The available encoder API is:
 *      int32_t getEncoderPosition();
        int32_t getEncoderDelta();
        void enableEncoderInterrupt();
        void disableEncoderInterrupt();
 */

#pragma once
#include <Arduino.h>
#include "Adafruit_seesaw.h"
#include "shared.h"

extern int32_t encoder_position, rotaryDelta;

extern const String SwitchNames[];
extern String keyPress;

uint8_t InitANO();
void IRAM_ATTR INTevent();
int8_t checkANO();
