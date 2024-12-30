#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "shared.h"

// uint8_t sdCard;

bool initSDcard();
bool writeToSDcard( const char * charBuffer );

bool initMQTT();
// bool publishDataToMQTT( const char * charBuffer );
