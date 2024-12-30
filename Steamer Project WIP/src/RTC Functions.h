#pragma once

#include <Arduino.h>
#include "RTClib.h"

#include "shared.h"

bool initRTC();
DateTime getNow();

char* getYYYYMMDD();
char* getTimestamp();
char* getHHMMSS();

uint8_t setRTCtime( uint8_t hh, uint8_t mm, uint8_t ss );

char* dirYYYYMMDD();
char* fileHHMMSS();
