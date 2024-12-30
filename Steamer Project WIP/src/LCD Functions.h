#pragma once

#include <Arduino.h>
// Note: the shared.h contains 'heater' and 'pump' declarations
#include "shared.h"
// #include "Operation Functions.h"

// uint8_t InitLCD();
// void UpdateLCD();

extern char hhmmss[9];

uint8_t InitLCD();
void calcUptime();
char* MMSS( int sec );
void ClearScreen();
void UpdateLCD( bool timerReset );
void TextAt( uint8_t line, uint8_t x, const char *textString );
void Backlight( bool );
void cursor( uint8_t line, uint8_t column, uint8_t block, bool blink );
void DisplayOff();

void initMsg( uint8_t onLine, const char* text, uint8_t erase=true );
void passFail(bool initResult);