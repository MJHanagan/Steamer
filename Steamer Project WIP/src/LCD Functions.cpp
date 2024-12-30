// #include <Arduino.h>
#include "LCD Functions.h"
// #include "shared.h"

uint32_t nextLCDupdate, updateInterval = 1000;
uint32_t initMillis, runTimeSec;

// Change these values if you're using the 4x20 LCD
#define LINES 4
#define COLS 20

volatile byte iLCD = 0;
char hhmmss[9]; // hh:mm:ss 8 characters plus one null = 9 total
char mmss[6];   // mm:ss 5 character plus null terminator.
bool flash;

uint16_t initLine, okDelay = 250, failDelay = 1000;

// t is time in seconds = millis()/1000;
void calcUptime()
{
  unsigned long nowMillis = millis() - initMillis;
  unsigned long seconds = nowMillis / 1000;
  runTimeSec = seconds;
  uint8_t hours = seconds / 3600;
  seconds %= 3600;
  uint8_t minutes = seconds / 60;
  seconds %= 60;
  snprintf(hhmmss, 9, "%02d:%02d:%02d", hours, minutes, seconds);
}

char *MMSS(int sec)
{
  sprintf(mmss, "%02d:%02d", sec / 60, sec % 60);
  return mmss;
}

void ClearScreen()
{
  Serial1.write(12);
  delay(50);
}
void GotoPos(uint8_t line, uint8_t x)
{
  uint8_t xy = 128 + line * COLS + x;
  Serial1.write(xy);
}
void Backlight(bool onOff)
{
  if (onOff)
  {
    Serial1.write(17);
    delay(50);
  }
  else
  {
    Serial1.write(18);
  }
}

void DisplayOff()
{
  Serial1.write(21);
  return;
}

void cursor(uint8_t line, uint8_t column, uint8_t block, bool blink)
{
  GotoPos(line, column);
  if (block and blink)
  {
    Serial1.write(25);
  }
  else if (block)
  {
    Serial1.write(24);
  }
  else if (blink)
  {
    Serial1.write(23);
  }
  else
  {
    Serial1.write(22);
  }
}

void TextAt(uint8_t line, uint8_t x, const char *textString)
{
  GotoPos(line, x);
  Serial1.print(textString);
}

void initMsg(uint8_t onLine, const char *text, uint8_t erase )
{
  if ( erase ) {
    TextAt( onLine, 0, "                    " );
  }
  TextAt(onLine, 0, text);
  return;
}

void passFail(bool initResult)
{
  if (initResult)
  {
    Serial1.print(" OK.");
    tone( PIEZO_PIN, 1000, okDelay );
    delay( okDelay );
  }
  else
  {
    Serial1.print(" FAILED");
    tone( PIEZO_PIN, 500, failDelay );
    delay( failDelay );
  }
  return;
}

uint8_t InitLCD()
{

  // Use the Serial "TX" pin to the LCD
  Serial1.begin(19200);

  // delay(50);
  // Clear out the LCD and place cursor at 0,0
  ClearScreen();
  // Turn on backlight
  Backlight(true);
  // Display opening text beginning at 0,0
  TextAt(0, 0, VERSION);
  cursor(1, 9, true, true);

  nextLCDupdate = millis() + updateInterval;

  return true;
}

void Flash()
{
  return;
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
void UpdateLCD(bool timerReset = false)
{

  if (timerReset)
  {
    nextLCDupdate = millis();
    iLCD = 0;
    ClearScreen();
  }

  // Return if not time for an update.
  if (millis() < nextLCDupdate)
    return;

  // Index the next update time
  nextLCDupdate += updateInterval;
  // Correct the time if we have gotten behind the regular interval updating time.
  if (millis() > nextLCDupdate)
    nextLCDupdate = millis() + updateInterval;

  calcUptime();
  getNow();

  if (opMode == "INIT")
  {
    // int secToSleep = ( sleepMillis-millis() ) /1000;
    // GotoPos(1,0); Serial1.printf( "Mode: %s", opMode );
    GotoPos(1, 0);
    Serial1.printf("Standby in %s", MMSS((sleepMillis - millis()) / 1000));
    GotoPos(2, 0);
    Serial1.printf("%3.0f/%3.0f %s=%3.0f%%", heater.setpoint, heater.maxTC1.avgT, heater.GetMode(), heater.heaterOutput);
    // GotoPos(2,0); Serial1.printf( "%3.1f mL C: %u P: %u", pump.doseVolume, pump.GetFlowCounts(), pump.pulseCount );
    GotoPos(3, 3);
    Serial1.printf("Time: %s", getHHMMSS());
    // GotoPos(3,12); Serial1.print( hhmmss );
    // cursor( 1, 9, true, true );
  }
  else if (opMode == "IDLE")
  {
    GotoPos(0, 2);
    Serial1.printf("STANDBY %s", getHHMMSS());
    GotoPos( 1,0 );
    Serial1.printf( "%3.0f %3.0f/%3.0f %u-%u", heater.maxTC1.avgT, heater.maxTC1.minT, heater.maxTC1.maxT, heater.maxTC1.errorCount, heater.maxTC1.errorCode );
    GotoPos( 2,0 );
    Serial1.printf( "%3.0f %3.0f/%3.0f %u-%u", heater.maxTC2.avgT, heater.maxTC2.minT, heater.maxTC2.maxT, heater.maxTC2.errorCount, heater.maxTC2.errorCode );
    GotoPos( 3,0 );
    Serial1.printf( "%3.0f %3.0f/%3.0f %u-%u", heater.maxTC3.avgT, heater.maxTC3.minT, heater.maxTC3.maxT, heater.maxTC3.errorCount, heater.maxTC3.errorCode );
    
    // GotoPos(1, 1);
    // Serial1.printf("%3.0f %3.0f %3.0f SP=%3.0f", heater.maxTC1.avgT, heater.maxTC2.avgT, heater.maxTC3.avgT, heater.setpoint );
    // GotoPos(2, 0);
    // Serial1.printf("%4u-%u%5u-%u%5u-%u", heater.maxTC1.errorCount, heater.maxTC1.errorCode,
                                            // heater.maxTC2.errorCount, heater.maxTC2.errorCode, 
                                            // heater.maxTC3.errorCount, heater.maxTC3.errorCode );
     
    // Serial1.printf("P=%.1f SP=%3.0f FC: %u", pressure.avgPress, Tsetpoint, pump.flowCount );
    // Serial1.printf("SP=%3.0f FC: %u", Tsetpoint, pump.GetFlowCounts() );
    // GotoPos(2,3); Serial1.printf( "Setpoint=%3.0f", setpoint );
    // GotoPos(3, 1);
    // Serial1.printf("%s C=%u V=%.1f",  swqStatus, pump.GetFlowCounts(), pump.doseVolume );
    // cursor(2, 12, false, true);

    // GotoPos(0, 7);
    // Serial1.printf("IDLE");
    // GotoPos(1, 1);
    // Serial1.printf("S=%3.0f T=%3.0f %s=%3.0f%%", heater.setpoint, heater.maxTC1.avgT, heater.GetMode(), heater.heaterOutput);
    // GotoPos(2, 0);
    // // Serial1.printf("P=%.1f SP=%3.0f FC: %u", pressure.avgPress, Tsetpoint, pump.flowCount );
    // Serial1.printf("SP=%3.0f FC: %u", Tsetpoint, pump.GetFlowCounts() );
    // // GotoPos(2,3); Serial1.printf( "Setpoint=%3.0f", setpoint );
    // GotoPos(3, 0);
    // Serial1.printf("%s t: %s",  swqStatus, getHHMMSS());
    // cursor(2, 12, false, true);
    // Serial.print( "Start LCD updated" );
    // GotoPos(2,13); // Go to setpoint value
  }
  else if (opMode == "PREHEAT")
  {
    if (modeStep == "WAIT")
    {
      GotoPos(0, 0);
      Serial1.printf("  WAITING... %s", MMSS((millis() - opStepTime) / 1000));
    }
    else
    {
      GotoPos(0, 0);
      Serial1.printf(" %s %s", modeStep, MMSS((opStepTime - millis()) / 1000));
    }

    GotoPos(1, 1);
    Serial1.printf("S=%3.0f T=%3.0f %s=%3.0f%%", heater.setpoint, heater.maxTC1.avgT, heater.GetMode(), heater.heaterOutput);
    GotoPos(2, 1);
    Serial1.printf("X%4.1f VAR:%4.1f", heater.tSlope, heater.tStd);
    // GotoPos(3,3); Serial1.printf( "REMAIN: %s", MMSS( (opStepTime-millis())/1000 ) );
    GotoPos(3, 3);
    Serial1.printf("%s t: %s",  swqStatus, getHHMMSS());
    cursor(2, 13, false, false);
  }
  else if (opMode == "READY")
  {
    GotoPos(0, 6);
    Serial1.printf("READY");
    GotoPos(1, 1);
    Serial1.printf("S=%3.0f T=%3.0f %s=%3.0f%%", heater.setpoint, heater.maxTC1.avgT, heater.GetMode(), heater.heaterOutput);
    GotoPos(2, 2);
    Serial1.printf("CNT:%3d t:%4.1f sec", pump.GetFlowCounts(), pump.pumpCycleTime / 1000.0);
    GotoPos(3, 0);
    Serial1.printf("%s  ST: %s", getHHMMSS(), MMSS(steamTime / 1000));
    // GotoPos(3,15);
    cursor(3, 14, false, true); // Go to steam cycle duration value
  }
  else if (opMode == "STEAM")
  {
    GotoPos(0, 1);
    Serial1.printf( "***STEAM*** %s", MMSS((opStepTime - millis())/1000) );
    // Serial.print( 'LCD line 0 finished.\n');
    GotoPos(1, 1);
    Serial1.printf( "S=%3.0f T=%3.0f %s=%3.0f%%", heater.setpoint, heater.maxTC1.avgT, heater.GetMode(), heater.heaterOutput );
    // Serial.print( 'LCD line 1 finished.\n');
    // GotoPos(2,2); Serial1.printf( "P:%3d T:%4.1f V: %4.1f", pump.pulseCount, pump.pumpCycleTime/1000.0, pump.doseVolume );
    GotoPos(2, 0);
    Serial1.printf( "P:%3d t:%4.1f V:%5.1f", pump.stopCount, pump.pumpCycleTime/1000.0, pump.doseVolume );
    // Serial.print( 'LCD line 3 finished.\n');
    GotoPos(3, 0);
    Serial1.printf("C:%3d %s ST: %s", pump.GetFlowCounts(), swqStatus, MMSS(steamTime/1000) );
    // Serial.print( 'LCD line 3 finished.\n');
    cursor(3, 13, false, true); // Go to steam cycle duration value
  }
  else if (opMode == "FOGGER")
  {
    GotoPos(0, 1);
    Serial1.printf(" ** FOGGER ** %s", MMSS((millis() - opStepTime) / 1000));
    GotoPos(1, 1);
    Serial1.printf("S=%3.0f T=%3.0f %s=%3.0f%%", heater.setpoint, heater.maxTC1.avgT, heater.GetMode(), heater.heaterOutput);
    GotoPos(2, 1);
    // Serial1.printf("%3.0f %3.0f %3.0f P=%3.0f", heater.maxTC1.avgT, heater.maxTC2.avgT, heater.maxTC3.avgT, pressure.avgPress);
    GotoPos(3, 0);
    Serial1.printf("%4d/%d %4.1fs%4.0fg", pump.stopCount, pump.GetFlowCounts(), pump.pumpCycleTime / 1000.0, pump.doseVolume);
    cursor(3, 13, false, true); // Go to steam cycle duration value
  }

  

  return;
}

char *calcMMSS(uint32_t msec)
{

  uint8_t minutes = msec / 60000;
  uint8_t seconds = (msec % 60000) / 1000;
  snprintf(mmss, 6, "%02d:%02d", minutes, seconds);
  return mmss;
}
