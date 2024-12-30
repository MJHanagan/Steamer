// #include <Arduino.h>

#include <Pressure Functions.h>

PressureControl::PressureControl( uint8_t _CS_PIN, double _Vdd ) : mcp3202( _CS_PIN, _Vdd ) {
    Vdd = _Vdd;
}

uint8_t PressureControl::Init() {

  mcp3202.begin();

  Startup();

  return true;
  
}

void PressureControl::Startup() {

  // Fill the data buffer with measurements
  index = 0;
  for ( uint8_t i=0; i<nAvg; i++ ) { MeasurePressure(); }

  nextMeasure = millis() + updateInterval;

  // Serial.printf( "@ReInit() index: %u iADC: %u press: %.1f PSI avgPress: %.1f PSI", index, iADC, press, avgPress );

  return;

}

void PressureControl::MeasurePressure() {

  // Calculate pressure directly from adc integer value
  iADC = mcp3202.readChannel( 0 );
  // iADC = 1000;//mcp3202.readChannel( 0 );
  press = m * double(iADC) + b;
  presData[ index ] = press;//m * double( mcp3202.readChannel( 0 ) ) + b;

  // if ( index==0 ) {
  //   Serial.printf( "index: %u iADC: %u press: %.1f", index, iADC, press );
  // }

  avgPress = 0.0; for ( uint8_t i=0; i<nAvg; i++ ) { avgPress += presData[i]; } // Sum all the values in presData[]
  avgPress /= double(nAvg);  // Calculate the average pressure

  // Index the array pointer for the next measurment - this limits index to 0,1,2... nAvg-1 only
  index = (index+1) % nAvg;
  
  return;
  
}


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
// This routine needs to be called at least every 50 mSec in the main loop() function
void PressureControl::Update() {

  // Return now if not time for next pressure measurement
  if ( millis() < nextMeasure ) return;
  
  // Measure the current pressure and calculate the average
  MeasurePressure();
  
  // Update the next check time
  nextMeasure += updateInterval;  

  return;
}

