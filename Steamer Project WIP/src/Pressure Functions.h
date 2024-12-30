#pragma once

#include <Arduino.h>

#include "MCP3202.h"

// #include "shared.h"

class PressureControl {

  public:

    PressureControl( uint8_t CS, double Vdd );

    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    uint8_t Init();
    void Startup();
    void Update();
    void MeasurePressure();

    double avgPress;
    
  private:
    static const uint8_t nAvg=20;    
    double presData[nAvg], Vdd;
    uint8_t index;
    uint32_t nextMeasure, updateInterval=250;
    unsigned int iADC;
    double press;

    // For the 0-30 PSI pressure sensor 0.5V=0 PSI 4.5V=30 PSI
    // The ADC input uses a 2/3 voltage divider to bring 5V max input down to 3.3V
    // The ADC value for 0 PSI is then 409.5 and for 30 PSI 3685.5
    const double m = 30.0 / (3685 - 409 );  // m = (x2-x1) / (y2-y1)
    const double b = -m * 409;  // b = -m*x, when y=0, i.e. x=409.5
    

    MCP3202 mcp3202;

};