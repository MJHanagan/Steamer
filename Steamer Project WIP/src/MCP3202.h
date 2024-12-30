//Release Date: 15-12-2018
//souvikssaha@gmail.com

/*===========================================================================
DAC7611 device library code is placed under the MIT license
Copyright (c) 2018 Souvik Saha

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
===========================================================================*/
#pragma once

#include "Arduino.h"

#ifndef MCP_H
#define MCP_H

class MCP3202 {
  public:
    MCP3202( int CS=10, double _Vdd=3.3 );
    void begin();
    unsigned int readChannel( uint8_t thisCh );
    double readChannelV( uint8_t thisCh );


  private:
    int chipSel;
    double Vdd;
};

#endif