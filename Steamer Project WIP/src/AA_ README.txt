Notes on WIP version:

(1) Changed all .cpp files to just #include "file.h" and moved any other #include statements from "file.cpp" to "file.h" (this is apparently the convention)
(2) Renamed "WiFi MQTT2" files to "WiFi MQTT"
(3) Added TC error count reset function in Heater Functions.cpp
(4) OTC and TC errors are cleared during 'wakeup' after a shutdown
(5) Changed 'Idle' screen to 'STANDBY' and added TC error counts to display
(6) Eliminated average temperature variables in Heater Functions.cpp and added maxTC.avgT, maxTC.minT, maxTC.maxT variables instead
(7) Added maxTC.errorMillis as millis() time of last error, this will be 0 if no errors.
(8) Added maxTC.reset() function to reset errors ans min/max temperature values.
(9) Changed auto off time to 1 hour

Needs:
Audio when OTC conditions
Improved startup LCD progress

Changes for V2.0 board?

**********************************************************************************************************************************

Notes on Version 1.0.0

Instituted the two-stage heating control mode:
    * When T < Tmin the output is set to maxOutput
    * When T > maxT the output is set to minOutput
    * Tmax is setpoint + deltaT
    * Tmin is setpoint - deltaT

For steaming mode with water at 7 pulses every 5500 msec (about 24 mL/min) use:
    * setpoint = 160.0
    * deltaT = 3.0
    * maxOutput = 100.0
    * minOutput = 80.0

Notes on high/low relay control:
    * When the line voltage is about 120 VAC the average required output is about 90%
    * When the voltage is about 110 VAC the average output required is about 95%

At idle conditions (i.e. no water is pumping) the settings should be:
    * setpoint = 160.0
    * deltaT = 3.0
    * maxOutput = 6.0
    * minOutput = 0.0

The nominal heating rate at 6% output is about 4.8°C/min (V=120)