// #include <Arduino.h>
#include "Operation Functions.h"


String opMode, modeStep, nextMode, fluidType="WATER";
String inputText, key, value;

uint32_t autoOffTime, autoOffDelay=60*60*1000, opStepTime, autoIdleTime;
uint32_t bootTime, startTime, endTime, steamTime=10*60*1000, endSteamTime, offTime, sleepMillis;

// Data logging times and interval time variables
uint32_t sdLogTime, sdLogInterval=2000;
uint32_t mqttTime, mqttInterval=5000;
uint32_t serialLogTime, serialLogInterval=5000;

char swqStatus[] = "XXX";

uint32_t loopStartTime, loopCounter, loopHz;

// uint32_t pumpCycleTime=5000, pulseIndex=6;
uint32_t pumpCycleTime=5500, pulseIndex=7;  // 5500/7 Maximum conditions for water with 95% power output (~1050 W)
//  For PID tuning use SP=160, pulseIndex=5 pumpCycleTime=5000 for about 71% average power demand
// For operating in pulsed heater mode
double Tsetpoint=160, Tdelta=5, maxOutput=100.0, minOutput=75.0;

// Heater high/low relay control parameters
double rcControl[4][3] = {  // [rows][columns]
        {  2.0,  0.0,   6.0 },  // Low power for idling at setpoint (no fluid flowing)
        {  2.0, 80.0, 100.0 },  // High power output for maximum fluid flow (i.e. STEAM or FOG mode)
        {  2.0,  0.0,  20.0 },  // Moderate power for faster recovery to setpoint in idle mode
        { 10.0,  0.0,   0.0 }   // User programmed values
    };

char dataBuffer[MAX_BUFFER_SIZE];  // Match the buffer sized used by the MQTT buffer
int buffLen;
static uint8_t result;

void OpUpdate() {

    // Check for ANO input activity
    if ( checkANO() == 0 ) {
        // No activity so clear the input variables
        keyPress = "";
        rotaryDelta = 0;
        if( opMode != "FOGGER" and opMode !="STEAM" and opMode !="READY" and millis() >= autoOffTime ) {
            ShutdownMode();
            return;
        }
    } else {
        // We have user activity from the ANO device so update the auto off timer
        autoOffTime = millis() + autoOffDelay;
        autoIdleTime = millis() + 10*60*1000;
    }

    // Chirp every 10 seconds if we are currently in an OTC condition
    if( heater.otcFlag ) {
        static uint32_t nextOTCbeep = 0;
        if( millis() >= nextOTCbeep ) {
            tone( PIEZO_PIN, 1200, 100 );
            nextOTCbeep = millis() + 10000;
        }
    }

    // Process according to the current opMode
    if( opMode == "IDLE" ) {
        IdleMode();
    } else if( opMode == "PREHEAT" ) {
        PreheatMode();
    } else if( opMode == "READY" ) {
        ReadyMode();
    } else if( opMode == "STEAM" ) {
        SteamMode();
    // } else if( opMode == "OVEN" ) {
    //     OvenMode();
    } else if( opMode == "FOGGER" ) {
        FogMode();
    } else {
        Serial.printf( "No function defined for %s mode.", opMode );
    }

    return;
}

uint8_t LogEvent( const char * text ) {

    getNow();
    sprintf( dataBuffer, "TS=%s,EVENT=%s", getTimestamp(), text );
    result = writeToSDcard( dataBuffer );
    result = sendMQTTdata( "steamer/data", dataBuffer );
    Serial.print( '\t' ); Serial.println( dataBuffer );

    return result;
}

uint8_t LogChange( const char * vName, double *vPointer, double newValue ) {
    sprintf( dataBuffer, "TS=%s,EVENT=VARIABLE_CHANGE,VAR=%s,FROM_=%f,TO_=%f", getTimestamp(), vName, *vPointer, newValue );
    *vPointer = newValue;
    return TxBuffer();
}

uint8_t LogChange( const char * vName, int *vPointer, int newValue ) {
    sprintf( dataBuffer, "TS=%s,EVENT=VARIABLE_CHANGE,VAR=%s,FROM_=%d,TO_=%d", getTimestamp(), vName, *vPointer, newValue );
    *vPointer = newValue;
    return TxBuffer();
}

uint8_t LogChange( const char * vName, uint32_t *vPointer, uint32_t newValue ) {
    sprintf( dataBuffer, "TS=%s,EVENT=VARIABLE_CHANGE,VAR=%s,FROM_=%u,TO_=%u", getTimestamp(), vName, *vPointer, newValue );
    *vPointer = newValue;
    return TxBuffer();
}

uint8_t TxBuffer() {

    result = writeToSDcard( dataBuffer );
    result = sendMQTTdata( "steamer/data", dataBuffer );
    Serial.print( '\t' ); Serial.println( dataBuffer );

    return result;
}
void OpLogData() {

    
    if( millis() >= sdLogTime or millis() >= mqttTime  ) {

        getNow();

        // Clear the dataBuffer array
        // memset( dataBuffer, 0, sizeof(dataBuffer));

        sprintf( dataBuffer, "TS=%s,millis=%u,MODE=%s,SP=%.1f,TC1=%.1f,TC2=%.1f,TC3=%.1f,OUT=%.1f,HCM=%s,FSC=%u,C1=%u,C2=%u,PRT=%u,VOL=%.1f,TC1err=%u,TC2err=%u,TC3err=%u,TC1code=%u,TC2code=%u,TC3code=%u,FF=%f,FL=%s,CT=%u,PC=%u,LHZ=%u,SWQ=%s,LSL=%d,FHM=%u,MHM=%u", 
            getTimestamp(), millis(), opMode, heater.setpoint, heater.maxTC1.avgT, heater.maxTC2.avgT, heater.maxTC3.avgT, heater.heaterOutput,
            heater.GetMode(), pump.GetFlowCounts(), pump.startCount, pump.stopCount, pump.GetRunTime(), pump.doseVolume, 
            heater.maxTC1.errorCount, heater.maxTC2.errorCount, heater.maxTC3.errorCount,
            heater.maxTC1.errorCode, heater.maxTC2.errorCode, heater.maxTC3.errorCode, pump.FlowFactor, fluidType, pump.pumpCycleTime, pump.pulseIndex, 
            loopHz, swqStatus, buffLen, esp_get_free_heap_size(), esp_get_minimum_free_heap_size() );
        
        buffLen = strlen(dataBuffer);

        // Write 'dataBuffer' to SD card as a single line appended to current file
        if( sdLogInterval and millis() >= sdLogTime ) { 
            result = writeToSDcard( dataBuffer );
            swqStatus[0] = ( result ) ? 'S' : 'X';
            sdLogTime += sdLogInterval;
            if ( sdLogTime < millis() ) sdLogTime = millis() + sdLogInterval;
        }

        // If MQTT is active send 'dataBuffer' to steamer/data topic
        if( mqttInterval and millis() > mqttTime ) {
            swqStatus[1] = ( wifiState==3 ) ? 'W' : 'X';
            
            if ( mqttActive ) {
                result = sendMQTTdata( "steamer/data", dataBuffer );
                // Serial.printf( "MQTT send result: %d\n", result );
                swqStatus[2] = ( result ) ? 'Q' : 'X';
            } else {
                swqStatus[2] = '?';
                // Serial.print( "No current MQTT connection.\n" );
            } 

            mqttTime += mqttInterval;
            if ( mqttTime < millis() ) mqttTime = millis() + mqttInterval;
        }

        // Write 'dataBuffer' to serial port
        if( serialLogInterval and millis() >= serialLogTime ) { 
            Serial.print( '\t' ); Serial.println( dataBuffer );

            serialLogTime += serialLogInterval;
            if ( serialLogTime < millis() ) serialLogTime = millis() + serialLogInterval;
        }       
    }
    return;
}

void OpCheckSerial() {

    // Return if nothing in the serial buffer
    if( Serial.available()==0 ) return;

    // Read serial buffer up to /n
    inputText = Serial.readStringUntil( '\n' );

    Serial.printf( "cmd string is: %s", inputText );

    inputText.trim();
    ChangeCommand( inputText );

    return;
}

void OpCheckMQTT() {

    updateMQTT();

    if ( mqttNewMsg ) {
        Serial.printf( "\n*** New MQTT RX, topic: %s  Message: %s", mqttTopic, mqttMessage );
        mqttNewMsg = false;
        ChangeCommand( mqttMessage );
    }

    return;
}

void ChangeCommand( String cmdText ) {

    cmdText.trim();
    cmdText.toUpperCase();

    // Clear the dataBuffer array so we can determine if new data was written to it
    memset( dataBuffer, 0, sizeof(dataBuffer));

    getNow();

    // Look for '=' to identify a key=value pair command
    int pos = cmdText.indexOf('=');

    if ( pos == -1 ) {  // No '=' in cmdText so check for single keyword command
        Serial.printf( "Command: %s", cmdText );
        if ( cmdText=="REBOOT" ) {  // Performs a software reboot
            LogEvent( "REBOOT" );
            delay( 100 );
            esp_restart();
        } else if ( cmdText=="OFF" ) { // Go to SHUTDOWN mode
            LogEvent( "SERIAL COMMAND SHUTDOWN" );
            ShutdownMode();
        } else if ( cmdText=="LRC" ) { // Low-power relay control (for maintaining idle/standby setpoint temperature)
            heater.HighLowRelayMode( rcControl[0][0], rcControl[0][1], rcControl[0][2] );
        } else if ( cmdText=="OTR" ) { // Over temperature reset
            LogEvent( "OTR" );
            heater.ResetOTC();
            heater.ResetTCs();
            heater.Startup( Tsetpoint );
        } else if ( cmdText=="RTE" ) { // Reset TC errors
            LogEvent( "RTE" );
            heater.ResetTCs();
        } else if ( cmdText=="#" ) { // Debug query
            Serial.print( "\n\nSetting relay control mode to (160, 0, 25 )\n" );
            heater.HighLowRelayMode( 160.0, 2.0, 25.0 );
        } else if ( cmdText=="?" ) { // Debug query
            Serial.printf( "\n\n*** Debug print\nSetpoint: %.1f Output: %.1f%% Mode: %s Enabled: %u\n", heater.setpoint, heater.heaterOutput, heater.GetMode(), heater.enabled );
            Serial.printf( "minTemp: %.1f maxOutput: %.1f%% maxTemp: %.1f minOutput: %.1f%%\n", heater.minTemp, heater.maxOutput, heater.maxTemp, heater.minOutput );
            Serial.printf( "loopHz: %u\n", loopHz );
        } else {
            Serial.printf( "Unknown command: %s", inputText );
        }

        } else {  // The command contains an '=' so parse the key=value string to 'key' and 'value' strings
            key = cmdText.substring( 0, pos );
            value = cmdText.substring( pos+1 );

            Serial.printf( "Command key: %s value: %s", key, value );

            // Check for the a keyword match and process the value
            if( key == "TS" ) {  // Time sync TS=hh:mm:ss        
                uint8_t hh = value.substring(0,2).toInt(); // 0123456789012345678
                uint8_t mm = value.substring(3,5).toInt(); // hh:mm:ss\n
                uint8_t ss = value.substring(6,8).toInt();

                setRTCtime( hh, mm, ss );

                sprintf( dataBuffer, "TS=%s,EVENT=TIMESYNC,TO_='%s'", getTimestamp(), value );
                TxBuffer();
            } else if( key == "RC" ) {  // Relay contol mode
                if ( value == "ON")  {
                    LogEvent( "RELAY CONTROL MODE ON" );
                    heater.HighLowRelayMode( rcControl[3][0], rcControl[3][1], rcControl[3][2] );
                } else if ( value=="LOW" ) { // Low-power realy control mode (for maintaining setpoint with no fluid flow)
                    LogEvent( "RELAY CONTROL MODE LOW" );
                    heater.HighLowRelayMode( rcControl[0][0], rcControl[0][1], rcControl[0][2] );
                } else if ( value=="HIGH" ) { // High-power realy control mode (for maintaining setpoint at maximum fluid flow)
                    LogEvent( "RELAY CONTROL MODE HIGH" );
                    heater.HighLowRelayMode( rcControl[1][0], rcControl[1][1], rcControl[1][2] );
                } else { // Turn off relay mode
                    LogEvent( "RELAY CONTROL MODE OFF" );
                    heater.SetMode( "OFF" );
                }
            } else if( key == "SP" ) {  // Change setpoint (°C)
                LogChange( "Tsetpoint", &Tsetpoint, value.toDouble() );
                heater.Setpoint( Tsetpoint );
            } else if( key == "CT" ) {  // Change pump cycle time (msec)  pump.pumpCycleTime, pump.pulseIndex
                LogChange( "CT", &pumpCycleTime, value.toInt() );
                pump.pumpCycleTime = pumpCycleTime;
            } else if( key == "PC" ) {  // Pump pulse index
                LogChange( "PC", &pulseIndex, value.toInt() );
                pump.pulseIndex = pulseIndex;
            } else if( key == "FF" ) { // Change fluid flow factor
                LogChange( "pump.FlowFactor", &pump.FlowFactor, value.toDouble() );
            } else if( key == "KP" ) { // Change PID KP value, proportional band
                LogChange( "heater.Kp", &heater.Kp, value.toDouble() );
            } else if( key == "KI" ) { // Change PID KI value, integral time (sec)
                LogChange( "heater.Ki", &heater.Ki, value.toDouble() );
            } else if( key == "KD" ) { // Change PID KD value, derivative time (sec)
                LogChange( "heater.Kd", &heater.Kd, value.toDouble() );
            // } else if( key == "PID" ) { // Change PID control automatic=1/manual=0
            //     sprintf( dataBuffer, "TS=%s,EVENT=VARIABLE_CHANGE,VAR=PID,FROM_=%u,TO_=%u", getTimestamp(), heater.GetMode(), value.toInt() );
            //     // heater.heaterPID.SetMode( value.toInt() );
            //     Serial.printf( "PID mode set to %s\n", heater.GetMode() );
            } else if( key == "OUT" ) { // Change heater output %
                LogChange( "heater.heaterOutput", &heater.heaterOutput, value.toDouble() );
            // } else if( key == "TD" ) { // Change temperature delta
            //     sprintf( dataBuffer, "TS=%s,EVENT=VARIABLE_CHANGE,VAR=Tdelta,FROM_=%f,TO_=%f", getTimestamp(), Tdelta, value.toDouble() );
            //     Tdelta = value.toDouble();
            //     Serial.printf( "Tdelta=%f\n", Tdelta );
            // } else if( key == "OH" ) { // Change high output level for relay mode control
            //     sprintf( dataBuffer, "TS=%s,EVENT=VARIABLE_CHANGE,VAR=maxOutput,FROM_=%f,TO_=%f", getTimestamp(), maxOutput, value.toDouble() );
            //     maxOutput = value.toDouble();
            //     Serial.printf( "maxOutput=%f\n", maxOutput );
            // } else if( key == "OL" ) { // Change temperature delta
            //     sprintf( dataBuffer, "TS=%s,EVENT=VARIABLE_CHANGE,VAR=minOutput,FROM_=%f,TO_=%f", getTimestamp(), minOutput, value.toDouble() );
            //     minOutput = value.toDouble();
            //     Serial.printf( "minOutput=%f\n", minOutput );
            } else if( key == "MQTT" ) { // Change MQTT Tx interval time
                LogChange( "mqttInterval", &mqttInterval, value.toInt() );
            } else if( key == "SD" ) { // Change SD card log interval time
                LogChange( "sdLogInterval", &sdLogInterval, value.toInt() );
            } else if( key == "USB" ) {
                LogChange( "serialLogInterval", &serialLogInterval, value.toInt() );
            } else if( key == "PUMP" ) {
                if ( value == "OFF" ) {
                    LogEvent( "PUMP OFF" );
                    pump.StopPump();
                } else if ( value == "ON" ) 
                    LogEvent( "PUMP ON" );{
                    pump.RunPump();
                }
            } else if( key == "FLUID" ) {
                if ( value == "WATER" ) {
                    LogEvent( "FLUID TYPE TO WATER" );
                    fluidType = value;
                    setFluid( fluidType );
                } else if ( value == "FOG" ) {
                    LogEvent( "FLUID TYPE TO FOG" );
                    fluidType = value;
                    setFluid( fluidType );
                } else {
                    Serial.printf( " Unknown fluid type: %s\n", value );
                }
            } else {
                Serial.print( " *** Unknown command.\n" );
            }
        }

    return;

}

void setFluid( String fluid ) {

    if ( fluid == "WATER" ) {
        fluidType = fluid;
        Tsetpoint = 160;
        pump.FlowFactor = 0.314839;  // mL of water per flow sensor pulse (measure at 7/5500 pulse mode for 10 minutes)
    } else if ( fluid == "FOG" ) {
        fluidType = fluid;
        Tsetpoint = 200;
        pump.FlowFactor = 0.363;  // Units: g/pulse, fog fluid tested 31-Oct-2024, (tested at 6 pulses every 6 seconds)  
    } else {
        Serial.print( "Unknown fluid type: %s\n");
    }

    return;
}

void PreheatMode( String command ) {

    // Serial.printf( "@PreheatMode command: %s", command );

    if( command == "START" ) {
        opMode = "PREHEAT";
        // Log the event
        sprintf( dataBuffer, "TS=%s,EVENT=ENTERED PREHEAT MODE\n", getTimestamp() );
        writeToSDcard( dataBuffer );
        sendMQTTdata( "steamer/data", dataBuffer );

        heater.Startup( Tsetpoint );  // This will also reset an OTC condition if warrented
        
        double deltaT = Tsetpoint - heater.maxTC1.avgT;

        Serial.printf( "@PreheatMode SP: %.1f  PV: %.1f deltaT: %.1f Fluid: %s\n", Tsetpoint, heater.maxTC1.avgT, deltaT, fluidType );
        
        if ( deltaT > 10.0 ) {
            
            // Heat capacity of aluminum = 0.896, power available is 1050 Watts
            // Mass small boiler=600, fogger boiler=1016
            // heatup time = mass * Cp * ΔT / power
            if ( fluidType == "FOG" ) {
                opStepTime = ( deltaT * 1016.0 * 0.896 / 1050.0 );  //  Gaggia Classic boiler at 1016 g and 1050 Watts
            } else {
                opStepTime = ( deltaT *  600.0 * 0.896 / 1050.0 );  // For mini Gaggia boiler at 600 g and 1050 Watts
            }
            modeStep = "POWER";
            // Heater to 100% in manual control mode
            heater.SetMode( "MAN" );
            heater.SetOutput( 100.0 );
            Serial.printf( " POWER mode, heatup time: %u sec.\n", opStepTime );
            opStepTime = millis() + opStepTime * 1000;

            // Log the event
            sprintf( dataBuffer, "TS=%s,EVENT=ENTERED POWER MODE\n", getTimestamp() );
            writeToSDcard( dataBuffer );
            sendMQTTdata( "steamer/data", dataBuffer );

        } else {
            // Change heater control to low-output High/Low relay control mode
            heater.HighLowRelayMode( rcControl[0][0], rcControl[0][1], rcControl[0][2] );
            // We are already within 5°C of the setpoint so go to PID control with 5% output
            // heater.SetOutput( 5.0, AUTOMATIC );
            modeStep = "WAIT";
            Serial.print( " WAIT mode\n" );
            // Log the event
            sprintf( dataBuffer, "TS=%s,EVENT=ENTERED WAIT MODE\n", getTimestamp() );
            writeToSDcard( dataBuffer );
            sendMQTTdata( "steamer/data", dataBuffer );
        }

        UpdateLCD( true );

        return;
    }

    if( command == "CANCEL" or keyPress == "UP" ) {
        // Turn off heater
        heater.SetOutput( 0.0 );
        // Go to idle mode
        IdleMode( "START" );
        return;
    }

    if( keyPress == "CENTER" ) {
        // A center key press cancels the preheat and returns to "Ready" mode
        ReadyMode( "START" );
        return;
    }

    if( rotaryDelta != 0 ) {
        Tsetpoint += double( rotaryDelta );
        heater.Setpoint( Tsetpoint );
        UpdateLCD( false );
        return;
    }

    if( modeStep == "POWER" ) {
        if( millis() > opStepTime ) {
            modeStep = "PAUSE";
            opStepTime = millis() + 30 * 1000;
            Serial.print( "Transition from POWER to PAUSE mode, starting relay mode\n");
            heater.SetOutput( 0.0 );
            UpdateLCD( true );
            Serial.printf( "POWER to PAUSE mode transition, opStepTime: %u\n", opStepTime );
            return;
        }
        return;
    }

    if( modeStep == "PAUSE" and millis() > opStepTime ) {
        modeStep = "WAIT";
        // Go to low-output mode
        Serial.print( "Transition from PAUSE to WAIT mode, starting relay mode\n");

        heater.HighLowRelayMode( rcControl[0][0], rcControl[0][1], rcControl[0][2] );
        UpdateLCD( true );
        Serial.print( "PAUSE to WAIT mode transition.\n" );
        return;
    }

    if( modeStep == "WAIT" and abs(heater.maxTC1.avgT-heater.setpoint) < 5.0 and heater.tStd<3.0 and abs(heater.tSlope)<2.0 ) {
        modeStep = "DONE";
        ReadyMode( "START" );
        Serial.print( "WAIT to READY mode transition.\n" );
        return;
    }

}

void IdleMode( String command ) {

    if( command == "START" ) {
        opMode = "IDLE";
        heater.Shutdown();
        pump.StopPump();
        autoOffTime = millis() + autoOffDelay;
        UpdateLCD( true );
        
        return;
    }

    if( keyPress == "CENTER" ) {
        PreheatMode( "START" );
        return;
    }

    if( keyPress == "UP" ) {
        ShutdownMode();
        return;
    }

    if( keyPress == "RIGHT" ) {
        pump.RunPump();
        Serial.print( "\n** Pump started.\n");
        return;
    }

    if( keyPress == "LEFT" ) {
        pump.StopPump();
        Serial.print( "\n** Pump stopped.\n");
        return;
    }

}

void SteamMode( String command ) {

    if( command == "START" ) {
        opMode = "STEAM";
        modeStep = "";
        // Initiate high output relay control
        heater.HighLowRelayMode( rcControl[1][0], rcControl[1][1], rcControl[1][2] );

        // Start the pump in pulse mode
        pump.InitPulseCycling( pulseIndex, pumpCycleTime );
        
        // Calculate the steam cycle stop time
        opStepTime = millis() + steamTime;

        // Log the event
        LogEvent( "STEAM CYCLE STARTED" );

        // Make some noise to signal the steam cycle has started
        tone( PIEZO_PIN, 400, 200 ); delay(20); tone( PIEZO_PIN, 500, 200 ); delay(20); tone( PIEZO_PIN, 600, 200 );
        UpdateLCD( true );
        
        return;
    }

    if( command == "DONE" or command == "CANCEL" ) {
        // Change heater control to low-output for idling
        heater.HighLowRelayMode( rcControl[0][0], rcControl[0][1], rcControl[0][2] );
        // Stop all pump activity
        pump.StopPump();      
        // Make some noise to signal the steam cycle has ended
        if( command == "DONE" ) {
            LogEvent( "STEAM CYCLE COMPLETED" );
            tone( PIEZO_PIN, 600, 250 ); delay(50); tone( PIEZO_PIN, 500, 250 ); delay(50); tone( PIEZO_PIN, 400, 250 );
        } else {
            LogEvent( "STEAM CYCLE CANCELLED" );
            tone( PIEZO_PIN, 600, 250 ); delay(50); tone( PIEZO_PIN, 600, 250 ); delay(50); tone( PIEZO_PIN, 600, 250 );
            Serial.print( "\n** Steam cycle CANCELED.\n");
        }

        autoOffTime = millis() + autoOffDelay;
        // modeStep = "RECOVERY";
        // Return to READY mode
        ReadyMode( "START" );       
        
        return;
    }

    if( keyPress == "UP" or keyPress == "CENTER" ) {        
        SteamMode( "CANCEL" );
        return;
    }

    if( keyPress == "RIGHT" ) {
        // Reduce the pulse interval time by 1000 msec
        pump.pumpCycleTime -= 500;
        UpdateLCD( false );
        Serial.printf( "\n** Reduced pump cycle interval time to %u mSec\n", pump.pumpCycleTime );
        return;
    }

    if( keyPress == "LEFT" ) {
        // Increase the pulse interval time by 1000 msec
        pump.pumpCycleTime += 500;
        UpdateLCD( false );
        Serial.printf( "\n** Increased pump cycle interval time to %u mSec\n", pump.pumpCycleTime );
        return;
    }

    if( rotaryDelta != 0 ) {
        long deltaTime = rotaryDelta * 10*1000;
        steamTime += deltaTime;

        // opStepTime is uint23_t so we need to do this:
        if( deltaTime > 0 ) {
            opStepTime += deltaTime;
        } else {
            opStepTime -= abs( deltaTime );
        }
        UpdateLCD( false );
        
        Serial.printf( "\n** Changed steam cycle time to %u mSec\n", steamTime );
        return;
    }

    // if ( modeStep == "RECOVERY" ) {
    //     if ( heater.maxTC1.avgT >= heater.minTemp ) {
    //         //Reduce the maximum output to minimim amount
    //         heater.HighLowRelayMode( rcControl[0][0], rcControl[0][1], rcControl[0][2] );
    //         // Enter READY mode
    //         ReadyMode( "START" );
    //         return;
    //     }
    // }

    // Check for an end of cycle
    if( millis() >= opStepTime ) {
        SteamMode( "DONE" );
        return;
    }

}

// void OvenMode( String command="" ) {
//     return;
// }

void FogMode( String command ) {

    // Tsetpoint=160, Tdelta=10, OutHigh=90.0, OutLow=70, TQdelta=15, Qhigh=1.0, Qlow=0.7, TQmin=20, Tover;
    // uint32_t Qfull=7000;

    if( command == "START" ) {
        opMode = "FOGGER";
        // Change to current Tsetpoint and manual output control
        heater.Setpoint( Tsetpoint );
        // Change heater control to low-output
        heater.HighLowRelayMode( rcControl[1][0], rcControl[1][1], rcControl[1][2] );
        // Start the automatic pump cycling protocol
        pump.InitPulseCycling( pulseIndex, pumpCycleTime );
        // Record start time of fog mode
        opStepTime = millis();
        // Make some noise to signal the for mode cycling has started
        tone( PIEZO_PIN, 400, 200 ); delay(20); tone( PIEZO_PIN, 500, 200 ); delay(20); tone( PIEZO_PIN, 600, 200 );
        UpdateLCD( true );
        return;
    }

    if( command == "DONE" or command == "CANCEL" ) {
        // Change heater control to low-output
        heater.HighLowRelayMode( rcControl[0][0], rcControl[0][1], rcControl[0][2] );
        // Return to PID control
        // heater.Setpoint( Tsetpoint, AUTOMATIC );
        pump.StopPump();        
        // Make some noise to signal the steam cycle has ended
        if( command = "DONE" ) {
            tone( PIEZO_PIN, 600, 250 ); delay(50); tone( PIEZO_PIN, 500, 250 ); delay(50); tone( PIEZO_PIN, 400, 250 );
            Serial.print( "\n** Fog cycle ended.\n");
        } else {
            tone( PIEZO_PIN, 600, 250 ); delay(50); tone( PIEZO_PIN, 600, 250 ); delay(50); tone( PIEZO_PIN, 600, 250 );
            Serial.print( "\n** Fog cycle CANCELED.\n");
        }
        // Enter READY mode
        ReadyMode( "START" );
        return;
    }

    if( keyPress == "UP" or keyPress == "CENTER" ) {
        // A center key press cancels the preheat and returns to "Ready" mode
        Serial.print( "\n** Canceling FOGGER cycle.\n" );
        FogMode( "CANCEL" );
        return;
    }

    if( keyPress == "LEFT" ) {
        // Increase the pulse interval time by 500 msec (decreases fluid flow)
        pumpCycleTime += 500;
        pump.pumpCycleTime = pumpCycleTime;
        UpdateLCD( false );
        Serial.printf( "\n** Reducing pump cycle time to %u mSec\n", pump.pumpCycleTime );
        return;
    }

    if( keyPress == "RIGHT" ) {
        // Reduce the pulse interval time by 500 msec (increases fluid flow)
        pumpCycleTime -= 500;
        pump.pumpCycleTime = pumpCycleTime;
        UpdateLCD( false );
        Serial.printf( "\n** Increasing pump cycle time to %u mSec\n", pump.pumpCycleTime );
        return;
    }

    if( rotaryDelta != 0 ) {
        // heater.heaterOutput += double( rotaryDelta );
        
        // if( heater.GetMode() ) {
        //     // Rotary inputs while in PID mode change the setpoint value
        //     Tsetpoint += double( rotaryDelta );
        //     heater.Setpoint( Tsetpoint, AUTOMATIC );
        //     Serial.printf( "\n** Changed fog setpoint to %.0f mSec\n", Tsetpoint );
        // } else {
        //     // Rotary inputs while in MANUAL mode change the heater output value
        //     heater.heaterOutput += double( rotaryDelta );
        //     Serial.printf( "\n** Changed heater output to %.0f%%\n", heater.heaterOutput );
        // }
        // Tsetpoint += double( rotaryDelta );
        // heater.Setpoint( Tsetpoint, MANUAL );
        // Update the LCD with the new value
        UpdateLCD( false );        
        
        return;
    }

}

void ReadyMode( String command ) {

    if( command == "START" ) {
        opMode = "READY";
        // Change heater control to low-output
        heater.HighLowRelayMode( rcControl[0][0], rcControl[0][1], rcControl[0][2] );
        // heater.Setpoint( Tsetpoint, AUTOMATIC );
        // Make some noise to signal we are now in Ready mode
        tone( PIEZO_PIN, 1000, 500 );
        UpdateLCD( true );
        // Log the event
        LogEvent( "ENTERED READY MODE" );

        nextMode = "IDLE";
        autoIdleTime = millis() + 10*60*1000;
        
        return;
    }

    if( keyPress == "CENTER" ) {
        SteamMode( "START" );
        return;
    }

    if( keyPress == "DOWN" ) {
        FogMode( "START" );
        return;
    }

    if( keyPress == "UP" ) {
        IdleMode( "START" );
        return;
    }

    if( rotaryDelta != 0 ) {
        // steamTime += rotaryDelta * 10*1000;
        Tsetpoint += double( rotaryDelta );
        heater.Setpoint( Tsetpoint );
        UpdateLCD( false );
        return;
    }

    if( keyPress == "LEFT" ) {
        pump.pumpCycleTime -= 500;
        UpdateLCD( false );
        return;
    }

    if( keyPress == "RIGHT" ) {
        pump.pumpCycleTime += 500;
        UpdateLCD( false );
        return;
    }

    // Check if we need to go to Idle mode
    if ( millis() > autoIdleTime ) {
        IdleMode( "START" );        
    }

}

void ShutdownMode() {

    DEBUG_PRINTLN("@ShutdownMode()");
    Serial.print( "\n\n**** SHUTDOWN MODE ****\n");

    heater.Shutdown();
    pump.StopPump();
    ClearScreen();
    Backlight( false );
    DisplayOff();

    LogEvent( "SHUTDOWN" );
    
    // Wait for user activity to wakeup
    // *** Need to add wakeup option via MQTT command or USB serial input
    while ( !checkANO() ) {
        updateMQTT();  // Keep the MQTT connection active during shutdown
        delay( 100 );
    }

    LogEvent( "WAKEUP" );

    Serial.print( "\n\n**** WAKING UP ****\n");

    keyPress = "";

    DEBUG_PRINTLN("Waking up!!");

    Backlight( true ); delay(50);
    heater.ResetTCs();
    heater.ResetOTC();
    IdleMode( "START" );

    OpResetLogTimes();
    
    return;
}

void OpResetLogTimes() {

    // Reset the SD card and MQTT update times
    serialLogTime = sdLogTime = mqttTime = millis();

    return;

}