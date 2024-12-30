#pragma once
// #include <Arduino.h>

#include "shared.h"

//  Variable from this module that are shared externally
extern String opMode, modeStep, fluidType;
extern uint32_t opStepTime, bootTime, startTime, endTime, offTime, sleepMillis, steamTime;
extern double Tsetpoint;
extern uint32_t loopStartTime, loopCounter, loopHz;
extern char swqStatus[];

void OpUpdate();
void OpLogData();
void OpCheckSerial();
void OpCheckMQTT();

uint8_t LogEvent( const char * text );
uint8_t LogChange( const char * vName, double *vPointer, double newValue );
uint8_t LogChange( const char * vName, int *vPointer, int newValue );
uint8_t LogChange( const char * vName, uint32_t *vPointer, uint32_t newValue );
uint8_t TxBuffer();

void IdleMode( String command="" );
void PreheatMode( String command="" );
void SteamMode( String command="" );
void FogMode( String command="" );
void ReadyMode( String command="" );
// void OvenMode( String command="" );
// void StandbyMode( String command="" );
void ShutdownMode();

void ChangeCommand( String cmdText );

void setFluid( String );

void OpResetLogTimes();


