#pragma once

#include <Arduino.h>

extern char mqttTopic[255], mqttMessage[255], timeSyncData[255];
extern bool wifiActive, mqttActive, mqttNewMsg, mqttDebug;
extern uint32_t mqttMsgCount, timeSyncDelay;

bool initWiFi( uint8_t );
bool initMQTT();
bool sendMQTTdata( const char* topic, const char* message );
bool updateMQTT();
bool recoverConnection();

bool timeSync( const char* requestedTime );
long getRSSI();