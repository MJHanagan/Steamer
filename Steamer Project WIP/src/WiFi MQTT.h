#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// #include "WiFi MQTT.h"
#include "shared.h"

// Be sure MAX_BUFFER_SIZE is defined to something larege enough to hold the longest publish or subscribe message

// extern char mqttTopic[MAX_BUFFER_SIZE], mqttMessage[MAX_BUFFER_SIZE], timeSyncData[MAX_BUFFER_SIZE];
extern char mqttTopic[], mqttMessage[], timeSyncData[];
extern bool wifiActive, mqttActive, mqttNewMsg, mqttDebug;
extern uint32_t mqttMsgCount, timeSyncDelay;
extern int mqttState, wifiState;

bool initWiFi( uint8_t );
bool initMQTT();
void IRAM_ATTR mqttCallback( char* topic, byte* message, unsigned int length );
bool sendMQTTdata( const char* topic, const char* message );
bool updateMQTT();
bool recoverConnection();

bool timeSync( const char* requestedTime );
long getRSSI();