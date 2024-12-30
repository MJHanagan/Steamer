#include <Arduino.h>
#include "WiFi MQTT.h"

// Future needs:
// Automatic WiFi connection loss and reconnection.
// Timed reconnection events to not swamp the WiFi router with connection requests.

// Verify these are the setting for the WiFi network
#define WIFI_SSID "The Bird House" // This is the WiFi network name (aka: SSID)
#define WIFI_PASSWORD "BlackCrow97" // The SSID password
#define DEVICE_NAME "Stanley Steamer" // Device name that should appear on the server.

// Verify these are the same as the MQTT mosquito broker running on the RPi.
#define MQTT_SERVER "192.168.1.125"  // This is the IP address of the RPi hosting the MQTT broker (runnning the mosquitto broker)
#define MQTT_USERNAME "Buzzy" // the mosquitto mqtt username (defined by the mosquito setup on the RPi)
#define MQTT_PASSWORD "Splat" // the mosquitto mqtt password (defined by the mosquito setup on the RPi)
#define MQTT_CLIENT_ID "Stan" // The client id identifies the NodeMCU (defined by the mosquito setup on the RPi)

// #include <ESP8266WiFi.h> // Library for the ESP8266 WiFi module
// #include <WiFi.h>  // Library used for ESP32 boards
// Intantiate a WiFi Client object
WiFiClient wifiClient;

// #include <PubSubClient.h> // Library for the MQTT client routines

// Intantiate an MQTT Client object, see https://pubsubclient.knolleary.net/ and https://pubsubclient.knolleary.net/api
PubSubClient client( MQTT_SERVER, 1883, wifiClient ); // 1883 is the MQTT listener port for the Broker

// Subscribe topics
const char *subscribeTopics[] = { "logon", "steamer/cmd" };
const char *timeSyncPubTopic = "time/request";
const char *timeSyncSubTopic = "time/data";

// Where incoming MQTT messages get written
char mqttTopic[MAX_BUFFER_SIZE], mqttMessage[MAX_BUFFER_SIZE], timeSyncData[MAX_BUFFER_SIZE];


static bool result;

uint8_t timeSyncActive;
bool wifiActive, mqttActive, mqttNewMsg;
uint32_t mqttUpdateTime, timeSyncStart, timeSyncDelay, mqttMsgCount;
int mqttState, wifiState;

bool initWiFi( uint8_t maxTries=10  ) {
    
    WiFi.mode( WIFI_STA );  // Set the device as a station (client) on the WiFi network
    WiFi.config( INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE );
    // Change the hostname name of WiFi connected device.
    WiFi.hostname( DEVICE_NAME );

    // Connect to the specified WiFi network name
    Serial.print("Connecting to "); Serial.print( WIFI_SSID );
    WiFi.begin( WIFI_SSID, WIFI_PASSWORD );

    // Wait until the connection has been confirmed before continuing
    while ( WiFi.status() != WL_CONNECTED and maxTries > 0 ) {
        digitalWrite( LED_BUILTIN, !digitalRead(LED_BUILTIN) );
        Serial.print(".");
        delay(2000);
        maxTries--;
    }

    if ( WiFi.status() != WL_CONNECTED ) {
        Serial.printf( "\n*** Unable to connect to %s.",  WIFI_SSID );
        return false;
    }

    // We got here so we are now connected to the WiFi network
    wifiActive = true;

    // Debugging - Output the IP Address of the NodeMCU
    Serial.println( "\nWiFi connected" );
    Serial.print( "Connected as IP address: " );
    Serial.println( WiFi.localIP() );

    Serial.printf( "WiFi connection name is: %s\n", WiFi.getHostname() );

    Serial.println("WiFi setup complete.");

    return true;

}

long getRSSI() {

    // Returns the current WiFi signal strength
    // -30 dBm = Maximum achievable signal strength. The client can only be a few feet from the AP to achieve this. Not typical or desirable in the real world.
    // -67 dBm = Very Good.  Minimum signal strength for applications that require very reliable, timely delivery of data packets.
    // -70 dBm = Okay.  Minimum signal strength for reliable packet delivery. Email, web
    // -80 dBm = Not Good.  Minimum signal strength for basic connectivity. Packet delivery may be unreliable.
    // -90 dBm = Unusable.  Approaching or drowning in the noise floor. Any functionality is highly unlikely.
    
    return WiFi.RSSI();
}

bool initMQTT( ) {

    // Verify we are still connected to the WiFi, if not return with false
    if ( WiFi.status() != WL_CONNECTED ) return false;
  
    // Connect to the MQTT mosquito running on the RPi.
    Serial.print("Connecting to RPi MQTT broker... ");
    client.setServer( MQTT_SERVER, 1883 ); // RPi IP address and use TCP socket 1883 for the MQTT data
    client.setBufferSize( MAX_BUFFER_SIZE );  // The default packet size is only 128 bytes, so increase it here

    // Verify we are connected to the MQTT broker running on the RPi
    if ( client.connect( MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD ) ) {
        Serial.println("Connection successful.");
        mqttActive = true;
    } else {
        Serial.println("Connection to MQTT Broker failed!");
        mqttActive = false;
        return false;
    }

    // Set the callback routine for all subscribed MQTT topics
    client.setCallback( mqttCallback );

    // Subscribe to all topics found in 'subscribeTopics'
    uint8_t nTopics = sizeof(subscribeTopics) / sizeof(*subscribeTopics);
    Serial.printf( "Number of subscription topics: %u\n", nTopics );
    for( uint8_t i=0; i<nTopics; i++ ) {
        Serial.printf( "(%u) Subscribing to '%s'", i, subscribeTopics[i] );
        result = client.subscribe( subscribeTopics[i] );
        Serial.printf( " Result: %u\n", result );
    }

    result = client.publish( "logon", "Stanley Steamer connected" );

    if( result ) {
        Serial.print( "Logon message sent.\n" );
    } else {
        Serial.print( "Logon message  failed.\n" );
    }

    Serial.println("MQTT setup complete.");

    return true;
}

bool updateMQTT() {  // This must be run periodically to allow for the message callback to run

    // Only need to update the MQTT functions every 100 mSec
    if( millis() < mqttUpdateTime ) return true;

    client.loop();  // This updates the MQTT functions

    // Get the state of the MQTT connection with the broker
    //  client.state() returns:
    //  -4 : MQTT_CONNECTION_TIMEOUT - the server didn't respond within the keepalive time
    //  -3 : MQTT_CONNECTION_LOST - the network connection was broken
    //  -2 : MQTT_CONNECT_FAILED - the network connection failed
    //  -1 : MQTT_DISCONNECTED - the client is disconnected cleanly
    //   0 : MQTT_CONNECTED - the client is connected
    //   1 : MQTT_CONNECT_BAD_PROTOCOL - the server doesn't support the requested version of MQTT
    //   2 : MQTT_CONNECT_BAD_CLIENT_ID - the server rejected the client identifier
    //   3 : MQTT_CONNECT_UNAVAILABLE - the server was unable to accept the connection
    //   4 : MQTT_CONNECT_BAD_CREDENTIALS - the username/password were rejected
    //   5 : MQTT_CONNECT_UNAUTHORIZED - the client was not authorized to connect

    mqttState = client.state();

    //  WL_NO_SHIELD        = 255,
    //  WL_IDLE_STATUS      = 0,
    //  WL_NO_SSID_AVAIL    = 1,
    //  WL_SCAN_COMPLETED   = 2,
    //  WL_CONNECTED        = 3,
    //  WL_CONNECT_FAILED   = 4,
    //  WL_CONNECTION_LOST  = 5,
    //  WL_DISCONNECTED     = 6
    wifiState = WiFi.status();
    
    // Update the next check time
    mqttUpdateTime = millis() + 100;

    // If MQTT is currently connected, and therefore WiFi as well, return with a true
    if ( mqttState == MQTT_CONNECTED and wifiState == WL_CONNECTED ) return true;

    // If not connected try reconnecting and return the status result (true=reconnected, false=failed to reconnect)    
    return recoverConnection();
}

bool recoverConnection() {

  if ( WiFi.status() == WL_CONNECTED ) {
    Serial.println( "Attempting an MQTT reconnection." );
    result = initMQTT();
  } else {
    Serial.println( "Attempting a WiFi reconnection." );
    result = initWiFi( 5 );
    // Return if unable to reconnect to WiFi
    if ( !result ) return result;
    Serial.println( "Attempting an MQTT reconnection." );
    result = initMQTT();
    // Update the current state of the MQTT conection
    mqttState = client.state();
  }

  return result;
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
void IRAM_ATTR mqttCallback(char* topic, byte* message, unsigned int length) {

    // Index the message counter
    ++mqttMsgCount;

    Serial.printf( "\n*** MQTT message #%u arrived on topic: %s at %u\n", mqttMsgCount, topic, millis() );

    // Copy the topic text to the externally shared 'mqttTopic'
    strncpy( mqttTopic, topic, sizeof(topic) );

    // The MQTT message is sent as byte data, not char or String (so you can send numerical data 
    // in compact binary form, i.e. not a character representation of the value).
    for (int i = 0; i < length; i++) {
        mqttMessage[i] = (char)message[i];
    }

    mqttMessage[length] = 0;  // NULL terminate the char array

    Serial.printf( " Message: %s\n", mqttMessage );

    mqttNewMsg = true;

    // if we are waiting for a timeSync message then test for this
    if( timeSyncActive and strcmp(topic,timeSyncSubTopic)==0 ) {
        // Copy the topic text to the externally shared 'mqttTopic'
        strncpy( timeSyncData, mqttMessage, sizeof(mqttMessage) ); 
        timeSyncActive = false;
    }

    return;
}

bool timeSync( const char* requestedTime ) {

    timeSyncActive = false;
    // Clear out any previous time sync data
    timeSyncData[0] = 0;  // Just need to put a NULL character in the first array element

    // Subscribe to the time/data topic
    result = client.subscribe( timeSyncSubTopic );
    if( !result ) {
        Serial.printf( "Failed to subscribe to timesync topic: %s\n", timeSyncSubTopic );
        return false;
    }
    // Send the specified time request, e.g. "etime?"
    timeSyncStart = millis();
    result = client.publish( timeSyncPubTopic, requestedTime );
    
    if( !result ) {
        Serial.printf( "Failed to publish the timesync request to %s.\n", timeSyncPubTopic );
        // Unsubscribe to the time/data topic
        result = client.unsubscribe( timeSyncSubTopic );
        return false;
    }

    timeSyncActive = true;

    // Wait for timesync reply from the MQTT broker
    uint32_t endTime = millis() + 5000;  // Wait up to 5 seconds for a timesync reply
    // When the requested timesync response arrives in 'mqttCallback()' then timeSyncActive is set to false
    while ( timeSyncActive and millis() < endTime ) {
        Serial.print( ". " );
        delay( 5 );
        client.loop();  // This forces the MQTT client to look for new messages
    }

    // If no data was written to 'timeSyncData' then we must have timedout
    if( timeSyncData[0] == 0 ) {
        Serial.print( "Timesync request failed.\n" );
        return false;
    }

    // We had a valid reply to the request so calculate the millis delay time from request to receipt
    timeSyncDelay = millis() - timeSyncStart;

    return true;
}

bool sendMQTTdata( const char* pubTopic, const char* pubMsg ) {

    int msgLen = strlen( pubMsg );

    result = client.publish( pubTopic, pubMsg, msgLen );
    client.loop();  // This forces the update of the MQTT functions
    
    // if ( result ) {
    //     // Serial.printf( "Sent '%s' on topic '%s'  RSSI: %d", pubMsg, pubTopic, WiFi.RSSI() );
        
    // } else {
    //     // Serial.printf( "Failed to send '%s' on topic '%s'  RSSI: %d", pubTopic, pubMsg, WiFi.RSSI() );

    // }

  return result;

}