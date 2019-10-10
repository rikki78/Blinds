#ifndef MQTT_HANDLER
#define MQTT_HANDLER

  #ifdef ESP32
    #include <FS.h> 
    #include <WiFi.h>
    #include <ArduinoOTA.h>
    #include <WiFiUdp.h>
    #include <WiFiManager.h>  
  // #ifdef MDNS_SD
    #include <ESPmDNS.h>
    #include "SPIFFS.h"
  //  #endif
  #elif defined(ESP8266)
    #include <FS.h> 
    #include <ESP8266WiFi.h>
    #include <ArduinoOTA.h>
    #include <ESP8266WebServer.h>
    #include <WiFiManager.h>  
  //  #ifdef MDNS_SD
      #include <ESP8266mDNS.h>
  //  #endif
  #endif

  #include <DNSServer.h>
  
  #include <PubSubClient.h>
  #include <ArduinoJson.h>


#ifndef WILL_QOS
  #define WILL_QOS 0
#endif

#ifndef WILL_RETAIN 
  #define WILL_RETAIN true
#endif

#ifndef WILL_MESSAGE
  #define WILL_MESSAGE "Offline"
#endif

#ifndef ANNOUNCEMENT_MSG
  #define ANNOUNCEMENT_MSG "Online"
#endif

#ifndef FORCE_WIFI_AP_MODE
  #define FORCE_WIFI_AP_MODE 4  // input to force going into AP mode. Connect to GND to force
#endif

#define   WIFIMANAGER_CONFIGPORTAL_TIMEOUT 300
#define   MINIMUM_WIFI_SIGNAL_QUALITY 0
#define   WIFIMANAGER_SSID      "Wifi_Manager"
#define   WIFIMANAGER_PASSWORD  "123456789"


#define CONFIG_TOPIC  "mqtt"
#define RESET_TOPIC   "reset"
#define START_AP_MODE_TOPIC "start_ap_mode"
#define TOPSZ                  60           // Max number of characters in topic string
#define JSON_MSG_BUFFER       400

void setup_wifimanager(boolean reset_settings);

void setupWifiMqtt(String devName);
void setupMqtt();
boolean reconnect(); 
void mqttHandle();
void saveConfigCallback();
void setSubscribeCallback(uint8_t (*fptr)(uint8_t data));
void setPublishCallback(void (*fptr)(char* topic, byte* payload, unsigned int length));
void mqttCallback(char* topic, byte* data, unsigned int data_len);

char * getMqttPort();

char * getMqttServer();

/* 
// in main program
subscriptions()
{
    
  Serial.println(" connected");
  mqttClient.subscribe(CONTROLLER_WATCHDOG, 1);
  mqttClient.subscribe(PING_CMD, 1);
  mqttClient.subscribe(BANK_SET, 1);
  // OpenMQTT subscriptions
  if (mqttClient.subscribe(subjectMQTTtoX)) {
      mqttClient.subscribe(subjectMultiGTWRF);
  }
  
}
 */
 
 #endif // MQTT_HANDLER