#include <mqttHandler.h>
#define MQTT_DEBUG

WiFiClient wifi_client;
PubSubClient mqttClient(wifi_client);

char _mqtt_server[] = "192.168.1.120";
char _mqtt_port[] = "1883";
char _mqtt_user[] = "";
char _mqtt_pass[] = "";
String savedSSID;
String savedPass;

uint8_t (*subscribeCallback)(uint8_t);
void (*publishCallback)(char *, uint8_t *, unsigned int);

long lastReconnectAttempt = 0;
uint8_t wifiDisconnect;
uint32_t firstDisconnectTime;

static String clientId;
static String deviceName;

void setupWifiMqtt(String devName)
{
  deviceName = devName; // + "-" + String(random(0xffff), HEX);;

  setup_wifimanager(false);
#ifdef ESP32
  WiFi.setHostname(deviceName.c_str());
#elif defined(ESP8266)
  WiFi.hostname(deviceName.c_str());
#endif
  setupMqtt();
  savedSSID = WiFi.SSID();
  savedPass = WiFi.psk();
  Serial.println("Pass and SSID");
  Serial.println(WiFi.SSID());
  Serial.println(WiFi.psk());
}

void setupMqtt()
{

  // Create a random client ID

  clientId = deviceName + "-" + String(random(0xffff), HEX);
  mqttClient.setServer(_mqtt_server, 1883);
  Serial.printf("MQTT server %s, port %s\r\n", _mqtt_server, _mqtt_port);
  mqttClient.setCallback(mqttCallback);
}

void setSubscribeCallback(uint8_t (*fptr)(uint8_t data))
{
  subscribeCallback = fptr;
}

void setPublishCallback(void (*fptr)(char *topic, byte *payload, unsigned int length))
{
  publishCallback = fptr;
}

char *getMqttPort()
{
  return (_mqtt_port);
}

char *getMqttServer()
{
  return (_mqtt_server);
}

boolean reconnect()
{
  String willTopic = deviceName + "/status" + "/LWT";
#ifdef MQTT_DEBUG
  Serial.print(String("Connecting with clientId " + clientId + "\n\r"));
#endif
  // Attempt to connect
  if (mqttClient.connect(clientId.c_str(), willTopic.c_str(), WILL_QOS, WILL_RETAIN, WILL_MESSAGE))
  {
    mqttClient.publish(willTopic.c_str(), ANNOUNCEMENT_MSG, WILL_RETAIN);
    String topic = deviceName + "/config/#";
    mqttClient.subscribe(topic.c_str());
#ifdef MQTT_DEBUG
    Serial.println(topic);
#endif
    //subscriptions();
    if (subscribeCallback)
    {
      subscribeCallback(false);
    }
  }
#ifdef MQTT_DEBUG
  Serial.printf("Leaving reconnect\n\r");
#endif
  return mqttClient.connected();
}

void mqttCallback(char *topic, byte *data, unsigned int data_len)
{
  char *str;
  char topicBuf[TOPSZ], dataBuf[data_len + 1];
  char *p;
  const char *mtopic = "NULL", *type = "NULL", *command = "NULL";
  uint16_t i = 0;

  strncpy(topicBuf, topic, sizeof(topicBuf));
  memcpy(dataBuf, data, sizeof(dataBuf));
  dataBuf[sizeof(dataBuf) - 1] = 0;
  // mtopic/type/command
  i = 0;
  for (str = strtok_r(topicBuf, "/", &p); str && i < 3; str = strtok_r(NULL, "/", &p))
  {
    switch (i++)
    {
    case 0: // Topic / GroupTopic / DVES_123456
      mtopic = str;
      break;
    case 1: // TopicIndex / Text
      type = str;
      break;
    case 2: //
      command = str;
    }
  }
#ifdef MQTT_DEBUG
  Serial.printf("Topic extract is %s, %s, %s\r\n", mtopic, type, command);
#endif
  if (strcmp(command, CONFIG_TOPIC) == 0)
  {
    uint8_t jsonOk;
#if ARDUINOJSON_VERSION_MAJOR < 6
    StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject &jsondata = jsonBuffer.parseObject(dataBuf);
    jsonOk = jsondata.success();
#else
    StaticJsonDocument<200> jsondata;
    DeserializationError error = deserializeJson(jsondata, dataBuf);
    jsonOk = (error == 0);
#endif
    if (jsonOk)
    {
#ifdef MQTT_DEBUG
      Serial.printf("JSON config success \r\n");
#endif
      File configFile = SPIFFS.open("/config.json", "w");
      if (!configFile)
      {
        Serial.printf("failed to open config file for writing");
      }
#if ARDUINOJSON_VERSION_MAJOR < 6
#ifdef MQTT_DEBUG
      jsondata.printTo(Serial);
#endif
      jsondata.printTo(configFile);
#else
#ifdef MQTT_DEBUG
      serializeJson(jsondata, Serial);
#endif
      serializeJson(jsondata, configFile);
#endif
      configFile.close();
    }
  }
  else if (strcmp(command, RESET_TOPIC) == 0)
  {
    Serial.printf("Reset command \r\n");
    ESP.restart();
  }
  else if (strcmp(command, START_AP_MODE_TOPIC) == 0)
  {

    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;
    //Set timeout before going to portal
    wifiManager.setConfigPortalTimeout(WIFIMANAGER_CONFIGPORTAL_TIMEOUT);
    if (!wifiManager.startConfigPortal(deviceName.c_str()))
    {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
  }

  if (publishCallback)
    publishCallback(topic, data, data_len);
}

void reconnectBlocking()
{
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("arduinoClient"))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqttClient.publish("outTopic", "hello world");
      // ... and resubscribe
      mqttClient.subscribe("inTopic");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
uint32_t wifiReconnectTmr;
uint8_t wifiReconnectStatus;
uint16_t wifiReconnectCount;
#define WIFI_RECONNECT_RETRY 10

/* 
#define NON_BLOCKING_RECONNECT

#ifdef NON_BLOCKING_RECONNECT
 */
void mqttHandle()
{
  if (!mqttClient.connected())
  {
    //  reconnectBlocking();

    long now = millis();
    if (WiFi.status() != WL_CONNECTED)
    {
    //  Serial.printf("Wifi disconnected %d, %d\r\n", wifiReconnectStatus, WiFi.status());

      // delay(1000);
      //return;
      if (wifiReconnectStatus == 0) // was connected
      {
        if (wifiDisconnect < 0xFF)
          wifiDisconnect++;
        Serial.printf("Wifi disconnected %d\r\n", wifiDisconnect);
        wifiReconnectCount = 0;
        firstDisconnectTime = millis();
        wifiReconnectStatus++;
      }
      else if (wifiReconnectStatus == 1)
      {
        wifiReconnectTmr = millis() + 10000;
        Serial.printf("Disabling WiFi %d\r\n", wifiReconnectCount);
        WiFi.mode(WIFI_OFF);
        WiFi.persistent(false); // Solve possible wifi init errors (re-add at 6.2.1.16 #4044, #4083)
        WiFi.disconnect(true);  // Delete SDK wifi config
        wifiReconnectStatus++;
        wifiReconnectTmr = millis() + 200;
      }
      else
      {
        if (wifiReconnectStatus == 2)
        {
          if (millis() > wifiReconnectTmr) // reconnect now
          {
            Serial.printf("Reconnecting WiFi %d\r\n", wifiReconnectCount);
            WiFi.mode(WIFI_STA); // Disable AP mode
            if (!WiFi.getAutoConnect())
            {
              WiFi.setAutoConnect(true);
            }
#ifdef ESP32
            WiFi.setHostname(deviceName.c_str());
#elif defined(ESP8266)
            WiFi.hostname(deviceName.c_str());
#endif
            Serial.println("Connecting to WiFi...");
            WiFi.begin(savedSSID.c_str(), savedPass.c_str());
            wifiReconnectStatus++;
            wifiReconnectTmr = millis() + 5000; // give connection time to reestablish
          }
        }
        else if (wifiReconnectStatus == 3) // check if we are connected now
        {
          if (WiFi.status() == WL_CONNECTED)
          {
            Serial.println("Connected");
          }
          else 
          {
            if (millis() > wifiReconnectTmr) 
            {
              Serial.printf("Not connected (%d), retry...\r\n", WiFi.status());
              wifiReconnectCount++;
              wifiReconnectTmr = millis() + 1000;
              wifiReconnectStatus = 1; // retry
  #if 0
                if (wifiReconnectCount > WIFI_RECONNECT_RETRY)
                {
                  Serial.printf("Restarting to reconnect\r\n");
                  ESP.restart();
                }
  #endif

            }
          }
        }
        
      }
    }
    else
    {
      if (wifiDisconnect)
        Serial.printf("Wifi connected\r\n");
     

      wifiDisconnect = 0;
      wifiReconnectStatus = 0;
      if (now - lastReconnectAttempt > 5000)
      {
        Serial.printf("Reconnecting MQTT\r\n");
        lastReconnectAttempt = now;
        // Attempt to reconnect
        if (reconnect())
        {
          lastReconnectAttempt = 0;
        }
      }
    }
  }
  else
  {
    mqttClient.loop();
  }
}

//Wifi manager parameters
//flag for saving data
bool shouldSaveConfig = true;
//do we have been connected once to mqtt

//callback notifying us of the need to save config
void saveConfigCallback()
{
  Serial.printf("Should save config");
  shouldSaveConfig = true;
}

void configModeCallback(WiFiManager *myWiFiManager)
{
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void setup_wifimanager(boolean reset_settings)
{

  // else
  /*
  if (reset_settings)  
  {
    Serial.println("Formatting SPIFFS");
    SPIFFS.format();
  }*/
  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin())
  {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json"))
    {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        Serial.println("opened config file");

        // Allocate a buffer to store contents of the file.

        uint8_t jsonOk = false;
#if ARDUINOJSON_VERSION_MAJOR < 6
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject &json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        jsonOk = json.success();
#else
        StaticJsonDocument<512> json;
        DeserializationError error = deserializeJson(json, configFile);
        serializeJson(json, Serial);
        jsonOk = (error == 0);
        if (error)
        {
          Serial.printf("deserializeJson() failed: ");
          Serial.println(error.c_str());
          return;
        }
#endif

        if (jsonOk)
        {
          //    char rikTest[12];
          Serial.printf("\nparsed json\n\r");
          strcpy(_mqtt_server, json["mqtt_server"]);
          strcpy(_mqtt_port, json["mqtt_port"]);
          strcpy(_mqtt_user, json["mqtt_user"]);
          strcpy(_mqtt_pass, json["mqtt_pass"]);
        }
        else
        {
          Serial.println("failed to load json config");
        }
      }
    }
  }
  else
  {
    Serial.printf("failed to mount FS");
  }

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom__mqtt_server("server", "mqtt server", _mqtt_server, 40);
  WiFiManagerParameter custom__mqtt_port("port", "mqtt port", _mqtt_port, 6);
  WiFiManagerParameter custom__mqtt_user("user", "mqtt user", _mqtt_user, 20);
  WiFiManagerParameter custom__mqtt_pass("pass", "mqtt pass", _mqtt_pass, 30);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  wifiManager.setDebugOutput(true);
  wifiManager.setEnableConfigPortal(true);
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);
  pinMode(FORCE_WIFI_AP_MODE, INPUT_PULLUP);
  delay(50);

  if (digitalRead(FORCE_WIFI_AP_MODE) == false) // Connect to GND to force
  {
    //   WiFiManager wm;
    Serial.printf("Resetting WifiManager settings %d", digitalRead(FORCE_WIFI_AP_MODE));

    //reset settings - for testing
    wifiManager.resetSettings();
    /* 
    wm.setTimeout(180);
    if (!wm.autoConnect(deviceName.c_str(), WIFIMANAGER_PASSWORD))
    {
      Serial.printf("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
    //if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)"); */
  }
  //Set timeout before going to portal
  wifiManager.setConfigPortalTimeout(WIFIMANAGER_CONFIGPORTAL_TIMEOUT);
  wifiManager.setConnectTimeout(300);
  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //add all your parameters here
  wifiManager.addParameter(&custom__mqtt_server);
  wifiManager.addParameter(&custom__mqtt_port);
  wifiManager.addParameter(&custom__mqtt_user);
  wifiManager.addParameter(&custom__mqtt_pass);

  //  if(reset_settings)  wifiManager.resetSettings();
  //set minimu quality of signal so it ignores AP's under that quality
  wifiManager.setMinimumSignalQuality(MINIMUM_WIFI_SIGNAL_QUALITY);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //and goes into a blocking loop awaiting configuration
  wifiManager.setTimeout(300);
  wifiManager.setWiFiAutoReconnect(true);
  if (!wifiManager.autoConnect(deviceName.c_str()))
  {
    Serial.printf("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.printf("connected...yeey :)\r\n");
  Serial.printf("Copying values\r\n");
  //read updated parameters
  strcpy(_mqtt_server, custom__mqtt_server.getValue());
  strcpy(_mqtt_port, custom__mqtt_port.getValue());
  strcpy(_mqtt_user, custom__mqtt_user.getValue());
  strcpy(_mqtt_pass, custom__mqtt_pass.getValue());
  Serial.printf("Copied values\r\n");
  delay(100);
  //save the custom parameters to FS
  if (shouldSaveConfig)
  {
    Serial.printf("saving config");
#if ARDUINOJSON_VERSION_MAJOR < 6
    DynamicJsonBuffer jsonBuffer;
    JsonObject &json = jsonBuffer.createObject();
#else
    StaticJsonDocument<512> json;
#endif

    json["mqtt_server"] = _mqtt_server;
    json["mqtt_port"] = _mqtt_port;
    json["mqtt_user"] = _mqtt_user;
    json["mqtt_pass"] = _mqtt_pass;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile)
    {
      Serial.printf("failed to open config file for writing");
    }
    delay(100);
#if ARDUINOJSON_VERSION_MAJOR < 6
    json.printTo(Serial);
    json.printTo(configFile);
#else
    serializeJson(json, Serial);
    serializeJson(json, configFile);
#endif
    configFile.close();
    //end save
  }
}

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