#define NON_BLOCKING_RECONNECT

#include <Arduino.h>
#include <main.h>

motorRequest request[3];
uint8_t motorIndex;
motorStatus status[3];
motorConfig config[3];
command moveCommand;
uint32_t statusReportTimer;

uint8_t lastState;

bool ledStatus;
uint32_t ledBlinkTimer;

uint32_t every100msTimer;
uint32_t every10msTimer;

L298N motors[3] =
    {
        L298N(EN_M1, IN1_M1, IN2_M1),
        L298N(EN_M1, IN1_M1, IN2_M1),
        L298N(EN_M1, IN1_M1, IN2_M1)};

uint16_t feedback[3] =
    {
        FEEDBACK_M1,
        FEEDBACK_M2,
        FEEDBACK_M3};

// motorCmd motorCommand[3];
const char *stateStrings[] =
    {
        "Idle",
        "Opening",
        "Closing",
        "Positioning",
        "Moving",
        "Stopping"};

extern PubSubClient mqttClient;

uint8_t monitorEndPosition(uint8_t motor, uint8_t direction)
{
  if (config[motor].calibrated)
  {
    if (direction == DIR_CLOSE)
    {
      if (analogRead(feedback[motor]) <= config[motor].startPos + END_POS_DEVIATE)
      {
        Debug_printf("Close end position %d reached\r\n", status[motor].curPos);
        return (true);
      }
    }
    else if (direction == DIR_OPEN)
    {
      if (analogRead(feedback[motor]) >= config[motor].endPos - END_POS_DEVIATE)
      {
        Debug_printf("Open end position %d reached\r\n", status[motor].curPos);
        return (true);
      }
    }
  }
  return false;
}

uint8_t monitorEndPosition(uint8_t motor)
{
  return monitorEndPosition(motor, status[motor].direction);
}

uint8_t posReached(uint8_t motor, uint8_t position)
{
  return posReached(motor, position, POS_DEVIATE);
}

uint8_t posReached(uint8_t motor, uint8_t position, uint8_t deviate)
{
  if (position >= status[motor].curPos - deviate && position <= status[motor].curPos + deviate)
  {
    Debug_printf("Position %d reached\r\n", status[motor].curPos);
    return (true);
  }
  return (false);
}

/* 
starts motor using given speed
returns false if started okay
*/
uint8_t motorStart(uint8_t motor, uint8_t direction, uint8_t speed)
{
  motorSetMoveTimeOut(motor, status[motor].timeoutMs);
  motorSetSpeed(motor, speed);
  if (!monitorEndPosition(motor, direction))
  {
    if (direction == DIR_CLOSE)
      motors[motor].backward();
    else
      motors[motor].forward();
  }
  else
  {
    Debug_printf("Motor %d on end position\r\n", motor);
    motorStop(motor);
    return true;
  }
  Debug_printf("Motor %d started, speed %d, dir %d \r\n", motor, speed, direction);
  return false;
}

/* 
if force, reset previous timeout. Else only set timeout if no timeout set yet
returns false if not set
*/
bool motorSetMoveTimeOut(uint8_t motor, uint16_t timeout, bool force)
{
  if (force || status[motor].moveTimeOut == 0)
  {
    status[motor].moveTimeOut = millis() + timeout;
  }
  else
    return false;
  return true;
}

bool motorSetMoveTimeOut(uint8_t motor, uint16_t timeout)
{
  return motorSetMoveTimeOut(motor, timeout, false);
}

void motorSetSpeed(uint8_t motor, uint8_t speed)
{
  motors[motor].setSpeed(speed * 255ul / 100);
  status[motor].speed = speed;
}

/* 
starts motor using previous set speed
returns false if started okay
*/
uint8_t motorStart(uint8_t motor, uint8_t direction)
{
  return motorStart(motor, direction, status[motor].speed);
}

/* starts motor using previous set speed and direction */
uint8_t motorStart(uint8_t motor)
{
  return motorStart(motor, status[motor].direction);
}

// starts the movement of the motor in the right direction
uint8_t moveToPosition(uint8_t motor, uint8_t position)
{

  uint8_t direction;
  if (position > 100)
    return true; // invalid position

  if (posReached(motor, position, POS_DEVIATE))
  {
    Debug_println("Already on position");
    return true;
  }
  if (position < status[motor].curPos)
  {
    direction = DIR_CLOSE;
    Debug_println(F("Positioning close direction"));
  }
  else if (position > status[motor].curPos)
  {
    direction = DIR_OPEN;
    Debug_println(F("Positioning open direction"));
  }
  else
  {
    Debug_println("Already on position");
    return true;
  }
  status[motor].desiredPos = position;
  /*
  if (closeToPosition)  
    slow speed
  else
    normal speed*/

  return motorStart(motor, direction); // , speed);
}
/*
Todo
- verwerken commands als een actie al begonnen is
- pas gaan communiceren als verbonden
- communicatie in 100ms loop
- error reset command
 */

/* 
calibrated:
open / close: set open/close position and position
manual move until release button -> set open/close position and stop on button release

not calibrated:
set direction, speed, start and stop manually
set open/close position
 */

void motorStop(uint8_t motor)
{
  motors[motor].stop();
  status[motor].moveTimeOut = 0;
}

void setDuration(uint8_t motor)
{
  if (request[motor].duration)
  {
    status[motor].moveEndTime = millis() + request[motor].duration;
  }
  else
    status[motor].moveEndTime = 0;
}

uint8_t doCommand(uint8_t motor, uint8_t cmd)
{
  if (!cmd)
    return false;
  uint8_t state = 0;

  switch (cmd) // translate the commands to a return state
  {
  case CMD_POSITION:

    state = ST_POSITIONING;
    break;
  case CMD_STOP:
    state = ST_STOP;
    break;
  case CMD_START:
    state = ST_START;
    break;
  case CMD_CLOSE:
    state = ST_CLOSE;
    break;
  case CMD_OPEN:
    state = ST_OPEN;
    break;
  default:
    break;
  }
  switch (state) // use return state to initiate the action
  {
  case ST_POSITIONING:
    if (!moveToPosition(motor, request[motor].position))
    {
      Debug_println(F("Positioning"));
    }
    else
      state = ST_STOP;
    break;
  case ST_STOP:
    motorStop(motor);
    break;
  case ST_START:
    if (!motorStart(motor))
    {
      setDuration(motor);
    }
    else
      state = ST_STOP;
    break;
  case ST_OPEN:
    Debug_println(F("Opening"));
    if (!motorStart(motor, DIR_OPEN)) // started ok
    {
      setDuration(motor);
    }
    else
      state = ST_STOP;
    break;
  case ST_CLOSE:
    Debug_println(F("Closing"));
    if (!motorStart(motor, DIR_CLOSE)) // started ok
    {
      setDuration(motor);
    }
    else
      state = ST_STOP;
    break;
  default:
    state = ST_STOP;
    motorStop(motor);
    break;
  }
  return (state);
}

/* status structure
motor : 1,2,3
speed : 0 - 100
direction : 0 / 1
state : string
position : 0 - 100

*/

/*
sends status for all motors
*/
void sendStatus()
{
  if (!mqttClient.connected())
  {
    return;
  }
  bool changedStatus = false;
  bool moving = false;
  for (uint8_t i = 0; i < MAX_MOTORS; i++)
  {
    switch (status[i].state)
    {
    case ST_OPEN:
    case ST_CLOSE:
    case ST_POSITIONING:
    case ST_START:
      moving = true;
      break;

    default:
      break;
    }
    changedStatus |= status[i].change;
  }

  if (changedStatus || millis() > statusReportTimer)
  {
    // https://wandbox.org/permlink/608elR7f8rSxwIJO
    StaticJsonDocument<MQTT_MAX_PACKET_SIZE> doc;
    JsonArray motors = doc.createNestedArray("motors");
    for (uint8_t i = 0; i < MAX_MOTORS; i++)
    {
      JsonObject motorData = motors.createNestedObject();
      motorData["direction"] = status[i].direction;
      motorData["state"] = stateStrings[status[i].state];
      motorData["position"] = status[i].curPos;
    }
    char buffer[MQTT_MAX_PACKET_SIZE];
    serializeJson(doc, buffer);
    if (mqttClient.publish(STATUS_TOPIC, buffer))
    {
      for (uint8_t i = 0; i < MAX_MOTORS; i++)
        status[i].change = false;
      if (moving)
        statusReportTimer = millis() + PER_STATUS_REPORT_MOVING;
      else
        statusReportTimer = millis() + PER_STATUS_REPORT;
    }
    else
      statusReportTimer = millis() + 100; // retry upon failed sending
    Debug_printf("Sending status, next in %ld \r\n", statusReportTimer - millis());
  }
}

void calculatePosition(uint8_t motor)
{
  uint32_t position;
  uint16_t adcReadValue = analogRead(feedback[motor]);
  if (config[motor].startPos - config[motor].endPos == 0)
    position = 50;
  else
    position = ((adcReadValue - config[motor].startPos) * 100UL) / (config[motor].endPos - config[motor].startPos);
  if (position != status[motor].curPos) // position changed
  {
    /* Debug_printf("M%d, St %d end %d \r\n", motor, config[motor].startPos, config[motor].endPos);
    Debug_printf("M%d changed position from %d to %d: %d\r\n", motor, status[motor].curPos, position, adcReadValue); */
    status[motor].curPos = position;
    status[motor].posChange = true;
  }
}

void every100ms()
{
  for (uint8_t i = 0; i < 1; i++)
  {
    calculatePosition(i);
  }
  sendStatus();
}

void every10ms()
{
}

void motorHandle()
{
  if (millis() > every100msTimer)
  {
    every100msTimer = millis() + 100;
    every100ms();
  }
  if (millis() > every10msTimer)
  {
    every10msTimer = millis() + 10;
    every10ms();
  }

  for (uint8_t i = 0; i < 1; i++)
  {
    if (status[i].state != lastState)
    {
      Debug_printf("New state %s\r\n", stateStrings[status[i].state]);
    }
    lastState = status[i].state;
    if (status[i].moveTimeOut && (millis() > status[i].moveTimeOut))
    {
      motorStop(i);
      status[i].moveTimeOut = 0;
      status[i].error = ERR_TIME_OUT;
      status[i].state = ST_STOP;
      Debug_printf("Motor %d timeout on moving\r\n", i);
    }
    switch (status[i].state)
    {
    case ST_IDLE:
      if (request[i].newRequest)
      {
        status[i].error = 0;
        status[i].state = doCommand(i, request[i].command);
        Debug_printf("Motor %d new request %d, going to %d\r\n", i, request[i].command, status[i].state);
        request[i].newRequest = false;
      }
      break;

    case ST_POSITIONING:
      if (posReached(i, status[i].desiredPos))
      {
        motorStop(i);
        Debug_printf("Motor %d on end position %d\r\n", i, status[i].desiredPos);
        status[i].state = ST_STOP;
      }
      if (request[i].newRequest)
        status[i].state = ST_IDLE;
      break;

    case ST_START:
      // ### todo, Do something here
      break;

    case ST_STOP:
      motorStop(i);
      Debug_printf("Motor %d stop state\r\n", i);
      /*     if (status[i].error)
        {
          Debug_printf("Motor %d, error %02x\r\n", i, status[i].error);
          status[i].state = ST_ERROR;
        }
        else */
      status[i].state = ST_IDLE;

      break;

    case ST_OPEN:
      if (status[i].moveEndTime && millis() > status[i].moveEndTime)
      {
        status[i].moveEndTime = 0;
        motorStop(i);
        Debug_printf("Motor %d open end time\r\n", i);
        status[i].state = ST_IDLE;
      }
      if (request[i].newRequest)
        status[i].state = ST_IDLE;
      break;

    case ST_CLOSE:
      if (status[i].moveEndTime && millis() > status[i].moveEndTime)
      {
        status[i].moveEndTime = 0;
        motorStop(i);
        Debug_printf("Motor %d close end time\r\n", i);
        status[i].state = ST_IDLE;
      }
      if (request[i].newRequest)
        status[i].state = ST_IDLE;
      break;

    case ST_ERROR:
      // if (request[i].newRequest && request[i].command == CMD_RESET_ERR)
      if (1)
      {
        Debug_printf("Error reset\r\n");
        status[i].state = ST_IDLE;
      }
      break;

    default:
      break;
    }
    if (status[i].state != lastState)
    {
      status[i].change = true;
    }
  }
}

/* 
sets the requested position for each motor in request struct
motor is not started here yet
*/
// void setPosition(int position, bool * motorUse)
void setPosition(int position, bool *motorUse)
{
  /*
  bool motorUse[3];
  motorUse[0] = motor_1;
  motorUse[1] = motor_2;
  motorUse[2] = motor_3;
*/

  for (uint8_t i = 0; i < MAX_MOTORS; i++)
  {
    if (motorUse[i])
    {
      request[i].position = position;
      request[i].command = CMD_POSITION;
      request[i].newRequest = true;
    }
  }
}

/*
sets speed to the motors
minimum 75% or the motor will stall
 */
void setSpeed(int speed, bool *motorUse)
{

  for (uint8_t i = 0; i < MAX_MOTORS; i++)
  {
    if (motorUse[i])
    {
      motorSetSpeed(i, speed);
    }
  }
}

/*

 */
void setDirRequest(bool direction, bool *motorUse)
{
  uint8_t dirCmd = direction == DIR_OPEN ? CMD_OPEN : CMD_CLOSE;

  for (uint8_t i = 0; i < MAX_MOTORS; i++)
  {
    if (motorUse[i])
    {
      request[i].command = dirCmd;
      request[i].newRequest = true;
    }
  }
}

/*
Sets the max time the blinds are allowed to move (protection)
 */
void setMoveTimeOut(uint16_t time, bool *motorUse)
{
  for (uint8_t i = 0; i < MAX_MOTORS; i++)
  {
    if (motorUse[i])
    {
      status[i].timeoutMs = time;
    }
  }
}

/*
Sets the duration of moving the blinds, so stop after x time
 */
void setMoveDuration(uint16_t time, bool *motorUse)
{
  for (uint8_t i = 0; i < MAX_MOTORS; i++)
  {
    if (motorUse[i])
    {
      request[i].duration = time;
    }
  }
}

void stopMoving(bool *motorUse)
{
  for (uint8_t i = 0; i < MAX_MOTORS; i++)
  {
    if (motorUse[i])
    {
      request[i].command = CMD_STOP;
      request[i].newRequest = true;
    }
  }
}

void startMoving(bool *motorUse)
{
  for (uint8_t i = 0; i < MAX_MOTORS; i++)
  {
    if (motorUse[i])
    {
      request[i].command = CMD_START;
      request[i].newRequest = true;
    }
  }
}

void blinkLed()
{
  if (millis() > ledBlinkTimer)
  {
    ledStatus ^= 1;
    ledBlinkTimer = millis() + 500;
    digitalWrite(BUILTIN_LED, ledStatus);
  }
}

/*
command structure
speed : 0 - 100
position : 0 - 100
direction : open / close
timeout : 0 - 60000 
motor_1 : true / false
motor_2 : true / false
motor_3 : true / false
command : stop / start / position
*/
/*

const size_t capacity = JSON_OBJECT_SIZE(7) + 80;
DynamicJsonDocument doc(capacity);

const char* json = "{\"speed\":0,\"position\":50,\"direction\":\"open\",\"command\":\"position\",\"motor_1\":true,\"motor_2\":true,\"motor_3\":true}";

deserializeJson(doc, json);

int speed = doc["speed"]; // 0
int position = doc["position"]; // 50
const char* direction = doc["direction"]; // "open"
const char* command = doc["command"]; // "position"
bool motor_1 = doc["motor_1"]; // true
bool motor_2 = doc["motor_2"]; // true
bool motor_3 = doc["motor_3"]; // true

open/close 
-> use speed

speed
-> set speed


*/

#define TOPSZ 60 // Max number of characters in topic string

void callback(char *topic, byte *payload, unsigned int length)
{
  char *str;
  char topicBuf[TOPSZ], dataBuf[length + 1];
  char *p;
  const char *mtopic = "NULL", *type = "NULL", *command = "NULL";
  uint16_t i = 0;

  strncpy(topicBuf, topic, sizeof(topicBuf));
  memcpy(dataBuf, payload, sizeof(dataBuf));
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
  Debug_printf("Topic extract is %s, %s, %s\r\n", mtopic, type, command);

  StaticJsonDocument<MQTT_MAX_PACKET_SIZE> doc;
  byte *json = (byte *)malloc(length + 1);
  // Copy the payload to the new buffer
  memcpy(json, payload, length);
  // Conversion to a printable string
  json[length] = '\0';
  // blinds/status -> send to controller
  // blinds/control -> send from controller
  if (strcmp(type, "control") == 0) // blinds/control
  {
    DeserializationError error = deserializeJson(doc, json);
    // Test if parsing succeeds.
    if (error)
    {
      Debug_print(F("deserializeJson() failed: "));
      Debug_println(error.c_str());
      return;
    }
    else
      serializeJson(doc, Serial);
    Debug_println();

    if (strcmp(command, "movement") == 0) // blinds/control/movement
    {
      moveCommand.speed = doc["speed"] | moveCommand.speed;             // 0
      moveCommand.position = doc["position"] | moveCommand.position;    // 50
      moveCommand.direction = doc["direction"] | moveCommand.direction; // "open"
      moveCommand.command = doc["command"] | moveCommand.command;       // "position"

      JsonArray motor = doc["motor"];
      moveCommand.motor[0] = motor[0] | moveCommand.motor[0];      // true
      moveCommand.motor[1] = motor[1] | moveCommand.motor[1];      // true
      moveCommand.motor[2] = motor[2] | moveCommand.motor[2];      // true
      moveCommand.timeoutMs = doc["timeoutMs"] | DEF_MOVE_TIMEOUT; // 10000
      moveCommand.duration = doc["duration"] | 0;
      moveCommand.newCommand = true;
      // uint8_t cmd;
      setMoveTimeOut(moveCommand.timeoutMs, moveCommand.motor);
      if (strcmp(moveCommand.command, "position") == 0)
      {
        // setPosition(moveCommand.position, moveCommand.motor[0], moveCommand.motor[1], moveCommand.motor[2]);
        setPosition(moveCommand.position, moveCommand.motor);
        setSpeed(moveCommand.speed, moveCommand.motor);
      }
      else if (strcmp(moveCommand.command, "speed") == 0)
      {
        setSpeed(moveCommand.speed, moveCommand.motor);
      }
      else if (strcmp(moveCommand.command, "open") == 0)
      {
        setDirRequest(DIR_OPEN, moveCommand.motor);
        if (moveCommand.duration)
          setMoveDuration(moveCommand.duration, moveCommand.motor);
        setSpeed(moveCommand.speed, moveCommand.motor);
      }
      else if (strcmp(moveCommand.command, "close") == 0)
      {
        setDirRequest(DIR_CLOSE, moveCommand.motor);
        if (moveCommand.duration)
          setMoveDuration(moveCommand.duration, moveCommand.motor);
        setSpeed(moveCommand.speed, moveCommand.motor);
      }
      else if (strcmp(moveCommand.command, "stop") == 0)
      {
        stopMoving(moveCommand.motor);
      }
      else if (strcmp(moveCommand.command, "start") == 0)
      {
        startMoving(moveCommand.motor);
        setSpeed(moveCommand.speed, moveCommand.motor);
        // cmd = CMD_START;
      }
      else
      {
        Debug_printf("invalid movement command %s", moveCommand.command);
        return; // invalid command
      }
    }
    else if (strcmp(command, "set_pos") == 0)
    {
      bool motor[3];
      const char *setCommand = doc["command"] | ""; // default empty
      JsonArray motorUse = doc["motor"];
      motor[0] = motorUse[0] | false;
      motor[1] = motorUse[1] | false;
      motor[2] = motorUse[2] | false;
      bool saveFile = false;
      int motorId;
      for (int i = 0; i < MAX_MOTORS; i++)
      {
        if (motor[i])
        {
          motorId = i;
          if (strcmp(setCommand, "startPos") == 0)
          {
            config[motorId].startPos = analogRead(feedback[motorId]);
            Debug_printf("set startpos motor %d to %d \r\n", motorId, config[motorId].startPos);
            saveFile = true;
          }
          else if (strcmp(setCommand, "endPos") == 0)
          {
            config[motorId].endPos = analogRead(feedback[motorId]);
            Debug_printf("set endpos motor %d to %d \r\n", motorId, config[motorId].endPos);
            saveFile = true;
          }
          else
          {
            Debug_printf("invalid setpos %s", setCommand);
            return; // invalid command
          }
        }
      }
      if (saveFile)
      {
        saveMotorConfig("/motor.json");
      }
    }
    else
    {
      Debug_printf("invalid command %s", command);
      return; // invalid command
    }
  }
  else
  {
    Debug_printf("invalid type %s", type);
    return; // invalid command
  }
}

uint8_t subscriptions(uint8_t status)
{
  mqttClient.subscribe(CONTROL_TOPIC, 1);
  Debug_printf("Subscribed to %s\r\n", CONTROL_TOPIC);
  return false;
}

// Loads the configuration from a file
uint8_t loadMotorConfig(const char *filename)
{
  Debug_printf("Loading motor cfg\r\n");
  if (SPIFFS.begin())
  {
    Debug_println("mounted file system");
    if (SPIFFS.exists(filename))
    {
      // Open file for reading
      File file = SPIFFS.open(filename, "r");
      const size_t capacity = JSON_ARRAY_SIZE(3) + JSON_OBJECT_SIZE(1) + 3 * JSON_OBJECT_SIZE(3) + 100;
      // Allocate a temporary JsonDocument
      StaticJsonDocument<capacity> doc;

      // Deserialize the JSON document
      DeserializationError error = deserializeJson(doc, file);
      if (error)
        Debug_println(F("Failed to read file, using default configuration"));
      else
        Debug_println(F("Loaded motor cfg success"));
      JsonArray motorArray = doc["motors"]; // get the arrays from the json

      for (uint8_t i = 0; i < MAX_MOTORS; i++)
      {
        JsonObject motorObj = motorArray[i];
        config[i].startPos = motorObj["startPos"] | 0;
        config[i].endPos = motorObj["endPos"] | 4095;
        config[i].calibrated = motorObj["calibrated"] | false;
      }

      // Close the file (Curiously, File's destructor doesn't close the file)
      file.close();
    }
    else
    {
      Debug_printf("failed to open %s\r\n", filename);
      return false;
    }
  }
  else
  {
    Debug_printf("failed to mount FS");
    return false;
  }
  return true;
}

// Saves the configuration to a file
void saveMotorConfig(const char *filename)
{
  // Delete existing file, otherwise the configuration is appended to the file
  //  SD.remove(filename);

  // Open file for writing
  File file = SPIFFS.open(filename, FILE_WRITE);
  if (!file)
  {
    Debug_println(F("Failed to create config file"));
    return;
  }

  const size_t capacity = JSON_ARRAY_SIZE(3) + JSON_OBJECT_SIZE(1) + 3 * JSON_OBJECT_SIZE(3) + 100;
  // Allocate a temporary JsonDocument
  StaticJsonDocument<capacity> doc;
  JsonArray motors = doc.createNestedArray("motors");
  for (uint8_t i = 0; i < MAX_MOTORS; i++)
  {
    JsonObject motorData = motors.createNestedObject();
    motorData["startPos"] = config[i].startPos;
    motorData["endPos"] = config[i].endPos;
    motorData["calibrated"] = config[i].calibrated;
  }
  serializeJson(doc, Serial);
  // Serialize JSON to file
  if (serializeJson(doc, file) == 0)
  {
    Debug_println(F("Failed to write to file"));
  }

  // Close the file
  file.close();
}

void setup()
{
  // hardware
  Serial.begin(115200);
  pinMode(BUILTIN_LED, OUTPUT);
  analogWriteFrequency(10000);
  analogWriteResolution(8);
  //  analogWriteResolution(EN_M2, 8);
  //  analogWriteResolution(EN_M3, 8);
  loadMotorConfig("/motor.json");
  // wifi / mqtt
  setupWifiMqtt(DEV_NAME);
  setPublishCallback(callback);
  setSubscribeCallback(subscriptions);

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });

  ArduinoOTA.begin();
}

void loop()
{
  mqttHandle();
  motorHandle();
  blinkLed();
  ArduinoOTA.handle();
}

TaskHandle_t CommunicationTask;
TaskHandle_t MotorTask;

// LED pins
const int led1 = 2;
const int led2 = 4;

void setup()
{/* 
  Serial.begin(115200);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT); */

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
      CommunicationTaskCode, /* Task function. */
      "Communication task",  /* name of task. */
      10000,                 /* Stack size of task */
      NULL,                  /* parameter of the task */
      1,                     /* priority of the task */
      &CommunicationTask,    /* Task handle to keep track of created task */
      0);                    /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
      MotorTaskCode, /* Task function. */
      "Motor task",  /* name of task. */
      10000,         /* Stack size of task */
      NULL,          /* parameter of the task */
      1,             /* priority of the task */
      &MotorTask,    /* Task handle to keep track of created task */
      1);            /* pin task to core 1 */
  delay(500);
}

//CommunicationTask: blinks an LED every 1000 ms
void CommunicationTaskCode(void *pvParameters)
{
  Serial.print("CommunicationTask running on core ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    commLoop();
  }
}

//Task2code: blinks an LED every 700 ms
void MotorTaskCode(void *pvParameters)
{
  Serial.print("MotorTask running on core ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    motorLoop();
  }
}

void commLoop()
{
}

void motorLoop()
{
}