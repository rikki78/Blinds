
#ifndef MAIN_H
#define MAIN_H

#define MQTT_MAX_PACKET_SIZE  256
#define MAX_MOTORS  3
#define NON_BLOCKING_RECONNECT

#include <mqttHandler.h>
#include <analogWrite.h>
#include <L298N.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>

/* IO defines motor control */
#define EN_M1 15
#define IN1_M1 16 // rxd2
#define IN2_M1 17 // txd2

#define EN_M2 14
#define IN1_M2 12
#define IN2_M2 13
#define EN_M3 23
#define IN1_M3 18
#define IN2_M3 19

#define FEEDBACK_M1 33
#define FEEDBACK_M2 34
#define FEEDBACK_M3 35
#define LED_BUILTIN 2

// directions
#define DIR_CLOSE 0 // retract the motors
#define DIR_OPEN 1  // extend the motors

#define POS_DEVIATE 1          // the deviation in position which is allowed in percents
#define PER_STATUS_REPORT 10000  // ms period in between status reports 
#define PER_STATUS_REPORT_MOVING  250   // ms period in between status reports when one of the motors is moving
#define END_POS_DEVIATE 10     // the deviation which is allowed, directly from motor. To monitor the end positions
#define DEF_MOVE_TIMEOUT 10000 // default timeout in which a movement should be completed

/* MQTT defines */
#define DEV_NAME "blinds"
#define CONTROL_CMD "control"
#define STATUS_CMD  "motor_status"
#define CONTROL_TOPIC DEV_NAME "/" CONTROL_CMD "/#"
#define STATUS_TOPIC  DEV_NAME "/" STATUS_CMD 

int debugLevel = 4;
#define LEVEL_VERBOSE 3 // show all
#define LEVEL_DEBUG_HI   2
#define LEVEL_DEBUG_LO   1
#define LEVEL_DEBUG_NONE   0

#define DEBUG_LEVEL LEVEL_DEBUG_HI

#if DEBUG_LEVEL >= LEVEL_VERBOSE 
  #define Debug_printfV       Debug_printf(x)
  #define Debug_printlnV(x)   Debug_println(x)
  #define Debug_printV(x)     Debug_print(x)
#else
  #define Debug_printfV  
  #define Debug_printlnV(x)
  #define Debug_printV(x)
#endif


#ifndef REMOTE_DEBUG
#define Debug_printf Serial.printf
#define Debug_println(x) Serial.println(x)
#define Debug_print(x) Serial.print(x)
#else
#define Debug_printf Debug.printf
#define Debug_println(x) Debug.println(x)
#define Debug_print(x) Debug_print(x)
#endif

#define DebugLevel_printf(level, x)    { if (level > debugLevel) Debug_printf(x);  }
#define DebugLevel_println(level, x)   { if (level > debugLevel) Debug_println(x); }
#define DebugLevel_print(level, x)     { if (level > debugLevel) Debug_print(x);   }

typedef struct
{
  const char *command;
  const char *direction;
  int speed;
  int position;
  uint16_t duration;
  uint16_t timeoutMs;
  bool motor[3];
  bool newCommand;
} command;

typedef struct
{
  int16_t curPos;      // between 0 and 100. 0 is closed, 100 is open
  uint16_t desiredPos; // between 0 and 100. 0 is closed, 100 is open
  uint8_t direction;   // ### maybe change to running, stopped,?
  uint8_t speed;
  uint8_t speedSet;
  uint8_t state;
  uint16_t  timeoutMs;
  uint32_t moveTimeOut; // end time of the movement
  uint8_t error;        // error status. 0 is no error
  uint32_t moveEndTime;
  bool change;
  bool posChange;
} motorStatus;

/* typedef struct 
{
  uint8_t   cmd;
  uint8_t   position; 
  uint8_t   speed; 
  uint16_t  timeoutMs;
  bool      changed;
}motorCmd;
 */
typedef struct
{
  uint16_t endPos;   // absolute max end position (coming from ADC value). Should be a high value
  uint16_t startPos; // absolute min end position (coming from ADC value). Should be a low value
  bool calibrated;   // set if end positions are calibrated
} motorConfig;

typedef struct
{
  int position;
  uint16_t duration; // move for a fixed duration
  uint8_t command;
  bool newRequest;
} motorRequest;

enum motorStates
{
  ST_IDLE,
  ST_OPEN,
  ST_CLOSE,
  ST_POSITIONING,
  ST_START,
  ST_STOP,
  ST_ERROR
};

enum motorCommands
{
  CMD_OPEN = 10, // move in opening direction
  CMD_CLOSE,     // move in closing direction
  CMD_POSITION,  // position automatically and start
  CMD_START,     // start moving, using previous direction
  CMD_STOP,      // stop moving
  CMD_DURATION,  // move for a certain time
  CMD_SPEED = 20, // set speed
  CMD_RESET_ERR   // reset error
};

enum motorErrors
{
  ERR_TIME_OUT = 1,  // moving took too long, position can't be reached
  ERR_TIME_DURATION, // relatieve inschakelduur overschreden
};

uint8_t monitorEndPosition(uint8_t motor, uint8_t direction);
uint8_t monitorEndPosition(uint8_t motor);
uint8_t posReached(uint8_t motor, uint8_t position);
uint8_t posReached(uint8_t motor, uint8_t position, uint8_t deviate);
uint8_t motorStart(uint8_t motor, uint8_t direction, uint8_t speed);
bool motorSetMoveTimeOut(uint8_t motor, uint16_t timeout, bool force);
bool motorSetMoveTimeOut(uint8_t motor, uint16_t timeout);
void motorSetSpeed(uint8_t motor, uint8_t speed);
uint8_t motorStart(uint8_t motor, uint8_t direction);
uint8_t motorStart(uint8_t motor);
uint8_t moveToPosition(uint8_t motor, uint8_t position);
void motorStop(uint8_t motor);
void setDuration(uint8_t motor);
uint8_t doCommand(uint8_t motor, uint8_t cmd);
void sendStatus();
void calculatePosition(uint8_t motor);
void every100ms();
void every10ms();
void motorHandle();
void setPosition(int position, bool * motorUse);
void setSpeed(int speed, bool * motorUse);
void setDirRequest(bool direction, bool * motorUse);
void setMoveTimeOut(uint16_t time, bool * motorUse);
void setMoveDuration(uint16_t time, bool * motorUse);
void stopMoving(bool * motorUse);
void startMoving(bool * motorUse);
void blinkLed();
void callback(char *topic, byte *payload, unsigned int length);
uint8_t subscriptions(uint8_t status);
uint8_t loadMotorConfig(const char *filename);
void saveMotorConfig(const char *filename);
void setup();
void setupComm();
void setupMotor();
void commLoop();
void motorLoop();
void loop();
void CommunicationTaskCode(void *pvParameters);
void MotorTaskCode(void *pvParameters);



#endif // MAIN_H