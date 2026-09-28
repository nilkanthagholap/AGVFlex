#ifndef AGV_FLEX_H
#define AGV_FLEX_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Adafruit_PWMServoDriver.h>

enum AGVCompass { AGV_NORTH = 0, AGV_EAST = 1, AGV_SOUTH = 2, AGV_WEST = 3 };

enum AGVStationKind {
  AGV_BAY,       
  AGV_TRUNK_END  
};

struct AGVStation {
  int node;               
  AGVCompass direction;    
  AGVStationKind kind;
  unsigned long dwellTimeMs;
};

class AGVFlex {
public:
  AGVFlex();
  void begin();
  void update();

  void setSpeeds(float forwardSpeed, float maxTurnRate);
  void setLineFollowGains(float kp, float kd);
  void setTimings(unsigned long junctionOvershootMs, unsigned long stationOvershootMs,
                   unsigned long turnBlindMs, unsigned long uturnBlindMs,
                   unsigned long autoAdvanceDelayMs, unsigned long junctionSettleMs = 1200);

private:
  enum State {
    STATE_IDLE,
    STATE_MOVING_TO_TRUNK,
    STATE_NAVIGATING_TRUNK,
    STATE_OVERSHOOTING,
    STATE_TURNING,
    STATE_ENTERING_STATION,
    STATE_STATION_OVERSHOOT,
    STATE_STATION_WAIT,
    STATE_PERFORMING_UTURN,
    STATE_ARRIVAL_UTURN
  };

  static const int PWM_PIN[4];
  static const int IN1_PIN[4];
  static const int IN2_PIN[4];
  static const int SENSOR_PIN[8];
  static const int MAGNET_PIN = 2;
  static const int BT_RX_PIN = 3;
  static const int BT_TX_PIN = 4;
  static const int MAX_SPEED = 3000;
  static const int MAX_ROUTE_STOPS = 12;
  static const int CMD_BUF_LEN = 48;
  static const int MAX_STATIONS = 20; 
  static const unsigned long MAGNET_DEBOUNCE_MS = 1500;
  static const unsigned long LINE_GONE_MS = 300;

  float _forwardSpeed;
  float _kp, _kd;
  float _maxOmega;
  float _searchOmegaMax;
  unsigned long _junctionOvershoot, _stationOvershoot, _turnBlind, _uturnBlind, _autoAdvanceDelay, _junctionSettle;

  AGVStation _stations[MAX_STATIONS];
  int _stationCount;

  Adafruit_PWMServoDriver _pwm;
  SoftwareSerial _bt;

  int _sensorVal[8];
  float _previousError, _lastError;
  State _state;
  AGVCompass _currentHeading;
  int _currentNode;
  int _targetStationId;
  AGVCompass _targetHeading;
  int _stationTickCount;
  unsigned long _stateTimer, _overshootTimer, _ignoreStationTimer, _turnStartTime;
  unsigned long _ignoreJunctionUntil;
  unsigned long _lineGoneSince;  
  int _missionQueue[MAX_ROUTE_STOPS];
  int _queueLength, _queueIndex;
  bool _pendingAdvance;
  char _cmdBuf[CMD_BUF_LEN];
  int _cmdLen;

  static volatile bool s_magnetTriggered;
  static volatile unsigned long s_lastMagnetTime;
  static void magnetISR();

  void driveMotorSigned(int motor, float value);
  void stopAll();
  void driveVector(float vx, float vy, float omega);
  void pivotInPlace(float omega);
  void readLineSensors(int &activeCount, float &weightedSum);
  void followLineTick(int activeCount, float weightedSum);
  int getTurnDirection(AGVCompass current, AGVCompass target);
  AGVCompass getOpposite(AGVCompass c);

  void pollBluetooth();
  void pollStream(Stream& stream);
  void emit(const char* s);
  void emit(char c);
  void emit(int n);
  void emitLn();
  void emitLn(const char* s);
  void handleCommand(char* cmd);
  void calibratePosition(int id);
  void beginLeg(int toId);
  void startRoute(const int* ids, int count);
  void advanceOrFinish();
};

#endif
