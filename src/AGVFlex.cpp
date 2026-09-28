#include "AGVFlex.h"
#include <string.h>
#include <stdlib.h>

const int AGVFlex::PWM_PIN[4]    = {0, 3, 6, 9};
const int AGVFlex::IN1_PIN[4]    = {1, 4, 7, 10};
const int AGVFlex::IN2_PIN[4]    = {2, 5, 8, 11};
const int AGVFlex::SENSOR_PIN[8] = {5, 6, 7, 8, 9, 10, 11, 12};

volatile bool AGVFlex::s_magnetTriggered = false;
volatile unsigned long AGVFlex::s_lastMagnetTime = 0;

void AGVFlex::magnetISR() {
  if (millis() - s_lastMagnetTime > MAGNET_DEBOUNCE_MS) {
    s_magnetTriggered = true;
    s_lastMagnetTime = millis();
  }
}

AGVFlex::AGVFlex() : _bt(BT_RX_PIN, BT_TX_PIN)
{
  _stationCount = 0;
  _forwardSpeed = 0.32;
  _kp = 0.18;
  _kd = 0.60;
  _maxOmega = 0.6;
  _searchOmegaMax = 0.45;
  _junctionOvershoot = 500;
  _stationOvershoot  = 1000;
  _turnBlind         = 400;
  _uturnBlind        = 800;
  _autoAdvanceDelay  = 2000;
  _junctionSettle    = 1200;

  _previousError = 0; _lastError = 0;
  _state = STATE_IDLE;
  _currentHeading = AGV_NORTH;
  _currentNode = 0;
  _targetStationId = 0;
  _stationTickCount = 0;
  _stateTimer = 0; _overshootTimer = 0; _ignoreStationTimer = 0; _turnStartTime = 0;
  _ignoreJunctionUntil = 0;
  _lineGoneSince = 0;
  _queueLength = 0; _queueIndex = 0; _pendingAdvance = false;
  _cmdLen = 0;
}

void AGVFlex::setSpeeds(float forwardSpeed, float maxTurnRate) {
  _forwardSpeed = forwardSpeed;
  _maxOmega = maxTurnRate;
}
void AGVFlex::setLineFollowGains(float kp, float kd) {
  _kp = kp; _kd = kd;
}
void AGVFlex::setTimings(unsigned long junctionOvershootMs, unsigned long stationOvershootMs,
                              unsigned long turnBlindMs, unsigned long uturnBlindMs,
                              unsigned long autoAdvanceDelayMs, unsigned long junctionSettleMs) {
  _junctionOvershoot = junctionOvershootMs;
  _stationOvershoot  = stationOvershootMs;
  _turnBlind         = turnBlindMs;
  _uturnBlind        = uturnBlindMs;
  _autoAdvanceDelay  = autoAdvanceDelayMs;
  _junctionSettle    = junctionSettleMs;
}

void AGVFlex::begin() {
  Serial.begin(9600);
  _bt.begin(9600);
  _pwm.begin();
  _pwm.setPWMFreq(1000);
  delay(300);
  for (int i = 0; i < 8; i++) pinMode(SENSOR_PIN[i], INPUT);

  pinMode(MAGNET_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MAGNET_PIN), magnetISR, FALLING);

  _state = STATE_IDLE;
}

void AGVFlex::driveMotorSigned(int motor, float value) {
  bool forward = value >= 0;
  int speed = (int)(fabs(value) * MAX_SPEED);
  if (speed > MAX_SPEED) speed = MAX_SPEED;
  if (forward) { _pwm.setPin(IN1_PIN[motor], 4095); _pwm.setPin(IN2_PIN[motor], 0); }
  else         { _pwm.setPin(IN1_PIN[motor], 0);    _pwm.setPin(IN2_PIN[motor], 4095); }
  _pwm.setPin(PWM_PIN[motor], speed);
}

void AGVFlex::stopAll() {
  for (int m = 0; m < 4; m++) {
    _pwm.setPin(IN1_PIN[m], 0); _pwm.setPin(IN2_PIN[m], 0); _pwm.setPin(PWM_PIN[m], 0);
  }
}

void AGVFlex::driveVector(float vx, float vy, float omega) {
  float fl = vy + vx + omega;
  float fr = vy - vx - omega;
  float rl = vy - vx + omega;
  float rr = vy + vx - omega;

  float maxMag = max(max(fabs(fl), fabs(fr)), max(fabs(rl), fabs(rr)));
  if (maxMag > 1.0) { fl /= maxMag; fr /= maxMag; rl /= maxMag; rr /= maxMag; }

  driveMotorSigned(0, fl); driveMotorSigned(1, fr);
  driveMotorSigned(2, rr); driveMotorSigned(3, rl);
}

void AGVFlex::pivotInPlace(float omega) {
  driveMotorSigned(0, omega);  driveMotorSigned(1, -omega);
  driveMotorSigned(2, -omega); driveMotorSigned(3, omega);
}

void AGVFlex::readLineSensors(int &activeCount, float &weightedSum) {
  static const float weight[8] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};
  activeCount = 0;
  weightedSum = 0;
  for (int i = 0; i < 8; i++) {
    _sensorVal[i] = digitalRead(SENSOR_PIN[i]);
    if (_sensorVal[i]) { weightedSum += weight[i]; activeCount++; }
  }
}

void AGVFlex::followLineTick(int activeCount, float weightedSum) {
  if (activeCount > 0) {
    float error = weightedSum / activeCount;
    float derivative = error - _previousError;
    float omega = (_kp * error) + (_kd * derivative);
    omega = constrain(omega, -_maxOmega, _maxOmega);
    driveVector(0, _forwardSpeed, omega);
    _previousError = error;
    _lastError = error;
  } else {
    float omega = (_kp * _lastError * 0.4);
    omega = constrain(omega, -_maxOmega / 2, _maxOmega / 2);
    driveVector(0, _forwardSpeed, omega);
  }
}

int AGVFlex::getTurnDirection(AGVCompass current, AGVCompass target) {
  int diff = (target - current);
  if (diff < 0) diff += 4;
  if (diff == 0) return 0;
  if (diff == 1) return 1;
  if (diff == 3) return -1;
  return 2;
}

AGVCompass AGVFlex::getOpposite(AGVCompass c) {
  return (AGVCompass)((c + 2) % 4);
}

void AGVFlex::emit(const char* s) { _bt.print(s); Serial.print(s); }
void AGVFlex::emit(char c)        { _bt.print(c); Serial.print(c); }
void AGVFlex::emit(int n)         { _bt.print(n); Serial.print(n); }
void AGVFlex::emitLn()            { _bt.println(); Serial.println(); }
void AGVFlex::emitLn(const char* s) { _bt.println(s); Serial.println(s); }

void AGVFlex::calibratePosition(int id) {
  _currentNode = _stations[id].node;
  _currentHeading = _stations[id].direction;
  _targetStationId = id;
  _queueLength = 0; _queueIndex = 0; _pendingAdvance = false;
  stopAll();
  _state = STATE_STATION_WAIT;
  emit("POS,"); emit(id); emitLn();
}

void AGVFlex::advanceOrFinish() {
  emit("ARR,"); emit(_targetStationId); emitLn();
  _queueIndex++;
  if (_queueIndex < _queueLength) {
    _pendingAdvance = true;
    _stateTimer = millis();
  } else {
    _queueLength = 0; _queueIndex = 0;
    emitLn("DONE");
  }
}

void AGVFlex::beginLeg(int toId) {
  int fromId = _targetStationId;
  if (toId == fromId) {
    advanceOrFinish();
    return;
  }
  bool fromIsBay = (_stations[fromId].kind == AGV_BAY);
  emit("LEG,"); emit(fromId); emit(','); emit(toId); emitLn();
  _targetStationId = toId;
  if (fromIsBay) {
    _turnStartTime = millis();
    _state = STATE_PERFORMING_UTURN;
  } else {
    _ignoreJunctionUntil = millis() + _junctionSettle;
    _lineGoneSince = 0;
    _state = STATE_NAVIGATING_TRUNK;
  }
}

void AGVFlex::startRoute(const int* ids, int count) {
  _queueLength = count < MAX_ROUTE_STOPS ? count : MAX_ROUTE_STOPS;
  for (int i = 0; i < _queueLength; i++) _missionQueue[i] = ids[i];
  _queueIndex = 0;
  _pendingAdvance = false;
  beginLeg(_missionQueue[0]);
}

void AGVFlex::handleCommand(char* cmd) {
  if (cmd[0] == '\0') return;
  char type = cmd[0];
  char* rest = cmd + 1;

  if (type == 'C') { // CLEAR
    _stationCount = 0;
    emitLn("ACK,C");
    return;
  }

  if (type == 'M') { // MAP
    char* tId = strtok(rest, ",");
    char* tNode = strtok(NULL, ",");
    char* tDir = strtok(NULL, ",");
    char* tKind = strtok(NULL, ",");
    char* tDwell = strtok(NULL, ",");
    
    if (tId && tNode && tDir && tKind && tDwell) {
      int id = atoi(tId);
      if (id >= 0 && id < MAX_STATIONS) {
        _stations[id].node = atoi(tNode);
        
        char d = tDir[0];
        if (d == 'N') _stations[id].direction = AGV_NORTH;
        else if (d == 'E') _stations[id].direction = AGV_EAST;
        else if (d == 'S') _stations[id].direction = AGV_SOUTH;
        else _stations[id].direction = AGV_WEST;
        
        _stations[id].kind = (tKind[0] == 'B') ? AGV_BAY : AGV_TRUNK_END;
        _stations[id].dwellTimeMs = strtoul(tDwell, NULL, 10);
        
        if (id >= _stationCount) _stationCount = id + 1;
        emit("ACK,M,"); emitLn(id);
      }
    }
    return;
  }

  if (type == 'P') {
    int id = atoi(rest);
    if (id >= 0 && id < _stationCount) calibratePosition(id);
    return;
  }

  if (_state == STATE_IDLE) { emitLn("ERR,NOT_CALIBRATED"); return; }
  if (_state != STATE_STATION_WAIT) { emitLn("ERR,BUSY"); return; }

  if (type == 'G') {
    int id = atoi(rest);
    if (id >= 0 && id < _stationCount) {
      int ids[1] = { id };
      startRoute(ids, 1);
    }
  } else if (type == 'R') {
    int ids[MAX_ROUTE_STOPS];
    int count = 0;
    char* token = strtok(rest, ",");
    while (token != NULL && count < MAX_ROUTE_STOPS) {
      int id = atoi(token);
      if (id >= 0 && id < _stationCount) ids[count++] = id;
      token = strtok(NULL, ",");
    }
    if (count > 0) startRoute(ids, count);
  }
}

void AGVFlex::pollStream(Stream& stream) {
  while (stream.available()) {
    char c = stream.read();
    if (c == '\n' || c == '\r') {
      if (_cmdLen > 0) {
        _cmdBuf[_cmdLen] = '\0';
        handleCommand(_cmdBuf);
        _cmdLen = 0;
      }
    } else if (_cmdLen < CMD_BUF_LEN - 1) {
      _cmdBuf[_cmdLen++] = c;
    }
  }
}

void AGVFlex::pollBluetooth() {
  pollStream(_bt);
  pollStream(Serial);
}

void AGVFlex::update() {
  pollBluetooth();

  int activeCount;
  float weightedSum;
  readLineSensors(activeCount, weightedSum);

  bool junctionDetected = false;
  if (s_magnetTriggered) {
    junctionDetected = true;
    s_magnetTriggered = false;
  }

  switch (_state) {
    case STATE_IDLE:
      stopAll();
      break;

    case STATE_MOVING_TO_TRUNK:
      followLineTick(activeCount, weightedSum);
      if (junctionDetected) {
        int targetNode = _stations[_targetStationId].node;
        if (targetNode > _currentNode) {
          _targetHeading = AGV_NORTH;
          _overshootTimer = millis();
          _state = STATE_OVERSHOOTING;
        } else if (targetNode < _currentNode) {
          _targetHeading = AGV_SOUTH;
          _overshootTimer = millis();
          _state = STATE_OVERSHOOTING;
        } else {
          _targetHeading = _stations[_targetStationId].direction;
          _currentHeading = _targetHeading;
          _stationTickCount = 0;
          _ignoreStationTimer = millis();
          _state = STATE_ENTERING_STATION;
        }
      }
      break;

    case STATE_NAVIGATING_TRUNK: {
      followLineTick(activeCount, weightedSum);

      if (millis() < _ignoreJunctionUntil) {
        junctionDetected = false;
        _lineGoneSince = 0;
      }

      if (junctionDetected) {
        if (_currentHeading == AGV_NORTH) _currentNode++;
        if (_currentHeading == AGV_SOUTH) _currentNode--;
      }

      if (activeCount == 0) {
        if (_lineGoneSince == 0) _lineGoneSince = millis();
      } else {
        _lineGoneSince = 0;
      }

      const AGVStation& target = _stations[_targetStationId];
      if (target.kind == AGV_TRUNK_END) {
        unsigned long lineGoneFor = (_lineGoneSince != 0) ? (millis() - _lineGoneSince) : 0;
        
        // RESTORED: Stop if we see the magnet OR if the line ends
        bool arrivedByMagnet = junctionDetected && (_currentNode == target.node);
        bool arrivedByDeadReckoning = (lineGoneFor >= LINE_GONE_MS) && (_currentNode == target.node);
        
        if (arrivedByMagnet || arrivedByDeadReckoning) {
          emit("DBG,ARR,"); emit(arrivedByMagnet ? "MAGNET" : "DEADRECK");
          emit(','); emit((int)lineGoneFor); emitLn();
          
          stopAll();
          _lineGoneSince = 0;
          _currentNode = target.node; 
          if (_currentHeading != target.direction) {
            _turnStartTime = millis();
            _state = STATE_ARRIVAL_UTURN;
          } else {
            _state = STATE_STATION_WAIT;
            advanceOrFinish();
          }
        }
      } else if (junctionDetected && _currentNode == target.node) {
        _targetHeading = target.direction;
        _overshootTimer = millis();
        _state = STATE_OVERSHOOTING;
      }
      break;
    }

    case STATE_OVERSHOOTING:
      driveVector(0, _forwardSpeed, 0);
      if (millis() - _overshootTimer >= _junctionOvershoot) {
        _turnStartTime = millis();
        _state = STATE_TURNING;
      }
      break;

    case STATE_TURNING: {
      int turnDir = getTurnDirection(_currentHeading, _targetHeading);
      pivotInPlace(_searchOmegaMax * turnDir);
      if (millis() - _turnStartTime > _turnBlind) {
        if (_sensorVal[3] == 1 || _sensorVal[4] == 1) {
          stopAll();
          _currentHeading = _targetHeading;
          if (_currentNode == _stations[_targetStationId].node) {
            _stationTickCount = 0;
            _ignoreStationTimer = millis();
            _state = STATE_ENTERING_STATION;
          } else {
            _ignoreJunctionUntil = millis() + _junctionSettle;
            _lineGoneSince = 0;
            _state = STATE_NAVIGATING_TRUNK;
          }
        }
      }
      break;
    }

    case STATE_ENTERING_STATION:
      followLineTick(activeCount, weightedSum);
      if (millis() - _ignoreStationTimer > 500) {
        if (activeCount >= 5) _stationTickCount++;
        else if (activeCount <= 2) _stationTickCount = 0;
        if (_stationTickCount >= 3) {
          _stationTickCount = 0;
          _stateTimer = millis();
          _state = STATE_STATION_OVERSHOOT;
        }
      }
      break;

    case STATE_STATION_OVERSHOOT:
      driveVector(0, _forwardSpeed, 0);
      if (millis() - _stateTimer >= _stationOvershoot) {
        stopAll();
        _state = STATE_STATION_WAIT;
        advanceOrFinish();
      }
      break;

    case STATE_STATION_WAIT:
      stopAll();
      if (_pendingAdvance) {
        unsigned long waitTime = _stations[_targetStationId].dwellTimeMs;
        if (waitTime == 0) waitTime = _autoAdvanceDelay;
        if (millis() - _stateTimer >= waitTime) {
          _pendingAdvance = false;
          beginLeg(_missionQueue[_queueIndex]);
        }
      }
      break;

    case STATE_PERFORMING_UTURN:
      pivotInPlace(_searchOmegaMax);
      if (millis() - _turnStartTime > _uturnBlind) {
        if (_sensorVal[3] == 1 || _sensorVal[4] == 1) {
          stopAll();
          _currentHeading = getOpposite(_currentHeading);
          _ignoreJunctionUntil = millis() + _junctionSettle;
          _lineGoneSince = 0;
          _state = STATE_MOVING_TO_TRUNK;
        }
      }
      break;

    case STATE_ARRIVAL_UTURN:
      pivotInPlace(_searchOmegaMax);
      if (millis() - _turnStartTime > _uturnBlind) {
        if (_sensorVal[3] == 1 || _sensorVal[4] == 1) {
          stopAll();
          _currentHeading = getOpposite(_currentHeading);
          _state = STATE_STATION_WAIT;
          advanceOrFinish();
        }
      }
      break;
  }
}
