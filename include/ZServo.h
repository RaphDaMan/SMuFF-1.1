/**
 * SMuFF Firmware
 * Copyright (C) 2019-2024 Technik Gegg
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include <stdlib.h>
#include <Arduino.h>
#include "Config.h"
#include "HAL/HAL.h"

#define MAX_SERVOS              5
#define US_PER_PULSE_0DEG       800       // microseconds for 0 degrees
#define US_PER_PULSE_180DEG     2200      // microseconds for 180 degrees
#ifdef USE_HIGHSPEED_SERVO
#define DUTY_CYCLE              4000      // servo cycle in us (>= 4ms, which gives a frequency of about 250Hz)
#else 
#define DUTY_CYCLE              20000     // servo cycle in us (>= 20ms, which gives a frequency of about 50Hz)
#endif
#define MOVING_SPEED            200       // moving speed of the servo for 60° in milliseconds (typical)
                                          // take into account: Servos move faster the higher the voltage
                                          // the real value is in the datasheet; this is some conservative value
#ifdef __ESP32__
#define SERVO_CHANNEL           8
#define SERVO_FREQ              50
#endif

#include "Debug.h"

void isrServoHandler();

static volatile bool timerSet = false;

class ZServo {
public:
  ZServo() { _pin = 0; };
  ZServo(int8_t servoIndex, pin_t pin) { attach(pin); setIndex(servoIndex); }

  void    attach(pin_t pin);
  void    attach(pin_t pin, bool useTimer, int8_t servoIndex = -1);
  void    attach(pin_t pin, int8_t servoIndex) { attach(pin); setIndex(servoIndex); }
  void    setIndex(int8_t servoIndex);
  void    detach();
  void    write(uint16_t val);
  void    writeMicroseconds(uint16_t microseconds);
  bool    setServoPos(uint8_t degree);           // sets the servo position in degree (between 0 and 180)
  void    setServoMS(uint16_t microseconds);     // sets the frequency in milliseconds
  void    setServo();                            // method called cyclically from an interrupt to refresh the servo position
  void    setDegreeMinMax(uint8_t min, uint8_t max) { _minDegree = min; _maxDegree = max; }
  void    setPulseWidthMinMax(uint16_t min, uint16_t max) { setPulseWidthMin(min); setPulseWidthMax(max); }
  void    setPulseWidthMin(uint16_t min) { _minPw = min; }
  void    setPulseWidthMax(uint16_t max) { _maxPw = max; }
  void    setTickRes(uint16_t res) { _tickRes = res; }
  uint16_t getTickRes() { return _tickRes; }
  void    setTickResAdjust(uint16_t ticks) { _tickAdjust = ticks; }
  uint16_t getTickResAdjust() { return _tickAdjust; }
  bool    hasTimer() { return _useTimer; }
  void    setMaxCycles(uint8_t val) { _maxCycles = val; }
  uint8_t getMaxCycles() { return _maxCycles; }
  void    disable();
  void    enable();
  bool    isDisabled() { return _disabled; }
  bool    isPulseComplete() { return _pulseComplete; }
  void    setDelay();

  uint8_t read() { return _degree; }
  uint8_t getDegree() { return _degree; }
  uint8_t getLastDegree() { return _lastDegree; }
  void    getDegreeMinMax(uint8_t* min, uint8_t* max) { *min = _minDegree; *max = _maxDegree; }
  void    getPulseWidthMinMax(uint16_t* min, uint16_t* max) { *min = _minPw; *max = _maxPw; }

private:
  void    setServoPin() { if(_pin != 0) digitalWrite(_pin, HIGH); }
  void    resetServoPin() { if(_pin != 0) digitalWrite(_pin, LOW); }

  volatile pin_t    _pin;
  volatile int8_t   _pinState;
  bool              _useTimer = false;
  bool              _disabled = false;
  int8_t            _servoIndex;
  volatile uint8_t  _degree;
  volatile uint8_t  _lastDegree;
  volatile uint32_t _lastUpdate;
  volatile uint32_t _tickCnt;
  volatile uint8_t  _dutyCnt;
  volatile uint8_t  _maxCycles;
  volatile uint16_t _pulseLen;
  uint16_t          _minPw = US_PER_PULSE_0DEG;
  uint16_t          _maxPw = US_PER_PULSE_180DEG;
  uint8_t           _minDegree = 0;
  uint8_t           _maxDegree = 180;
  volatile uint16_t _tickRes = 50;
  volatile uint16_t _tickAdjust = 0;
  volatile bool     _pulseComplete = true;
  volatile uint32_t* _fastPin;
  uint32_t          _pinMask_S;
  uint32_t          _pinMask_R;

};
