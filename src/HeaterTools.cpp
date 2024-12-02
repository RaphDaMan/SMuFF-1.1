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
#include "SMuFF.h"

static uint32_t lastTempTime = 0;
float           avgHeaterDelta;
uint32_t        avgHeaterDeltaSamples = 0; 
bool            isHeaterSet = false;
bool            isHeaterTooSlow = false;

void getHeaterTemp() {
#if defined(USE_DRYER)
  if(hasDryerHeaterSensor) {
    heater1 = readHeaterTemp(HEATER_TEMP_PIN, smuffConfig.thermistorUseBeta);
  }
#endif
}

void getHeaterDelta() {
#if defined(USE_DRYER)
  if(hasDryerHeaterSensor) {
    uint32_t deltaT = millis()-lastTempTime + 1;
    heater1delta = (heater1 - heater1prev)/((float)deltaT/1000);
    heater1prev = heater1;
    lastTempTime = millis();
    if(heater1delta != 0.0) {
        if(smuffConfig.dbgLevel & DEV4 == DEV4) { // don't bother converting if it aint going to be displayed
            char bufTemp[16], bufDelta[16];
            dtostrf(heater1, 3, 4, bufTemp);
            dtostrf(heater1delta, 2, 4, bufDelta);
            __debugS(DEV4, PSTR("Heater: %s°C Δ: %s°C (%d ms) (E:%d)"), bufTemp, bufDelta, deltaT, heaterDeltaErrors);
        }
        avgHeaterDelta += heater1delta;
        avgHeaterDeltaSamples++;
    }
  }
#endif
}

void setHeater(bool on) {
#if defined(USE_DRYER)
    #if defined(HEATER_PIN) && HEATER_PIN > 0
        if(on) {
            digitalWrite(HEATER_PIN, HIGH);
            isHeaterOn = true;
        }
        else {
            digitalWrite(HEATER_PIN, LOW);
            isHeaterOn = false;
        }
    #endif
#endif
}

void handleHeater() {
  #if defined(USE_DRYER)
    getHeaterTemp();
    
    if(isHeating) {
      getHeaterDelta();
      
      if(heater1 <= heaterTargetTemp-0.3) {
        setHeater(true);
      }
      else {
        isHeaterSet = (heater1 >= heaterTargetTemp-1 && heater1 < heaterTargetTemp+3);
        if(heater1 >= heaterTargetTemp+0.3)
          setHeater(false);
      }
      // never let it heat over the max. temperature configured (+ 5°C overshoot)
      if(heater1 > smuffConfig.heaterMaxTemp+5) {
        setHeater(false);
        heaterOverflow = true;
        __debugSInt(W, PSTR("Heater exceeded max. temperature: max=%d, temp=%s"), smuffConfig.heaterMaxTemp+5, String(heater1).c_str());
        return;
      }
      // stop heating if temperature rise is either too high or too low
      if(isHeaterOn && heater1delta > 0.0 && (heater1delta < smuffConfig.heaterDeltaMin || heater1delta > smuffConfig.heaterDeltaMax)) {
        /*
        char bufDeltaMin[16], bufDeltaMax[16], bufDelta[16];
        dtostrf(smuffConfig.heaterDeltaMin, 2, 6, bufDeltaMin);
        dtostrf(smuffConfig.heaterDeltaMax, 2, 6, bufDeltaMax);
        dtostrf(heater1delta, 2, 6, bufDelta);
        __debugSInt(W, PSTR("Heater exceeded delta: min=%s, max=%s Δ=%s°C"), bufDeltaMin, bufDeltaMax, bufDelta);
        */
       if(--heaterDeltaErrors == 0) {
          heaterOverflow = true;
          __debugSInt(DEV3, PSTR("Heater exceeded max. delta errors!"));
       }
      }
    }
  #endif
}

void handleDynamicFan() {
#if defined(USE_DRYER)
    uint8_t currentSpeed = dryerFan1Speed;
    if(humidity1 >= smuffConfig.humidityLevels[3]) {
        dryerFan1Speed = smuffConfig.humidityFanSpeeds[3];
    }
    else if(humidity1 >= smuffConfig.humidityLevels[2] && humidity1 < smuffConfig.humidityLevels[3]) {
        dryerFan1Speed = smuffConfig.humidityFanSpeeds[2];
    }
    else if(humidity1 >= smuffConfig.humidityLevels[1] && humidity1 < smuffConfig.humidityLevels[2]) {
        dryerFan1Speed = smuffConfig.humidityFanSpeeds[1];
    }
    else if(humidity1 <= smuffConfig.humidityLevels[0]) {
        dryerFan1Speed = smuffConfig.humidityFanSpeeds[0];
    }
    
    if(isHeaterSet && dryerFan1Speed != currentSpeed) {
        __debugS(DEV3, PSTR("Changing FAN1 speed dynamically to %d%%"), dryerFan1Speed);

        #if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
            fanDryer1.setFanSpeed(dryerFan1Speed);
        #else
            #if FAN_PIN1 > 0
                analogWrite(FAN1_PIN, map(dryerFan1Speed, 0, 100, 0, 255));
            #endif
        #endif
    }
#endif
}
