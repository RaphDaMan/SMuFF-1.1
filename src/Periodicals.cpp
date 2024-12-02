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

volatile bool leoNerdBlinkState  = false;
volatile bool leoNerdBlinkGreen  = false;
volatile bool leoNerdBlinkRed    = false;
volatile bool sendingStatesToggle = false;

static long fiveSecCounter = 0;

void sendStates(bool override) {
  if(!initDone)
    return;
  
  if(!override && (parserBusy || sendingResponse)) {
    __debugS(DEV, PSTR("Parser busy: %s  Sending Response: %s"), parserBusy ? P_Yes : P_No, sendingResponse ? P_Yes : P_No);
    return;
  }
  if(override)
    refreshStatus();
  // send status of endstops and current tool to all listeners, if configured
  if(!sendingResponse && smuffConfig.sendPeriodicalStats && initDone) {
    if(parserBusy && !override)
      return;
    printPeriodicalState(0);
    if(CAN_USE_SERIAL1 && (smuffConfig.hasPanelDue != 1 && smuffConfig.duet3Dport != 1))
      printPeriodicalState(1);
    if(CAN_USE_SERIAL2 && (smuffConfig.hasPanelDue != 2 && smuffConfig.duet3Dport != 2))
      printPeriodicalState(2);
    if(CAN_USE_SERIAL3 && (smuffConfig.hasPanelDue != 3 && smuffConfig.duet3Dport != 3))
      printPeriodicalState(3);
    sendingStatesToggle = !sendingStatesToggle;
  }
  // Add your periodical code here
}

void every10ms() {
  // Add your periodical code here
}

void every20ms() {
  // Add your periodical code here
}

void every50ms() {
  // Add your periodical code here
}

void every100ms() {
  // Add your periodical code here
}

void every250ms() {
  // Add your periodical code here
}

void every500ms() {
  #if defined(USE_DRYER)
    handleHeater();
  #endif
  refreshStatus();     // refresh main screen
  // Add your periodical code here
}

void every1s() {
  #if defined(USE_LEONERD_DISPLAY)
    if(leoNerdBlinkGreen || leoNerdBlinkRed) {
      leoNerdBlinkState = !leoNerdBlinkState;
      if(leoNerdBlinkGreen)
        encoder.setLED(LN_LED_GREEN, leoNerdBlinkState);
      if(leoNerdBlinkRed)
        encoder.setLED(LN_LED_RED, leoNerdBlinkState);
    }
    else {
      leoNerdBlinkState = false;
      encoder.setLED(LN_LED_GREEN, false);
      encoder.setLED(LN_LED_RED, false);
    }
  #endif

  #if defined(USE_DRYER)
    if(hasDryerHeaterSensor) {
      if(isHeating) {
        getHeaterDelta();
      }
      else 
        heater1prev = heater1;

      if(heaterOverflow) {
        __debugSInt(W, PSTR("Heater temp. reading failed. Please check sensors!"));
      }
    }
    if(isHeating && heaterTimeout > 0) {
      heaterTimeout--;
      if(heaterTimeout == 0) {
        stopDryer();
      }
    }
    if(isHeating && heaterEstimatedTime > 0) {
      heaterEstimatedTime--;
      if(heaterEstimatedTime == 0) {
        if(heater1 < heaterTargetTemp-10) {
          __debugSInt(W, PSTR("Heater target temperature not reached in estimated time!"));
          isHeaterTooSlow = true;
        }
      }
    }
  #endif

  // send states to WebInterface
  if(smuffConfig.webInterface) {
    sendStates();
  }

  // Add your periodical code here
}

void every2s() {
  if(!smuffConfig.webInterface) {
    sendStates();
  }
  if(aht10SensorsFound > 0) {
    flipTempHum++;
  }
  if(aht10SensorsFound == 2) {
    if(flipTempHum > 4)
      flipTempHum = 0;
  }
  else if(aht10SensorsFound == 1) {
    if(flipTempHum > 2)
      flipTempHum = 0;
  }
  // Add your periodical code here
}


void every5s() {

  fiveSecCounter++;
  if(fiveSecCounter % 12 == 0)
      showFreeMemory();       // dump memory info once a minute

#if defined(USE_SPOOLMOTOR) && defined(USE_DRYER)
  if(fiveSecCounter % (smuffConfig.spinInterval/5) == 0) {
    // check for interval spinning of spools every 15 seconds
    for(int i=0; i< MAX_TOOLS; i++) {
      if(intervalSpin[i] && i!= toolSelected) {
        windSpoolMotorCW(i, smuffConfig.spinSpeed, smuffConfig.spinDuration); // if set, spin spool for 1 second
      }
    }
  }
#endif

  if(fiveSecCounter % 6 == 0) {
    readTempHumidity();       // read temperature and humidity every 30 secs.
    if(dryerFan1Dynamic)
      handleDynamicFan();
      if(avgHeaterDeltaSamples > 0) {
        float avgDelta = avgHeaterDelta/avgHeaterDeltaSamples;
        __debugS(DEV4, PSTR("Avg. Heater Delta: %s°C/s"), String((avgDelta)).c_str());
        
        // check estimated heater delta after 2 minutes
        if(avgHeaterDeltaSamples % 240 == 0 && !isHeaterSet) {
          if(avgDelta < heaterEstimatedDelta-0.25f) {
            __debugS(W, PSTR("Avg. Heater Delta (%s°C/s) does not match calculated Delta (%s°C/s)"), String((avgDelta)).c_str(), String((heaterEstimatedDelta)).c_str());
            uint8_t t = heaterTargetTemp - heater1;
            heaterEstimatedTime = t / avgDelta + (60);
            __debugS(I, PSTR("Corrected estimated heat-up time to %d minutes"), heaterEstimatedTime / 60);
          }
        }
      }
    if(aht10SensorsFound > 0) {
      // __debugS(DEV4, PSTR("1st: %s°C\t%s%% RH\t2nd: %s°C\t%s%% RH"), String(temp1, 1).c_str(), String(humidity1, 1).c_str(), String(temp2, 1).c_str(), String(humidity2, 1).c_str());

      if(isHeating && sensorLog != nullptr) {
        char data[50];
        if(aht10SensorsFound >= 1 && smuffConfig.logSensor1) {
          snprintf_P(data, ArraySize(data)-1, P_LogFormat, millis(), 1, String(temp1, 1).c_str(), String(humidity1, 1).c_str()); 
          writeSensorLog(sensorLog, data);
        }
        if(aht10SensorsFound >= 2 && smuffConfig.logSensor2) {
          snprintf_P(data, ArraySize(data)-1,P_LogFormat, millis(), 2, String(temp2, 1).c_str(), String(humidity2, 1).c_str()); 
          writeSensorLog(sensorLog, data);
        }
      }
    }
  }

  // Add your periodical code here
}
