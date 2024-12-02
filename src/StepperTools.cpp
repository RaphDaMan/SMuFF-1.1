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

/*
    Supporting functions for stepper motors, mostly used by instances of ZStepper class.
*/

#include "SMuFF.h"

// forward declarations of some locally used functions
void startStepperInterval(timerVal_t duration=0);
void calcSyncMovementFactor();

volatile byte remainingSteppersFlag = 0;
volatile byte startStepperIndex = 0;

#if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)

#define DELAY_LOOP(STPR)  for(int i=0; i < smuffConfig.stepDelay[STPR]; i++) asm("NOP");

volatile uint32_t        *stepper_reg_X = &(digitalPinToPort(X_STEP_PIN)->BSRR);
volatile uint32_t        *stepper_reg_Y = &(digitalPinToPort(Y_STEP_PIN)->BSRR);
volatile uint32_t        *stepper_reg_Z = &(digitalPinToPort(Z_STEP_PIN)->BSRR);
// preset of set/reset values to make the interrupt routine faster
uint32_t                  pinMask_Xset = digitalPinToBitMask(X_STEP_PIN);
uint32_t                  pinMask_Yset = digitalPinToBitMask(Y_STEP_PIN);
uint32_t                  pinMask_Zset = digitalPinToBitMask(Z_STEP_PIN);
uint32_t                  pinMask_Xrst = digitalPinToBitMask(X_STEP_PIN) << 16;
uint32_t                  pinMask_Yrst = digitalPinToBitMask(Y_STEP_PIN) << 16;
uint32_t                  pinMask_Zrst = digitalPinToBitMask(Z_STEP_PIN) << 16;
#endif

//=====================================================================================================
// Override functions called by ZStepper library
//=====================================================================================================

void overrideStepX(pin_t pin, bool resetPin) {
  #if defined(X_STEP_PIN_NAME)
    *stepper_reg_X = resetPin ? pinMask_Xset : pinMask_Xrst;
    #if defined(USE_XSTEP_DELAY)
      if(resetPin && smuffConfig.stepDelay[SELECTOR] > 0)
        DELAY_LOOP(SELECTOR)
    #endif
  #else
    if(!resetPin)
      STEP_HIGH_X
    else {
      STEP_LOW_X
      #if defined(USE_XSTEP_DELAY)
        if(smuffConfig.stepDelay[SELECTOR] > 0)
          delayMicroseconds(smuffConfig.stepDelay[SELECTOR]);
      #endif
    }
  #endif
}

void overrideStepY(pin_t pin, bool resetPin) {
  #if defined(Y_STEP_PIN_NAME)
    *stepper_reg_Y = resetPin ? pinMask_Yset : pinMask_Yrst;
    #if defined(USE_YSTEP_DELAY)
      if(resetPin && smuffConfig.stepDelay[REVOLVER] > 0)
        DELAY_LOOP(REVOLVER)
    #endif
  #else
    if(!resetPin)
      STEP_HIGH_Y
    else {
      STEP_LOW_Y
      #if defined(USE_YSTEP_DELAY)
        if(smuffConfig.stepDelay[REVOLVER] > 0)
          delayMicroseconds(smuffConfig.stepDelay[REVOLVER]);
      #endif
    }
  #endif
}

void overrideStepZ(pin_t pin, bool resetPin) {
  #if defined(Z_STEP_PIN_NAME)
    *stepper_reg_Z = resetPin ? pinMask_Zset : pinMask_Zrst;
    #if defined(USE_ZSTEP_DELAY)
      if(resetPin && smuffConfig.stepDelay[FEEDER] > 0)
        DELAY_LOOP(FEEDER)
    #endif
  #else
    if(!resetPin)
      STEP_HIGH_Z
    else {
      STEP_LOW_Z
      #if defined(USE_ZSTEP_DELAY)
        if(smuffConfig.stepDelay[FEEDER] > 0)
          delayMicroseconds(smuffConfig.stepDelay[FEEDER]);
      #endif
    }
  #endif
}


//=====================================================================================================
// ISR handler and helper functions needed for ZStepper library
//=====================================================================================================

void isrStepperTimerHandler() {
  // fastFlipDbg();                              // for debugging only
  timerVal_t duration;
  byte i = startStepperIndex;              // startStepperIndex is either 0 if multiple steppers are in action or the according stepper
  do {
    if (!(_BV(i) & remainingSteppersFlag))    // current stepper doesn't need movement, continue with next one
      continue;

    if (steppers[i].getInterruptFactor() > 0) {
      if (!steppers[i].allowInterrupt())      // check whether a synced stepper needs movement too
        continue;
    }
    else
      duration = steppers[i].getDuration();   // get next interrupt interval

    if (steppers[i].handleISR()) {            // current stepper has to move, call it's handler
      remainingSteppersFlag &= ~_BV(i);       // mark current stepper as done, if handler returned true
      if (remainingSteppersFlag == 0)         // leave, if no more steppers need movement
        break;
    }

  } while(++i < NUM_STEPPERS);
  startStepperInterval(duration);              // start next interval if needed
  // fastFlipDbg();                              // for debugging only
}

void startStepperInterval(timerVal_t duration) {
  if (remainingSteppersFlag != 0) {                         // does any stepper still need to move?
    stepperTimer.setNextInterruptInterval(duration);        // yes, set timer value for next interrupt
  }
  else {
    stepperTimer.stop();                                    // no more stepper movements pending, stop timer, done
  }
}

void calcSyncMovementFactor() {

  // if(!smuffConfig.allowSyncSteppers)                // obsolete: don't care, if sync movement is disabled
  //   return;

  long maxTotal = 0;
  uint8_t maxTotalIndex = 0;

  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {      // find largest pending movement (most total steps)
    if (!(_BV(i) & remainingSteppersFlag))
      continue;
    long ts = steppers[i].getTotalSteps();
    if(ts > maxTotal) {
      maxTotal = ts-1;
      maxTotalIndex  = i;
    }
  }
  __debugS(DEV2, PSTR("\tStepper '%-8s' total steps: %7ld"), steppers[maxTotalIndex].getDescriptor(), maxTotal);

  // find all smaller movements and set the factor proportionally, i.e.
  // slow down movement so all steppers start and stop in sync
  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    if (!(_BV(i) & remainingSteppersFlag))
      continue;
    if(i == maxTotalIndex)              // ignore the stepper that's setting the pace
      continue;
    long tSteps = steppers[i].getTotalSteps()-1;
    // factor is now calculated differently to overcome issues with decimals
    timerVal_t factor = (timerVal_t)round(((double)tSteps/(double)maxTotal)*10000);
    steppers[i].setInterruptFactor(factor);
    __debugS(DEV2, PSTR("\tStepper '%-8s' total steps: %7ld  interrupt factor: %ld"), steppers[i].getDescriptor(), tSteps, steppers[i].getInterruptFactor());
  }
}

/** 
 * Find the shortest duration (a.k.a. smallest overflow value)
 */
timerVal_t getDuration() {
  timerVal_t minDuration = 0xFFFF;
  
  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    if (_BV(i) & remainingSteppersFlag) {             // only if the stepper is in action
      steppers[i].isLTDuration(&minDuration);         // compare and assign minDuration, if it's less than the current minDuration
    }
  }
  return minDuration; 
}

void runNoWait(int8_t index) {
  if (index != -1) {
    remainingSteppersFlag |= _BV(index);
    startStepperIndex = (byte)index;
  }
  else {
    calcSyncMovementFactor();                           // calculate movement factor, if more than one stepper has to move
    startStepperIndex = (byte)0;
  }
  startStepperInterval(getDuration());
}

void runAndWait(int8_t index) {
  bool dualFeeder = false;
  #if defined(USE_DDE)
    if(!asyncDDE && (remainingSteppersFlag & _BV(FEEDER)) && (remainingSteppersFlag & _BV(DDE_FEEDER))) {
      dualFeeder = true;
      if(index != -1)
        __debugS(DEV2, PSTR("[runAndWait] Stepper: %-8s"), steppers[index].getDescriptor());
      __debugS(DEV2, PSTR("[runAndWait] remainingSteppersFlag: %4lx  AsyncDDE: %s  DualFeeder: %s"), remainingSteppersFlag, asyncDDE ? P_Yes : P_No, dualFeeder ? P_Yes : P_No);
    }
  #endif
  runNoWait(index);
  while (remainingSteppersFlag)
  {
    if(smuffConfig.prusaMMU2)
      checkSerialPending(); // not a really nice solution but needed to check serials for "Abort" command in PMMU mode
    #if defined(USE_DDE)
      // stop internal feeder when the DDE feeder has stopped
      if(dualFeeder && ((remainingSteppersFlag & _BV(FEEDER)) && !(remainingSteppersFlag & _BV(DDE_FEEDER)))) {
        steppers[FEEDER].setMovementDone(true);
        remainingSteppersFlag = 0;
        __debugS(DEV2, PSTR("DDE Feeder has stopped, stopping Feeder too"));
        break;
      }
    #endif
    if(index != -1 && !(remainingSteppersFlag & _BV(index))) {
      __debugS(DEV2, PSTR("\tStepper '%-8s' has finished @ position: %s mm"), steppers[index].getDescriptor(), String(steppers[index].getStepPositionMM()).c_str());
      break;
    }
  }
}

