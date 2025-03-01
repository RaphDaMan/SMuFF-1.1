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

// used for the JSON parameters and M205 GCode command
const char* selector PROGMEM           = "Selector";
const char* revolver PROGMEM           = "Revolver";
const char* feeder PROGMEM             = "Feeder";
const char* feeder2 PROGMEM            = "Feeder2";
const char* toolCount PROGMEM          = "Tools";
const char* serialBaudrate PROGMEM     = "Baudrates";
const char* serial0Baudrate PROGMEM    = "Ser1Spd";
const char* serial1Baudrate PROGMEM    = "Ser1Spd";
const char* serial2Baudrate PROGMEM    = "Ser2Spd";
const char* serial3Baudrate PROGMEM    = "Ser3Spd";
const char* bowdenLength PROGMEM       = "BowdenLen";
const char* selectorDist PROGMEM       = "SelectorDist";
const char* selectorUnloadDist PROGMEM = "SelectorUnload";
const char* contrast PROGMEM           = "LCDCont";
const char* i2cAdr PROGMEM             = "I2CAddr";
const char* autoClose PROGMEM          = "MenuClose";
const char* fanSpeed PROGMEM           = "FanSpeed";
const char* psTimeout PROGMEM          = "PwrSavTime";
const char* sendAction PROGMEM         = "SendAction";
const char* emulatePrusa PROGMEM       = "EmulPrusa";
const char* unloadCommand PROGMEM      = "UnldCmd";
const char* hasPanelDue PROGMEM        = "HasPanelDue";
const char* servoMinPwm PROGMEM        = "ServoMinPwm";
const char* servoMaxPwm PROGMEM        = "ServoMaxPwm";
const char* periodicalStats  PROGMEM   = "SendStats";
const char* wipeSequence PROGMEM       = "WipeSeq";
const char* backlightColor PROGMEM     = "BackColor";
const char* encoderTicks PROGMEM       = "EncTicks";
const char* maxSpeed PROGMEM           = "MaxSpeed";
const char* accelSpeed PROGMEM         = "AccelSpeed";
const char* accelDist PROGMEM          = "AccelDist";
const char* invertDir PROGMEM          = "InvDir";
const char* endstopTrig PROGMEM        = "EStopTrg";
const char* endstopTest PROGMEM        = "EStopTest";
const char* stepDelay PROGMEM          = "StepDly";
const char* stepsPerMillimeter PROGMEM = "StepsMM";
const char* offset PROGMEM             = "Offset";
const char* power PROGMEM              = "Power";
const char* mode PROGMEM               = "Mode";
const char* tmode PROGMEM              = "TMode";
const char* rsense PROGMEM             = "RSense";
const char* msteps PROGMEM             = "MS";
const char* stall PROGMEM              = "Stall";
const char* cstepmin PROGMEM           = "CSMin";
const char* cstepmax PROGMEM           = "CSMax";
const char* cstepdown PROGMEM          = "CSDown";
const char* drvrAdr PROGMEM            = "DrvAddr";
const char* toff PROGMEM               = "TOff";
const char* stopOnStall PROGMEM        = "StopOnStall";
const char* maxStallCount PROGMEM      = "MaxStall";
const char* spacing PROGMEM            = "Spacing";
const char* externalControl PROGMEM    = "ExtCtrl";
const char* insertSpeed PROGMEM        = "InsSpeed";
const char* purgeSpeed PROGMEM         = "PurgeSpeed";
const char* purgeLength PROGMEM        = "PurgeLen";
const char* reinforceLength PROGMEM    = "ReinforceLen";
const char* unloadRetract PROGMEM      = "UnldRetract";
const char* unloadPushback PROGMEM     = "UnldPushback";
const char* pushbackDelay PROGMEM      = "PushbackDly";
const char* enableChunks PROGMEM       = "UseChunks";
const char* feedChunks PROGMEM         = "Chunks";
const char* insertLength PROGMEM       = "InsLen";
const char* useDuet PROGMEM            = "UseDuet";
const char* sharedStepper PROGMEM      = "SharedStepper";
const char* wiggle PROGMEM             = "Wiggle";
const char* useServo PROGMEM           = "UseServo";
const char* servoOffPos PROGMEM        = "ServoOpen";
const char* servoOnPos PROGMEM         = "ServoClose";
const char* servo1Cycles PROGMEM       = "Servo1Cyc";
const char* servo2Cycles PROGMEM       = "Servo2Cyc";
const char* resetBeforeFeed PROGMEM    = "ResetBeforeFd";
const char* homeAfterFeed PROGMEM      = "HomeAfterFd";
const char* stepsPerRevolution PROGMEM = "StepsRev";
const char* material PROGMEM           = "Material";
const char* materials PROGMEM          = "Materials";
const char* servoOutput PROGMEM        = "Output";
const char* servoClosed PROGMEM        = "Close";
const char* servoOpened PROGMEM        = "Open";
const char* wiper PROGMEM              = "Wiper";
const char* lid PROGMEM                = "Lid";
const char* spare1 PROGMEM             = "Spare1";
const char* spare2 PROGMEM             = "Spare2";
const char* tool PROGMEM               = "Tool";
const char* positions PROGMEM          = "Positions";
const char* swaps PROGMEM              = "SwapTools";
const char* lBtnDown PROGMEM           = "LBtnDown";
const char* lBtnHold PROGMEM           = "LBtnHold";
const char* rBtnDown PROGMEM           = "RBtnDown";
const char* rBtnHold PROGMEM           = "RBtnHold";
const char* speedsInMMS PROGMEM        = "SpdsInMms";
const char* xlateSpeed PROGMEM         = "XlateSpeed";
const char* ms3Config PROGMEM          = "MS3";
const char* motDelay PROGMEM           = "MDly";
const char* cutter PROGMEM             = "Cutter";
const char* useCutter PROGMEM          = "UseCutter";
const char* usePurge PROGMEM           = "UsePurge";
const char* cutterOpen PROGMEM         = "CutOpen";
const char* cutterClose PROGMEM        = "CutClose";
const char* purges PROGMEM             = "Purges";
const char* cutterLength PROGMEM       = "CutterLen";
const char* endstop2 PROGMEM           = "UseEStop2";
const char* color PROGMEM              = "Color";
const char* pfactor PROGMEM            = "PFactor";
const char* autoWipe PROGMEM           = "AutoWipe";
const char* toolColor PROGMEM          = "ToolColor";
const char* idleAnim PROGMEM           = "IdleAnim";
const char* colorVal PROGMEM           = "CValue";
const char* animBpm PROGMEM            = "AnimBPM";
const char* statusBpm PROGMEM          = "StatusBPM";
const char* relay PROGMEM              = "Relay";
const char* invertRelay PROGMEM        = "InvRelay";
const char* menuOnTerm PROGMEM         = "TermMenu";
const char* webInterface PROGMEM       = "WebInterface";
const char* revolverClosed PROGMEM     = "RevolverClose";
const char* useSplitter PROGMEM        = "UseSplitter";
const char* splitterDist PROGMEM       = "SplitterDist";
const char* feedState PROGMEM          = "FeedState";
const char* ddeDist PROGMEM            = "DDEDist";
const char* useDDE PROGMEM             = "UseDDE";
const char* purgeDDE PROGMEM           = "PurgeDDE";
const char* traceUsb PROGMEM           = "TraceUSB";
const char* devName PROGMEM            = "Device";
const char* cutterTop PROGMEM          = "CutterTop";
const char* dbgFreq PROGMEM            = "DebugFreq";
const char* invertDuet PROGMEM         = "InvDuet";
const char* syncSteppers PROGMEM       = "SyncSteppers";
const char* dbgLvl PROGMEM             = "DbgLevel";
const char* oscFreq PROGMEM            = "OscFreq";
const char* pwmFreq PROGMEM            = "PWMFreq";
const char* modePwm PROGMEM            = "PWM";
const char* modePin PROGMEM            = "OUTPUT";
const char* outX PROGMEM               = "User%d";
const char* user1 PROGMEM              = "User1";
const char* user2 PROGMEM              = "User2";
const char* dspSerialPort PROGMEM      = "DspSerial";
const char* duetSerialPort PROGMEM     = "DuetSerial";
const char* userMessage PROGMEM        = "UserMessage";
const char* userDialog PROGMEM         = "UserDialog";
const char* userDlgId PROGMEM          = "Id";
const char* userDlgButton PROGMEM      = "Button";
const char* userDlgStatus PROGMEM      = "Status";
const char* userDlgOption PROGMEM      = "Option";
const char* changeDebug PROGMEM        = "ChangeDbg";
const char* ledRefresh PROGMEM         = "LedRefresh";
const char* ledsPerTool PROGMEM        = "LedsPerTool";
const char* ledAnimation PROGMEM       = "LedAnimation";
const char* spi3Miso PROGMEM           = "Spi3Miso";
const char* spoolSpeed PROGMEM         = "SpoolSpeed";
const char* spool PROGMEM              = "Spool";
const char* autoRewind PROGMEM         = "AutoRewind";
const char* dirCCW PROGMEM             = "CCW";

const char* adcRes PROGMEM             = "ADC_Res";
const char* adcR1 PROGMEM              = "ADC_R1";
const char* thR25 PROGMEM              = "TH_R25";
const char* thBeta PROGMEM             = "TH_Beta";
const char* shcA PROGMEM               = "SHC_A";
const char* shcB PROGMEM               = "SHC_B";
const char* shcC PROGMEM               = "SHC_C";
const char* fan1speed PROGMEM          = "Fan1Speed";
const char* fan2speed PROGMEM          = "Fan2Speed";
const char* fan1dyn PROGMEM            = "Fan1Dyn";
const char* fan2dyn PROGMEM            = "Fan2Dyn";
const char* htMaxTemp PROGMEM          = "HeaterMaxTemp";
const char* htDeltaMin PROGMEM         = "HeaterDeltaMin";
const char* htDeltaMax PROGMEM         = "HeaterDeltaMax";
const char* htDeltaErr PROGMEM         = "HeaterDeltaErr";
const char* thUseBeta PROGMEM          = "UseBeta";
const char* humidityLevels PROGMEM     = "HumidityLevels";
const char* dynFanSpeeds PROGMEM       = "DynFanSpeeds";
const char* logSensor1 PROGMEM         = "LogSensor1";
const char* logSensor2 PROGMEM         = "LogSensor2";
const char* spinSpeed PROGMEM          = "SpinSpeed";
const char* spinDuration PROGMEM       = "SpinDuration";
const char* spinInterval PROGMEM       = "SpinInterval";
const char* spinSpool PROGMEM          = "SpinSpool";

