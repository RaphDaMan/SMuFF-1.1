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

#define FASTFLIPDBG       1                               // for firmware debugging only

#if defined(__BRD_SKR_MINI)
Stream                    *debugSerial = &Serial1;        // send debug output to...
Stream                    *terminalSerial = &Serial1;     // send terminal emulation output to ...
#else
Stream                    *debugSerial = &Serial2;        // send debug output to...
Stream                    *terminalSerial = &Serial2;     // send terminal emulation output to ...
#endif
Stream                    *logSerial = &Serial;           // send logs to USB by default

#if defined(USE_SW_SERIAL0)
SoftwareSerial            swSerial0(SW_SERIAL_RX_PIN, SW_SERIAL_TX_PIN, false);  // mainly for testing purpose
#endif

dspDriver                 display = INIT_DSP();           // initialize display driver (see macro in according Display_xxx.h)
ZStepper                  steppers[NUM_STEPPERS];
ZTimer                    stepperTimer;
ZTimer                    gpTimer;
#if defined(USE_FASTLED_TOOLS)
ZTimer                    ledTimer;
#endif
#if defined(USE_ZSERVO) && !defined(USE_MULTISERVO)
ZTimer                    servoTimer;
ZServo                    servoWiper;
ZServo                    servoLid;
ZServo                    servoCutter;
#else
  #if !defined(USE_MULTISERVO)
    Servo                 servoWiper;
    Servo                 servoLid;
    Servo                 servoCutter;
  #endif
#endif
ZFan                      fan;
ZFan                      fanDryer1;
ZFan                      fanDryer2;

#if defined(__STM32G0XX)
HardwareSerial            Serial4(RX4_PIN, TX4_PIN);
#endif


#if defined(USE_LEONERD_DISPLAY)
LeoNerdEncoder            encoder(I2C_ENCODER_ADDRESS);
#else
  #if !defined(USE_SERIAL_DISPLAY)
  ClickEncoder            encoder(ENCODER1_PIN, ENCODER2_PIN, ENCODER_BUTTON_PIN, 4);
  #else 
  ClickEncoder            encoder(0, 0, 0, 0);
  #endif
#endif

#if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
  #if defined(USE_SPLITTER_ENDSTOPS)
  ZEStopMux               splitterMux;
  #endif
#endif

#ifdef HAS_TMC_SUPPORT
TMC2209Stepper            *drivers[NUM_STEPPERS+1];
#endif

#if defined(USE_MULTISERVO)
  #if defined(USE_PCA9685_SW_I2C)
    Adafruit_PWMServoDriver   servoPwm = Adafruit_PWMServoDriver(I2C_SERVOCTL_ADDRESS, I2CBusMS);
  #else
    Adafruit_PWMServoDriver   servoPwm = Adafruit_PWMServoDriver(I2C_SERVOCTL_ADDRESS);
  #endif
int8_t                    servoMapping[16]   = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}; // servo output mappings for the Wiper, Lid, Cutter, Spare1 and Spare2 servos
int8_t                    outputMode[16]     = { 0,  0,  0,  0,  0,  1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}; // -1 = not used, 0 = PWM, 1 = OUTPUT
uint8_t                   servoPosClosed[16] = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};
#else
uint8_t                   servoPosClosed[MAX_TOOLS];
#endif
double                    stepperPosClosed[MAX_TOOLS];  // V6S only

#if defined(USE_SPOOLMOTOR)
  #if defined(USE_PCA9685_SW_I2C)
    Adafruit_PWMServoDriver   motor1Pwm = Adafruit_PWMServoDriver(I2C_MOTORCTL1_ADDRESS, I2CBusMS);
    Adafruit_PWMServoDriver   motor2Pwm = Adafruit_PWMServoDriver(I2C_MOTORCTL2_ADDRESS, I2CBusMS);
    Adafruit_PWMServoDriver   motor3Pwm = Adafruit_PWMServoDriver(I2C_MOTORCTL3_ADDRESS, I2CBusMS);
    Adafruit_PWMServoDriver   motor4Pwm = Adafruit_PWMServoDriver(I2C_MOTORCTL4_ADDRESS, I2CBusMS);
  #else
    Adafruit_PWMServoDriver   motor1Pwm = Adafruit_PWMServoDriver(I2C_MOTORCTL1_ADDRESS);
    Adafruit_PWMServoDriver   motor2Pwm = Adafruit_PWMServoDriver(I2C_MOTORCTL2_ADDRESS);
    Adafruit_PWMServoDriver   motor3Pwm = Adafruit_PWMServoDriver(I2C_MOTORCTL3_ADDRESS);
    Adafruit_PWMServoDriver   motor4Pwm = Adafruit_PWMServoDriver(I2C_MOTORCTL4_ADDRESS);
  #endif
int8_t                    spoolMappings[MAX_TOOLS]      = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; // tool to spool-motor mapping
uint32_t                  spoolDuration[MAX_TOOLS]      = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0 };
bool                      spoolDirectionCCW[MAX_TOOLS]  = {true, true, true, true, true, true, true, true, true,true, true, true};
int8_t                    motorPins[][3] = {{ MA_PWM, MA_IN1, MA_IN2}, { MB_PWM, MB_IN1, MB_IN2 }, { MC_PWM, MC_IN1, MC_IN2 }, { MD_PWM, MD_IN1, MD_IN2 }, { ME_PWM, ME_IN1, ME_IN2 }};
#endif
uint8_t                   spoolMotorsFound = 0;

uint8_t                   aht10SensorsFound = 0;
#if defined(USE_DRYER) && defined(USE_PCA9685_SW_I2C)
  AHTxx                   aht1 = AHTxx(I2C_AHT10_1_ADDRESS, AHT1x_SENSOR, I2CBusMS);
  AHTxx                   aht2 = AHTxx(I2C_AHT10_2_ADDRESS, AHT1x_SENSOR, I2CBusMS);
#elif defined(USE_DRYER)
  AHTxx                   aht1 = AHTxx(I2C_AHT10_1_ADDRESS, AHT1x_SENSOR);
  AHTxx                   aht2 = AHTxx(I2C_AHT10_2_ADDRESS, AHT1x_SENSOR);
#endif
float                     humidity1, temp1;
float                     humidity2, temp2;
float                     heater1;
float                     heater1prev, heater1delta;
int                       dryerFan1Speed, dryerFan2Speed;
bool                      dryerFan1Dynamic = false;
bool                      hasDryerHeaterSensor = false;
bool                      hasDryerHeater = false;
bool                      heaterOverflow = false;
bool                      isHeating = false;
bool                      isHeaterOn = false;
uint32_t                  heaterTimeout = 0;
uint8_t                   heaterTargetTemp = 0;
uint8_t                   heaterDeltaErrors = 0;
uint32_t                  heaterEstimatedTime = 0;
uint32_t                  heaterShutdown;
float                     heaterEstimatedDelta = 0.0;
_File*                    sensorLog;
bool                      intervalSpin[MAX_TOOLS] = { false, false, false, false, false, false, false, false, false, false, false, false };

bool                      hasMultiservo = false;

int8_t                    toolSelections[MAX_TOOLS];
uint8_t                   remoteKey = REMOTE_NONE;
static volatile uint16_t  intervalMask;       // bit-mask for interval reached
uint16_t                  mmsMin = 1;         // minimum moving speed for stepper in mm/s
uint16_t                  mmsMax = 800;       // maximum moving speed for stepper in mm/s
uint16_t                  speedIncrement = 5; // increment for speeds in menus
uint32_t                  pwrSaveTime;
volatile uint16_t         gpTimer1ms = 0;
volatile uint16_t         gpTimer20ms = 0;
volatile uint16_t         gpTimer50ms = 0;
volatile uint16_t         gpTimer100ms = 0;
static volatile unsigned  tickCounter = 0;
volatile unsigned long    lastEncoderButtonTime = 0;
volatile bool             sdRemoved = false;
volatile bool             initDone = false;   // enables sending periodical status information to serial ports
volatile bool             isPwrSave = false;
volatile bool             showMenu = false;
volatile bool             lastZEndstopState = false;
volatile bool             isIdle = false;
bool                      tmcWarning = false;
bool                      isUsingTmc = false;
bool                      isReceiving = false;
volatile bool             refreshingDisplay = false;
volatile uint16_t         flipDbgCnt;
volatile uint32_t         __systick = 0;        // for benchmarking only
uint32_t                  waitForDlgId = 0;
bool                      gotDlgId = false;
uint8_t                   dlgButton;
static int                lastTriggerState[4] = { -1, -1, -1, -1 };
static int                triggerStateCount[4] = { 1, 1, 1, 1 };
String                    axisNames[] = { "X", "Y", "Z", "Z2" };

volatile IntervalHandler intervalHandlers[] = {
    {  10, 0, every10ms}, 
    {  20, 0, every20ms}, 
    {  50, 0, every50ms}, 
    { 100, 0, every100ms}, 
    { 250, 0, every250ms}, 
    { 500, 0, every500ms}, 
    {1000, 0, every1s}, 
    {2000, 0, every2s}, 
    {5000, 0, every5s}
};

// forward declarations of some locally used functions
void fncKey1();
void fncKey2();
void fncKey3();
void fncKey4();
void loopEx();

//=====================================================================================================
// Benchmarking helper functions
//=====================================================================================================
#if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)

volatile uint32_t*  debugPin = 0;
uint32_t            pinMask_Dset;
uint32_t            pinMask_Drst;
volatile bool       debugToggle = false;

void fastFlipDbg() {
  #if FASTFLIPDBG == 1
    *debugPin = (debugToggle) ? pinMask_Dset : pinMask_Drst;
  #else
    digitalWrite(DEBUG_PIN, debugToggle);
  #endif
  debugToggle = !debugToggle;
}

#endif

void endstopEventX() {
  // add your code here it you want to hook into the endstop event for the Selector
  // must be enabled by defining USE_CUSTOM_ENDSTOP_EVENT_X
}

void endstopEventY() {
  // add your code here it you want to hook into the endstop event for the Revolver
  // must be enabled by defining USE_CUSTOM_ENDSTOP_EVENT_Y
}

void endstopEventZ() {
  // add your code here it you want to hook into the 1st endstop event for the Feeder
  // must be enabled by defining USE_CUSTOM_ENDSTOP_EVENT_Z
}

void endstopEventZ2() {
  // add your code here it you want to hook into the 2nd endstop event for the Feeder
  // must be enabled by defining USE_CUSTOM_ENDSTOP_EVENT_Z2
}


//=====================================================================================================
// ISR handler functions
//=====================================================================================================

void HAL_SYSTICK_Callback(void) {
  // if(initDone) fastFlipDbg();

  if(initDone) {
    #if defined(USE_DRYER)
    #endif
  }
}

/*
  Interrupt handler for Endstops
 */

void showTriggerMessage(int axis, bool state) {

  if(state == lastTriggerState[axis]) {
    triggerStateCount[axis]++;
    return;
  }
  else {
    int index = (axis == FEEDER2) ? FEEDER : axis;
    __debugSInt(DEV3, P_EstopTriggerMsg, axisNames[axis].c_str(), String(steppers[index].getStepPositionMM()).c_str(), state, triggerStateCount[axis]);
    lastTriggerState[axis] = state;
    triggerStateCount[axis] = 1;
  }
}

void isrEndstopX() {
  bool hit = steppers[SELECTOR].evalEndstopHit(1);
  #if defined(USE_CUSTOM_ENDSTOP_EVENT_X)
    if(hit)
      endstopEventX();
  #endif
  if(smuffConfig.dbgLevel & DEV3 == DEV3)
    showTriggerMessage(SELECTOR, hit);
}

void isrEndstopY() {
  bool hit = steppers[REVOLVER].evalEndstopHit(1);
  #if defined(USE_CUSTOM_ENDSTOP_EVENT_Y)
    if(hit)
      endstopEventY();
  #endif
  if(smuffConfig.dbgLevel & DEV3 == DEV3)
    showTriggerMessage(REVOLVER, hit);
}

void isrEndstopZ() {
  bool hit = steppers[FEEDER].evalEndstopHit(1);
  #if defined(USE_CUSTOM_ENDSTOP_EVENT_Z)
    if(hit)
      endstopEventZ();
  #endif
  if(smuffConfig.dbgLevel & DEV3 == DEV3)
    showTriggerMessage(FEEDER, hit);
}

void isrEndstopZ2() {
  bool hit = false;
  #if !defined(USE_DDE)
    if(steppers[FEEDER].getEndstopPin(2) > 0) {
      hit = steppers[FEEDER].evalEndstopHit(2);
    }
  #else
    hit = steppers[DDE_FEEDER].evalEndstopHit(1);
  #endif
  #if defined(USE_CUSTOM_ENDSTOP_EVENT_Z2)
    if(hit)
      endstopEventZ2();
  #endif
  if(smuffConfig.dbgLevel & DEV3 == DEV3) {
    showTriggerMessage(FEEDER2, hit);
  }
}

/*
  Interrupt handler for StallGuard on TMC2209
 */
void isrStallDetectedX() { steppers[SELECTOR].stallDetected(); }
void isrStallDetectedY() { steppers[REVOLVER].stallDetected(); }
void isrStallDetectedZ() { steppers[FEEDER].stallDetected(); }


/*
  Interrupt handler for the LED timer
*/
void isrLedTimerHandler() {

  #if defined(USE_FASTLED_TOOLS)
    static volatile uint8_t ledTickCounter = 0;
    #if !defined(USE_MULTISERVO)
      if(!initDone || isUpload || !servoLid.isPulseComplete() || !servoCutter.isPulseComplete() || !servoWiper.isPulseComplete())
        return;
    #else
      if(!initDone || isUpload)
        return;
    #endif

    ledTickCounter++;                     // increments every 10ms
    if(ledTickCounter == 10) {            // every 100ms
      ledTickCounter = 0;
      fastLedHue++;                       // used in some color changing/fading effects
      setFastLEDStatus(fastLedStatus);
    }
    else {
      uint32_t updateTime = (uint32_t)(((fastLedStatus > FASTLED_STAT_NONE) ? smuffConfig.ledRefresh[0] : smuffConfig.ledRefresh[1])*1000);
      // update NeoPixels when the interval has been reached and no steppers are in motion
      if(remainingSteppersFlag == 0 && (micros()-lastFastLedUpdate >= updateTime)) {
        updateToolLeds();
      }
    }

  #endif
}

/*
  Interrupt handler for the servo timer.
  Available only if MultiServo option isn't used.
*/
#if defined(USE_ZSERVO) && !defined(USE_MULTISERVO)
void isrServoTimerHandler() {
  isrServoHandler();        // call servo handler
}
#endif

/*
  Interrupt handler for the general purpose timer
  Fires every GPTIMER_RESOLUTION uS and thus increments the general counter
  every millisecond.
  Serves also as a handler/dispatcher for periodicals / encoder / fan / NeoPixels.
*/
void isrGPTimerHandler() {

  tickCounter++; // increment tick counter
  if ((tickCounter % gpTimer1ms) == 0) {
    // each millisecond...
    __systick++;    // 1000 Hz frequency
    if (initDone) {
      // decrement all milliseconds counters for periodic functions and
      // set a flag when the timeout has been reached
      for (uint8_t i = 0; i < ArraySize(intervalHandlers); i++) {
        intervalHandlers[i].val--;
        if (intervalHandlers[i].val <= 0) {
          intervalMask |= _BV(i);
          intervalHandlers[i].val = intervalHandlers[i].period;
        }
      }

      #if !defined(USE_LEONERD_DISPLAY) && !defined(USE_SERIAL_DISPLAY)
        encoder.service();      // service the rotary encoder
      #endif
      isrFanTimerHandler();     // call the fan interrupt routines also every GPTIMER_RESOLUTION uS
    }
  }
  
  /*
  #ifdef FLIPDBG
  if (initDone && ((tickCounter % flipDbgCnt) == 0)) {
    // flips the debug pin polarity which is supposed to generate 
    // a square wave signal (default: 500 Hz) for debug purposes
    // frequency can be modified in smuffConfig.dbgFreq
    #if FASTFLIPDBG == 0
      #if defined(USE_MULTISERVO)
        // debugToggle = !debugToggle;
        // servoPwm.setPin(servoMapping[SERVO_SPARE2], debugToggle);
      #else
        FLIPDBG
      #endif
    #endif
  }
  #endif
  */
}

void isrSdCardDetected() {
  sdRemoved = digitalRead(SD_DETECT_PIN) == HIGH;
}


//=====================================================================================================
// SETUP
//=====================================================================================================

void setup() {

  const char* after = PSTR("=== %-30s o.k. ===");
  
  __initDebug();
  serialBuffer0.reserve(30);      // initialize serial buffers
  serialBuffer1.reserve(30);
  serialBuffer2.reserve(30);
  serialBuffer3.reserve(30);
  readDebugLevel();               // read Debug Level settings from SD-Card

  // Setup a fixed baudrate until the config file was read.
  // This baudrate is the default setting on the ESP32 while
  // booting up, so exceptions thrown can be shown in terminal app
  Serial.begin(115200);
  if (CAN_USE_SERIAL1) Serial1.begin(115200);
  if (CAN_USE_SERIAL2) Serial2.begin(115200);
  if (CAN_USE_SERIAL3) Serial3.begin(115200);
  if (debugSerial == &Serial)
    delay(1500);   // allow USB serial to settle before the first message is printed out

  __debugS(DEV3, PSTR("setup start"));
  removeFirmwareBin();            // deletes the firmware.bin file; prevents flashing the firmware on each boot
  __debugS(SP, after, "remove firmware.bin");
  
  // ------------------------------------------------------------------------------------------------------
  // Disable JTAG/SWI on these boards:
  //  On the SKR E3-DIP it's because of the buzzer signal (PA15 / JTDI).
  //  On the SKR E3 V2.0 it's because of the SCL signal (PA15 / JTDI).
  // ------------------------------------------------------------------------------------------------------
  // IMPORTANT: If debug ports are disabled, flashing the firmware / bootloader using ST-Link will fail!
  //            Hence, if you need to flash the MCU using ST-Link at some point, better comment out
  //            -D DISABLE_DEBUG_PORT in platformio.ini for that particular board.
  // ------------------------------------------------------------------------------------------------------
  #if defined(DISABLE_DEBUG_PORT)
    pin_DisconnectDebug(PA_15);   // disable Serial wire JTAG configuration (STM32F1 only)
    __debugS(D, PSTR("\tSWDIO debug ports disabled"));
  #endif
  #if defined(STM32_REMAP_SPI)
    afio_remap(AFIO_REMAP_SPI1); // remap SPI3 to SPI1 if a "normal" display is being used
    __debugS(D, PSTR("\tSPI3 remapped to SPI1"));
  #endif

  __debugS(D, PSTR("\tInitializing USB..."));
  initUSB();                // init the USB serial so it's being recognized by the Windows-PC
  __debugS(D, PSTR("\tInitializing I2C..."));
  setupI2C();               // setup I2C/TWI devices
  __debugS(D, PSTR("\tInitializing display..."));
  setupDisplay();           // setup display first in order to show error messages if neccessary
  __debugS(SP, after, PSTR("setup Display"));

  #if defined(USE_I2C)
    __debugS(D, PSTR("\tScanning for I2C Devices on bus 1..."));
    // don't scan I2C if no I2C devices are being used; Scan will take ages
    drawSDStatus(STAT_SCANNING_I2C, 1);
    enumI2cDevices(1);
    __debugS(D, after, "enumerating I2C Devices on bus 1");
  #endif
  #if defined(USE_SPLITTER_ENDSTOPS)
    __debugS(D, PSTR("\tScanning for I2C Devices on bus 2..."));
    drawSDStatus(STAT_SCANNING_I2C, 2);
    enumI2cDevices(2);
    __debugS(D, after, "enumerating I2C Devices on bus 2");
  #endif
  #if defined(USE_MULTISERVO)
    __debugS(D, PSTR("\tScanning for I2C Devices on bus 3..."));
    drawSDStatus(STAT_SCANNING_I2C, 3);
    enumI2cDevices(3);
    __debugS(D, after, "enumerating I2C Devices on bus 3");
      if(hasMultiservo) {
        // RESET the D1 Mini (if available)
        servoPwm.setPin(D1_MINI_RESET, 0);
        delay(50);
        servoPwm.setPin(D1_MINI_RESET, 4095);
      }
  #endif

  #if !defined(USE_SERIAL_DISPLAY)
    delay(250);
    setupEncoder();                                   // setup encoder
    __debugS(SP, after, PSTR("setup Encoder"));
  #endif

  __debugS(DEV, PSTR("\treading Configs..."));
  if (readConfig()) {                                 // read SMUFF.json from SD-Card
    readTmcConfig();                                  // read TMCDRVR.json from SD-Card
    readServoMapping();                               // read SERVOMAP.json from SD-Card
    readMaterials();                                  // read MATERIALS.json from SD-Card
    readDryerConfig();                                // read DRYER.json from SD-Card
    #if defined(SMUFF_V6S)
      readRevolverMapping();                          // read REVOLVERMAPS.json from SD-Card
    #endif
  }
  __debugS(SP, after, PSTR("read Config"));
  initFastLED();                                      // init FastLED if configured
  setContrast(smuffConfig.lcdContrast);               // reset display contrast after reading config
  initHwDebug();                                      // init hardware debugging

  setupSerial();                                      // setup all serial ports
  __debugS(SP, after, PSTR("setup Serial"));

  if(smuffConfig.useDuet) {
    // Duet3D uses Serial 1 for communication, Serial 3 for PanelDue (optional)
    // in this case, all debug outputs are sent to USB serial
    debugSerial = &Serial;                            // send debug output to USB
    terminalSerial = &Serial;                         // send terminal emulation output to USB
    __debugS(SP, PSTR("\tdebug & terminal serials swapped for Duet3D/RRF"));
  }
  setupSteppers();                                    // setup all steppers
  __debugS(SP, after, PSTR("setup Steppers"));
  setupServos();                                      // setup all servos
  __debugS(SP, after, PSTR("setup Servos"));
  setupSpoolMotors();                                 // setup all spool-motors
  __debugS(SP, after, PSTR("setup SpoolMotors"));
  setupTimers();                                      // setup all timers
  __debugS(SP, after, PSTR("setup Timers"));
  setupRelay();                                       // setup relay board
  __debugS(SP, after, PSTR("setup Relay"));
  #ifdef HAS_TMC_SUPPORT
    setupTMCDrivers();                                // setup TMC drivers if any are used
    __debugS(SP, after, PSTR("setup TMC drivers"));
  #endif
  setupAHT10Sensors();                                // setup Temp./Humidity sensors
  __debugS(SP, after, PSTR("setup AHT sensors"));
  setupHeater();                                      // setup heater for Dryer
__debugS(SP, after, PSTR("setup Heater"));

  testFastLED(false);                                 // run a test sequence on backlight FastLEDs
  testFastLED(true);                                  // run a test sequence on tools FastLEDs

  setupSwSerial0();                                   // used only for testing purposes
  setupBacklight();                                   // setup display backlight
  setupDuetSignals();                                 // setup Duet3D signal pins
  setupFan();                                         // setup internal cooling fan
  setupDryerFans();                                   // setup fans for filament dryer
  __debugS(SP, after, PSTR("setup Misc."));
  getStoredData();                                    // read EEPROM.json from SD-Card; this call must happen after setupSteppers()
  
  #if defined(USE_SPLITTER_ENDSTOPS)
    setupEStopMux();                                  // setup enstops multiplexer if configured
    __debugS(SP, after, PSTR("setup EStop Mux"));
  #endif
  uint32_t now = millis();
  readSequences();                                    // read sound files from SD-Card sounds folder
  __debugS(D, PSTR("\tloading sequences took %ld ms"), millis()-now);
  __debugS(SP, after, PSTR("read Sequences"));

  if (smuffConfig.homeAfterFeed)
    moveHome(REVOLVER, false, false);                 // home Revolver / Lid-Servo
  else
    resetRevolver();

  #if defined(SD_DETECT_PIN)                          // setup SD-Detect ISR
    if(SD_DETECT_PIN > 0) {
      attachInterrupt(digitalPinToInterrupt(SD_DETECT_PIN), isrSdCardDetected, CHANGE);
      __debugS(D, PSTR("\tSD-Card detect interrupt attached"));
    }
  #endif

  #if defined(__BRD_SKR_MINI_E3)                      // applies only to E3 V1.2 and 2.0 / 3.0 with integrated stepper drivers
    if(drivers[SELECTOR] == nullptr || drivers[FEEDER] == nullptr) {
      __debugS(W, PSTR("\nWARNING: Your controller is equipped with onboard TMC stepper drivers"));
      __debugS(W, PSTR("but at least one of them has not being initialized for UART mode"));
      __debugS(W, PSTR("and thus, your stepper motor may overheat!"));
      __debugS(W, PSTR("Please check and change your configuration accordingly!"));
    }
  #endif
  
  __debugS(D, PSTR("startup tune"));
  startupBeep();                                        // signal startup has finished
  __debugS(DEV3, PSTR("setup end"));

  for (uint8_t i = 0; i < (uint8_t)ArraySize(intervalHandlers); i++) // init interval handler values
    intervalHandlers[i].val = intervalHandlers[i].period;
  gpTimer1ms = (uint16_t)(1000/GPTIMER_RESOLUTION);
  gpTimer20ms = gpTimer1ms*20;
  gpTimer50ms = gpTimer1ms*50;
  gpTimer100ms = gpTimer1ms*100;
  
  sendStartResponse(0);                                 // send "start<CR><LF>" to USB serial interface
  if (CAN_USE_SERIAL1)
    sendStartResponse(1);                               // send "start<CR><LF>" to all serial interfaces allowed to use
  if (CAN_USE_SERIAL2 && smuffConfig.hasPanelDue != 2)
    sendStartResponse(2);
  if (CAN_USE_SERIAL3 && smuffConfig.hasPanelDue != 3)
    sendStartResponse(3);
  
  setFastLEDToolIndex(toolSelected, smuffConfig.toolColor, true);
  pwrSaveTime = millis();                               // init value for LCD screen timeout
  initDone = true;                                      // mark init done; enable periodically sending status, if configured
  refreshStatus();
}

#ifdef HAS_TMC_SUPPORT
void reportTMC(uint8_t axis, const char *PROGMEM msg) {
  // for now, only debug message (main screen will show "!" next to "TMC")
  __debugS(W, PSTR("Driver %c: reports '%s'"), axis==FEEDER2 ? 'E' : 'X' + axis, msg);
}

void monitorTMC(uint8_t axis) {
  uint16_t temp;
  if (drivers[axis] != nullptr) {
    // has any error occured?
    if (drivers[axis]->drv_err()) {
      tmcWarning = true;
      // check overtemp. warning
      if (drivers[axis]->otpw()) {
        reportTMC(axis, P_TMC_Status09);
      }
      // check overtemp.
      if (drivers[axis]->ot()) {
        if (drivers[axis]->t157())
          temp = 157;
        if (drivers[axis]->t150())
          temp = 150;
        if (drivers[axis]->t143())
          temp = 143;
        if (drivers[axis]->t120())
          temp = 120;
        char msg[80];
        sprintf_P(msg, PSTR("Overtemp. >= %d°C"), temp);
        reportTMC(axis, msg);
      }
      // check open lines
      if (drivers[axis]->ola())
        reportTMC(axis, P_TMC_Status03);
      if (drivers[axis]->olb())
        reportTMC(axis, P_TMC_Status04);
      // check short to grounds
      if (drivers[axis]->s2ga())
        reportTMC(axis, P_TMC_Status05);
      if (drivers[axis]->s2gb())
        reportTMC(axis, P_TMC_Status06);
    }
  }
}
#endif


//=====================================================================================================
// LOOP and other functions
//=====================================================================================================

static bool sdRemovalSet = false;
static bool firstLoop = false;
static bool wasIdle = false;
static bool heaterFailureSet = false;

void shutdownHeater() {
  #if defined(USE_DRYER)
    // turn off heater instantly
    setHeater(false);
    isHeating = false;
    closeLogFile(sensorLog);
    heaterShutdown = millis()+10*60000;
    __debugS(heaterOverflow ? W : I, PSTR("Heater has been %sshut down!"), heaterOverflow ? "forcefully " : "");
    if(heaterOverflow)
      longBeep(6);
  #endif
}

void startDryer(uint8_t temperature, uint32_t duration, uint8_t fan1speed, uint8_t fan2speed, bool fan1dyn) {
  #if defined(USE_DRYER)
    if(!isHeating) {
      heaterShutdown = 0;
      heaterTimeout = duration;
      heaterTargetTemp = temperature;
      heaterDeltaErrors = smuffConfig.heaterDeltaErr;
      dryerFan1Speed = fan1speed;
      dryerFan2Speed = fan2speed;
      dryerFan1Dynamic = fan1dyn;
      uint8_t t = heaterTargetTemp - heater1;
      isHeaterTooSlow = false;
      heaterEstimatedDelta = (smuffConfig.heaterDeltaMin + smuffConfig.heaterDeltaMax) / 2;
      heaterEstimatedTime = t / heaterEstimatedDelta + (3*60);
      if (dryerFan1Speed >= 0 && dryerFan1Speed <= 100)
      {
        #if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
          fanDryer1.setFanSpeed(dryerFan1Speed);
        #else
          #if FAN_PIN1 > 0
            analogWrite(FAN1_PIN, map(dryerFan1Speed, 0, 100, 0, 255));
          #endif
        #endif
      }
      if (dryerFan2Speed >= 0 && dryerFan2Speed <= 100)
      {
        #if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
          fanDryer2.setFanSpeed(dryerFan2Speed);
        #else
          #if FAN_PIN2 > 0
            analogWrite(FAN2_PIN, map(dryerFan2Speed, 0, 100, 0, 255));
          #endif
        #endif
      }
      sensorLog = openLogFileWrite(SENSOR_LOGFILE);
      isHeating = true;
      isHeaterSet = false;
      __debugS(I, PSTR("Dryer started. Estimated heat-up time: %d minutes"), heaterEstimatedTime/60);
    }
  #endif
}

void stopDryer() {
  #if defined(USE_DRYER)
  if(isHeating) {
    __debugS(I, PSTR("Dryer stopped."));
    shutdownHeater();
    if(!heaterOverflow)
      dryerBeep();
    heaterTargetTemp = 0;
    dryerFan1Speed = 0;
    dryerFan2Speed = 0;
    #if defined(__STM32F1XX) || defined(__STM32F4XX) || defined(__STM32G0XX)
      fanDryer1.setFanSpeed(dryerFan1Speed);
      fanDryer2.setFanSpeed(dryerFan2Speed);
    #else
      #if FAN_PIN1 > 0
        analogWrite(FAN1_PIN, map(dryerFan1Speed, 0, 100, 0, 255));
      #endif
      #if FAN_PIN2 > 0
        analogWrite(FAN2_PIN, map(dryerFan2Speed, 0, 100, 0, 255));
      #endif
    #endif
  }
  #endif
}

// Call periodical functions as the timeout has triggered.
// Add your specific code there, if you need to have something
// managed periodically.
// The main loop is the better choice for dispatching, since it'll
// allow uninterrupted serial I/O.
void handlePeriodicals() {
  for (uint8_t i = 0; i < (uint8_t)ArraySize(intervalHandlers); i++) {
    if (_BV(i) & intervalMask) {
      if(intervalHandlers[i].func != nullptr)
        intervalHandlers[i].func();
      intervalMask &= ~_BV(i);
    }
  }
}

void loop() {

  // if(!firstLoop) {     // for extended debugging only
  //   __debugS(DEV3, PSTR("Looping..."));
  //   firstLoop = true;
  // }

  handlePeriodicals();

#if defined(USE_SPOOLMOTOR)
  for(uint8_t i=0; i< MAX_TOOLS; i++) {
    if(spoolDuration[i] > 0) {
      if(millis() >= spoolDuration[i]) {
        stopRewindingSpool(i);
        spoolDuration[i] = 0;
      }
    }
  }
#endif

#if defined(SD_DETECT_PIN)
  // message the user if the SD-Card gets removed
  if (sdRemoved) {
    if (!sdRemovalSet) {
      userBeep();
      leoNerdBlinkRed = true;
      setFastLEDStatus(FASTLED_STAT_ERROR);
      if (isPwrSave) {
        setPwrSave(0);
      }
    }
    sdRemovalSet = true;
    if((__systick % 500) == 0) {
      char tmp[50];
      sprintf_P(tmp, P_SDCardRemoved);
      drawUserMessage(tmp);
    }
    return;
  }
  else
  {
    if (sdRemovalSet) {
      if (initSD(false)) {
        sdRemovalSet = false;
        leoNerdBlinkRed = false;
        setFastLEDStatus(FASTLED_STAT_NONE);
        lastEvent = millis();
        refreshStatus();
      }
    }
  }
#endif

#if defined(USE_DRYER)
  if(heaterOverflow && !heaterFailureSet) {
    shutdownHeater();
    leoNerdBlinkRed = true;
    setFastLEDStatus(FASTLED_STAT_ERROR);
    if (isPwrSave) {
      setPwrSave(0);
    }
    char tmp[50];
    sprintf_P(tmp, P_HeaterFailure);
    drawUserMessage(tmp);
    heaterFailureSet = true;
    return;
  }
  else {
    leoNerdBlinkRed = false;
    setFastLEDStatus(FASTLED_STAT_NONE);
    lastEvent = millis();
    heaterFailureSet = false;
    refreshStatus();
  }
#endif

  bool state = feederEndstop();
  if (state != lastZEndstopState) {
    refreshStatus();
    lastZEndstopState = state;
  }
  #if !defined(USE_SERIAL_DISPLAY)
    if(checkUserMessage()) {
      pwrSaveTime = millis();
    }
  #endif

  wasIdle = isIdle;
  isIdle = ((uint16_t)((millis() - lastEvent) / 1000) >= smuffConfig.powerSaveTimeout);
  if(isIdle != wasIdle) {
    __debugS(DEV4, PSTR("Idle switched from %d to %d"), wasIdle, isIdle);
  }

  #if !defined(USE_SERIAL_DISPLAY)
  int16_t turn;
  uint8_t button;
  bool isHeld, isClicked;

  if (!showMenu) {
    switch (remoteKey)
    {
      case REMOTE_HOME:
        remoteKey = REMOTE_NONE;
        break;
      case REMOTE_PF1:
        fncKey1();
        remoteKey = REMOTE_NONE;
        break;
      case REMOTE_PF2:
        fncKey2();
        remoteKey = REMOTE_NONE;
        break;
      case REMOTE_PF3:
        fncKey3();
        remoteKey = REMOTE_NONE;
        break;
      case REMOTE_PF4:
        fncKey4();
        remoteKey = REMOTE_NONE;
        break;
    }

    getInput(&turn, &button, &isHeld, &isClicked);
    if (isClicked || turn != 0) {
      if(isClicked && isUpload)
        resetUpload();
      if (isPwrSave) {
        setPwrSave(0);
        return;
      }
      lastEvent = millis();
    }
    #endif
    
    #if defined(USE_FASTLED_TOOLS)
    if(!isIdle) {
      if(fastLedStatus != FASTLED_STAT_NONE) {
        setFastLEDStatus(FASTLED_STAT_NONE);
        __debugS(DEV3, PSTR("FastLED status: None"));
      }
    }
    else {
      // switch on idle animation only once
      if(smuffConfig.useIdleAnimation && fastLedStatus != smuffConfig.animationType) {
        setFastLEDStatus(smuffConfig.animationType);
        __debugS(DEV3, PSTR("FastLED status: Animation"));
      }
    }
    #endif

    #if !defined(USE_SERIAL_DISPLAY)
    if ((button == MainButton && isClicked) || (button == WheelButton && isHeld)) {
      showMenu = true;
      char title[] = {"Settings"};
      showSettingsMenu(title);
      showMenu = false;
      debounceButton();
    }
    else if (button == LeftButton) {
      // applies to LeoNerd's display only
      if (isClicked) {
        fncKey1();
      }
      else if (isHeld) {
        fncKey2();
      }
    }
    else if (button == RightButton) {
      if (isClicked) {
        fncKey3();
      }
      else if (isHeld) {
        fncKey4();
      }
    }
    else if (turn != 0) {
      resetAutoClose();
      displayingUserMessage = false;
      showMenu = true;
      if (turn == -1) {
        showMainMenu();
      }
      else {
        showToolsMenu();
      }
      pwrSaveTime = millis();
      showMenu = false;
    }
  }

  if ((millis() - pwrSaveTime) / 1000 >= smuffConfig.powerSaveTimeout && !isPwrSave) {
    setPwrSave(1);
  }
  #endif

#ifdef HAS_TMC_SUPPORT
  monitorTMC(SELECTOR);
  monitorTMC(REVOLVER);
  monitorTMC(FEEDER);
#endif

  if(isTestPending) {
    isTestPending = false;
    showMenu = true;
    testRun(testToRun);
    showMenu = false;
  }
  
  loopEx();
  __flushDebug();
}

/*
* For testing only
*/
void loopEx() {

#if defined(__BRD_SKR_MINI_E3) || defined(__BRD_SKR_MINI_E3DIP)
  #if defined(USE_SW_SERIAL0)         // for testing SoftwareSerial only
    if((__systick % 2000) == 0) {     // every two seconds send string "*TG" to SW-Serial
      swSerial0.println("*TG");
    }
  #endif
  
#endif

}

void fncKey1() {
  if (strlen(smuffConfig.lButtonDown) > 0) {
    parseGcode(String(smuffConfig.lButtonDown), -1);
  }
}

void fncKey2() {
  if (strlen(smuffConfig.lButtonHold) > 0) {
    parseGcode(String(smuffConfig.lButtonHold), -1);
  }
}

void fncKey3() {
  if (strlen(smuffConfig.rButtonDown) > 0) {
    parseGcode(String(smuffConfig.rButtonDown), -1);
  }
}

void fncKey4() {
  if (strlen(smuffConfig.rButtonHold) > 0) {
    parseGcode(String(smuffConfig.rButtonHold), -1);
  }
  else {
    // open / close LID servo by default
    if (lidOpen)
      setServoLid(SERVO_CLOSED);
    else
      setServoLid(SERVO_OPEN);
  }
}

void setPwrSave(int8_t state) {
  setDisplayPowerSave(state);

  isPwrSave = state == 1;
  if (!isPwrSave) {
    pwrSaveTime = millis();
    refreshStatus();
    #if defined(USE_FASTLED_BACKLIGHT)
      setBacklightIndex(smuffConfig.backlightColor);    // turn back on backlight
    #endif
    lastEvent = millis();
  }
  else {
    #if defined(USE_FASTLED_BACKLIGHT)
      setBacklightIndex(0);                             // turn off backlight
    #endif
  }
}

bool checkUserMessage() {
  #if !defined(USE_SERIAL_DISPLAY)
    bool isClicked = getEncoderButton(true);
    if (displayingUserMessage && (isClicked || (millis() - userMessageTime > USER_MESSAGE_RESET * 1000))) {
      displayingUserMessage = false;
    }
    return displayingUserMessage;
  #else
    return false;
  #endif
}
