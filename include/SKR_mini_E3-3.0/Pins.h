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
 * Pins configuration file for SKR mini E3 V3.0 board
 */
#pragma once

#define BOARD_INFO          "SKR mini E3 V3.0"
// SELECTOR (X)
#define STEP_HIGH_X         digitalWrite(X_STEP_PIN, HIGH);
#define STEP_LOW_X          digitalWrite(X_STEP_PIN, LOW);
#if defined(SWAP_X_STEPPER)
#if defined(__STM32G0XX)
#define X_STEP_PIN_NAME     PB_3
#endif
#define X_STEP_PIN          PB3
#define X_DIR_PIN           PB4
#define X_ENABLE_PIN        PD1
#else
#if defined(__STM32G0XX)
#define X_STEP_PIN_NAME     PB_13
#endif
#define X_STEP_PIN          PB13
#define X_DIR_PIN           PB12
#define X_ENABLE_PIN        PB14
#endif
#if defined(SWAP_SELECTOR_ENDSTOP)
#define X_END_PIN           PC1     // Y-STOP
#else
#define X_END_PIN           PC0     // X-STOP
#endif
// REVOLVER (Y)
#define STEP_HIGH_Y         digitalWrite(Y_STEP_PIN, HIGH);
#define STEP_LOW_Y          digitalWrite(Y_STEP_PIN, LOW);
#if defined(SWAP_Y_STEPPER) // flag swaps driver Y and E
#if defined(__STM32G0XX)
#define Y_STEP_PIN_NAME     PB_3
#endif
#define Y_STEP_PIN          PB3
#define Y_DIR_PIN           PB4
#define Y_ENABLE_PIN        PD1
#else
#if defined(__STM32G0XX)
#define Y_STEP_PIN_NAME     PB_10
#endif
#define Y_STEP_PIN          PB10
#define Y_DIR_PIN           PB2
#define Y_ENABLE_PIN        PB11
#endif
#if defined(SWAP_SELECTOR_ENDSTOP)
#define Y_END_PIN           PC1     // Y-STOP
#else
#define Y_END_PIN           PC0     // X-STOP
#endif
// FEEDER (Z)
#define STEP_HIGH_Z         digitalWrite(Z_STEP_PIN, HIGH);
#define STEP_LOW_Z          digitalWrite(Z_STEP_PIN, LOW);
#if defined(__STM32G0XX)
#define Z_STEP_PIN_NAME     PB_0
#endif
#define Z_STEP_PIN          PB0
#define Z_DIR_PIN           PC5
#define Z_ENABLE_PIN        PB1
#define Z_END_PIN           PC2     // Z-STOP
#define Z_END2_PIN          PC15    // E0-STOP
#define Z_END_DUET_PIN      Z_END2_PIN

// (E) - Not used yet, just in case
#define STEP_HIGH_E         digitalWrite(E_STEP_PIN, HIGH);
#define STEP_LOW_E          digitalWrite(E_STEP_PIN, LOW);
#if defined(SWAP_Y_STEPPER) // flag swaps driver Y and E
#define E_STEP_PIN          PB10
#define E_DIR_PIN           PB2
#define E_ENABLE_PIN        PB11
#else
#if defined(SWAP_X_STEPPER) // flag swaps driver Y and E
#define E_STEP_PIN          PB13
#define E_DIR_PIN           PB12
#define E_ENABLE_PIN        PB14
#else
#define E_STEP_PIN          PB3
#define E_DIR_PIN           PB4
#define E_ENABLE_PIN        PD1
#endif
#endif
#define E_END_PIN           PC15    // E0-STOP

#define RELAY_PIN           PC13    // PS-ON (Relay for stepper motor switching)

#define SERVO_OPEN_DRAIN    0
#if !defined(SWAP_SERVOS)
#define SERVO1_PIN          PC14    // Z-PROBE.1  (Wiper Servo)
#define SERVO2_PIN          PA1     // Z-PROBE.3  (Lid Servo)
// #define SERVO2_PIN          PD4     // Z-PROBE.3  (Lid Servo)
#define SERVO3_PIN          PC12    // PT-DET     (Cutter Servo)
#else
#define SERVO1_PIN          PC12    // PT-DET     (Wiper Servo)
#define SERVO2_PIN          PA1     // Z-PROBE.3  (Lid Servo)
#define SERVO3_PIN          PC14    // Z-PROBE.1  (Cutter Servo)
#endif

#define FAN_PIN             PC6     // FAN0
#define FAN1_PIN            PC7     // FAN1 (Dryer)
#define FAN2_PIN            PB15    // FAN2 (Dryer)
#define HEATER_PIN          PC9     // HB-Heater (Dryer)
#define HEATER2_PIN         PC8     // E0-Heater

#define NEOPIXEL_TOOL_PIN   PA8     // for tools (NEOPIXEL)
//#define NEOPIXEL_TOOL_PIN   PA9     // alternative for tools (EXT1.8); Important: _only_ usable with TWI / LEONERD display
//#define NEOPIXEL_TOOL_PIN   PD6     // alternative for tools (EXT1.3); Important: not! usable with TWI / LEONERD display
#define BRIGHTNESS_TOOL     127
//#define LED_TYPE_TOOL       WS2812B
#define COLOR_ORDER_TOOL    NEO_GRB + NEO_KHZ800

#define SDCS_PIN            0       // use default
#define DEBUG_PIN           0       // none

#define USB_CONNECT_PIN     0       // not available
#define SD_DETECT_PIN       PC3

#if defined(USE_SPLITTER_ENDSTOPS)
// using the same pins as for TWI displays (SW-I2C)
#if !defined(USE_SW_TWI)
#define SPLITTER_SCL        PA9     // EXP1.8
#define SPLITTER_SDA        PA10    // EXP1.6
#else
#define SPLITTER_SCL        PA15    // EXP1.9
#define SPLITTER_SDA        PD6     // EXP1.3
#endif
#endif

#if defined(USE_MULTISERVO)
// software I2C is being used on those pins
#define ADASERVO_SCL        PC14    // Z-PROBE.1
#define ADASERVO_SDA        PA1     // Z-PROBE.3
#endif


#define DUET_SIG_FED_PIN    PC4      // THB (thermistor output pins will work fine up to 100Hz - see schematic)
#if !defined(USE_DRYER)
#define DUET_SIG_SEL_PIN    PA0      // TH0
#else
#define HEATER_TEMP_PIN     PA0      // TH0
#endif

#define DEBUG_OFF_PIN       0

// only on V3.0 (spare pins... YAY!)
#define GPIO1               PD5     // I/O.1
#define GPIO2               PD4     // I/O.2
#define GPIO3               PD3     // I/O.3
#define GPIO4               PD2     // I/O.4
#define GPIO5               PD0     // I/O.5

#define STALL_X_PIN         GPIO5   // I/O.5 
#define STALL_Y_PIN         GPIO4   // I/O.4
#define STALL_Z_PIN         GPIO3   // I/O.3

// the following pins cannot be used directly from the according headers/terminals, since those are signals
// used to drive the Mosfets for heaters/fan. If you need one of those signals, you have to wire it up on the 
// according driver input pin of U8 (see schematic).
#define SPARE1              PC8     // HE0
#define SPARE2              PC9     // BED
#define SPARE3              PC7     // FAN1
#define SPARE4              PB15    // FAN2

// -----------------------------------------------------
// Serial Ports section
// -----------------------------------------------------
#define SW_SERIAL_TX_PIN    0
#define SW_SERIAL_RX_PIN    0

#define TMC_HW_SERIAL       1
#define TMC_SERIAL          Serial4

// SERIAL1 - Cannot be used for serial comm.
#if defined(USE_SERIAL_DISPLAY)
#define CAN_USE_SERIAL1     true
#else
#define CAN_USE_SERIAL1     false
#endif
#define TX1_PIN             PA9     // EXP1.8 - ENCODER1_PIN
#define RX1_PIN             PA10    // EXP1.6 - ENCODER2_PIN

// SERIAL2 - Can be used for serial comm.
#define CAN_USE_SERIAL2     true
#define TX2_PIN             PA2     // TX on TFT header
#define RX2_PIN             PA3     // RX on TFT header

// SERIAL3 - Cannot be used for serial comm.
#define CAN_USE_SERIAL3     false
#define TX3_PIN             PB10    // Y-Axis STEP
#define RX3_PIN             PB11    // Y-Axis ENABLE

// SERIAL4 - Cannot be used for serial comm.
#define CAN_USE_SERIAL4     false
#define TX4_PIN             PC10    // Common Stepper Driver TX
#define RX4_PIN             PC11    // Common Stepper Driver RX

#if !defined(__LIBMAPLE__)
#define SPI3_SCLK           0
#define SPI3_MISO           0
#define SPI3_MOSI           0
#define SPI3_CS             0
#endif

// -----------------------------------------------------
// Display section
// -----------------------------------------------------
#include "../Display/Displays.h"
