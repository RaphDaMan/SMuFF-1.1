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
    Calculate temperature based on the Steinhart-Hart equation for NTC.
    This module has two way of  calculating the temperature:
        calcTemp():             - calculate temp. based on coefficient values
        calcTempFromBeta():     - calculate temp. based on thermistor Beta value
    
    Both are percise enough, the later is easier to configure because there are no
    pre-calculated coefficient values needed.

    Basic ideas taken from Circuit Basics, see:
    https://www.circuitbasics.com/arduino-thermistor-temperature-sensor-tutorial/

    Steinhart-Hart coefficient values A, B and C must be pre-calculated here:
    https://www.thinksrs.com/downloads/programs/therm%20calc/ntccalibrator/ntccalculator.html
    and set up in smuffConfig.shc_A, .shc_B and .shc_C.
*/

#include "SMuFF.h"

#define TEMP_SAMPLE_CNT  5

float calcTempFromBeta(uint32_t raw, uint32_t adcResolution, uint32_t R1, float betaVal, float R25)
{
    uint32_t R2 = (uint32_t)R1 / ((float)adcResolution/raw - 1.0);
    float Rth = (float)R2/R25;
    float tK = (float)1/(1/(KELVIN+25)+1.0/betaVal*log(Rth));
    return tK - KELVIN;
}

float calcTemp(uint32_t raw, uint32_t adcResolution, uint32_t R1, float shc_A, float shc_B, float shc_C)
{
    uint32_t R2 = (uint32_t)R1 / ((float)adcResolution/raw - 1.0);
    float tK = 1/(shc_A + shc_B * log(R2) + shc_C * pow(log(R2),3));      // resulting temperature in Kelvin
    return tK - KELVIN;
}

// ------------------------------------------
// Read temperatur sensor
//
// returns: Either the meassured temp. in Celsius or -273.15 in case of error.
// ------------------------------------------
float readHeaterTemp(pin_t pin, bool useBeta, bool useFahrenheit) {

    uint32_t raw = 0;
    uint32_t mStart = micros();
    
    // 5 samples are supposed to take around 1300 microseconds
    for(uint8_t i=0; i < TEMP_SAMPLE_CNT; i++) {
        raw += analogRead(pin);
    }
    //__debugS(D, PSTR("readHeaterTemp took: %d us"), micros()-mStart);

    if(raw == 0) {
        heaterOverflow = true;
        return -KELVIN;
    }
    raw = (uint32_t)raw/TEMP_SAMPLE_CNT;
    if(raw == smuffConfig.adc_Res) {
        heaterOverflow = true;
        return -KELVIN;
    }

    float tC;
    if(!useBeta) {
        // calculation based on Steinhart-Hart pre-calculated values
        tC = calcTemp(raw, smuffConfig.adc_Res, smuffConfig.adc_R1, smuffConfig.shc_A, smuffConfig.shc_B, smuffConfig.shc_C);
    }
    else {
        // calculation based on thermistor Beta value
        tC = calcTempFromBeta(raw, smuffConfig.adc_Res, smuffConfig.adc_R1, smuffConfig.thermistorBeta, smuffConfig.thermistorR25);
    }
    float tF = (tC * 9.0) / 5.0 + 32.0;                                   // temp. in Fahrenheit (if needed)

    // overall temp. measurement is supposed to take not more than 2500 microseconds (2.5 ms)
    //__debugS(D, PSTR("readHeaterTemp calc took: %d us"), micros()-mStart);

    heaterOverflow = false;
    return useFahrenheit ? tF : tC;
}