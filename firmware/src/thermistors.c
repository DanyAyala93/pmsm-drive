/** @file thermistors.c
 * @brief Converting thermistor values and storing temperature history for 
 *        average calculations.
 *
 * @author kreu
 *
 * @copyright <2021/2022> The Chamberlain Group, LLC. All rights reserved.
 * All information within this file and associated files, including all information
 * and files transferred with this file are CONFIDENTIAL and the proprietary
 * property of The Chamberlain Group, LLC.
 *
 * In using the Licensed Software, Company shall not use with, combine, or
 * incorporate any viral open source software with or into any of the Licensed
 * Software in a manner that would require any portion of the Licensed software to
 * be (i) disclosed or distributed in source code form; (ii) licensed for the
 * purpose of making derivative works; or (iii) distributable or redistributable
 * at no charge.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "thermistors.h"

#define THERMISTOR_TABLE_SIZE 36
#define N_TEMPERATURE_READINGS_TO_AVERAGE   50  

uint16_t fetTemperatureLog1[N_TEMPERATURE_READINGS_TO_AVERAGE];
uint16_t fetTemperatureLog2[N_TEMPERATURE_READINGS_TO_AVERAGE];

uint16_t thermistorTable[THERMISTOR_TABLE_SIZE] = {
    3136,   // 0 degC
    2939,   // 5
    2726,   // 10
    2503,   // 15
    2275,   // 20
    2048,   // 25
    1827,   // 30
    1618,   // 35
    1423,   // 40
    1245,   // 45
    1084,   // 50
    941,    // 55
    815,    // 60
    705,    // 65
    609,    // 70
    527,    // 75
    456,    // 80
    395,    // 85
    342,    // 90
    297,    // 95
    259,    // 100
    226,    // 105
    197,    // 110
    173,    // 115
    152,    // 120
    134,    // 125
    118,    // 130
    104,    // 135
    93,     // 140
    82,     // 145
    73,     // 150
    64,     // 155
    55,     // 160
    46,     // 165
    37,     // 170
    28      // 175
};

/**
 * @brief Add new temperature value and get the average value.
 *
 * Takes a raw ADC value as an argument, converts it to a temperature
 * in degrees Celcius, adds it to an array that stores temperature
 * measurements, and then returns the average of the entire array.
 *              
 * No validation is done to the ADC values passed to this function.
 * All values will be converted to degrees and saved.
 *              
 * This function is called every 10ms from main.
 *
 * @param AdcVal New raw ADC value to add to the temperature log 

 * @return The average of all temperature measurements stored
 *         in the temperature log for this thermistor.  Units are 
 *         degrees Celcius.
 */
uint16_t thermistorGetFetTemp1(uint16_t AdcVal)
{
    static uint8_t tLogIdx = 0; // Index for temperature log
    uint16_t averageTemperature = 0;

    for (uint8_t i = 0; i < THERMISTOR_TABLE_SIZE; i++)
    {
        if ( (AdcVal >= thermistorTable[i]) || (i == (THERMISTOR_TABLE_SIZE - 1)) ) //can't go over max, just use largest value (will cause overtemp fault)
        {
            if (tLogIdx >= N_TEMPERATURE_READINGS_TO_AVERAGE)
            {
                tLogIdx = 0;
            }

            fetTemperatureLog1[tLogIdx++] =  i*5;
            break;
        }
    }

    for (uint8_t i = 0; i < N_TEMPERATURE_READINGS_TO_AVERAGE; i++)
    {
        averageTemperature += fetTemperatureLog1[i];
    }
    averageTemperature = averageTemperature / N_TEMPERATURE_READINGS_TO_AVERAGE;
    
    
    return averageTemperature;
}

/**
 * @brief Add new temperature value and get the average value.
 *
 * Takes a raw ADC value as an argument, converts it to a temperature
 * in degrees Celcius, adds it to an array that stores temperature
 * measurements, and then returns the average of the entire array.
 *              
 * No validation is done to the ADC values passed to this function.
 * All values will be converted to degrees and saved.
 *              
 * This function is called every 10ms from main.
 *
 * @param AdcVal New raw ADC value to add to the temperature log 

 * @return The average of all temperature measurements stored
 *         in the temperature log for this thermistor.  Units are 
 *         degrees Celcius.
 */
uint16_t thermistorGetFetTemp2(uint16_t AdcVal)
{
    static uint8_t tLogIdx = 0; // Index for temperature log
    uint16_t retVal = 0;

    for (uint8_t i = 0; i < THERMISTOR_TABLE_SIZE; i++)
    {
        if ( (AdcVal >= thermistorTable[i]) || (i == (THERMISTOR_TABLE_SIZE - 1)) ) //can't go over max, just use largest value (will cause overtemp fault)
        {
            if (tLogIdx >= N_TEMPERATURE_READINGS_TO_AVERAGE)
            {
                tLogIdx = 0;
            }

            fetTemperatureLog2[tLogIdx++] =  i*5;
            break;
        }
    }

    for (uint8_t i = 0; i < N_TEMPERATURE_READINGS_TO_AVERAGE; i++)
    {
        retVal += fetTemperatureLog2[i];
    }
    retVal = retVal / N_TEMPERATURE_READINGS_TO_AVERAGE;
    
    
    return retVal;
}