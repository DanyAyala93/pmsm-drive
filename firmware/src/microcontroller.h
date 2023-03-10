/* 
 * File:   microcontroller.h
 * Author: kreu
 *
 * Created on June 23, 2020, 1:05 PM
 */

#ifndef MICROCONTROLLER_H
#define MICROCONTROLLER_H

#include "./config/cgi_proto/peripheral/adc/plib_adc_common.h"

#define VIN_ADC_POSINPUT            ADC_POSINPUT_AIN9   // ADC1_AIN9
#define VBATT_ADC_POSINPUT          ADC_POSINPUT_AIN1   // ADC0_AIN1
#define U_CURRENT_ADC_POSINPUT      ADC_POSINPUT_AIN11  // ADC0_AIN11
#define V_CURRENT_ADC_POSINPUT      ADC_POSINPUT_AIN2   // ADC1_AIN2
#define W_CURRENT_ADC_POSINPUT      ADC_POSINPUT_AIN10  // ADC0_AIN10
#define BUS_CURRENT_ADC_POSINPUT    ADC_POSINPUT_AIN1   // ADC1_AIN1
#define SPEED_CMD_ADC_POSINPUT      ADC_POSINPUT_AIN0   // ADC0_AIN0
#define BAT_CURRENT_ADC_POSINPUT    ADC_POSINPUT_AIN6   // ADC1_AIN6
#define FET_THMSTR1_ADC_POSINPUT    ADC_POSINPUT_AIN3   // ADC1_AIN3
#define FET_THMSTR2_ADC_POSINPUT    ADC_POSINPUT_AIN4   // ADC0_AIN4
#define AMB_THMSTR_ADC_POSINPUT     ADC_POSINPUT_AIN7   // ADC1_AIN7
#define MOTOR_ID_ADC_POSINPUT       ADC_POSINPUT_AIN5   // ADC0_AIN5

#define HALL_ANALOG_A_POSINPUT      ADC_POSINPUT_AIN8   // ADC1_AIN8
#define HALL_ANALOG_B_POSINPUT      ADC_POSINPUT_AIN2   // ADC0_AIN2

#endif    /* MICROCONTROLLER_H */

