/* ************************************************************************** */
/** ADC.H

  @Company
    Chamberlain Group Inc

  @File Name
    adc.h

  @Summary
    Source file for ADC

 */
/* ************************************************************************** */

#ifndef _ADC_H    /* Guard against multiple inclusion */
#define _ADC_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */
#include "userparams.h"
#include "mc_app.h"
#include "microcontroller.h"
/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */



    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************
extern ADC_POSINPUT secondaryAdc0Channels[];
extern ADC_POSINPUT secondaryAdc1Channels[];
extern uint16_t                adc_0_offset;
extern uint16_t                adc_1_offset;
extern volatile uint16_t       adc_bus_current_offset;


extern bool adc_motor_id_error_f; //motor id error

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

extern void ADC_CALIB_IPHASE_ISR(ADC_STATUS status, uintptr_t context);
extern void ADC_wait_for_cal(void);

/*************************************************************************
 * Function: ADC_check_motor_ID
 *
 * Arguments: none
 *
 * Returns: none
 *
 * Description:  Checks ADC input for motor ID resistor and sets fault if ID 
 *              conflicts with board ID or is missing/undefined
 *
 * Original author: Olivia Robles
 ***************************************************************************/
extern void ADC_check_motor_ID(void);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _ADC_H */

/* *****************************************************************************
 End of File
 */
