/* ************************************************************************** */
/** Board ID

  @Company
    Chamberlain Group

  @File Name
    board_id.c

  @Summary
    Source file for board_id

 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "board_id.h"


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */
//UPDATE FIRMWARE VERSION HERE FOR ANY CHANGES - ** for now, do not use 0 **
#define VERSION_MAJOR 3
#define VERSION_MINOR 1

uint8_t versionNumbers[NUM_OF_VERSIONS_TO_COMM] = {VERSION_MAJOR, VERSION_MINOR, 0};

uint8_t Motor_ID_num = INVALID;   // Used to store the ID of the connected motor
                                  // (Initialized to a bad value, so we dont default to a motor)

uint8_t Board_ID_num = INVALID; // Used to store the ID of the motor drive board
                                // (Initialized to a bad value, so we dont default)

motorConfigParameters motorParams[MAX_MOTOR_NUM] = {
  {
    MOTOR_PER_PHASE_RESISTANCE_HDOP,
    MOTOR_PER_PHASE_INDUCTANCE_HDOP,
    MAX_FW_NEGATIVE_ID_REF_HDOP,
    MAX_MOTOR_CURRENT_HDOP,
    MAX_MOTOR_CURRENT_DURING_STALL_HDOP,
    SPEEDCNTR_PTERM_HDOP,
    SPEEDCNTR_ITERM_HDOP,
    SPEEDCNTR_CTERM_HDOP,
    SPEEDCNTR_PTERM_DURING_STALL_HDOP,
    SPEEDCNTR_ITERM_DURING_STALL_HDOP,
    MAX_MOTOR_CURRENT_SQUARED_HDOP,
    INVKFi_BELOW_BASE_SPEED_HDOP,
    MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC_HDOP,
    NOMINAL_SPEED_RAD_PER_SEC_ELEC_HDOP,
    POT_ADC_COUNT_FW_SPEED_RATIO_HDOP,
    NOMINAL_ELE_VEL_RAD_PER_SEC_HDOP,
    CTC_ELE_VEL_2_LOW_BOUND_HDOP,
    CTC_ELE_VEL_1_HIGH_BOUND_HDOP,
    CTC_ELE_VEL_1_LOW_BOUND_HDOP,
    CTC_ELE_VEL_2_HIGH_BOUND_HDOP,
    CLOSEDLOOP_SPEED_RAMP_RATE_DELTA_HDOP,
    CLOSEDLOOP_SPEED_HYSTERESIS_HDOP,
    SENSORED_TO_SENSORLESS_SWITCH_SPEED_HDOP,
    SENSORLESS_TO_SENSORED_SWITCH_SPEED_HDOP,
    TRAP_TO_FOC_SWITCH_SPEED_HDOP,
    FOC_TO_TRAP_SWITCH_SPEED_HDOP
  },
  {
    MOTOR_PER_PHASE_RESISTANCE_SPARTANH,
    MOTOR_PER_PHASE_INDUCTANCE_SPARTANH,
    MAX_FW_NEGATIVE_ID_REF_SPARTANH,
    MAX_MOTOR_CURRENT_SPARTANH,
    MAX_MOTOR_CURRENT_DURING_STALL_SPARTANH,
    SPEEDCNTR_PTERM_SPARTANH,
    SPEEDCNTR_ITERM_SPARTANH,
    SPEEDCNTR_CTERM_SPARTANH,
    SPEEDCNTR_PTERM_DURING_STALL_SPARTANH,
    SPEEDCNTR_ITERM_DURING_STALL_SPARTANH,
    MAX_MOTOR_CURRENT_SQUARED_SPARTANH,
    INVKFi_BELOW_BASE_SPEED_SPARTANH,
    MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC_SPARTANH,
    NOMINAL_SPEED_RAD_PER_SEC_ELEC_SPARTANH,
    POT_ADC_COUNT_FW_SPEED_RATIO_SPARTANH,
    NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANH,
    CTC_ELE_VEL_2_LOW_BOUND_SPARTANH,
    CTC_ELE_VEL_1_HIGH_BOUND_SPARTANH,
    CTC_ELE_VEL_1_LOW_BOUND_SPARTANH,
    CTC_ELE_VEL_2_HIGH_BOUND_SPARTANH,
    CLOSEDLOOP_SPEED_RAMP_RATE_DELTA_SPARTANH,
    CLOSEDLOOP_SPEED_HYSTERESIS_SPARTANH,
    SENSORED_TO_SENSORLESS_SWITCH_SPEED_SPARTANH,
    SENSORLESS_TO_SENSORED_SWITCH_SPEED_SPARTANH,
    TRAP_TO_FOC_SWITCH_SPEED_SPARTANH,
    FOC_TO_TRAP_SWITCH_SPEED_SPARTANH
  },
  {
    MOTOR_PER_PHASE_RESISTANCE_SPARTANL,
    MOTOR_PER_PHASE_INDUCTANCE_SPARTANL,
    MAX_FW_NEGATIVE_ID_REF_SPARTANL,
    MAX_MOTOR_CURRENT_SPARTANL,
    MAX_MOTOR_CURRENT_DURING_STALL_SPARTANL,
    SPEEDCNTR_PTERM_SPARTANL,
    SPEEDCNTR_ITERM_SPARTANL,
    SPEEDCNTR_CTERM_SPARTANL,
    SPEEDCNTR_PTERM_DURING_STALL_SPARTANL,
    SPEEDCNTR_ITERM_DURING_STALL_SPARTANL,
    MAX_MOTOR_CURRENT_SQUARED_SPARTANL,
    INVKFi_BELOW_BASE_SPEED_SPARTANL,
    MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC_SPARTANL,
    NOMINAL_SPEED_RAD_PER_SEC_ELEC_SPARTANL,
    POT_ADC_COUNT_FW_SPEED_RATIO_SPARTANL,
    NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANL,
    CTC_ELE_VEL_2_LOW_BOUND_SPARTANL,
    CTC_ELE_VEL_1_HIGH_BOUND_SPARTANL,
    CTC_ELE_VEL_1_LOW_BOUND_SPARTANL,
    CTC_ELE_VEL_2_HIGH_BOUND_SPARTANL,
    CLOSEDLOOP_SPEED_RAMP_RATE_DELTA_SPARTANL,
    CLOSEDLOOP_SPEED_HYSTERESIS_SPARTANL,
    SENSORED_TO_SENSORLESS_SWITCH_SPEED_SPARTANL,
    SENSORLESS_TO_SENSORED_SWITCH_SPEED_SPARTANL,
    TRAP_TO_FOC_SWITCH_SPEED_SPARTANL,
    FOC_TO_TRAP_SWITCH_SPEED_SPARTANL
  }
};

/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

/* ************************************************************************** */




/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */


void bi_determine_board_id(void)
{
    if(Variant_Heavy_Get() && !Variant_Lite_Get())
    {
        Board_ID_num = SPARTAN_HEAVY;
    }
    else if (!Variant_Heavy_Get() && Variant_Lite_Get())
    {
        Board_ID_num = SPARTAN_LITE;
    }
    else //HD OPERATOR not currently implemented
    {
       Board_ID_num = INVALID;
    }
}

/*******************************************************************************
* Function: bi_versionInfo_ISR
*
* Arguments: status - TC status provided by the NVIC
*            context - provided by the NVIC
*
* Returns: none
*
* Description: Used for communicating version/config info to the main board.
*              Controls the Fault output pin to the control board.  This ISR is
*              called every 30ms by TC4 OVF overflow interrupt.
*
* Original authors: Kyle Reu
*
* Copyright 2022 The Chamberlain Group, Inc. All Rights Reserved.
*******************************************************************************/
void bi_versionInfo_ISR(TC_TIMER_STATUS status, uintptr_t context) //NOSONAR - ISR format is defined this way in IDE
{
    volatile static uint8_t outputPulseCounter = 0; //pulses for version numbers
    volatile static uint8_t outputHeaderCounter = 1; //dead time between version number codes
    volatile static uint8_t outputPinState = 1;

    static uint8_t i = 0;  // counter for 3 different numbers to tx

    
    if (i > NUM_OF_VERSIONS_TO_COMM)
    {
        // After all 3 numbers are sent, we are done
    }
    else if (0 == outputPinState)
    {
        Drive_Fault_Out_Set();
        //Debug_LED_Set();
        outputPinState = 1;
    }
    else
    {// Output is high
        if (outputHeaderCounter < 1)    //stay high between version numbers for 90ms
                                        // 30ms timer period after the last low pulse
                                        // + 30ms after the else where we reset outputHeaderCounter
                                        // + 30ms here where we increment outputHeaderCounter
        {
            outputHeaderCounter++;
        }
        else if (outputPulseCounter < versionNumbers[i])
        {
            Drive_Fault_Out_Clear();
            //Debug_LED_Clear();
            outputPinState = 0;
            outputPulseCounter++;
        }
        else //adds 30 ms
        {
            // Output sequence complete, reset counters
            outputPulseCounter = 0;
            outputHeaderCounter = 0;
            if (i < 3)
            {
                i++;    // Increment counter so we move to next number
            }
            else
            {
            // We are done communicating version info, so disable timer
                TC4_TimerStop();
            }
        }
        
        
    }
}

void bi_check_id_error(void)
{
    if ( ( adc_motor_id_error_f //motor id resistor reading invalid
       || (Board_ID_num != Motor_ID_num) //board & motor ids do not match
       || (Board_ID_num == INVALID) ) //board id invalid
       && !((Board_ID_num == SPARTAN_HEAVY) && (Motor_ID_num == SPARTAN_LITE))  ) //allow heavy board with lite motor & lite main board (mainboard check with motor will be done on mainboard)
    {
        fh_faultHandler(ID_RESISTOR_FAULT);
    }
}


/* *****************************************************************************
 End of File
 */
