/*******************************************************************************
 Motor Control Application Header File

  File Name:
    mc_app.h

  Summary:
 Motor Control Application Variable and Function declarations.

  Description:
    This file contains the declarations for Motor Control application specific 
 * variables and functions (excluding variables and functions used by Motor Control
 * Library

 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _MC_APP_H    /* Guard against multiple inclusion */
#define _MC_APP_H
#include "definitions.h"
//#include "X2CScope.h"
//#include "X2CScopeCommunication.h"
#include "mc_Lib.h"
#include "hall.h"
#include "adc.h"
#include "fault_handler.h"

#ifdef UNIT_TEST

#ifdef NVIC_EnableIRQ
#undef NVIC_EnableIRQ
#endif
#define NVIC_EnableIRQ(PDEC_OTHER_IRQn)
#ifdef NVIC_DisableIRQ
#undef NVIC_DisableIRQ
#endif
#define NVIC_DisableIRQ(PDEC_OTHER_IRQn)
#ifdef TCC1_PWM24bitDutySet
#undef TCC1_PWM24bitDutySet
#endif
#define TCC1_PWM24bitDutySet(channel, duty)

#define asm(C)

#ifdef Dump_Resistor_FET_Control_Clear
#undef Dump_Resistor_FET_Control_Clear
#endif
#define Dump_Resistor_FET_Control_Clear()

#endif

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

/* MC Core Variables */

/*MC State Machine States*/
// #define WINDMILLING                 0
// #define WINDMILLING_DECIDE          1
// #define WINDMILLING_PASSIVE_BRAKE   2
// #define ALIGN                       3
// #define OPENLOOP_FOC                4
// #define OPENLOOP_TO_CLOSEDLOOP_FOC  5
#define CLOSEDLOOP_FOC              6

typedef struct {
    uint16_t count;
    uint16_t period;
} delay_gen_t;

typedef struct {
    uint8_t focStateMachine; // FOC State Machine
    uint8_t focStart;
    uint8_t inverterRunning;// Inverter on/off state: 0 - Inverter Disabled,  1- Inverter Enabled
    uint8_t motorDirection; // Motor Spin Direction : 0 - Positive Direction, 1 - Negative Direction
    uint8_t motorForceStop; // This flag is set to force the motor to stop
    uint8_t trap_mode;
    uint8_t sensorless_mode;
    uint8_t startup_f;  //flag used to signal for extra start up current if needed
} motor_status_t;

extern delay_gen_t          delay_10ms;
extern uint16_t timer_stall_detect_count;

extern uint8_t            motorStallDirection;

extern uint8_t motorIsMoving_f; //used to indicate motor is moving for check in main

//extern void ADC_ISR (ADC_STATUS status, uintptr_t context); where did this go?
extern void OC_FAULT_ISR (uintptr_t context);

extern void mcApp_SpeedRamp();
extern void mcApp_ADCISRTasks(ADC_STATUS status, uintptr_t context);
extern void mcApp_DumpResistorControl();
extern void mcApp_StallDetection();
extern void mcApp_DriveInitialize();
extern void mcApp_inverterDisable();
extern void delay_us(uint16_t time_us);
extern void mcApp_decideLowPowerMode();

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _MC_APP_H */

/* *****************************************************************************
 End of File
 */
