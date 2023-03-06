/* ************************************************************************** */
/** FAULT_HANDLER.H

  @Company
    Chamberlain Group Inc

  @File Name
    fault_handler.h

  @Summary
    Header file for fault handler

 */
/* ************************************************************************** */

#ifndef _FAULT_HANDLER_H    /* Guard against multiple inclusion */
#define _FAULT_HANDLER_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "definitions.h"
#include "userparams.h"
#include "mc_app.h"


#ifdef UNIT_TEST
#ifdef Motor_Enable_Get
#undef Motor_Enable_Get
#endif
extern uint8_t ut_motor_enable_get;
#define Motor_Enable_Get()      (ut_motor_enable_get)

#ifdef Motor_Direction_Get
#undef Motor_Direction_Get
#endif
extern uint8_t ut_motor_direction_get;
#define Motor_Direction_Get()   (ut_motor_direction_get)

// shadow inline function
#ifdef TCC1_PWM24bitDutySet
#undef TCC1_PWM24bitDutySet
#endif
#define TCC1_PWM24bitDutySet(X,Y)

// redefine TCC1 registers
#ifdef TCC1_REGS
#undef TCC1_REGS
#endif
tcc_registers_t dummy_regs;
#define TCC1_REGS (&dummy_regs)

#ifdef HALL_Voltage_Select_Clear
#undef HALL_Voltage_Select_Clear
#endif
#define HALL_Voltage_Select_Clear()

#define RUN_FOREVER 0

#else
#define RUN_FOREVER 1
#endif


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */

/*MC Fault Codes*/
#define NO_FAULT                    0   // No heartbeat when motor is running
#define IDLE                        1   // Used as a heartbeat
#define BRAKE_INPUT_ACTIVE          2
#define MOTOR_STALL                 3
#define LOW_SIDE_FET_FAULT          4
#define HIGH_SIDE_FET_FAULT         5
#define HALL_SENSOR_FAULT           6
#define BUS_CURRENT_FAULT_UNUSED    7   // NOT USED (legacy fault code)
#define BUS_UNDERVOLTAGE_FAULT      8
#define UNDERVOLTAGE_5V_RAIL_UNUSED 9   // NOT USED (legacy fault code)
#define OPAMP_FAULT                 10
#define FET_TEMPERATURE_FAULT       11
#define ID_RESISTOR_FAULT           12

    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************

typedef struct {
    uint8_t faultCode;  // Current fault code
    uint8_t faultCount; // Keeps a running count of all detected faults since power on
} drive_fault_status_t;

extern drive_fault_status_t    drive_fault_status;

extern volatile uint16_t     fetTemp1;
extern volatile uint16_t     fetTemp2;
extern volatile int16_t      busCurrent;
extern volatile float        busVoltage;


    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

extern void fh_ShortedFetISRTasks(ADC_STATUS status, uintptr_t context);
extern void fh_shortedFetTest();
extern void fh_FetTemperatureTest();
extern void fh_BusUnderVoltageTest();
extern void fh_faultHandler(uint8_t fault_code);
extern void TC4_TimerInitialize_FaultCodes();
extern void fh_TC4_ISR(TC_TIMER_STATUS status, uintptr_t context);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _FAULT_HANDLER_H */

/* *****************************************************************************
 End of File
 */
