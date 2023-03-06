/* ************************************************************************** */
/** Board ID

  @Company
    Chamberlain Group

  @File Name
    board_id.h

  @Summary
    Header file for board ID
 */
/* ************************************************************************** */

#ifndef _BOARD_ID_H    /* Guard against multiple inclusion */
#define _BOARD_ID_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "config/cgi_proto/peripheral/port/plib_port.h"
#include "fault_handler.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */
    
#define NUM_OF_VERSIONS_TO_COMM 3 //version high, version low, motor ID

#ifdef UNIT_TEST
  bool ut_variant_heavy_get (void);
  #ifdef Variant_Heavy_Get
      #undef Variant_Heavy_Get
  #endif
  #define Variant_Heavy_Get() (ut_variant_heavy_get())

  bool ut_variant_lite_get (void);
  #ifdef Variant_Lite_Get
      #undef Variant_Lite_Get
  #endif
  #define Variant_Lite_Get() (ut_variant_lite_get())

  #ifdef ADC0_REGS
  #undef ADC0_REGS
  #endif
  adc_registers_t dummy_adc0_regs;
  #define ADC0_REGS (&dummy_adc0_regs)

#endif

typedef struct {
  float     MOTOR_PER_PHASE_RESISTANCE;                          // Per Phase Resistance in Ohms
  float     MOTOR_PER_PHASE_INDUCTANCE;                          // Per Phase Inductance in Henrys  
  float     MAX_FW_NEGATIVE_ID_REF;                              // Maximum negative D axis reference current (in A) during Flux Weakening
  float     MAX_MOTOR_CURRENT;                                   // Maximum Motor Current in A
  float     MAX_MOTOR_CURRENT_DURING_STALL;                      // Maximum Motor Current allowed during stall condition

  float     SPEEDCNTR_PTERM;                                     // Speed Loop Proportional Gain
  float     SPEEDCNTR_ITERM;                                     // Speed Loop Integral Gain
  float     SPEEDCNTR_CTERM;                                     // Speed Loop Anti-Windup Gain
  float     SPEEDCNTR_PTERM_DURING_STALL;                        // Speed Loop Proportional Gain during stall
  float     SPEEDCNTR_ITERM_DURING_STALL;                        // Speed Loop Integral Gain during stall

  float     MAX_MOTOR_CURRENT_SQUARED;
  float     INVKFi_BELOW_BASE_SPEED;
  float     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC;
  float     NOMINAL_SPEED_RAD_PER_SEC_ELEC;
  float     POT_ADC_COUNT_FW_SPEED_RATIO;
  float     NOMINAL_ELE_VEL_RAD_PER_SEC;
  float     CTC_ELE_VEL_2_LOW_BOUND;
  float     CTC_ELE_VEL_1_HIGH_BOUND;
  float     CTC_ELE_VEL_1_LOW_BOUND;
  float     CTC_ELE_VEL_2_HIGH_BOUND;
  float     CLOSEDLOOP_SPEED_RAMP_RATE_DELTA;
  float     CLOSEDLOOP_SPEED_HYSTERESIS;
  float     SENSORED_TO_SENSORLESS_SWITCH_SPEED;
  float     SENSORLESS_TO_SENSORED_SWITCH_SPEED;
  float     TRAP_TO_FOC_SWITCH_SPEED;
  float     FOC_TO_TRAP_SWITCH_SPEED;
}motorConfigParameters;

extern motorConfigParameters motorParams[MAX_MOTOR_NUM];

extern uint8_t versionNumbers[NUM_OF_VERSIONS_TO_COMM];


    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************

extern uint8_t Motor_ID_num;
extern uint8_t Board_ID_num;


    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

/**
 * @brief board id determine board id
 *
 * Function to check board id resistor based on populated location, not resistor value
 */
extern void bi_determine_board_id(void);
extern void bi_versionInfo_ISR(TC_TIMER_STATUS status, uintptr_t context);

/**
 * @brief board id check id error
 *
 * Function to check if motor and board ids are both valid and matching
 * calls fault handler in the event of a mismatch or invalid reading
 */
extern void bi_check_id_error(void);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _BOARD_ID_H */

/* *****************************************************************************
 End of File
 */
