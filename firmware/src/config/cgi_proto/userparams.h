/*******************************************************************************
 * User Parameters 

  File Name:
    userparams.h

 Summary:
    Header file which defines Motor Specific and Board Specific constants 

  Description:
    This file contains the motor and board specific constants. It also defines
 * switches which allows algorithm to be run in debug modes like Open Loop Mode,
 * Torque mode, etc. 

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

#ifndef USERPARAMS_H
#define USERPARAMS_H

#include <math.h>

// Add by HL
#ifndef M_PI
    #define M_PI 3.14159265358979323846	/* pi */
#endif

#define     PWM_CLK                                             (120000000ul)   // PWM Peripheral Input Clock Frequency in Hz
#define     PWM_FREQ                                            20000           // PWM Frequency in Hz
#define     DELAY_MS                                            (float)10  // Delay in milliseconds after which Speed Ramp loop is executed
#define     SW_DEBOUNCE_DLY_MS                                  (float)500  // Switch debounce delay in mS

/*Application Configuration Switches*/
/*
 * TRAPEZOIDAL_MODE      FOC CONTROL         Mode
 *      0                    0            PLL Sensorless Mode
 *      0                    1            Starts and operates in FOC mode (Sensored Hall/Hybrid/Sensorless mode)
 *      1                    0            Starts and operates in Sensored TRAPEZOIDAL mode
 *      1                    1            Starts in Sensored Trapezoidal mode and switches to FOC mode (Sensored Hall/Hybrid/Sensorless mode)
 * 
 * PLL Sensorless Mode : Use sensorless angle and velocity estimate 
 *      HALL_ANGLE_RUN =  0u
 *      HALL_SENSORLESS_HYBRID_RUN = 0u
 * Hall + Sensorless Hybrid : Use sensored and sensorless angle and velocity estimate
 *      HALL_ANGLE_RUN =  0u
 *      HALL_SENSORLESS_HYBRID_RUN = 1u
 * Hall FOC Mode : Use Sensored angle and velocity estimate
 *      HALL_ANGLE_RUN =  1u
 *      HALL_SENSORLESS_HYBRID_RUN = 0u
 *  *  */

#define TRAPEZOIDAL_CONTROL 0u // Enables/ Disables Trapezoidal Control
#define FOC_CONTROL         1u // Enables/Disables FOC Control

#if !TRAPEZOIDAL_CONTROL
#define IPD_ENABLE 0u // Enables/Disables IPD Detect
#endif 

#if FOC_CONTROL
    #define HALL_ANGLE_RUN 1u
    #if !HALL_ANGLE_RUN
        #define HALL_SENSORLESS_HYBRID_RUN 0u
    #endif // HALL_ANGLE_RUN
#endif 
/*Defining this Macro, enables current PI tuning mode in which a step current
 *  reference is generated to observe step response of the current controller*/
#undef CURPI_TUN
#ifdef CURPI_TUN
    #define CUR_STEP_AMP  (float) 5.0
    #define CUR_STEP_TIM  (float) 0.05
#endif

/*Define macro OPEN_LOOP_FUNCTIONING to run the speed control in open loop.
 In this mode, the rotor angle is assumed and is not in sync with the actual rotor
 angle. This mode can be used to debug current loops and rotor position
 estimator*/

/*Undefine macro OPEN_LOOP_FUNCTIONING to run the speed control in closed loop.
 In this mode, the rotor angle is estimated using PLL based estimator. */
#undef OPEN_LOOP_FUNCTIONING 

/*Define macro TORQUE_MODE to run the motor in TORQUE Control Mode
 Undefine macro TORQUE_MODE to run the motor in Speed Control Mode*/
#undef TORQUE_MODE


/*Define macro ENABLE_FLUX_WEAKENING to enable Flux Weakening
 Undefine macro ENABLE_FLUX_WEAKENING to disable Flux Weakening*/
//#define ENABLE_FLUX_WEAKENING

/*Define macro ENABLE_WINDMILLING to enable Windmilling Capability*/
#if (!TRAPEZOIDAL_CONTROL && !HALL_ANGLE_RUN)
    #undef     ENABLE_WINDMILLING
#endif


//===============================================================
// Following parameters for Liftmaster 001D9878-1 board
// Gain of phase current opamp = 10 
// shunt resistor = 0.001 ohms
// DC offset = 1.65V
// max current = +/- 165 A
// (165 * 0.001 * 10) + 1.65V = 3.3V
#define     MAX_BOARD_CURRENT                                   (float)(165)            // Max Board Current in A
#define     DCBUS_SENSE_TOP_RESISTOR                            (float)220.0             // DC Bus voltage Divider - Top Side Resistor in Kohm
#define     DCBUS_SENSE_BOTTOM_RESISTOR                         (float)10.0              // DC Bus voltage Divider - Bottom Side Resistor in Kohm


//---------------Motor Specifications: HD Operator Motor 123-0232-000 
#define     MOTOR_PER_PHASE_RESISTANCE_HDOP                            ((float)0.0159)         // Per Phase Resistance in Ohms
#define     MOTOR_PER_PHASE_INDUCTANCE_HDOP                            ((float)0.0000325)      // Per Phase Inductance in Henrys
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_Line_Line_KRPM_MECH_HDOP     (float)6.1              // Back EMF Constant in Vpeak(L-L)/KRPM 
#define     NOPOLESPAIRS_HDOP                                          4                       // Number of Pole Pairs of the PMSM Motor     
#define     STAR_CONNECTED_MOTOR_HDOP                                  1                       // 1 - Motor is Star Connected, 0 - Motor is Delta Connected
#define     NOMINAL_SPEED_RPM_HDOP                                     (float)3700             // Nominal Rated Speed of the Motor - Value in RPM
#define     FW_SPEED_RPM_HDOP                                          (float)4100             // Maximum Speed of the Motor in Flux Weakening Mode - Value in RPM
#define     MAX_FW_NEGATIVE_ID_REF_HDOP                                (float)(-10.0)          // Maximum negative D axis reference current (in A) during Flux Weakening
#define     MAX_MOTOR_CURRENT_HDOP                                     (float)(100)            // Maximum Motor Current in A
#define     MAX_MOTOR_CURRENT_DURING_STALL_HDOP                        MAX_MOTOR_CURRENT_HDOP  //Not in use currently, set to same as max motor 

//*** Speed Control Loop Coefficients *****
#define     SPEEDCNTR_PTERM_HDOP                                     ((float)0.12)                  // Speed Loop Proportional Gain
#define     SPEEDCNTR_ITERM_HDOP                                     ((float)0.00003)              // Speed Loop Integral Gain
#define     SPEEDCNTR_CTERM_HDOP                                     (float)0.1                     // Speed Loop Anti-Windup Gain
#define     SPEEDCNTR_PTERM_DURING_STALL_HDOP                        ((float)0.170)                  // Speed Loop Proportional Gain
#define     SPEEDCNTR_ITERM_DURING_STALL_HDOP                        ((float)0.0003)              // Speed Loop Integral Gain
        
//*** Defines that use other motor specific values
#define     MAX_MOTOR_CURRENT_SQUARED_HDOP                             (float)((float)MAX_MOTOR_CURRENT_HDOP*(float)MAX_MOTOR_CURRENT_HDOP)
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RPM_MECH_HDOP          (float)((MOTOR_BACK_EMF_CONSTANT_Vpeak_Line_Line_KRPM_MECH_HDOP/(1+(0.732*STAR_CONNECTED_MOTOR_HDOP)))/1000)
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RPS_MECH_HDOP          (float)(MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RPM_MECH_HDOP * 60)
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_MECH_HDOP  (float)(MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RPS_MECH_HDOP/(2*M_PI))
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC_HDOP  (float)(MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_MECH_HDOP/NOPOLESPAIRS_HDOP)
#define     INVKFi_BELOW_BASE_SPEED_HDOP                               (float)(1/MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC_HDOP)
#define     MOTOR_PER_PHASE_INDUCTANCE_DIV_2_PI_HDOP                   ((float)(MOTOR_PER_PHASE_INDUCTANCE_HDOP/(2*M_PI)))
#define     NOMINAL_SPEED_RAD_PER_SEC_ELEC_HDOP                        (float)(((NOMINAL_SPEED_RPM_HDOP/60)*2*M_PI)*NOPOLESPAIRS_HDOP)   // Value in Electrical Radians/Sec
#define     MIN_WM_SPEED_SPEED_ELEC_RAD_PER_SEC_HDOP                   (float)(((MIN_WM_SPEED_IN_RPM/60)*2*M_PI)*NOPOLESPAIRS_HDOP)    // Value in Electrical Radians/Sec
#define     CLOSEDLOOP_RAMP_RATE_RADS_PER_SEC2_ELEC_HDOP               (float)(CLOSEDLOOP_RAMP_RATE_RADS_PER_SEC2_MECH*NOPOLESPAIRS_HDOP) // CLosed Loop  Speed Ramp rate in Electrical Radians/Sec^2
#define     FW_SPEED_RAD_PER_SEC_ELEC_HDOP                             (float)(((FW_SPEED_RPM_HDOP/60)*2*M_PI)*NOPOLESPAIRS_HDOP)
#ifdef ENABLE_FLUX_WEAKENING
    #define     POT_ADC_COUNT_FW_SPEED_RATIO_HDOP                      (float)(FW_SPEED_RAD_PER_SEC_ELEC_HDOP/MAX_ADC_COUNT)
#else
    #define     POT_ADC_COUNT_FW_SPEED_RATIO_HDOP                      (float)(NOMINAL_SPEED_RAD_PER_SEC_ELEC_HDOP/MAX_ADC_COUNT)
#endif
#define NOMINAL_ELE_VEL_RAD_PER_SEC_HDOP                               (float)(NOMINAL_SPEED_RPM_HDOP * (float)NOPOLESPAIRS_HDOP * RL_TWO_PI / 60.0f)
#define CTC_ELE_VEL_2_LOW_BOUND_HDOP                                   (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_HDOP * 0.4f)
#define CTC_ELE_VEL_1_HIGH_BOUND_HDOP                                  (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_HDOP * 0.1f)
#define CTC_ELE_VEL_1_LOW_BOUND_HDOP                                   (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_HDOP * 0.05f)
#define CTC_ELE_VEL_2_HIGH_BOUND_HDOP                                  (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_HDOP * 0.6f)
#define CLOSEDLOOP_SPEED_RAMP_RATE_DELTA_HDOP                          (float)(CLOSEDLOOP_RAMP_RATE_RADS_PER_SEC2_ELEC_HDOP*DELAY_MS*0.001) // CLosed Loop  Speed Ramp Rate in Electrical Radians/sec^2 in each control loop time
#define CLOSEDLOOP_SPEED_HYSTERESIS_HDOP                               (float)(5*CLOSEDLOOP_SPEED_RAMP_RATE_DELTA_HDOP)
#define SENSORED_TO_SENSORLESS_SWITCH_SPEED_HDOP                       (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_HDOP * 0.5f)
#define SENSORLESS_TO_SENSORED_SWITCH_SPEED_HDOP                       (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_HDOP * 0.4f)
#define TRAP_TO_FOC_SWITCH_SPEED_HDOP                                  (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_HDOP * 0.2f)
#define FOC_TO_TRAP_SWITCH_SPEED_HDOP                                  (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_HDOP * 0.0f)

//---------------Motor Specifications: Spartan Heavy Motor 123-0240-000
#define     MOTOR_PER_PHASE_RESISTANCE_SPARTANH                          ((float)0.0071)         // Per Phase Resistance in Ohms
#define     MOTOR_PER_PHASE_INDUCTANCE_SPARTANH                          ((float)0.0000195)      // Per Phase Inductance in Henrys
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_Line_Line_KRPM_MECH_SPARTANH   (float)6.3              // Back EMF Constant in Vpeak(L-L)/KRPM 
#define     NOPOLESPAIRS_SPARTANH                                        5                       // Number of Pole Pairs of the PMSM Motor     
#define     STAR_CONNECTED_MOTOR_SPARTANH                                1                       // 1 - Motor is Star Connected, 0 - Motor is Delta Connected
#define     NOMINAL_SPEED_RPM_SPARTANH                                   (float)3900             // Nominal Rated Speed of the Motor - Value in RPM
#define     FW_SPEED_RPM_SPARTANH                                        (float)4600             // Maximum Speed of the Motor in Flux Weakening Mode - Value in RPM
#define     MAX_FW_NEGATIVE_ID_REF_SPARTANH                              (float)(-10.0)          // Maximum negative D axis reference current (in A) during Flux Weakening
#define     MAX_MOTOR_CURRENT_SPARTANH                                   (float)(160)            // Maximum Motor Current in A *could be reduced to 120 maybe*
#define     MAX_MOTOR_CURRENT_DURING_STALL_SPARTANH                      (float)(160)            // Maximum Motor Current during stall condition

//*** Speed Control Loop Coefficients *****
#define     SPEEDCNTR_PTERM_SPARTANH                                     ((float)0.070)                  // Speed Loop Proportional Gain
#define     SPEEDCNTR_ITERM_SPARTANH                                     ((float)0.00003)              // Speed Loop Integral Gain
#define     SPEEDCNTR_CTERM_SPARTANH                                     (float)0.1                     // Speed Loop Anti-Windup Gain
#define     SPEEDCNTR_PTERM_DURING_STALL_SPARTANH                        ((float)0.150)                  // Speed Loop Proportional Gain
#define     SPEEDCNTR_ITERM_DURING_STALL_SPARTANH                        ((float)0.00003)              // Speed Loop Integral Gain
        
//*** Defines that use other motor specific values
#define     MAX_MOTOR_CURRENT_SQUARED_SPARTANH                             (float)((float)MAX_MOTOR_CURRENT_SPARTANH*(float)MAX_MOTOR_CURRENT_SPARTANH)
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RPM_MECH_SPARTANH          (float)((MOTOR_BACK_EMF_CONSTANT_Vpeak_Line_Line_KRPM_MECH_SPARTANH/(1+(0.732*STAR_CONNECTED_MOTOR_SPARTANH)))/1000)
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RPS_MECH_SPARTANH          (float)(MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RPM_MECH_SPARTANH * 60)
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_MECH_SPARTANH  (float)(MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RPS_MECH_SPARTANH/(2*M_PI))
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC_SPARTANH  (float)(MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_MECH_SPARTANH/NOPOLESPAIRS_SPARTANH)
#define     INVKFi_BELOW_BASE_SPEED_SPARTANH                               (float)(1/MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC_SPARTANH)
#define     MOTOR_PER_PHASE_INDUCTANCE_DIV_2_PI_SPARTANH                   ((float)(MOTOR_PER_PHASE_INDUCTANCE_SPARTANH/(2*M_PI)))
#define     NOMINAL_SPEED_RAD_PER_SEC_ELEC_SPARTANH                        (float)(((NOMINAL_SPEED_RPM_SPARTANH/60)*2*M_PI)*NOPOLESPAIRS_SPARTANH)   // Value in Electrical Radians/Sec
#define     MIN_WM_SPEED_SPEED_ELEC_RAD_PER_SEC_SPARTANH                   (float)(((MIN_WM_SPEED_IN_RPM/60)*2*M_PI)*NOPOLESPAIRS_SPARTANH)    // Value in Electrical Radians/Sec
#define     CLOSEDLOOP_RAMP_RATE_RADS_PER_SEC2_ELEC_SPARTANH               (float)(CLOSEDLOOP_RAMP_RATE_RADS_PER_SEC2_MECH*NOPOLESPAIRS_SPARTANH) // CLosed Loop  Speed Ramp rate in Electrical Radians/Sec^2
#define     FW_SPEED_RAD_PER_SEC_ELEC_SPARTANH                             (float)(((FW_SPEED_RPM_SPARTANH/60)*2*M_PI)*NOPOLESPAIRS_SPARTANH)
#ifdef ENABLE_FLUX_WEAKENING
    #define     POT_ADC_COUNT_FW_SPEED_RATIO_SPARTANH                      (float)(FW_SPEED_RAD_PER_SEC_ELEC_SPARTANH/MAX_ADC_COUNT)
#else
    #define     POT_ADC_COUNT_FW_SPEED_RATIO_SPARTANH                      (float)(NOMINAL_SPEED_RAD_PER_SEC_ELEC_SPARTANH/MAX_ADC_COUNT)
#endif
#define NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANH                               (float)(NOMINAL_SPEED_RPM_SPARTANH * (float)NOPOLESPAIRS_SPARTANH * RL_TWO_PI / 60.0f)
#define CTC_ELE_VEL_2_LOW_BOUND_SPARTANH                                   (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANH * 0.4f)
#define CTC_ELE_VEL_1_HIGH_BOUND_SPARTANH                                  (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANH * 0.1f)
#define CTC_ELE_VEL_1_LOW_BOUND_SPARTANH                                   (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANH * 0.05f)
#define CTC_ELE_VEL_2_HIGH_BOUND_SPARTANH                                  (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANH * 0.6f)
#define CLOSEDLOOP_SPEED_RAMP_RATE_DELTA_SPARTANH                          (float)(CLOSEDLOOP_RAMP_RATE_RADS_PER_SEC2_ELEC_SPARTANH*DELAY_MS*0.001) // CLosed Loop  Speed Ramp Rate in Electrical Radians/sec^2 in each control loop time
#define CLOSEDLOOP_SPEED_HYSTERESIS_SPARTANH                               (float)(5*CLOSEDLOOP_SPEED_RAMP_RATE_DELTA_SPARTANH)
#define SENSORED_TO_SENSORLESS_SWITCH_SPEED_SPARTANH                       (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANH * 0.5f)
#define SENSORLESS_TO_SENSORED_SWITCH_SPEED_SPARTANH                       (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANH * 0.4f)
#define TRAP_TO_FOC_SWITCH_SPEED_SPARTANH                                  (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANH * 0.2f)
#define FOC_TO_TRAP_SWITCH_SPEED_SPARTANH                                  (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANH * 0.0f)

//---------------Motor Specifications: Spartan Lite Motor 123-0245-000
#define     MOTOR_PER_PHASE_RESISTANCE_SPARTANL                          ((float)0.0294)         // Per Phase Resistance in Ohms
#define     MOTOR_PER_PHASE_INDUCTANCE_SPARTANL                          ((float)0.0000447)      // Per Phase Inductance in Henrys
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_Line_Line_KRPM_MECH_SPARTANL   (float)6.0              // Back EMF Constant in Vpeak(L-L)/KRPM
#define     NOPOLESPAIRS_SPARTANL                                        4                       // Number of Pole Pairs of the PMSM Motor     
#define     STAR_CONNECTED_MOTOR_SPARTANL                                1                       // 1 - Motor is Star Connected, 0 - Motor is Delta Connected
#define     NOMINAL_SPEED_RPM_SPARTANL                                   (float)4150             // Nominal Rated Speed of the Motor - Value in RPM
#define     FW_SPEED_RPM_SPARTANL                                        (float)4400             // Maximum Speed of the Motor in Flux Weakening Mode - Value in RPM
#define     MAX_FW_NEGATIVE_ID_REF_SPARTANL                              (float)(-15.0)          // Maximum negative D axis reference current (in A) during Flux Weakening
#define     MAX_MOTOR_CURRENT_SPARTANL                                   (float)(60)             // Maximum Motor Current in A
#define     MAX_MOTOR_CURRENT_DURING_STALL_SPARTANL                      (float)(120)            // Maximum Motor Current during stall condition

//*** Speed Control Loop Coefficients *****
#define     SPEEDCNTR_PTERM_SPARTANL                                     ((float)0.070)                   // Speed Loop Proportional Gain
#define     SPEEDCNTR_ITERM_SPARTANL                                     ((float)0.000003)              // Speed Loop Integral Gain
#define     SPEEDCNTR_CTERM_SPARTANL                                     (float)0.1                     // Speed Loop Anti-Windup Gain
#define     SPEEDCNTR_PTERM_DURING_STALL_SPARTANL                        ((float)0.170)                  // Speed Loop Proportional Gain
#define     SPEEDCNTR_ITERM_DURING_STALL_SPARTANL                        ((float)0.0003)              // Speed Loop Integral Gain

//*** Defines that use other motor specific values
#define     MAX_MOTOR_CURRENT_SQUARED_SPARTANL                             (float)((float)MAX_MOTOR_CURRENT_SPARTANL*(float)MAX_MOTOR_CURRENT_SPARTANL)
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RPM_MECH_SPARTANL          (float)((MOTOR_BACK_EMF_CONSTANT_Vpeak_Line_Line_KRPM_MECH_SPARTANL/(1+(0.732*STAR_CONNECTED_MOTOR_SPARTANL)))/1000)
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RPS_MECH_SPARTANL          (float)(MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RPM_MECH_SPARTANL * 60)
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_MECH_SPARTANL  (float)(MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RPS_MECH_SPARTANL/(2*M_PI))
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC_SPARTANL  (float)(MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_MECH_SPARTANL/NOPOLESPAIRS_SPARTANL)
#define     INVKFi_BELOW_BASE_SPEED_SPARTANL                               (float)(1/MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC_SPARTANL)
#define     MOTOR_PER_PHASE_INDUCTANCE_DIV_2_PI_SPARTANL                   ((float)(MOTOR_PER_PHASE_INDUCTANCE_SPARTANL/(2*M_PI)))
#define     NOMINAL_SPEED_RAD_PER_SEC_ELEC_SPARTANL                        (float)(((NOMINAL_SPEED_RPM_SPARTANL/60)*2*M_PI)*NOPOLESPAIRS_SPARTANL)   // Value in Electrical Radians/Sec
#define     MIN_WM_SPEED_SPEED_ELEC_RAD_PER_SEC_SPARTANL                   (float)(((MIN_WM_SPEED_IN_RPM/60)*2*M_PI)*NOPOLESPAIRS_SPARTANL)    // Value in Electrical Radians/Sec
#define     CLOSEDLOOP_RAMP_RATE_RADS_PER_SEC2_ELEC_SPARTANL               (float)(CLOSEDLOOP_RAMP_RATE_RADS_PER_SEC2_MECH*NOPOLESPAIRS_SPARTANL) // CLosed Loop  Speed Ramp rate in Electrical Radians/Sec^2
#define     FW_SPEED_RAD_PER_SEC_ELEC_SPARTANL                             (float)(((FW_SPEED_RPM_SPARTANL/60)*2*M_PI)*NOPOLESPAIRS_SPARTANL)
#ifdef ENABLE_FLUX_WEAKENING
    #define     POT_ADC_COUNT_FW_SPEED_RATIO_SPARTANL                      (float)(FW_SPEED_RAD_PER_SEC_ELEC_SPARTANL/MAX_ADC_COUNT)
#else
    #define     POT_ADC_COUNT_FW_SPEED_RATIO_SPARTANL                      (float)(NOMINAL_SPEED_RAD_PER_SEC_ELEC_SPARTANL/MAX_ADC_COUNT)
#endif
#define NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANL                               (float)(NOMINAL_SPEED_RPM_SPARTANL * (float)NOPOLESPAIRS_SPARTANL * RL_TWO_PI / 60.0f)
#define CTC_ELE_VEL_2_LOW_BOUND_SPARTANL                                   (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANL * 0.4f)
#define CTC_ELE_VEL_1_HIGH_BOUND_SPARTANL                                  (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANL * 0.1f)
#define CTC_ELE_VEL_1_LOW_BOUND_SPARTANL                                   (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANL * 0.05f)
#define CTC_ELE_VEL_2_HIGH_BOUND_SPARTANL                                  (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANL * 0.6f)
#define CLOSEDLOOP_SPEED_RAMP_RATE_DELTA_SPARTANL                          (float)(CLOSEDLOOP_RAMP_RATE_RADS_PER_SEC2_ELEC_SPARTANL*DELAY_MS*0.001) // CLosed Loop  Speed Ramp Rate in Electrical Radians/sec^2 in each control loop time
#define CLOSEDLOOP_SPEED_HYSTERESIS_SPARTANL                               (float)(5*CLOSEDLOOP_SPEED_RAMP_RATE_DELTA_SPARTANL)
#define SENSORED_TO_SENSORLESS_SWITCH_SPEED_SPARTANL                       (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANL * 0.5f)
#define SENSORLESS_TO_SENSORED_SWITCH_SPEED_SPARTANL                       (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANL * 0.4f)
#define TRAP_TO_FOC_SWITCH_SPEED_SPARTANL                                  (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANL * 0.2f)
#define FOC_TO_TRAP_SWITCH_SPEED_SPARTANL                                  (float)(NOMINAL_ELE_VEL_RAD_PER_SEC_SPARTANL * 0.0f)

// List of motors
enum
{
    INVALID,
    SPARTAN_HEAVY,
    SPARTAN_LITE,
    HD_OPERATOR,
    MAX_MOTOR_NUM
};

//--------------Motor Startup Behavior Configuration----------//
#define     ALIGN_TIME_IN_SEC                                   2                       // Duration of Motor Alignment in seconds
#define     OPENLOOP_RAMP_TIME_IN_SEC                           3                       // Ramp time to reach from 0 to Open Loop Speed in seconds

#if HALL_ANGLE_RUN || HALL_SENSORLESS_HYBRID_RUN
#define     OPENLOOP_END_SPEED_RPM                              100    
#else
#define     OPENLOOP_END_SPEED_RPM                              500                     // Speed at which the motor switches from open loop to closed loop in RPM
#endif

#define     CLOSEDLOOP_RAMP_RATE_RPM_SEC                        4625                    // Closed Loop Speed Ramp rate in Rev/min/Sec
#define     ALIGN_D_CURRENT_REF                                 0.4                     // Maximum Torque Reference during Motor Alignment in A
#define     OPENLOOP_D_CURRENT_REF                              0.4                     // Maximum Torque Reference during Open Loop Mode in A

#define     TORQUE_MODE_MAX_CUR                                 0.4                     // Maximum Torque Mode Current Reference in A
#define     WINDMILL_TIME_SEC                                   0.5                     // Duration of Motor Windmilling in seconds
#define     WINDMILL_START_Q_AXIS_REF                           0.4
#define     REGEN_BRAKE_CURRENT_REF                             0.4
#define     PASSIVE_BRAKE_TIME_IN_SEC                           2
#define     D_CURRENT_REF_FALL_TIME_SEC                         (float)(1.0)            // D axis Current Reference Fall Time in Seconds
#define     REGEN_BRAKE_CURRENT_RAMP_TIME_SEC                   (float)(1.0)
#define     MIN_WM_SPEED_IN_RPM                                 (float)(200)
/* PI controllers tuning values - */

//******** D Control Loop Coefficients *******
#define     D_CURRCNTR_PTERM                                    0.01                    // D axis Proportional Gain
#define     D_CURRCNTR_ITERM                                    (0.0005)                // D axis Integral Gain
#define     D_CURRCNTR_CTERM                                    0.5                     // D axis Anti-Windup Gain
#define     D_CURRCNTR_OUTMAX                                   1.3//0.999                   // D axis PI Controller Maximum Output - Max D axis Voltage (Normalized)

//******** Q Control Loop Coefficients *******
#define     Q_CURRCNTR_PTERM                                    0.01                    // Q axis Proportional Gain
#define     Q_CURRCNTR_ITERM                                    (0.0005)               // Q axis Integral Gain
#define     Q_CURRCNTR_CTERM                                    0.5                     // Q axis Anti-Windup Gain
#define     Q_CURRCNTR_OUTMAX                                   1.3//0.999                   // Q axis PI Controller Maximum Output - Max D axis Voltage (Normalized)

//*** Trapezoidal Speed Control Loop Coefficients *****
#define     TRAP_SPEEDCNTR_PTERM                                     (0.0001)                   // Speed Loop Proportional Gain
#define     TRAP_SPEEDCNTR_ITERM                                     (0.0000003)             // Speed Loop Integral Gain
#define     TRAP_SPEEDCNTR_CTERM                                     0.8                     // Speed Loop Anti-Windup Gain
#define     TRAP_SPEEDCNTR_OUTMAX                                    0.8       // Speed Loop PI Controller Maximum Output - Max Q axis Current Reference in A

/* motor dependent HALL constants */
#define     RL_30_HALL_JUMP_CW  0x0302U
#define     RL_30_HALL_JUMP_CCW  0x0203U
#define     RL_90_HALL_JUMP_CW  0x0206U
#define     RL_90_HALL_JUMP_CCW  0x0602U
#define     RL_150_HALL_JUMP_CW  0x0604U
#define     RL_150_HALL_JUMP_CCW  0x0406U
#define     RL_210_HALL_JUMP_CW  0x0405U
#define     RL_210_HALL_JUMP_CCW  0x0504U
#define     RL_270_HALL_JUMP_CW  0x0501U
#define     RL_270_HALL_JUMP_CCW  0x0105U
#define     RL_330_HALL_JUMP_CW  0x0103U
#define     RL_330_HALL_JUMP_CCW  0x0301U


//--------------PLL Estimator Configuration----------//
#define     KFILTER_ESDQ                (float)((float)400/(float)32767)                // D,Q Axis BEMF Filter Coefficient
#define     KFILTER_VELESTIM            (float)((float)(374)/(float)32767)              // PLL Estimator Speed Filter Coefficient
#define     KFILTER_IDREF               (float)((float)(10)/(float)32767)               // Flux Weakening mode - D axis Current Reference Filter

/* HALL constants */
#define RL_ONE_OVER_SIX_PI (float)(1.0 / 6.0 * M_PI)
#define RL_ONE_OVER_TWO_PI (float)(0.5 * M_PI)
#define RL_FIVE_OVER_SIX_PI (float)(5.0 / 6.0 * M_PI)
#define RL_SEVEN_OVER_SIX_PI (float)(7.0 / 6.0 * M_PI)
#define RL_THREE_OVER_TWO_PI (float)(1.5 * M_PI)
#define RL_ELEVEN_OVER_SIX_PI (float)(11.0 / 6.0 * M_PI)
/**/
#define RL_ONE_OVER_THREE_PI (float)(1.0 / 3.0 * M_PI)
#define RL_TWO_OVER_THREE_PI (float)(2.0 / 3.0 * M_PI)
#define RL_PI (float)(M_PI)
#define RL_FOUR_OVER_THREE_PI (float)(4.0 / 3.0 * M_PI)
#define RL_FIVE_OVER_THREE_PI (float)(5.0 / 3.0 * M_PI)
#define RL_TWO_PI (float)(2.0 * M_PI)
#define RL_DUMM (0.0f)
#define RL_ONE_OVER_TWELVE_PI (float)(1.0 / 12.0 * M_PI)    // 15 degrees - used for angle iterator method for increasing locked rotor torque
/**/
#define RL_TC0_FREQ (30000000.0f)
#define RL_TCO_TS (float)(1.0 / RL_TC0_FREQ)
#define RL_2PI_TC0_FREQ (float)(RL_TWO_PI * RL_TC0_FREQ)


/**/
#define RL_LPF_BW 200.0f
#define MC_ISR_T (float)(1.0f / (float)PWM_FREQ)
#define RL_LPF_COEFF_2 (float)(RL_LPF_BW * MC_ISR_T)
#define RL_LPF_COEFF_1 (float)(1.0 - RL_LPF_COEFF_2)

#define SPEED_STEP_ON_TIME 2.5
#define SPEED_STEP_PERIOD   5
#define SPEED_STEP_ON_TIME_COUNT  (uint32_t) (SPEED_STEP_ON_TIME*100)
#define SPEED_STEP_PERIOD_COUNT  (uint32_t) (SPEED_STEP_PERIOD*100)
#define SPEED_STEP_SIZE_ELEC_RAD_PER_SEC    300

//#define SPEED_PI_TUNING
#define SENSORLESS_MODE_ENABLE 6u // value of 6 was chosen because while plotting on X2CScope, it's amplitude matches the max rotor angle in radians i.e. 6.28. Hence no scaling is needed
#define SENSORLESS_MODE_DISABLE 0u
#define TRAP_MODE_ENABLE 5u
#define TRAP_MODE_DISABLE 0u
// <editor-fold defaultstate="collapsed" desc=" Derived Macros from Motor Control Board Specifications, Motor Specifications and Motor Dyanmics">

#define     PWM_PERIOD_COUNT                                    (((PWM_CLK/PWM_FREQ)/2))
#define     PWM_HALF_PERIOD_COUNT                               PWM_PERIOD_COUNT>>1

#define     VREF_DAC_VALUE                                      (int) 2048
#define     ADC_PHASE_CURRENT_SCALE                             (float)(MAX_BOARD_CURRENT/(float)2048)
#define     MAX_MEASURED_BUS_CURRENT                            (float) 95.65   // 3.3V on ADC pin represents this amoutn of current (in Amps)
#define     ADC_BUS_CURRENT_SCALE                               (float) (MAX_MEASURED_BUS_CURRENT/MAX_ADC_COUNT)
//#define     CURRENT_LIMIT_CMP_REF                               (int)(((float)2048*(MAX_MOTOR_CURRENT/MAX_BOARD_CURRENT))+VREF_DAC_VALUE)

#define     MAX_ADC_COUNT                                       (float)4095     // for 12-bit ADC
#define     MAX_ADC_INPUT_VOLTAGE                               (float)3.3      // volts
#define     ADC_VOLTS_TO_BITS                                   (float)(MAX_ADC_COUNT / MAX_ADC_INPUT_VOLTAGE) //used to convert ADC volts to bits
#define     DCBUS_SENSE_RATIO                                   (float)(DCBUS_SENSE_BOTTOM_RESISTOR/(DCBUS_SENSE_BOTTOM_RESISTOR + DCBUS_SENSE_TOP_RESISTOR))
#define     VOLTAGE_ADC_TO_PHY_RATIO                            (float)(MAX_ADC_INPUT_VOLTAGE/(MAX_ADC_COUNT * DCBUS_SENSE_RATIO))
#define     SINGLE_ELEC_ROT_RADS_PER_SEC                        (float)(2*M_PI)
#define     MAX_DUTY                                            (PWM_PERIOD_COUNT)
#define     LOOPTIME_SEC                                        (float)(1/((float)PWM_FREQ))           // PWM Period - 50 uSec, 20Khz PWM
#define     COUNT_FOR_ALIGN_TIME                                (unsigned int)((float)ALIGN_TIME_IN_SEC/(float)LOOPTIME_SEC)
#define     COUNT_FOR_WINDMILLING_TIME                          (unsigned int)((float)WINDMILL_TIME_SEC/(float)LOOPTIME_SEC)
#define     COUNT_FOR_PASSIVE_BRAKE_TIME                        (unsigned int)((float)PASSIVE_BRAKE_TIME_IN_SEC/(float)LOOPTIME_SEC)
#define     ALIGN_CURRENT_STEP                                  (float)(2*ALIGN_D_CURRENT_REF/COUNT_FOR_ALIGN_TIME) // Current reference during aligning is ramped up for 50% of align time.
#define     OPENLOOP_END_SPEED_RPS                              ((float)OPENLOOP_END_SPEED_RPM/60)
#define     OPENLOOP_END_SPEED_RADS_PER_SEC_MECH                (float)(OPENLOOP_END_SPEED_RPS * SINGLE_ELEC_ROT_RADS_PER_SEC)
#define     OPENLOOP_END_SPEED_RADS_PER_SEC_ELEC                (float)(OPENLOOP_END_SPEED_RADS_PER_SEC_MECH * NOPOLESPAIRS)
#define     OPENLOOP_END_SPEED_RADS_PER_SEC_ELEC_IN_LOOPTIME    (float)(OPENLOOP_END_SPEED_RADS_PER_SEC_ELEC * LOOPTIME_SEC)
#define     OPENLOOP_RAMPSPEED_INCREASERATE                     (float)(OPENLOOP_END_SPEED_RADS_PER_SEC_ELEC_IN_LOOPTIME/(OPENLOOP_RAMP_TIME_IN_SEC/LOOPTIME_SEC))
#define     CLOSEDLOOP_RAMP_RATE_RPS_SEC                        ((float)CLOSEDLOOP_RAMP_RATE_RPM_SEC/60) // CLosed Loop  Speed Ramp rate in Rev/sec^2 
#define     CLOSEDLOOP_RAMP_RATE_RADS_PER_SEC2_MECH             (float)(CLOSEDLOOP_RAMP_RATE_RPS_SEC*2*M_PI) // CLosed Loop  Speed Ramp Rate in Mechanical Radians/Sec^2



#define     DELAY_10MS_COUNT                                     (float)(PWM_FREQ*DELAY_MS*(float)0.001)
#define     SW_DEBOUNCE_DLY_500MS                                (uint32_t)(SW_DEBOUNCE_DLY_MS/DELAY_MS)  // Switch debounce duration in multiple of 10mS
#define     RHO_OFFSET_ELEC_RAD                                  (float)(RHO_OFFSET_ELEC_DEG*M_PI/180)
#define     MIN_RHO_OFFSET_ELEC_RAD                              (float)(RHO_OFFSET_ELEC_RAD/PWM_FREQ)

#define     MAX_NORM                                            (float) 0.95
#define     MAX_NORM_SQ                                         (float) (MAX_NORM*MAX_NORM)
#define     TORQUE_MODE_POT_ADC_RATIO                           (float) (TORQUE_MODE_MAX_CUR/MAX_ADC_COUNT)

#define     STALL_DETECT_RPM_THRESHOLD                          (float) 200     // Speed threshold for detecting a stall
#define     STALL_DETECT_CURRENT_THRESHOLD_RATIO                (float) 0.4     // Percentage of MAX_MOTOR_CURRENT used as current threshold for detecting a stall
#define     STALL_DETECT_CURRENT_REDUCTION_RATIO                (float) 0.5     // After stall is detected, reduce speed controller output by this percentage
#define     TIMER_STALL_DETECT_MS                               (float) 3000    // After this long, shut down drive and
                                                                                // report a stall fault
#define     TIMER_STALL_DETECT_PERIOD                            (float) (TIMER_STALL_DETECT_MS / DELAY_MS)
#define     TIMER_STALL_DETECT_DELAY_MS                         (float) 390 // Allow max phase current during a stall for 
                                                                            // this long before reducing current limit
#define     TIMER_STALL_DETECT_DELAY                            (float) (TIMER_STALL_DETECT_DELAY_MS / DELAY_MS)
#define     TIMER_STALL_DETECT_ANGLE_ITERATE_INTERVAL_MS        (float) 130 
#define     TIMER_STALL_DETECT_ANGLE_ITERATE_INTERVAL           (uint16_t) (TIMER_STALL_DETECT_ANGLE_ITERATE_INTERVAL_MS / DELAY_MS)

#define     HALL_UNCHANGED_SO_ASSUME_ZERO_SPEED                 1000000

#define     RAPID_SPEED_INPUT_DELTA_THRESHOLD                   1000        // Threshold for overriding speed ramp limit
#define     SPEED_RAMP_OVERRIDE_FACTOR                          3           // When overriding the speed ramp limit, use this factor

//#define     DECIMATE_NOMINAL_SPEED                              ((NOMINAL_SPEED_RPM *(M_PI/30))*NOPOLESPAIRS/10)
#ifdef      CURPI_TUN
    #define CPT_CNT_VAL   (float)(CUR_STEP_TIM*PWM_FREQ)
#endif 

#define     D_CURRENT_REF_STEP                  (float)(ALIGN_D_CURRENT_REF/(D_CURRENT_REF_FALL_TIME_SEC*PWM_FREQ))
#define     REGEN_BRAKE_CURRENT_STEP            (float)(REGEN_BRAKE_CURRENT_REF/(REGEN_BRAKE_CURRENT_RAMP_TIME_SEC*PWM_FREQ))


#define     DCBUS_DUMP_RESISTOR_VOLTAGE_THRESHOLD       (float) 50.0    // Value of DC bus voltage to turn on the dump resistor
#define     SPEED_INPUT_IGNORE_RISING_THRESHOLD_VOLTS   (float) 0.16   // Voltage where a stopped motor will start moving (Spec says ignore under 0.3V out of 5V)
#define     SPEED_INPUT_IGNORE_FALLING_THRESHOLD_VOLTS  (float) 0.12     // Voltage where a spinning motor will come to a stop
#define     SPEED_INPUT_IGNORE_RISING_THRESHOLD_COUNTS  (float)((SPEED_INPUT_IGNORE_RISING_THRESHOLD_VOLTS / MAX_ADC_INPUT_VOLTAGE) * MAX_ADC_COUNT)
#define     SPEED_INPUT_IGNORE_FALLING_THRESHOLD_COUNTS (float)((SPEED_INPUT_IGNORE_FALLING_THRESHOLD_VOLTS / MAX_ADC_INPUT_VOLTAGE) * MAX_ADC_COUNT)

//--------------Fault Detection Thresholds----------//
#define     DCBUS_UNDERVOLTAGE_THRESHOLD_VOLTS      (float) (14.0-0.3)    // Threshold for undervoltage fault (including .3 schottky diode drop)
#define     DCBUS_UNDERVOLTAGE_RECOVERY_VOLTS       (float) (18.0)        // Threshold for recovery from undervoltage fault
#define     SHORTED_FET_CURRENT_THRESHOLD_AMPS      (float) 8.0     // Threshold for shorted FET fault
#define     SHORTED_FET_CURRENT_THRESHOLD_COUNTS    (float) (SHORTED_FET_CURRENT_THRESHOLD_AMPS / ADC_PHASE_CURRENT_SCALE)     // Threshold for shorted FET fault
#define     PHASE_CURNT_OFFSET_MAX_THRESHOLD_COUNTS (float) 2110    // Max threshold for phase current opamp fault
#define     PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS (float) 1986    // Min threshold for phase current opamp fault
#define     BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS   (float) 490     // Max threshold for bus current opamp fault
#define     BUS_CURNT_OFFSET_MIN_THRESHOLD_COUNTS   (float) 342     // Min threshold for bus current opamp fault
#define     FET_TEMPERATURE_MAX_THRESHOLD_DEGC      (float) 150     // Max threshold for FET temperature fault
#define     FET_TEMPERATURE_RECOVERY_DEGC           (float) 130     // Threshold for recovery from FET temperature fault

//--------------Other----------//
#define     LOW_POWER_MODE_DELAY_TIME_MS            1000                                        // Time delay (in ms) before entering low power mode
#define     LOW_POWER_MODE_DELAY_TIME_COUNTS        LOW_POWER_MODE_DELAY_TIME_MS / DELAY_MS     // Time delay in number of counts

#endif
 
// </editor-fold>

