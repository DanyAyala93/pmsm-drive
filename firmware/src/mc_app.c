/** @file mc_app.c
 * @brief Motor Control Application Variable and Function definitions.
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

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END

#include "userparams.h"
#include <math.h>
#include "mc_app.h"
#include "mc_Lib.h"
#include "definitions.h"
#include "microcontroller.h"
#include "thermistors.h"
#include "PowerControl.h"
#include "component/tcc.h"
#include "board_id.h"


#define DUMP_RESISTOR_TIME_LIMIT_TOGGLE_COUNT      10  //10 counts * 10 ms = 100ms - time limit for how long dump resistor can stay on continuously, toggle every 10 counts
#define DUMP_RESISTOR_TIME_LIMIT_MAX_COUNT         500 // 5 seconds / 10ms = 500 counts - resistor rated for 50W for 5 seconds so turn off completely after 5s
#define MOTOR_ENCODER_POWERUP_DELAY_US      14000   // Delay time needed for encoder IC after power is applied
                                                    // Min delay required: 9ms (by experimentation)

uint16_t patt_reg_cw_lut[8u] = {
    0x0077, /*Invalid Hall Code - Disable all PWM*/
    0x0467, /*1: 300 degree */
    0x0157, /*2: 60 degree */
    0x0457, /*3: 0 degree */
    0x0237, /*4: 180 degree   */
    0x0267, /*5: 240 degree  */
    0x0137, /*6: 120 degree */
    0x0077  /*7: Invalid Hall Code - Disable all PWM*/            
};

uint16_t patt_reg_ccw_lut[8u] = {
    0x0077, /*Invalid Hall Code - Disable all PWM*/
    0x0137, /*1: 300 degree */
    0x0267, /*2: 60 degree */
    0x0237, /*3: 0 degree */
    0x0457, /*4: 180 degree   */
    0x0157, /*5: 240 degree  */
    0x0467, /*6: 120 degree */
    0x0077  /*7: Invalid Hall Code - Disable all PWM*/
};

typedef struct {
    uint32_t inputVal; /* read value of button input pin */
    uint16_t state;
    uint32_t cnt;
} button_response_t;

typedef struct {
    char windmilling;
    char windmilling_decide;
    char passive_brake;
    char regen_brake;
    char align;
    char open_loop;
    char open_loop_to_closed_loop;
    char closed_loop;
}state_machine_status_t;

uint8_t trigger = 0;
float align_rotor_angle_degrees =0;
float align_rotor_angle_radians =0;
float trap_speed_input = 0.1;
uint32_t trap_duty = PWM_HALF_PERIOD_COUNT;

mcParam_PIController    mcApp_Q_PIParam;      // Parameters for Q axis Current PI Controller 
mcParam_PIController    mcApp_D_PIParam;      // Parameters for D axis Current PI Controller 
mcParam_PIController    mcApp_Speed_PIParam;  // Parameters for Speed PI Controller 
mcParam_PIController    mcApp_Trap_Speed_PIParam;  // Parameters for Trap Speed PI Controller 
mcParam_FOC             mcApp_focParam;       // Parameters related to Field Oriented Control
mcParam_SinCos          mcApp_SincosParam;    // Parameters related to Sine/Cosine calculator
mcParam_SVPWM           mcApp_SVGenParam;     // Parameters related to Space Vector PWM
mcParam_ControlRef      mcApp_ControlParam;   // Parameters related to Current and Speed references
mcParam_PLLEstimator    mcApp_EstimParam;     // Parameters related to PLL Estimator
mcParam_AlphaBeta       mcApp_I_AlphaBetaParam; // Alpha and Beta (2 Phase Stationary Frame) axis Current values
mcParam_AlphaBeta       mcApp_IRef_AlphaBetaParam; // Alpha and Beta (2 Phase Stationary Frame) axis Current Reference values
mcParam_DQ              mcApp_I_DQParam;// D and Q axis (2 Phase Rotating Frame) current values
mcParam_DQ              mcApp_IRef_DQParam; // D and Q axis (2 Phase Rotating Frame) Current Reference Values
mcParam_ABC             mcApp_I_ABCParam; // A,B,C axis (3 Phase Stationary Frame) current values
mcParam_AlphaBeta       mcApp_V_AlphaBetaParam; // Alpha and Beta (2 Phase Stationary Frame) axis voltage values
mcParam_DQ              mcApp_V_DQParam;// D and Q axis (2 Phase Rotating Frame) voltage values
motor_status_t          mcApp_motorState;
delay_gen_t             delay_10ms;
state_machine_status_t  motor_state_machine;
float                   OpenLoop_Ramp_Angle_Rads_Per_Sec = 0;    // ramp angle variable for initial ramp 
uint32_t                Align_Counter = 0;             // lock variable for initial ramp 
unsigned int            Windmilling_Counter = 0;
unsigned int            Passive_Brake_Counter = 0;
short                   speedCommand;
float                   speedIgnoreThreshold = SPEED_INPUT_IGNORE_RISING_THRESHOLD_COUNTS;
int16_t                 phaseCurrentA;
int16_t                 phaseCurrentB;
float                   DoControl_Temp1;
float                   DoControl_Temp2;

float                   d_intermediate;
mcParam_DQ              I_dq_old,I_dq_new;
float                   ol_angle,cl_angle;

uint32_t                curpi_counter = 0;
uint32_t                speed_step_counter = 0;

char                    state_count = 0;
float                   trap_mod_modulation_index;

uint8_t                 secondaryAdcSequenceNumber = 0;

uint8_t                 motorStallDirection = 0;

uint8_t                 motorIsMoving_f = 0; //used to indicate motor is moving for check in main

//ADC variables, declared in adc.h
uint16_t                adc_0_offset = 0;
uint16_t                adc_1_offset = 0;
volatile uint16_t       adc_bus_current_offset = 0;

//Local Function prototypes
void mcApp_inverterEnable();
void PWM_Output_Enable();
static void rotorAngleEstimate(tagHALLdata * ptData);

//global function prototypes
void delay_us(uint16_t time_us);

//Function definitions
void mcApp_SpeedRamp() //called every 10ms
{
#ifdef SPEED_PI_TUNING
    if(mcApp_motorState.motorDirection ==0)
    {
        if(speed_step_counter < SPEED_STEP_ON_TIME_COUNT)
        {
            mcApp_ControlParam.VelInput = (float)((float)speedCommand * motorParams[Motor_ID_num].POT_ADC_COUNT_FW_SPEED_RATIO) + SPEED_STEP_SIZE_ELEC_RAD_PER_SEC;
        }
        else
        {
            mcApp_ControlParam.VelInput = (float)((float)speedCommand * motorParams[Motor_ID_num].POT_ADC_COUNT_FW_SPEED_RATIO);
        }
    }
    else
    {
        if(speed_step_counter < SPEED_STEP_ON_TIME_COUNT)
        {
            mcApp_ControlParam.VelInput = (float)((float)-speedCommand * motorParams[Motor_ID_num].POT_ADC_COUNT_FW_SPEED_RATIO) - SPEED_STEP_SIZE_ELEC_RAD_PER_SEC;
        }
        else
        {
            mcApp_ControlParam.VelInput = (float)((float)-speedCommand * motorParams[Motor_ID_num].POT_ADC_COUNT_FW_SPEED_RATIO);
        }
    }
#else

    motorIsMoving_f = ( (mcApp_Speed_PIParam.qInMeas > 10) || (mcApp_Speed_PIParam.qInMeas < -10) );
    static uint16_t motorStoppedTimer = 0;
    
    //check qInMeas to determine if startup was successful/ is finished
    if (((mcApp_Speed_PIParam.qInMeas > STALL_DETECT_RPM_THRESHOLD) || (mcApp_Speed_PIParam.qInMeas < -STALL_DETECT_RPM_THRESHOLD)) 
            && mcApp_motorState.startup_f)
    {
        mcApp_motorState.startup_f = false; 
    }

    if(mcApp_motorState.motorDirection == 0)
    {
    // Set speedIgnoreThreshold to use for the speed input
        if (mcApp_Speed_PIParam.qInMeas <= (SPEED_INPUT_IGNORE_FALLING_THRESHOLD_COUNTS * motorParams[Motor_ID_num].POT_ADC_COUNT_FW_SPEED_RATIO))
        {
            speedIgnoreThreshold = SPEED_INPUT_IGNORE_RISING_THRESHOLD_COUNTS;
        }
        else if (mcApp_Speed_PIParam.qInMeas > (SPEED_INPUT_IGNORE_RISING_THRESHOLD_COUNTS * motorParams[Motor_ID_num].POT_ADC_COUNT_FW_SPEED_RATIO))
        {
            speedIgnoreThreshold = SPEED_INPUT_IGNORE_FALLING_THRESHOLD_COUNTS;
        }

    // Check speedCommand against speedIgnoreThreshold
        if (speedCommand < speedIgnoreThreshold)
        {
            mcApp_ControlParam.VelInput = 0;
        }
        else
        {
            mcApp_ControlParam.VelInput = ((float)speedCommand * motorParams[Motor_ID_num].POT_ADC_COUNT_FW_SPEED_RATIO);
        }
    }
    else
    {
    // Set the active threshold to use for the speed input
        if (-mcApp_Speed_PIParam.qInMeas <= (SPEED_INPUT_IGNORE_FALLING_THRESHOLD_COUNTS * motorParams[Motor_ID_num].POT_ADC_COUNT_FW_SPEED_RATIO))
        {
            speedIgnoreThreshold = SPEED_INPUT_IGNORE_RISING_THRESHOLD_COUNTS;
        }
        else if (-mcApp_Speed_PIParam.qInMeas > (SPEED_INPUT_IGNORE_RISING_THRESHOLD_COUNTS * motorParams[Motor_ID_num].POT_ADC_COUNT_FW_SPEED_RATIO))
        {
            speedIgnoreThreshold = SPEED_INPUT_IGNORE_FALLING_THRESHOLD_COUNTS;
        }

    // Check speedCommand against the active threshold
        if (speedCommand < speedIgnoreThreshold)
        {
            mcApp_ControlParam.VelInput = 0;
        }
        else
        {
            mcApp_ControlParam.VelInput = ((float)-speedCommand * motorParams[Motor_ID_num].POT_ADC_COUNT_FW_SPEED_RATIO);
        }
    }

    if (mcApp_motorState.motorDirection != Motor_Direction_Get())
    {
        if (motorIsMoving_f)
        {
           mcApp_motorState.motorForceStop = 1;
        }
        else
        {
           mcApp_inverterDisable(); //**inverter should be disabled always before changing direction**
           //before this line was added, if direction changed without disabling inverter, current measurement
           //sign change from + to - or vice versa caused overcurrent protection to set output to max erroneously.
           //This issue has been resolved by added the above line. Backup measures are also in place
           mcApp_motorState.motorDirection = !mcApp_motorState.motorDirection;
        }
    }

// Enable or disable the inverter if needed
    if (mcApp_motorState.inverterRunning)
    {
        if (mcApp_motorState.motorForceStop)
        {
            if (motorIsMoving_f)
            {
                mcApp_ControlParam.VelInput = 0;   // Force the speed to ramp down to zero
            }
            else
            {
                mcApp_inverterDisable(); //**inverter should be disabled always before changing direction**

                // Change direction if needed.  This is needed in case a direction change set the
                //   motorForceStop flag and we are changing directions quickly (before speedRamp
                //   is called again).
                if (mcApp_motorState.motorDirection != Motor_Direction_Get())
                {
                    mcApp_motorState.motorDirection = !mcApp_motorState.motorDirection;
                }
                mcApp_motorState.motorForceStop = 0;    // Clear the flag since the motor has stopped
            }
        }
        else if (1 == Motor_Enable_Get()) //not enabled and not force stop (Enable line is active-low at MCU pin)
        {
            if (!motorIsMoving_f)
            {
                mcApp_inverterDisable();
            }
            else
            {
                mcApp_motorState.motorForceStop = 1;
            }
            
        }
        else if ((0 == mcApp_ControlParam.VelInput) && (!motorIsMoving_f))
        {
            // Only disable inverter if motor has been stopped for some time
            //TODO Olivia review - timing and magic number
            if (motorStoppedTimer < 5) //times 10 ms
            {
                motorStoppedTimer++;
            }
            else
            {
                motorStoppedTimer = 0;
                mcApp_inverterDisable();
            } 
        }
    }
    else
    {// Inverter is not running
        if ( (0 == Motor_Enable_Get()) && (mcApp_ControlParam.VelInput != 0))   // (Enable line is active-low at MCU pin)
             
        {
            // Make sure the hall value is valid before enabling the inverter
            if (CheckIsHallValueValid())
            {
                // Hall value is valid, so enable the inverter
                mcApp_inverterEnable();
            }
            else
            {
                // Hall value is invalid, so go to fault handler
                fh_faultHandler(HALL_SENSOR_FAULT);
            }
            
            
            
        }
    }

#endif

    mcApp_ControlParam.Diff =  mcApp_ControlParam.VelInput - mcApp_ControlParam.VelRef;
      
    //Speed Rate Limiter implementation.
    if (mcApp_ControlParam.Diff > RAPID_SPEED_INPUT_DELTA_THRESHOLD)
    {
      // Rapid VelInput increase overrides normal ramp rate limit
      mcApp_ControlParam.VelRef+= ( motorParams[Motor_ID_num].CLOSEDLOOP_SPEED_RAMP_RATE_DELTA * SPEED_RAMP_OVERRIDE_FACTOR );
    }
    else if (mcApp_ControlParam.Diff < -RAPID_SPEED_INPUT_DELTA_THRESHOLD)
    {
      // Rapid VelInput increase overrides normal ramp rate limit
      mcApp_ControlParam.VelRef-= ( motorParams[Motor_ID_num].CLOSEDLOOP_SPEED_RAMP_RATE_DELTA * SPEED_RAMP_OVERRIDE_FACTOR);
    } 
    else if(mcApp_ControlParam.Diff >= motorParams[Motor_ID_num].CLOSEDLOOP_SPEED_HYSTERESIS)
    {
      mcApp_ControlParam.VelRef += motorParams[Motor_ID_num].CLOSEDLOOP_SPEED_RAMP_RATE_DELTA;   

    }
    else if(mcApp_ControlParam.Diff <=-motorParams[Motor_ID_num].CLOSEDLOOP_SPEED_HYSTERESIS)
    {
      mcApp_ControlParam.VelRef -= motorParams[Motor_ID_num].CLOSEDLOOP_SPEED_RAMP_RATE_DELTA; 
    }
    else
    {
      mcApp_ControlParam.VelRef = mcApp_ControlParam.VelInput;
    }
        
#if (!HALL_ANGLE_RUN && !TRAPEZOIDAL_CONTROL && !HALL_SENSORLESS_HYBRID_RUN)
    if(mcApp_motorState.motorDirection ==0)
    {
        if(mcApp_ControlParam.VelRef < OPENLOOP_END_SPEED_RADS_PER_SEC_ELEC)
        {
            mcApp_ControlParam.VelRef = OPENLOOP_END_SPEED_RADS_PER_SEC_ELEC;
        }
        else
        {
            
        }
    }
    else
    {
        if(mcApp_ControlParam.VelRef > -OPENLOOP_END_SPEED_RADS_PER_SEC_ELEC)
        {
            mcApp_ControlParam.VelRef = -OPENLOOP_END_SPEED_RADS_PER_SEC_ELEC;
        } 
        else
        {
            
        }
    }
#endif

#ifdef SPEED_PI_TUNING
    speed_step_counter++;
    if(speed_step_counter> SPEED_STEP_PERIOD_COUNT)
    {
        speed_step_counter = 0;
    }
#endif
        
}

inline static void PWM_Output_Disable()
{
    #if TRAPEZOIDAL_CONTROL
    // For trap control, set all PWMs to 0%
    // NOTE: The low side FETs are always overridden by the pattern register.
    //       Set the high side FETs to 0%
        TCC1_PWM24bitDutySet(TCC1_CHANNEL0,(uint32_t) 0 );  // U
        TCC1_PWM24bitDutySet(TCC1_CHANNEL1,(uint32_t) 0 );  // V
        TCC1_PWM24bitDutySet(TCC1_CHANNEL2,(uint32_t) 0 );  // W
    #else
    // For FOC, set all PWMs to 50%
    // With all 3 phases at the same duty, no current flows through windings
        TCC1_PWM24bitDutySet(TCC1_CHANNEL0,(uint32_t) PWM_HALF_PERIOD_COUNT );  // U
        TCC1_PWM24bitDutySet(TCC1_CHANNEL1,(uint32_t) PWM_HALF_PERIOD_COUNT );  // V
        TCC1_PWM24bitDutySet(TCC1_CHANNEL2,(uint32_t) PWM_HALF_PERIOD_COUNT );  // W
    #endif //TRAPEZOIDAL_CONTROL

    // Override all PWM outputs to low
    // NOTE: This is an asynchronous PWMPatternSet call so that PWMs are disabled immediately (without
    //   waiting for the next PWM period)
    TCC1_PWMForceUpdate();
    TCC1_PWMPatternSet(
        (TCC_PATT_PGE0_Msk|TCC_PATT_PGE1_Msk|TCC_PATT_PGE2_Msk|TCC_PATT_PGE4_Msk|TCC_PATT_PGE5_Msk|TCC_PATT_PGE6_Msk),
        (TCC_PATT_PGE0(0)|TCC_PATT_PGE1(0)|TCC_PATT_PGE2(0)|TCC_PATT_PGE4(0)|TCC_PATT_PGE5(0)|TCC_PATT_PGE6(0)));
    TCC1_PWMForceUpdate();
}

void PWM_Output_Enable()
{
    #if TRAPEZOIDAL_CONTROL
    // For trap control, set all PWMs to 0%
    // NOTE: The low side FETs are always overridden by the pattern register.
    //       Set the high side FETs to 0%
        TCC1_PWM24bitDutySet(TCC1_CHANNEL0,(uint32_t) 0 );  // U
        TCC1_PWM24bitDutySet(TCC1_CHANNEL1,(uint32_t) 0 );  // V
        TCC1_PWM24bitDutySet(TCC1_CHANNEL2,(uint32_t) 0 );  // W
    #else
    // For FOC, set all PWMs to 50%
    // With all 3 phases at the same duty, no current flows through windings
        TCC1_PWM24bitDutySet(TCC1_CHANNEL0,(uint32_t) PWM_HALF_PERIOD_COUNT );  // U
        TCC1_PWM24bitDutySet(TCC1_CHANNEL1,(uint32_t) PWM_HALF_PERIOD_COUNT );  // V
        TCC1_PWM24bitDutySet(TCC1_CHANNEL2,(uint32_t) PWM_HALF_PERIOD_COUNT );  // W
    #endif //TRAPEZOIDAL_CONTROL

    TCC1_PWMForceUpdate();
    TCC1_PWMPatternSet(0x00,0x00);/*Disable PWM override*/
    TCC1_PWMForceUpdate();
}

void mcApp_DriveInitialize()
{
// Fault status line
    drive_fault_status.faultCode = IDLE;    // Start with IDLE
    //Debug_LED_Set();    // Start with LED on
    TC4_TimerStart();   // Start drive fault output comm to main board


// Current measurement calibration
    delay_us(5000); // Allow phase current outputs to settle before measuring
    ADC0_ChannelSelect(U_CURRENT_ADC_POSINPUT,ADC_NEGINPUT_GND); // Select Phase U for calibration
    ADC1_ChannelSelect(V_CURRENT_ADC_POSINPUT,ADC_NEGINPUT_GND); // Select Phase V for calibration
    ADC0_CallbackRegister((ADC_CALLBACK) ADC_CALIB_IPHASE_ISR, (uintptr_t)NULL);

//    EIC_CallbackRegister ((EIC_PIN)EIC_PIN_2, (EIC_CALLBACK) OC_FAULT_ISR,(uintptr_t)NULL); // Overcurrent fault trigger

    TCC1_PWMStart();  // U, V, and W phase PWMs
    ADC0_Enable();  // Start calibration
    X2CScope_Init();
    
    delay_us(MOTOR_ENCODER_POWERUP_DELAY_US);    // For motors with a position sensor IC instead of simple hall sensors,
                                                 // a delay here is needed to allow hall signals to come up properly.
    hall_sensor_init();

    TC2_CaptureStart(); // Hall input
    
}

static inline void lowSpeedCorrect(tagHALLdata * ptData)
{
    float vel, tcMCisr, tcHALLisr;
    
    if((STT_HALL_ISR_ONE_JUMP_LOW_SPEED != ptData->stateLth) || (0u == ptData->tcMCisr)){
        return;
    }
    
    vel = ptData->eleVelocityLth;
    tcMCisr = (float)(ptData->tcMCisr);
    tcHALLisr = (float)(ptData->tcHALLisr);
    
    if(tcMCisr > tcHALLisr){
        ptData->eleVelocityLth = vel * tcHALLisr / tcMCisr;
    }    
}

static inline void angleCal(tagHALLdata * ptData)
{
    float tmp;
    
    tmp = ptData->eleVelocityLth * (float)ptData->tcMCisr * RL_TCO_TS;
    
    switch(ptData->dirLth){
        case 1:
            if(tmp < ptData->offsetAngleBound){
                ptData->offsetAngle = tmp;
            } else {
                ptData->offsetAngle = ptData->offsetAngleBound;
            }
            break;
        case -1:
            if(tmp > ptData->offsetAngleBound){
                ptData->offsetAngle = tmp;
            } else {
                ptData->offsetAngle = ptData->offsetAngleBound;
            }            
            break;
        case 0:
            ptData->offsetAngle = 0.0f;
            break;
        default:           
            break;
    }
       
    tmp = ptData->angleHALLjumpLth + ptData->offsetAngle;
    
    if(RL_TWO_PI < tmp){
        ptData->estimatedRotorAngle = tmp - RL_TWO_PI;
    } else if(0.0f > tmp){
        ptData->estimatedRotorAngle = tmp + RL_TWO_PI;
    } else{
        ptData->estimatedRotorAngle = tmp;
    } 
}

static void rotorAngleEstimate(tagHALLdata * ptData)
{
    float newAngleHall; // What should this be initialized to?
    static float prevAngleHall; // What should this be initialized to?

    switch(ptData->stateLth){
        case STT_HALL_ISR_INI: 
        case STT_HALL_ISR_1ST_JUMP:
        case STT_HALL_ISR_ONE_JUMP_LOW_SPEED:            
        
            /*
                The next switch statement attempts to increase locked rotor torque by adjusting
                the angle estimate.  The first/default estimate is that the rotor is in the center
                of the hall singal range (e.g. 30 degrees for hall signal repesenting 0-60 degree
                range).  If the actual rotor angle is on the edge of the hall range (e.g. 1 degree),
                then the original estimate will result in low torque.  This guess and try method
                should provide higher torque and allow the drive to move the motor out of a locked
                rotor condition

                **This method assumes we have 6 hall signals/codes for 360 degrees of mechanical rotation
                **It is OK if we go outside the range 0-2pi.  mcLib will correct the value when using angle data

            */ 

            newAngleHall = ptData->angleHALL;
            if (mcApp_motorState.startup_f) //only run on startup (for reversals)
            {
                // Use the counter, to iterate through different angle estimates
                switch (HALLdata.iterateHallAngle_counter)
                {
                case 0:
                    // If the counter is at 0, then dont change the angle
                    ptData->estimatedRotorAngle = newAngleHall;
                    break;
                case 1:
                    // Check if there has been a hall transition since the last time
                    if (newAngleHall != prevAngleHall)
                    {
                        // Hall value changed, so use the midpoint between the two
                        newAngleHall = ( (newAngleHall + prevAngleHall) / 2 );
                    }
                    else
                    {
                        // Change the angle to +30 degrees (right at the hall sensor transition point)
                        ptData->estimatedRotorAngle = newAngleHall + RL_ONE_OVER_SIX_PI;
                    }

                    break;
                case 2:
                    // Check if there has been a hall transition since the last time
                    if (newAngleHall != prevAngleHall)
                    {
                        // Hall value changed, so use the midpoint between the two
                        newAngleHall = ( (newAngleHall + prevAngleHall) / 2 );
                    }
                    else
                    {
                        // Change the angle to -30 degrees (right at the hall sensor transition point)
                        ptData->estimatedRotorAngle = newAngleHall - RL_ONE_OVER_SIX_PI;
                    }
                    
                    break;
                default:
                    // If we make it through the iterations and still haven't moved, just go back to the original angle
                    ptData->estimatedRotorAngle = newAngleHall;
                    break;
                }
            }
            else // not startup, so just use the angle value as normal
            {
               ptData->estimatedRotorAngle = newAngleHall; 
            }

            prevAngleHall = newAngleHall;   // Save the angle for next time

            break;

        case STT_HALL_ISR_ONE_JUMP_HIGH_SPEED: 
        case STT_HALL_ISR_SIX_JUMP:
            angleCal(ptData);
            break;
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: MC ADC ISR TASKS
// *****************************************************************************
// *****************************************************************************
//runs every 50 microseconds
void mcApp_ADCISRTasks(ADC_STATUS status, uintptr_t context)
{
    NVIC_DisableIRQ(PDEC_OTHER_IRQn);
    latchHALLdata(&HALLdata);
    HALLdata.tcMCisr = TC0_Timer32bitCounterGet();

    // If we havent seen a hall transition in a while, force our speed estimation to zero
    //  This is necessary because HALLdata.eleVelocityFil is only updated in the Hall ISR, which
    //  is not triggered if we dont see any hall transitions.
    if (HALLdata.tcMCisr > HALL_UNCHANGED_SO_ASSUME_ZERO_SPEED)
    {
        HALLdata.eleVelocityFil = 0;
        mcApp_Speed_PIParam.qInMeas = 0;
    }
    
#if 1
    lowSpeedCorrect(&HALLdata);
#endif
    NVIC_EnableIRQ(PDEC_OTHER_IRQn);
    rotorAngleEstimate(&HALLdata);
    /* LPF */
    HALLdata.eleVelocityFil = (RL_LPF_COEFF_1 * HALLdata.eleVelocityFil) + (RL_LPF_COEFF_2 * HALLdata.eleVelocityLth);    
    
    X2CScope_Update();

    phaseCurrentA = (int16_t)ADC0_ConversionResultGet() - (int16_t)adc_0_offset;// Phase Current A (U) measured using ADC0
    phaseCurrentB = (int16_t)ADC1_ConversionResultGet() - (int16_t)adc_1_offset;// Phase Current B (V) measured using ADC1
    
    /* Clear all interrupt flags */
       ADC0_REGS->ADC_INTFLAG = ADC_INTFLAG_Msk;
       ADC0_REGS->ADC_INTENCLR = ADC_INTFLAG_RESRDY_Msk;// Disable ADC interrupt
    /* select the next channel using the sequency number to cycle through additional adc inputs*/
    ADC1_ChannelSelect(secondaryAdc1Channels[secondaryAdcSequenceNumber],ADC_NEGINPUT_GND); // DC Bus Voltage on ADC0
    ADC0_ChannelSelect(secondaryAdc0Channels[secondaryAdcSequenceNumber],ADC_NEGINPUT_GND);  // Thermistor #2
    
    ADC0_REGS->ADC_SWTRIG |= ADC_SWTRIG_START_Msk; 
    

   // Check for start and Ignore speed input below the defined threshold
if(mcApp_motorState.focStart)
{
    mcApp_I_ABCParam.a = (float)phaseCurrentA*ADC_PHASE_CURRENT_SCALE * (-1); 
    mcApp_I_ABCParam.b = (float)phaseCurrentB*ADC_PHASE_CURRENT_SCALE * (-1);
    
    mcLib_ClarkeTransform(&mcApp_I_ABCParam, &mcApp_I_AlphaBetaParam);
   
    
    
    mcLib_ParkTransform(&mcApp_I_AlphaBetaParam, &mcApp_SincosParam, 
                        &mcApp_I_DQParam);
 
    #if !HALL_ANGLE_RUN
    mcLib_PLLEstimator(&mcApp_EstimParam, &mcApp_SincosParam, &mcApp_focParam, 
                       &mcApp_I_AlphaBetaParam, &mcApp_V_AlphaBetaParam);
    #endif

#ifndef CURPI_TUN  
        motor_state_machine.closed_loop = state_count;
        mcApp_ControlParam.AssertActiveVector = 1;
        /* Use the rotor angle estimated by PLL estimator as the angle reference for rotating frame*/
        if(mcApp_motorState.motorDirection == 0)
        {
        #if (TRAPEZOIDAL_CONTROL && FOC_CONTROL)
            if((HALLdata.eleVelocityFil > motorParams[Motor_ID_num].TRAP_TO_FOC_SWITCH_SPEED)\
                && (mcApp_motorState.trap_mode == TRAP_MODE_ENABLE)\
                && (mcApp_motorState.sensorless_mode == SENSORLESS_MODE_DISABLE))
            {
                mcApp_motorState.trap_mode = TRAP_MODE_DISABLE;
                mcApp_V_DQParam.d = mcApp_D_PIParam.qOut = mcApp_D_PIParam.qdSum = 0;
                mcApp_V_DQParam.q = mcApp_Q_PIParam.qOut\
                = mcApp_Q_PIParam.qdSum =  trap_mod_modulation_index;
              
            }

            
            if((HALLdata.eleVelocityFil <= motorParams[Motor_ID_num].FOC_TO_TRAP_SWITCH_SPEED)\
                && (mcApp_motorState.trap_mode == TRAP_MODE_DISABLE)\
                && (mcApp_motorState.sensorless_mode == SENSORLESS_MODE_DISABLE))
            {
                mcApp_motorState.trap_mode = TRAP_MODE_ENABLE;
                mcApp_Trap_Speed_PIParam.qOut = mcApp_Trap_Speed_PIParam.qdSum =  mcApp_Q_PIParam.qOut;
            }
        #endif // TRAPEZOIDAL_CONTROL && FOC_CONTROL 

        #if HALL_SENSORLESS_HYBRID_RUN    
            if ((mcApp_EstimParam.qVelEstim > motorParams[Motor_ID_num].SENSORED_TO_SENSORLESS_SWITCH_SPEED)\
                &&(mcApp_motorState.sensorless_mode == SENSORLESS_MODE_DISABLE)\
                &&(mcApp_motorState.trap_mode == TRAP_MODE_DISABLE))
            {
                mcApp_motorState.sensorless_mode = SENSORLESS_MODE_ENABLE;
            }
            
            if ((mcApp_EstimParam.qVelEstim < motorParams[Motor_ID_num].SENSORLESS_TO_SENSORED_SWITCH_SPEED)\
                &&(mcApp_motorState.sensorless_mode == SENSORLESS_MODE_ENABLE)\
                &&(mcApp_motorState.trap_mode == TRAP_MODE_DISABLE))
            {
                mcApp_motorState.sensorless_mode = SENSORLESS_MODE_DISABLE;
            }
        #endif // HALL_SENSORLESS_HYBRID_RUN
        }
        else
        {
        #if (TRAPEZOIDAL_CONTROL && FOC_CONTROL)
            if((HALLdata.eleVelocityFil < -motorParams[Motor_ID_num].TRAP_TO_FOC_SWITCH_SPEED)\
                && (mcApp_motorState.trap_mode == TRAP_MODE_ENABLE)\
                && (mcApp_motorState.sensorless_mode == SENSORLESS_MODE_DISABLE))
            {
                mcApp_motorState.trap_mode = TRAP_MODE_DISABLE;
                mcApp_V_DQParam.d = mcApp_D_PIParam.qOut = mcApp_D_PIParam.qdSum = 0;
                mcApp_V_DQParam.q =  mcApp_Q_PIParam.qOut\
                = mcApp_Q_PIParam.qdSum = -trap_mod_modulation_index;
               
            }

            if((HALLdata.eleVelocityFil >= -motorParams[Motor_ID_num].FOC_TO_TRAP_SWITCH_SPEED)\
                && (mcApp_motorState.trap_mode == TRAP_MODE_DISABLE)\
                && (mcApp_motorState.sensorless_mode == SENSORLESS_MODE_DISABLE))
            {
                mcApp_motorState.trap_mode = TRAP_MODE_ENABLE;
                mcApp_Trap_Speed_PIParam.qOut = mcApp_Trap_Speed_PIParam.qdSum  = -mcApp_Q_PIParam.qOut;
            }
        #endif // TRAPEZOIDAL_CONTROL && FOC_CONTROL

        
        #if HALL_SENSORLESS_HYBRID_RUN    
            if ((mcApp_EstimParam.qVelEstim < -motorParams[Motor_ID_num].SENSORED_TO_SENSORLESS_SWITCH_SPEED)\
                &&(mcApp_motorState.sensorless_mode == SENSORLESS_MODE_DISABLE)\
                &&(mcApp_motorState.trap_mode == TRAP_MODE_DISABLE))
            {
                mcApp_motorState.sensorless_mode = SENSORLESS_MODE_ENABLE;
            }
            
            if ((mcApp_EstimParam.qVelEstim > -motorParams[Motor_ID_num].SENSORLESS_TO_SENSORED_SWITCH_SPEED)\
                &&(mcApp_motorState.sensorless_mode == SENSORLESS_MODE_ENABLE)\
                &&(mcApp_motorState.trap_mode == TRAP_MODE_DISABLE))
            {
                mcApp_motorState.sensorless_mode = SENSORLESS_MODE_DISABLE;
            }
        #endif // HALL_SENSORLESS_HYBRID_RUN    
        }



#if HALL_ANGLE_RUN
            mcApp_SincosParam.Angle = HALLdata.estimatedRotorAngle;
#elif HALL_SENSORLESS_HYBRID_RUN
            if(mcApp_motorState.sensorless_mode == SENSORLESS_MODE_DISABLE)
            {
                mcApp_SincosParam.Angle = HALLdata.estimatedRotorAngle;
            }
            else
            {
                mcApp_SincosParam.Angle = mcApp_EstimParam.qRho;
            }
            
#else
                
            mcApp_SincosParam.Angle = mcApp_EstimParam.qRho;

            if(mcApp_ControlParam.ol_cl_complete == 0)
            {
                if(mcApp_motorState.motorDirection == 0)
                {
                    mcApp_ControlParam.VelRef = OPENLOOP_END_SPEED_RADS_PER_SEC_ELEC;
                }
                else
                {
                    mcApp_ControlParam.VelRef = -OPENLOOP_END_SPEED_RADS_PER_SEC_ELEC;
                }
                // calculate max allowable d axis reference with priority to q axis current.
                //In order to ensure that total current vector does not exceed max motor current rating
                d_intermediate =  sqrtf(motorParams[Motor_ID_num].MAX_MOTOR_CURRENT_SQUARED - (mcApp_IRef_DQParam.q*mcApp_IRef_DQParam.q));

                //If current D axis reference is higher than max allowable D axis reference then limit its value to the max allowable D axis reference.
                if(mcApp_IRef_DQParam.d > d_intermediate)
                {
                    mcApp_IRef_DQParam.d = d_intermediate;
                }

                if(mcApp_IRef_DQParam.d > D_CURRENT_REF_STEP)
                {
                    mcApp_IRef_DQParam.d -= D_CURRENT_REF_STEP;
                }
                else
                {
                    mcApp_IRef_DQParam.d = 0;
                    mcApp_ControlParam.ol_cl_complete = 1;
                }
            }
#endif
            
            //if TORQUE MODE skip the speed and Field Weakening controller               
#ifndef TORQUE_MODE
            // Execute the velocity control loop
#if HALL_ANGLE_RUN
            mcApp_Speed_PIParam.qInMeas = HALLdata.eleVelocityFil;
#elif HALL_SENSORLESS_HYBRID_RUN
            if(mcApp_motorState.sensorless_mode == SENSORLESS_MODE_DISABLE)
            {
               mcApp_Speed_PIParam.qInMeas = HALLdata.eleVelocityFil;
            }
            else
            {
                mcApp_Speed_PIParam.qInMeas = mcApp_EstimParam.qVelEstim;
            }
#else
            mcApp_Speed_PIParam.qInMeas = mcApp_EstimParam.qVelEstim;
#endif
            mcApp_Speed_PIParam.qInRef  = mcApp_ControlParam.VelRef;
            mcLib_CalcPI(&mcApp_Speed_PIParam);
            mcApp_IRef_DQParam.q= mcApp_Speed_PIParam.qOut;

    #ifdef ENABLE_FLUX_WEAKENING
            // Implement Field Weakening if backEMF is greater than bus voltage
            if(mcApp_motorState.motorDirection == 0)
            {
                    if( ( SQRT3 * mcApp_EstimParam.qEsqf) > (mcApp_focParam.DCBusVoltage) )
                    {
                        mcApp_focParam.Vds = mcApp_V_DQParam.d*mcApp_V_DQParam.d;

                        if(mcApp_focParam.Vds>MAX_NORM_SQ)
                        {
                        mcApp_focParam.Vds = MAX_NORM_SQ;
                        }

                        mcApp_focParam.Vqs = sqrtf(MAX_NORM_SQ-mcApp_focParam.Vds);
                        mcApp_focParam.VqRefVoltage = mcApp_focParam.MaxPhaseVoltage*mcApp_focParam.Vqs;

                         //Calculating Flux Weakening value of Id, Id_flux_Weakening = (Vqref- Rs*Iq - BEMF)/omega*Ls

                        mcApp_ControlParam.Id_FW_Raw = (mcApp_focParam.VqRefVoltage - (motorParams[Motor_ID_num].MOTOR_PER_PHASE_RESISTANCE * mcApp_IRef_DQParam.q) 
                                                -(mcApp_ControlParam.VelRef  * motorParams[Motor_ID_num].MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC))/(mcApp_ControlParam.VelRef  * motorParams[Motor_ID_num].MOTOR_PER_PHASE_INDUCTANCE);

                        mcApp_ControlParam.Id_FW_Filtered = mcApp_ControlParam.Id_FW_Filtered +
                                          ((mcApp_ControlParam.Id_FW_Raw - mcApp_ControlParam.Id_FW_Filtered) * mcApp_ControlParam.qKfilterIdRef) ;

                        mcApp_IRef_DQParam.d= mcApp_ControlParam.Id_FW_Filtered;
                          //Limit Id such that MAX_FW_NEGATIVE_ID_REF < Id < 0
                        if(mcApp_IRef_DQParam.d> 0)
                            mcApp_IRef_DQParam.d= 0; 

                        if(mcApp_IRef_DQParam.d< motorParams[Motor_ID_num].MAX_FW_NEGATIVE_ID_REF)
                            mcApp_IRef_DQParam.d= motorParams[Motor_ID_num].MAX_FW_NEGATIVE_ID_REF;

                        // Limit Q axis current such that sqrtf(Id^2 +Iq^2) <= MAX_MOTOR_CURRENT
                        mcApp_ControlParam.Iqmax = sqrtf((motorParams[Motor_ID_num].MAX_MOTOR_CURRENT_SQUARED) - (mcApp_IRef_DQParam.d*mcApp_IRef_DQParam.d));
                    }
                    else
                    {
                        mcApp_IRef_DQParam.d= 0;
                        mcApp_ControlParam.Iqmax = mcApp_Speed_PIParam.qOutMax;
                        mcApp_ControlParam.Id_FW_Filtered = 0;
                        mcApp_ControlParam.Id_FW_Raw = 0;
                    }
            }
            else
            {
                    if( ((-1 * SQRT3) * mcApp_EstimParam.qEsqf) > (mcApp_focParam.DCBusVoltage) )
                    {
                        mcApp_focParam.Vds = mcApp_V_DQParam.d*mcApp_V_DQParam.d;

                        if(mcApp_focParam.Vds>MAX_NORM_SQ)
                        {
                        mcApp_focParam.Vds = MAX_NORM_SQ;
                        }

                        mcApp_focParam.Vqs = sqrtf(MAX_NORM_SQ-mcApp_focParam.Vds);
                        mcApp_focParam.VqRefVoltage = -mcApp_focParam.MaxPhaseVoltage*mcApp_focParam.Vqs;

                         //Calculating Flux Weakening value of Id, Id_flux_Weakening = (Vqref- Rs*Iq - BEMF)/omega*Ls

                        mcApp_ControlParam.Id_FW_Raw = (mcApp_focParam.VqRefVoltage - (motorParams[Motor_ID_num].MOTOR_PER_PHASE_RESISTANCE * mcApp_IRef_DQParam.q) 
                                                -(mcApp_ControlParam.VelRef  * motorParams[Motor_ID_num].MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC))/(mcApp_ControlParam.VelRef  * motorParams[Motor_ID_num].MOTOR_PER_PHASE_INDUCTANCE);

                        mcApp_ControlParam.Id_FW_Filtered = mcApp_ControlParam.Id_FW_Filtered +
                                          ((mcApp_ControlParam.Id_FW_Raw - mcApp_ControlParam.Id_FW_Filtered) * mcApp_ControlParam.qKfilterIdRef) ;    
                        mcApp_IRef_DQParam.d= mcApp_ControlParam.Id_FW_Filtered;
                          //Limit Id such that MAX_FW_NEGATIVE_ID_REF < Id < 0
                        if(mcApp_IRef_DQParam.d> 0)
                            mcApp_IRef_DQParam.d= 0; 

                        if(mcApp_IRef_DQParam.d< motorParams[Motor_ID_num].MAX_FW_NEGATIVE_ID_REF)
                            mcApp_IRef_DQParam.d= motorParams[Motor_ID_num].MAX_FW_NEGATIVE_ID_REF;

                        // Limit Q axis current such that sqrtf(Id^2 +Iq^2) <= MAX_MOTOR_CURRENT
                        mcApp_ControlParam.Iqmax = -sqrtf((motorParams[Motor_ID_num].MAX_MOTOR_CURRENT_SQUARED) - (mcApp_IRef_DQParam.d*mcApp_IRef_DQParam.d));
                    }
                    else
                    {
                        mcApp_IRef_DQParam.d= 0;
                        mcApp_ControlParam.Iqmax = mcApp_Speed_PIParam.qOutMin; // Yes setting the max to a min value.  mcApp_ControLParam only has a max, not a min but should be a negative here
                        mcApp_ControlParam.Id_FW_Filtered = 0;
                    }
            }


    #endif 

    #else  // else for #ifndef TORQUE_MODE
    if(mcApp_motorState.motorDirection == 0)
    {
            mcApp_IRef_DQParam.q= (float)((float)speedCommand * TORQUE_MODE_POT_ADC_RATIO); // During torque mode, Iq = Potentiometer provides torque reference in terms of current, Id = 0
            mcApp_IRef_DQParam.d= 0;
            mcApp_ControlParam.Iqmax = TORQUE_MODE_MAX_CUR;
    }
    else
    {
            mcApp_IRef_DQParam.q= (float)((float)-speedCommand * TORQUE_MODE_POT_ADC_RATIO); // During torque mode, Iq = Potentiometer provides torque reference in terms of current, Id = 0
            mcApp_IRef_DQParam.d= 0;
            mcApp_ControlParam.Iqmax = -TORQUE_MODE_MAX_CUR;
    }
    #endif  // endif for #ifndef TORQUE_MODE
    
#else // CURPI_TUN
    mcApp_IRef_DQParam.d= CUR_STEP_AMP;
    mcApp_ControlParam.AssertActiveVector = 1;
    if(curpi_counter < CPT_CNT_VAL)
    {
        curpi_counter++;
    }
    else
    {
        mcApp_inverterDisable();
        curpi_counter = 0;
    }

#endif //CURPI_TUN    

#if TRAPEZOIDAL_CONTROL
    if(mcApp_motorState.trap_mode == TRAP_MODE_ENABLE)
    {
                if(mcApp_motorState.motorDirection == 0)
                {
                    TCC1_REGS->TCC_PATTBUF =(uint16_t)(patt_reg_cw_lut[HALLdata.HALLvalue]);
                    mcApp_Trap_Speed_PIParam.qInRef =  mcApp_ControlParam.VelRef;
                    mcApp_Trap_Speed_PIParam.qInMeas =  HALLdata.eleVelocityFil;
         
                }
                else
                {
                    TCC1_REGS->TCC_PATTBUF =(uint16_t)(patt_reg_ccw_lut[HALLdata.HALLvalue]);  
                    mcApp_Trap_Speed_PIParam.qInRef =  -mcApp_ControlParam.VelRef;
                    mcApp_Trap_Speed_PIParam.qInMeas =  -HALLdata.eleVelocityFil;
                }
                mcLib_CalcPI(&mcApp_Trap_Speed_PIParam);
                trap_mod_modulation_index =  mcApp_Trap_Speed_PIParam.qOut;          


                trap_duty = (uint32_t)(mcApp_SVGenParam.PWMPeriod*trap_mod_modulation_index);
                TCC1_PWM24bitDutySet(TCC1_CHANNEL0,(uint32_t) trap_duty );  // U
                TCC1_PWM24bitDutySet(TCC1_CHANNEL1,(uint32_t) trap_duty );  // V
                TCC1_PWM24bitDutySet(TCC1_CHANNEL2,(uint32_t) trap_duty );  // W


    }
    else
    {

        if(mcApp_motorState.focStart)
        {
            TCC1_REGS->TCC_PATTBUF = 0x0000;
        }
        else
        {
            TCC1_REGS->TCC_PATTBUF = 0x0077;
        }
#endif // TRAPEZOIDAL_CONTROL  

        // PI control for D
        mcApp_D_PIParam.qInMeas = mcApp_I_DQParam.d;          // This is in Amps
        mcApp_D_PIParam.qInRef  = mcApp_IRef_DQParam.d;      // This is in Amps
        mcLib_CalcPI(&mcApp_D_PIParam);
        mcApp_V_DQParam.d    =  mcApp_D_PIParam.qOut;          // This is in %. If should be converted to volts, multiply with DCBus/sqrt(3)

        // dynamic d-q adjustment
        // with d component priority
        // vq=sqrt (vs^2 - vd^2)
        // limit vq maximum to the one resulting from the calculation above
         DoControl_Temp2 = mcApp_D_PIParam.qOut * mcApp_D_PIParam.qOut;
         DoControl_Temp1 = MAX_NORM_SQ - DoControl_Temp2;
         mcApp_Q_PIParam.qOutMax = sqrtf(DoControl_Temp1);
         mcApp_Q_PIParam.qOutMin = -mcApp_Q_PIParam.qOutMax;        

            if(mcApp_motorState.motorDirection == 0)
            {
                //Limit Q axis current
                if(mcApp_IRef_DQParam.q>mcApp_ControlParam.Iqmax)
                {
                    mcApp_IRef_DQParam.q= mcApp_ControlParam.Iqmax;
                }
                else
                {

                }
            }
            else
            {
                //Limit Q axis current
                if(mcApp_IRef_DQParam.q<mcApp_ControlParam.Iqmax)
                {
                    mcApp_IRef_DQParam.q= mcApp_ControlParam.Iqmax;
                }   
                else
                {

                }
            }
        // PI control for Q
        mcApp_Q_PIParam.qInMeas = mcApp_I_DQParam.q;          // This is in Amps
        mcApp_Q_PIParam.qInRef  = mcApp_IRef_DQParam.q;      // This is in Amps
        mcLib_CalcPI(&mcApp_Q_PIParam);
        mcApp_V_DQParam.q    =  mcApp_Q_PIParam.qOut;          // This is in %. If should be converted to volts, multiply with DCBus/sqrt(3)   




        mcLib_SinCosGen(&mcApp_SincosParam);
        mcLib_InvParkTransform(&mcApp_V_DQParam,&mcApp_SincosParam, &mcApp_V_AlphaBetaParam);
          // Update LastValpha and LastVbeta
        mcApp_EstimParam.qLastValpha = (mcApp_focParam.MaxPhaseVoltage * mcApp_V_AlphaBetaParam.alpha);
        mcApp_EstimParam.qLastVbeta = (mcApp_focParam.MaxPhaseVoltage * mcApp_V_AlphaBetaParam.beta);
        mcLib_SVPWMGen(&mcApp_V_AlphaBetaParam , &mcApp_SVGenParam);
        if(mcApp_ControlParam.AssertActiveVector == 1)
        {
            //For overmodulation, limit max to 95% duty = 2850
            if (mcApp_SVGenParam.dPWM_A > 2850)
            {
                mcApp_SVGenParam.dPWM_A = 2850;
            }

            if (mcApp_SVGenParam.dPWM_B > 2850)
            {
                mcApp_SVGenParam.dPWM_B = 2850;
            }

            if (mcApp_SVGenParam.dPWM_C > 2850)
            {
                mcApp_SVGenParam.dPWM_C = 2850;
            }

            TCC1_PWM24bitDutySet(TCC1_CHANNEL0,(uint32_t) mcApp_SVGenParam.dPWM_A );  // U
            TCC1_PWM24bitDutySet(TCC1_CHANNEL1,(uint32_t) mcApp_SVGenParam.dPWM_B );  // V
            TCC1_PWM24bitDutySet(TCC1_CHANNEL2,(uint32_t) mcApp_SVGenParam.dPWM_C );  // W
        }
        else
        {
            #if TRAPEZOIDAL_CONTROL
                TCC1_REGS->TCC_PATTBUF = 0x0000;    //TODO: should this be 0x0077???

                TCC1_PWM24bitDutySet(TCC1_CHANNEL0,(uint32_t) 0 );  // U
                TCC1_PWM24bitDutySet(TCC1_CHANNEL1,(uint32_t) 0 );  // V
                TCC1_PWM24bitDutySet(TCC1_CHANNEL2,(uint32_t) 0 );  // W
            #else
                TCC1_PWM24bitDutySet(TCC1_CHANNEL0,(uint32_t) PWM_HALF_PERIOD_COUNT );  // U
                TCC1_PWM24bitDutySet(TCC1_CHANNEL1,(uint32_t) PWM_HALF_PERIOD_COUNT );  // V
                TCC1_PWM24bitDutySet(TCC1_CHANNEL2,(uint32_t) PWM_HALF_PERIOD_COUNT );  // W
            #endif
        }
    #if TRAPEZOIDAL_CONTROL            
        }   //  close bracket: if(mcApp_motorState.trap_mode == TRAP_MODE_ENABLE)
    #endif //TRAPEZOIDAL_CONTROL 
}
else
{
    #if TRAPEZOIDAL_CONTROL
        TCC1_REGS->TCC_PATTBUF = 0x0077;            

        TCC1_PWM24bitDutySet(TCC1_CHANNEL0,(uint32_t) 0 );  // U
        TCC1_PWM24bitDutySet(TCC1_CHANNEL1,(uint32_t) 0 );  // V
        TCC1_PWM24bitDutySet(TCC1_CHANNEL2,(uint32_t) 0 );  // W
    #else
        TCC1_PWM24bitDutySet(TCC1_CHANNEL0,(uint32_t) PWM_HALF_PERIOD_COUNT );  // U
        TCC1_PWM24bitDutySet(TCC1_CHANNEL1,(uint32_t) PWM_HALF_PERIOD_COUNT );  // V
        TCC1_PWM24bitDutySet(TCC1_CHANNEL2,(uint32_t) PWM_HALF_PERIOD_COUNT );  // W
    #endif
} // close bracket: if(mcApp_motorState.focStart == 1)

    // Wait for ADC results to be ready
    while(ADC0_REGS->ADC_INTFLAG != ADC_INTFLAG_RESRDY_Msk); //TODO Olivia FIXME should not have this in ISR

    // Store the ADC results in the appropriate variables
    switch (secondaryAdcSequenceNumber)
    {
    case 0: // SpeedCmd & BusCurrent
        speedCommand = ADC0_ConversionResultGet();
        busCurrent = ((ADC1_ConversionResultGet() - adc_bus_current_offset) * ADC_BUS_CURRENT_SCALE);
        break;
    case 1: // FetTemp2 & FetTemp1
        fetTemp2 = thermistorGetFetTemp2(ADC0_ConversionResultGet());
        fetTemp1 = thermistorGetFetTemp1(ADC1_ConversionResultGet());
        break;
    case 2: // BLANK & BusVoltage
        ADC0_ConversionResultGet();// Nothing needed from ADC0, but read the register to prevent an overflow interrupt
        mcApp_focParam.DCBusVoltage = ((float)ADC1_ConversionResultGet())* VOLTAGE_ADC_TO_PHY_RATIO; // Reads and translates to actual bus voltage
        
        busVoltage = mcApp_focParam.DCBusVoltage;   // busVoltage variable used for fault detection
        mcApp_focParam.MaxPhaseVoltage = (float)(mcApp_focParam.DCBusVoltage*ONE_BY_SQRT3);
        break;
    default:
        break;
    }

    // Increment the ADC sequence number
    if (++secondaryAdcSequenceNumber > 2)
    {
        secondaryAdcSequenceNumber = 0; // reset sequence number
    }

    /* select the next ADC channel for conversion */
    ADC0_ChannelSelect(U_CURRENT_ADC_POSINPUT,ADC_NEGINPUT_GND); // Phase U to ADC0
    ADC1_ChannelSelect(V_CURRENT_ADC_POSINPUT,ADC_NEGINPUT_GND); // Phase V to ADC1
    ADC0_REGS->ADC_INTENSET = ADC_INTFLAG_RESRDY_Msk;// Enable ADC interrupt
    /* Clear all interrupt flags */
    ADC0_REGS->ADC_INTFLAG = ADC_INTFLAG_Msk;

       
    delay_10ms.count++;
}

// Turn on the dump resistor if bus voltage is above threshold ****run every 10ms****
//time limit in place for 50% DC (100ms on, 100ms off, repeat as needed)
void mcApp_DumpResistorControl()
{
   static uint32_t time_limit_counter = 0;
    if (mcApp_focParam.DCBusVoltage < (DCBUS_DUMP_RESISTOR_VOLTAGE_THRESHOLD - 0.5) && (time_limit_counter < DUMP_RESISTOR_TIME_LIMIT_MAX_COUNT)) //below hysteresis value && not at max
    {
        Dump_Resistor_FET_Control_Clear();
        time_limit_counter = 0; //reset to allow for future use/toggles
    }
    else if (mcApp_focParam.DCBusVoltage > DCBUS_DUMP_RESISTOR_VOLTAGE_THRESHOLD || time_limit_counter > 0) //counter check needed to make sure we still toggle in hysteresis range
    {
        if(time_limit_counter >= DUMP_RESISTOR_TIME_LIMIT_MAX_COUNT)
        {
           //reached overall max time on, turn off forever (until fresh power cycle) something must be wrong
           //consider adding error code in future updates?
           Dump_Resistor_FET_Control_Clear();
           //DO NOT INCREMENT COUNTER - need to stay at max to make sure we don't come back in here again
        }
        else if ((time_limit_counter % DUMP_RESISTOR_TIME_LIMIT_TOGGLE_COUNT) == 0) //10 counts (100ms) have passed
        {
           //on/off time limit exceeded, toggle
           Dump_Resistor_FET_Control_Toggle();
           time_limit_counter++;
        }
        else //just increment counter
        {
           time_limit_counter++;
        }
    }
}

/**
 * @brief Detect a stalled motor and take appropriate action to prevent HW damage.
 *
 *        This function uses motor state, motor speed, and motor phase current
 *        to detect if the motor is stalled.  When a stall is initially
 *        detected, the drive allows full MAX_MOTOR_CURRENT to be applied to the
 *        motor.  After a specified "delay" amount of time, the drive will reduce
 *        output by a "reduction" amount.  After 3 seconds of continuous stall,
 *        the drive will shut down and report a stall fault.
 *              
 *        This function is called every 10ms from main.
 *
 * @return void
 */
void mcApp_StallDetection()
{
    if(mcApp_motorState.motorDirection == 0)
    {
        if ( (mcApp_motorState.focStart) &&
            (mcApp_Speed_PIParam.qInMeas < STALL_DETECT_RPM_THRESHOLD) &&
            (mcApp_I_DQParam.q > (motorParams[Motor_ID_num].MAX_MOTOR_CURRENT * STALL_DETECT_CURRENT_THRESHOLD_RATIO)))
        {
            timer_stall_detect_count++;

            if (0 == (timer_stall_detect_count % TIMER_STALL_DETECT_ANGLE_ITERATE_INTERVAL))
            {
                // At each interval, iterate the estimated angle to try to get better starting torque
                HALLdata.iterateHallAngle_counter++;    // Increment the counter to iterate the angle estimate
            }

            if (timer_stall_detect_count >= TIMER_STALL_DETECT_DELAY)
            {
                // Delay time allows for high startup torque 
                // After the delay period, limit current to 50% max
                mcApp_Speed_PIParam.qOutMax = (motorParams[Motor_ID_num].MAX_MOTOR_CURRENT * STALL_DETECT_CURRENT_REDUCTION_RATIO);
            }
            else if ( mcApp_motorState.startup_f )
            {
                //set max output current to 120A during locked rotor when the motor has not moved yet
                mcApp_Speed_PIParam.qOutMax = motorParams[Motor_ID_num].MAX_MOTOR_CURRENT_DURING_STALL;
                mcApp_ControlParam.Iqmax = motorParams[Motor_ID_num].MAX_MOTOR_CURRENT_DURING_STALL;

                // Increase pi paramaters here to respond more quickly during a motor stall
                mcApp_Speed_PIParam.qKp = motorParams[Motor_ID_num].SPEEDCNTR_PTERM_DURING_STALL;
                mcApp_Speed_PIParam.qKi = motorParams[Motor_ID_num].SPEEDCNTR_ITERM_DURING_STALL;
            }
            if (timer_stall_detect_count >= TIMER_STALL_DETECT_PERIOD)
            {
                // After the detect period, a motor Stall has been
                // detected, so throw a fault
                motorStallDirection = 0;
                fh_faultHandler(MOTOR_STALL);
            }
        }
        // else we dont meet the stall detect criteria
        else if (timer_stall_detect_count > 0)
        {
            // Reset qOutMax and counter
            mcApp_Speed_PIParam.qOutMax = motorParams[Motor_ID_num].MAX_MOTOR_CURRENT;
            mcApp_ControlParam.Iqmax = motorParams[Motor_ID_num].MAX_MOTOR_CURRENT;
            timer_stall_detect_count = 0;
            HALLdata.iterateHallAngle_counter = 0;
            
            // Reset pi paramaters once we are out of the stall condition
            mcApp_Speed_PIParam.qKp = motorParams[Motor_ID_num].SPEEDCNTR_PTERM;
            mcApp_Speed_PIParam.qKi = motorParams[Motor_ID_num].SPEEDCNTR_ITERM;
        }
    }
    else
    {
        if ( (mcApp_motorState.focStart) &&
            (mcApp_Speed_PIParam.qInMeas > -STALL_DETECT_RPM_THRESHOLD) &&
            (-mcApp_I_DQParam.q > (motorParams[Motor_ID_num].MAX_MOTOR_CURRENT * STALL_DETECT_CURRENT_THRESHOLD_RATIO)))
        {
            timer_stall_detect_count++;

            if (0 == (timer_stall_detect_count % TIMER_STALL_DETECT_ANGLE_ITERATE_INTERVAL))
            {
                // At each interval, iterate the estimated angle to try to get better starting torque
                HALLdata.iterateHallAngle_counter++;    // Increment the counter to iterate the angle estimate
            }

            if (timer_stall_detect_count >= TIMER_STALL_DETECT_DELAY)
            {
                // Delay time allows for high startup torque 
                // After the delay period, limit current to 50% max
                mcApp_Speed_PIParam.qOutMin = -(motorParams[Motor_ID_num].MAX_MOTOR_CURRENT * STALL_DETECT_CURRENT_REDUCTION_RATIO);
            }
            else if ( mcApp_motorState.startup_f )
            {
                //set max output current to 120A during locked rotor when the motor has not moved yet
                mcApp_Speed_PIParam.qOutMin = -motorParams[Motor_ID_num].MAX_MOTOR_CURRENT_DURING_STALL;
                mcApp_ControlParam.Iqmax    = -motorParams[Motor_ID_num].MAX_MOTOR_CURRENT_DURING_STALL;  // Setting max to negative is correct for mcApp_ControlParam

                // Increase pi paramaters here to respond more quickly during a motor stall
                mcApp_Speed_PIParam.qKp = motorParams[Motor_ID_num].SPEEDCNTR_PTERM_DURING_STALL;
                mcApp_Speed_PIParam.qKi = motorParams[Motor_ID_num].SPEEDCNTR_ITERM_DURING_STALL;
            }
            if (timer_stall_detect_count >= TIMER_STALL_DETECT_PERIOD)
            {
                // After the detect period, a motor Stall has been
                // detected, so throw a fault
                motorStallDirection = Motor_Direction_Get();
                fh_faultHandler(MOTOR_STALL);
            }
        }
        // else we dont meet the stall detect criteria
        else if (timer_stall_detect_count > 0)
        {
            // Reset qOutMin and counter
            mcApp_Speed_PIParam.qOutMin = -motorParams[Motor_ID_num].MAX_MOTOR_CURRENT;
            mcApp_ControlParam.Iqmax    = -motorParams[Motor_ID_num].MAX_MOTOR_CURRENT;  // Setting max to negative is correct for mcApp_ControlParam
            timer_stall_detect_count = 0;
            HALLdata.iterateHallAngle_counter = 0;

                        
            // Reset pi paramaters once we are out of the stall condition
            mcApp_Speed_PIParam.qKp = motorParams[Motor_ID_num].SPEEDCNTR_PTERM;
            mcApp_Speed_PIParam.qKi = motorParams[Motor_ID_num].SPEEDCNTR_ITERM;
        }
    }
}


// *****************************************************************************
// *****************************************************************************
// Section: MC PI Controller Routines
// *****************************************************************************
// *****************************************************************************

static inline void mcApp_InitControlParameters(void)
{
    // PI Trap Speed
    mcApp_Trap_Speed_PIParam.qKp = TRAP_SPEEDCNTR_PTERM;       
    mcApp_Trap_Speed_PIParam.qKi = TRAP_SPEEDCNTR_ITERM;              
    mcApp_Trap_Speed_PIParam.qKc = TRAP_SPEEDCNTR_CTERM;
    mcApp_Trap_Speed_PIParam.qdSum = 0;
    mcApp_Trap_Speed_PIParam.qOutMax = TRAP_SPEEDCNTR_OUTMAX;
    mcApp_Trap_Speed_PIParam.qOutMin = 0;
    mcApp_Trap_Speed_PIParam.qErr = 0;
    mcApp_Trap_Speed_PIParam.qInMeas = 0;
    mcApp_Trap_Speed_PIParam.qInRef = 0;

    mcLib_InitPI(&mcApp_Trap_Speed_PIParam);    
    
    // PI D Term     
    mcApp_D_PIParam.qKp = D_CURRCNTR_PTERM;       
    mcApp_D_PIParam.qKi = D_CURRCNTR_ITERM;              
    mcApp_D_PIParam.qKc = D_CURRCNTR_CTERM;
    mcApp_D_PIParam.qdSum = 0;
    mcApp_D_PIParam.qOutMax = D_CURRCNTR_OUTMAX;
    mcApp_D_PIParam.qOutMin = -mcApp_D_PIParam.qOutMax;
    mcApp_D_PIParam.qErr = 0;
    mcApp_D_PIParam.qInMeas = 0;
    mcApp_D_PIParam.qInRef = 0;

    mcLib_InitPI(&mcApp_D_PIParam);


    // PI Q Term 
    mcApp_Q_PIParam.qKp = Q_CURRCNTR_PTERM;    
    mcApp_Q_PIParam.qKi = Q_CURRCNTR_ITERM;
    mcApp_Q_PIParam.qKc = Q_CURRCNTR_CTERM;
    mcApp_Q_PIParam.qdSum = 0;
    mcApp_Q_PIParam.qOutMax = Q_CURRCNTR_OUTMAX;
    mcApp_Q_PIParam.qOutMin = -mcApp_Q_PIParam.qOutMax;
    if(mcApp_motorState.motorDirection == 0)
    {
        mcApp_ControlParam.Iqmax = motorParams[Motor_ID_num].MAX_MOTOR_CURRENT;
    }
    else
    {
       mcApp_ControlParam.Iqmax = -motorParams[Motor_ID_num].MAX_MOTOR_CURRENT; 
    }
    mcApp_Q_PIParam.qErr = 0;
    mcApp_Q_PIParam.qInMeas = 0;
    mcApp_Q_PIParam.qInRef = 0;

    mcLib_InitPI(&mcApp_Q_PIParam);

    // PI Qref Term
    mcApp_Speed_PIParam.qKp = motorParams[Motor_ID_num].SPEEDCNTR_PTERM;       
    mcApp_Speed_PIParam.qKi = motorParams[Motor_ID_num].SPEEDCNTR_ITERM;       
    mcApp_Speed_PIParam.qKc = motorParams[Motor_ID_num].SPEEDCNTR_CTERM;  
    mcApp_Speed_PIParam.qdSum = 0;
    mcApp_Speed_PIParam.qOutMax = motorParams[Motor_ID_num].MAX_MOTOR_CURRENT;   
    mcApp_Speed_PIParam.qOutMin = -mcApp_Speed_PIParam.qOutMax;
    mcApp_Speed_PIParam.qErr = 0;
    mcApp_Speed_PIParam.qInMeas = 0;
    mcApp_Speed_PIParam.qInRef = 0;

    mcLib_InitPI(&mcApp_Speed_PIParam);


    mcApp_ControlParam.qKfilterIdRef = KFILTER_IDREF;


    return;
}
// *****************************************************************************
// *****************************************************************************
// Section: MC PLL Estimator Parameter Initialization Routine
// *****************************************************************************
// *****************************************************************************
static inline void mcApp_InitEstimParm(void)
{
    mcApp_EstimParam.qLsDt = (motorParams[Motor_ID_num].MOTOR_PER_PHASE_INDUCTANCE/LOOPTIME_SEC);
    mcApp_EstimParam.qRs = motorParams[Motor_ID_num].MOTOR_PER_PHASE_RESISTANCE;
    mcApp_EstimParam.qKFi = (motorParams[Motor_ID_num].MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC);
    mcApp_EstimParam.qInvKFi= motorParams[Motor_ID_num].INVKFi_BELOW_BASE_SPEED;
    mcApp_EstimParam.qNominal_Speed = motorParams[Motor_ID_num].NOMINAL_SPEED_RAD_PER_SEC_ELEC;
    mcApp_EstimParam.qDecimate_Nominal_Speed = (motorParams[Motor_ID_num].NOMINAL_SPEED_RAD_PER_SEC_ELEC/10);
       mcApp_EstimParam.qOmegaMr=0;
     
    mcApp_EstimParam.qKfilterEsdq = KFILTER_ESDQ;
    mcApp_EstimParam.qVelEstimFilterK = KFILTER_VELESTIM;

    mcApp_EstimParam.qDeltaT = LOOPTIME_SEC;

    mcApp_EstimParam.qLastIalpha = 0;
    mcApp_EstimParam.qLastIbeta = 0;
    mcApp_EstimParam.qDIalpha = 0;
    mcApp_EstimParam.qDIbeta = 0;
    mcApp_EstimParam.qEsa = 0;
    mcApp_EstimParam.qEsb = 0;
    mcApp_EstimParam.qEsd = 0;
    mcApp_EstimParam.qEsq = 0;
    mcApp_EstimParam.qVIndalpha = 0;
    mcApp_EstimParam.qVIndbeta = 0;
    mcApp_EstimParam.qEsdf = 0;
    mcApp_EstimParam.qEsqf = 0;
    mcApp_EstimParam.qVelEstim = 0;
    mcApp_EstimParam.qLastIalpha = 0;
    mcApp_EstimParam.qLastVbeta = 0;
}


void mcApp_inverterEnable()
{
    drive_fault_status.faultCode = NO_FAULT;    // Stop the heartbeat while inverter is running

    mcApp_InitControlParameters();
    mcApp_InitEstimParm();
    resetHALLdata(&HALLdata);  /* Initialize HALL data. */
    mcApp_IRef_DQParam.d= 0;
    mcApp_IRef_DQParam.q= 0;
    mcApp_ControlParam.Id_FW_Filtered = 0;
    mcApp_ControlParam.VelInput = 0;
    mcApp_ControlParam.VelRef = 0;
    #ifdef ENABLE_WINDMILLING
    mcApp_motorState.focStateMachine = WINDMILLING;
    #else
        #if HALL_ANGLE_RUN || HALL_SENSORLESS_HYBRID_RUN
        mcApp_motorState.focStateMachine = CLOSEDLOOP_FOC;
        
        #else
        mcApp_motorState.focStateMachine = ALIGN;
        #endif
    #endif
    mcApp_SincosParam.Angle = 0;
    mcApp_SVGenParam.PWMPeriod = (float)MAX_DUTY;
    Align_Counter = 0;
    mcApp_motorState.sensorless_mode = SENSORLESS_MODE_DISABLE;
#if TRAPEZOIDAL_CONTROL
    mcApp_motorState.trap_mode = TRAP_MODE_ENABLE;
#else
    mcApp_motorState.trap_mode =TRAP_MODE_DISABLE;
#endif // TRAPEZOIDAL_CONTROL
    mcApp_motorState.focStart = 1;
    mcApp_motorState.inverterRunning = 1;
    
    mcApp_motorState.startup_f = true;

    PWM_Output_Enable();
}

void mcApp_inverterDisable()
{
    PWM_Output_Disable();

    mcApp_motorState.inverterRunning = 0;
    mcApp_motorState.focStart = 0;
    mcApp_IRef_DQParam.d= 0;
    mcApp_IRef_DQParam.q= 0;
    mcApp_I_DQParam.d = 0;
    mcApp_I_DQParam.q = 0;
    state_count = 0;
    mcApp_motorState.sensorless_mode = SENSORLESS_MODE_DISABLE;

    #if TRAPEZOIDAL_CONTROL    
        mcApp_motorState.trap_mode = TRAP_MODE_DISABLE;
    #else
        mcApp_motorState.trap_mode = TRAP_MODE_DISABLE;
    #endif  // TRAPEZOIDAL_CONTROL

    motor_state_machine.align = 0;
    motor_state_machine.closed_loop = 0;
    motor_state_machine.open_loop = 0;
    motor_state_machine.open_loop_to_closed_loop = 0;
    motor_state_machine.passive_brake = 0;
    motor_state_machine.windmilling = 0;
    motor_state_machine.windmilling_decide = 0;
    Passive_Brake_Counter = 0;
    
    Passive_Brake_Counter = 0;
    
    mcApp_V_DQParam.d = 0;
    mcApp_V_DQParam.q = 0;

    drive_fault_status.faultCode = IDLE;    // Provide a heartbeat when inverter is not running
}

// *Called every 10ms
void mcApp_decideLowPowerMode()
{
    static uint16_t lowPowerModeTimer = 0;

    if (((1 == Motor_Enable_Get()) && (0 == mcApp_motorState.inverterRunning)) )   // (Enable line is active-low at MCU pin)
    {
        // If conditions are met, delay before going into low power mode.  When the main board clears a fault, it 
        // toggles the ENABLE line and the drive should not enter low power mode during this toggle.
        if (lowPowerModeTimer <= LOW_POWER_MODE_DELAY_TIME_COUNTS)
        {
            lowPowerModeTimer++;
        }
        else
        {
            // Enter low power mode
            
            PDEC_HALLStop();    // Stop PDEC so that interrupt is not triggered when we turn off power rails
            
            Dump_Resistor_FET_Control_Clear(); //turn off just in case

            turnOff12VSwRail();   // Turn off the switched 12V rail
            turnOff3v3SwRail();  // Turn off the switched 3.3V rail
            
            WDT_Disable();

            PM_StandbyModeEnter();
            // ** MCU stays in standby mode until external interrupt pin (ENABLE line) transitions from high to low
            
            // After exiting low power mode, get everything back up and running
            WDT_Enable();
            turnOn12VSwRail();
            turnOn3v3SwRail();
            lowPowerModeTimer = 0;   // Clear timer for when we wake up
            
            delay_us(MOTOR_ENCODER_POWERUP_DELAY_US);    // For motors with a position sensor IC instead of simple hall sensors,
                                                        // a delay here is needed to allow hall signals to come up properly.
            PDEC_HALLStart();           // Start the PDEC again
            resetHALLdata(&HALLdata);   // Reset here so that any hall signal changes during sleep to not trigger
                                        // the hall interrupt
        }
    }
    else
    {
    // Dont go into low power mode, so reset the delay timer
        lowPowerModeTimer = 0;
    }
    
    
    
}

inline void mcApp_motorDirectionSet(uint8_t newDirection)
{
    if (mcApp_motorState.motorDirection != newDirection)
    {
        // Verify motor is stopped before changing direction
        if ( (0 == mcApp_motorState.inverterRunning)  ||
             ( (HALLdata.eleVelocityFil < 1) && (HALLdata.eleVelocityFil > -1) ) )
        {
            mcApp_motorState.motorDirection = newDirection;
        }
        else
        {
            mcApp_motorState.motorForceStop = 1;    // Set the force stop flag and do NOT change the state machine direction yet
        }
        
    }
    // else do nothing
}

void delay_us(uint16_t time_us)
{
    for (int i = 0; i < (time_us * 24); i++)
    {
        asm("nop");
    }
}
