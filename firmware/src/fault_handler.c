/* ************************************************************************** */
/** FAULT_HANDLER.C

  @Company
    Chamberlain Group Inc

  @File Name
    fault_handler.c

  @Summary
    Source file for fault handler

 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "fault_handler.h"
#include "board_id.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */

#define PERCENT_TO_LOW_DC(x)  ((uint32_t)(MAX_DUTY * (x)/100)) //x is in percent
#define PERCENT_TO_HIGH_DC(x) ((uint32_t)(MAX_DUTY * (100 - (x))/100)) //x is in percent
#define U_PHASE_SHORT_TEST_DUTY (PERCENT_TO_LOW_DC(3)) //3% DC
#define U_PHASE_HIGH_SIDE_TEST_DUTY (PERCENT_TO_HIGH_DC(10))// U phase low side PWM to 10%, rest to 0 (invert the value since these are the low side FETs)
    // Low duty is used here so that a failed high U FET does not cause high current that could damage the low U FET.
#define VW_PHASE_HIGH_SIDE_TEST_DUTY (PERCENT_TO_HIGH_DC(40))// Set U phase low side PWM to 40%, rest to 0 (invert the value since these are the low side FETs)
    // Higher duty is used here since the current (in a failed case) will be flowing through the motor
    // and will be affected by the inductance and resistance of the windings
#define VW_PHASE_LOW_SIDE_TEST_DUTY (PERCENT_TO_LOW_DC(10)) //U phase low side PWM to 10%

uint16_t timer_stall_detect_count = 0;
drive_fault_status_t    drive_fault_status;

volatile uint16_t       fetTemp1;
volatile uint16_t       fetTemp2;
volatile int16_t        busCurrent;

volatile float          busVoltage;
volatile int16_t        fetTestCurrentMeasurement;
volatile int16_t        fetTestMeasurementCounter;

static uint8_t          underVoltageFaultDirection; //used to determine if direction changed since undervoltage detected
/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */



/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*******************************************************************************
* Function: fh_ShortedFetISRTasks
*
* Arguments: status - ADC status provided by the NVIC
*            context - provided by the NVIC
*
* Returns: none
*
* Description: Used by the fh_shortedFetTest function to get the ADC conversion
*              results.  A counter is incremented for use by the test function.
*
* Original file: 
* Original authors: Kyle Reu
*
* Function created:
* Product Release History:
*
* Copyright 2021 The Chamberlain Group, Inc. All Rights Reserved.
*******************************************************************************/
void fh_ShortedFetISRTasks(ADC_STATUS status, uintptr_t context)
{
    (void)status;
    (void)context;
    fetTestCurrentMeasurement = ADC0_ConversionResultGet();
    fetTestMeasurementCounter++; // Increment flag for test routine
    ADC0_REGS->ADC_INTFLAG = ADC_INTFLAG_Msk;   // Clear all interrupt flags
}

/*******************************************************************************
* Function: fh_shortedFetTest
*
* Arguments: none
*
* Returns: none
*
* Description: Test the inverter for any shorted FETs.  See Firmware Spec
*              document 190A2701 for detailed methodology.
*
* Original file: 
* Original authors: Kyle Reu
*
* Function created:
* Product Release History:
*
* Copyright 2021 The Chamberlain Group, Inc. All Rights Reserved.
*******************************************************************************/

//called after the 5 second calibration start up, only on boot up
void fh_shortedFetTest()
{
    volatile int16_t testPhaseCurrent, testPhaseCurrentHighOn;
    volatile int16_t testPhaseCurrentHighOff[4];

    ADC0_ChannelSelect(U_CURRENT_ADC_POSINPUT,ADC_NEGINPUT_GND);    // Select the U phase current measurement ADC channel

    TCC1_PWMStop(); // Make sure PWM is stopped before configuring for this test

// Test for U phase high side shorted FET
//////////////////////////////
    
    // Disable all FETs except U phase low
    TCC1_PWMPatternSet(
        (TCC_PATT_PGE1_Msk|TCC_PATT_PGE2_Msk|TCC_PATT_PGE4_Msk|TCC_PATT_PGE5_Msk|TCC_PATT_PGE6_Msk),
        (TCC_PATT_PGE1(0)|TCC_PATT_PGE2(0)|TCC_PATT_PGE4(0)|TCC_PATT_PGE5(0)|TCC_PATT_PGE6(0)));

    TCC1_PWM24bitDutySet(TCC1_CHANNEL0, U_PHASE_HIGH_SIDE_TEST_DUTY  );  // set U phase to low DC, see macro comment for details

    TCC1_PWMForceUpdate();

    // Start PWM and ADC measurements
    TCC1_PWMStart();
    
    // Multiple PWM cycles are needed here to dissipate charge in the RC snubbers
    fetTestMeasurementCounter = 0;
    while(fetTestMeasurementCounter < 7);   // Wait for 7 PWM cycles and use the last measurement

    TCC1_PWMStop();
    
    testPhaseCurrent = (fetTestCurrentMeasurement - adc_0_offset);

    // Check current measurement against u phase threshold
    if ( testPhaseCurrent > SHORTED_FET_CURRENT_THRESHOLD_COUNTS )
    {
        fh_faultHandler(HIGH_SIDE_FET_FAULT); // This function loops forever with this fault code
    }
    // If the U phase is not shorted, extend the PWM pulse and test for other phases
    
// Test for V/W phase high side shorted FET
//////////////////////////////

    TCC1_PWM24bitDutySet(TCC1_CHANNEL0, VW_PHASE_HIGH_SIDE_TEST_DUTY );  // set U phase mid DC to test VW high side, see macro comment for details
    TCC1_PWMForceUpdate();
    
    // Start PWM and ADC measurements
    TCC1_PWMStart();

    fetTestMeasurementCounter = 0;
    while(fetTestMeasurementCounter < 4);    // Wait for conversions to complete

    TCC1_PWMStop();
    
    testPhaseCurrent = (fetTestCurrentMeasurement - adc_0_offset);

    // Check current measurement against u phase threshold
    if ( testPhaseCurrent > SHORTED_FET_CURRENT_THRESHOLD_COUNTS )
    {
        fh_faultHandler(HIGH_SIDE_FET_FAULT); // This function loops forever with this fault code
    }

// Test for U phase low side shorted FET
//////////////////////////////

    // Disable all FETs except U phase high
    TCC1_PWMPatternSet(
        (TCC_PATT_PGE0_Msk|TCC_PATT_PGE1_Msk|TCC_PATT_PGE2_Msk|TCC_PATT_PGE5_Msk|TCC_PATT_PGE6_Msk),
        (TCC_PATT_PGE0(0)|TCC_PATT_PGE1(0)|TCC_PATT_PGE2(0)|TCC_PATT_PGE5(0)|TCC_PATT_PGE6(0)));

    TCC1_REGS->TCC_WAVE &= ~TCC_WAVE_WAVEGEN_Msk;   // Clear previous WAVEGEN config
    TCC1_REGS->TCC_WAVE |= TCC_WAVE_WAVEGEN_DSBOTH; // Configure for top and bottom events
                                                    // **This is done so ADC captures occur when
                                                    //   the FET is on and when its off

    // First, check for U phase short with a 3% duty pulse
    TCC1_PWM24bitDutySet(TCC1_CHANNEL0, U_PHASE_SHORT_TEST_DUTY );  // set U phase duty
    TCC1_PWMStart();    // Start PWM and ADC measurements

    // Take two ADC captures per pulse since there are two captures per PWM period
    // The first pulse doesn't get measured well by the ADC, so ignore it and wait for the second pulse
    fetTestMeasurementCounter = 0;
    while(fetTestMeasurementCounter < 4);    // Wait for conversions to complete
    
    TCC1_PWMStop();
    testPhaseCurrentHighOn = (fetTestCurrentMeasurement - adc_0_offset);
    // Check current measurement against threshold
    if ( testPhaseCurrentHighOn > SHORTED_FET_CURRENT_THRESHOLD_COUNTS )
    {
        fh_faultHandler(LOW_SIDE_FET_FAULT); // This function loops forever with this fault code
    }

    // If the U phase passes, continue to test V and W phases

// Test for V/W phase low side shorted FET
//////////////////////////////
    // Due to winding impedance, 4 short pulses are used to detect a short on the V or W phases

    TCC1_PWM24bitDutySet(TCC1_CHANNEL0, VW_PHASE_LOW_SIDE_TEST_DUTY );  // set U phase duty
    TCC1_PWMStart();    // Start PWM and ADC measurements
    fetTestMeasurementCounter = 0;
    while(fetTestMeasurementCounter < 1);    // Wait for conversion to complete
    // Throw away first sample because it occurs before the first pwm period

    for (int i = 0; i < 4; i++) //4 pulses
    {
    // When the FET is on (first/odd ADC measurement), we can ignore the ADC capture value because the U phase has already been tested
    // When the FET is off (second/even ADC measurements, the V and W phases are tested by looking for current through the U phase body diode
        fetTestMeasurementCounter = 0;
        while(fetTestMeasurementCounter < 2);    // Wait for conversion to complete

        testPhaseCurrentHighOff[i] = (fetTestCurrentMeasurement - adc_1_offset);    // Save measurement in array
    }
    
    TCC1_PWMStop();
    TCC1_PWM24bitDutySet(TCC1_CHANNEL0, PWM_PERIOD_COUNT);  // Set PWM duty to 50%
    
    for (int i = 0; i < 3; i++)
    {
        if ( (testPhaseCurrentHighOff[i+1] + 5) > testPhaseCurrentHighOff[i])
            //next current reading must be at least 5 counts greater than the last for it to pass
        {
        // The current is not growing more negative, then we do not detect a V or W phase short
            break;
        }
        else if (i == 2)//last check did not pass the test TODO Kyle review
        {
        // We always detected an increasing (in magnitude) current, so we have a V or W phase short
            fh_faultHandler(LOW_SIDE_FET_FAULT); // This function loops forever with this fault code
        }
    }


// if PASS, then set ADC ISR, enable ADC, reset PWM settings, and return
//////////////////////////////
    ADC0_Disable();
    ADC0_CallbackRegister((ADC_CALLBACK) mcApp_ADCISRTasks, (uintptr_t)NULL);
    ADC0_Enable();
    
    // Reset the configuration changes we made for this test
    TCC1_REGS->TCC_WAVE &= ~TCC_WAVE_WAVEGEN_Msk;   // Clear previous WAVEGEN config
    TCC1_REGS->TCC_WAVE |= TCC_WAVE_WAVEGEN_DSTOP; // Configure for top and bottom events
    mcApp_inverterDisable();  // Reset TCC settings back to normal
    TCC1_PWMStart();
}

// Test FET temperature against threshold
// **This function blocks if test fails
void fh_FetTemperatureTest()
{
    if ( (fetTemp1 > FET_TEMPERATURE_MAX_THRESHOLD_DEGC) ||
         (fetTemp2 > FET_TEMPERATURE_MAX_THRESHOLD_DEGC) )
    {
        fh_faultHandler(FET_TEMPERATURE_FAULT);
    }
    //else do nothing and return
}

// Test bus voltage against threshold
// **This function blocks if test fails
void fh_BusUnderVoltageTest()
{
    if (busVoltage < DCBUS_UNDERVOLTAGE_THRESHOLD_VOLTS)
    {
        underVoltageFaultDirection = Motor_Direction_Get();
        fh_faultHandler(BUS_UNDERVOLTAGE_FAULT);
    }
}

//only called when a fault is detected
/*******************************************************************************
* Function: fh_faultHandler
*
* Arguments: fault_code - Number representing the fault that was detected
*                          
*
* Returns: none
*
* Description: Handle drive faults and keep track of the number of faults
*              detected since last power up.
*
* Original file: 
* Original authors: Kyle Reu
*
* Function created:
* Product Release History:
*
* Copyright 2021 The Chamberlain Group, Inc. All Rights Reserved.
*******************************************************************************/
void fh_faultHandler(uint8_t fault_code)
{
    mcApp_inverterDisable();

    drive_fault_status.faultCount++;
    drive_fault_status.faultCode = fault_code;

    switch (fault_code)
    {
    case BRAKE_INPUT_ACTIVE:
        //will not be called for this legacy "fault"
        break;

    case OPAMP_FAULT:
    case LOW_SIDE_FET_FAULT:
    case HIGH_SIDE_FET_FAULT:
    case ID_RESISTOR_FAULT:
        //****************************************************************
        //****WARNING FATAL ERRORS: must stay here until clean reboot*****
        //****************************************************************
        PDEC_HALLStop(); //disable HALL interrupt (could lead to HALL fault, which can recover)
        while (RUN_FOREVER)
        {
            // Do not allow any motor functionality until board reboots without this fault
            WDT_Clear();
            X2CScope_Communicate();
            X2CScope_Update();
        }

        break;
    case BUS_UNDERVOLTAGE_FAULT:
        //WARNING: BLOCKING HERE. NO MOTION OR OTHER FUNCTIONALITY ALLOWED UNTIL BUS VOLTAGE RECOVERED AND ENABLE OR DIRECTION TOGGLE
        // Wait for enable or direction toggle before continuing, direction toggle is to handle undervoltage on reversal/obstruction
        while ( ((0 == Motor_Enable_Get()) && (underVoltageFaultDirection == Motor_Direction_Get())) // (Enable line is active-low at MCU pin)
           ||  (busVoltage <= DCBUS_UNDERVOLTAGE_RECOVERY_VOLTS) ) //if busVoltage is low, stay in here regardless of other toggling
        {
            WDT_Clear();
            X2CScope_Communicate();
            X2CScope_Update();
        }

        //recovery
        drive_fault_status.faultCode = IDLE;
        break;

    case FET_TEMPERATURE_FAULT:
        //WARNING: BLOCKING HERE. NO MOTION OR OTHER FUNCTIONALITY ALLOWED UNTIL FET TEMP RECOVERED AND ENABLE TOGGLE
        while ( (0 == Motor_Enable_Get()) // (Enable line is active-low at MCU pin)
           ||  (fetTemp2 >= FET_TEMPERATURE_RECOVERY_DEGC) || (fetTemp1 >= FET_TEMPERATURE_RECOVERY_DEGC) ) //if fetTemp is too high, stay in here regardless of enable toggling
        {
            WDT_Clear();
            X2CScope_Communicate();
            X2CScope_Update();
        }

        //recovery
        drive_fault_status.faultCode = IDLE;
        break;

    case MOTOR_STALL:
        timer_stall_detect_count = 0;   // Reset stall timer
        //WARNING: BLOCKING HERE. NO MOTION OR OTHER FUNCTIONALITY ALLOWED UNTIL ENABLE OR DIRECTION TOGGLE
        // Wait for enable or direction toggle before continuing, direction toggle is to handle reversal/obstruction
        while ((0 == Motor_Enable_Get()) && (motorStallDirection == Motor_Direction_Get()))
        {
            WDT_Clear();
            X2CScope_Communicate();
            X2CScope_Update();
        }

        //recovery
        drive_fault_status.faultCode = IDLE;
        break;

    case HALL_SENSOR_FAULT:
       //lower encoder voltage setting
       HALL_Voltage_Select_Clear();

        //WARNING: NO MOTION OR OTHER FUNCTIONALITY ALLOWED UNTIL ENABLE TOGGLE, BLOCKING HERE
        ADC0_Disable();
        while ( (0 == Motor_Enable_Get()) || (0 == CheckIsHallValueValid()) )    // Wait for enable switch to go low before continuing (Enable line is active-low at MCU pin)
        {
            WDT_Clear();
            X2CScope_Communicate();
            X2CScope_Update();
        }
        
        ADC0_Enable();
        drive_fault_status.faultCode = IDLE;

        //Hall sensor reconnected, maybe motor changed? check ID resistor again
        ADC_check_motor_ID();
        bi_check_id_error(); //also check for error now that motor may have changed

        break;
    default:
        break;
    }

    return;
}

void TC4_TimerInitialize_FaultCodes( void )
{
    /* Reset TC */
    TC4_REGS->COUNT16.TC_CTRLA = TC_CTRLA_SWRST_Msk;

    while((TC4_REGS->COUNT16.TC_SYNCBUSY & TC_SYNCBUSY_SWRST_Msk) == TC_SYNCBUSY_SWRST_Msk)
    {
        /* Wait for Write Synchronization */
    }

    /* Configure counter mode & prescaler */
    TC4_REGS->COUNT16.TC_CTRLA = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV64 | TC_CTRLA_PRESCSYNC_PRESC ;

    /* Configure in Match Frequency Mode */
    TC4_REGS->COUNT16.TC_WAVE = (uint8_t)TC_WAVE_WAVEGEN_MPWM;

    /* Configure timer period */
    TC4_REGS->COUNT16.TC_CC[0U] = 4687U;

    /* Clear all interrupt flags */
    TC4_REGS->COUNT16.TC_INTFLAG = (uint8_t)TC_INTFLAG_Msk;

//    TC4_CallbackObject.callback = NULL;
    /* Enable interrupt*/
    TC4_REGS->COUNT16.TC_INTENSET = (uint8_t)(TC_INTENSET_OVF_Msk);


    while((TC4_REGS->COUNT16.TC_SYNCBUSY) != 0U)
    {
        /* Wait for Write Synchronization */
    }

    TC4_TimerCallbackRegister((TC_TIMER_CALLBACK)fh_TC4_ISR, (uintptr_t)NULL); // TC for drive fault output NOSONAR - microchip function
}


/*******************************************************************************
* Function: fh_faultHandler
*
* Arguments: status - TC status provided by the NVIC
*            context - provided by the NVIC
*
* Returns: none
*
* Description: Controls the Fault output pin to the control board.  This ISR is
*              called every 100ms by TC4 OVF overflow interrupt
*
* Original file: 
* Original authors: Kyle Reu
*
* Function created:
* Product Release History:
*
* Copyright 2021 The Chamberlain Group, Inc. All Rights Reserved.
*******************************************************************************/
// Fault Line output pin ISR, runs every ~100ms
void fh_TC4_ISR(TC_TIMER_STATUS status, uintptr_t context)
{
    volatile static uint8_t outputPulseCounter = 0; //pulses for fault codes
    volatile static uint8_t outputHeaderCounter = 0; //dead time between fault codes
    volatile static uint8_t outputPinState = 1;


    if (0 == outputPinState)
    {
        Drive_Fault_Out_Set();
        //Debug_LED_Set();
        outputPinState = 1;
    }
    else
    {// Output is high
        if (outputHeaderCounter < 2) //stay high between fault codes for 300ms
        {
            outputHeaderCounter++;
        }
        else if (outputPulseCounter < drive_fault_status.faultCode)
        {
            Drive_Fault_Out_Clear();
            //Debug_LED_Clear();
            outputPinState = 0;
            outputPulseCounter++;
        }
        else //adds 100 ms
        {
            // Output sequence complete, reset counters
            outputPulseCounter = 0;
            outputHeaderCounter = 0;
        }
        
        
    }
    
}

void OC_FAULT_ISR(uintptr_t context)
{
    mcApp_inverterDisable();

    while(1)
    {
        X2CScope_Communicate();
        X2CScope_Update();
    }
    
}

/* *****************************************************************************
 End of File
 */
