/* ************************************************************************** */
/** ADC.C

  @Company
    Chamberlain Group Inc

  @File Name
    adc.c

  @Summary
    Header file for ADC

 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

#include "adc.h"
#include "board_id.h"


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */
/* ************************************************************************** */
//#defines below are taken from Centerpiece code base
// Value in ohms for the ID resistors
#define ID_PULL_UP_OHMS    750.

#define IDRES_0_OHM         00.000000001    //  no such thing as zero, so says pc-lnt.
#define IDRES_1_OHM         50.                       // white
#define IDRES_2_OHM        107.                       // black
#define IDRES_3_OHM        174.                       // brown
#define IDRES_4_OHM        249.                       // orange
#define IDRES_5_OHM        340.                       // undefined
#define IDRES_6_OHM        453.                       // green
#define IDRES_7_OHM        590.                       // red
#define IDRES_8_OHM        750.                       // grey
#define IDRES_9_OHM        953.                       // undefined
#define IDRES_10_OHM      1240.                       // yellow
#define IDRES_11_OHM      1650.                       // undefined
#define IDRES_12_OHM      2260.                       // purple
#define IDRES_13_OHM      3240.                       // undefined
#define IDRES_14_OHM      5230.                       // blue
#define IDRES_15_OHM     11300.                       // undefined
#define BFR_OHM         999999.

// Macro to compute intermediate result for voltage divider
#define ID_VDIV_M( res1)          ( res1 /  (res1 + ID_PULL_UP_OHMS) )

// Macro to compute the midpoint between two voltage dividers
#define ID_VALUE_M(resA, resB)      (uint16_t)(((ID_VDIV_M(resA) + ID_VDIV_M(resB))   /2 ) * MAX_ADC_INPUT_VOLTAGE * ADC_VOLTS_TO_BITS )

//ID harness thresholds in A/D counts
#define  IDLOW_ERROR  ID_VALUE_M(  IDRES_0_OHM,  IDRES_1_OHM)
#define  ID1          ID_VALUE_M(  IDRES_1_OHM,  IDRES_2_OHM)
#define  ID2          ID_VALUE_M(  IDRES_2_OHM,  IDRES_3_OHM)
#define  ID3          ID_VALUE_M(  IDRES_3_OHM,  IDRES_4_OHM)
#define  ID4          ID_VALUE_M(  IDRES_4_OHM,  IDRES_5_OHM)
#define  ID5          ID_VALUE_M(  IDRES_5_OHM,  IDRES_6_OHM)
#define  ID6          ID_VALUE_M(  IDRES_6_OHM,  IDRES_7_OHM)
#define  ID7          ID_VALUE_M(  IDRES_7_OHM,  IDRES_8_OHM)
#define  ID8          ID_VALUE_M(  IDRES_8_OHM,  IDRES_9_OHM)
#define  ID9          ID_VALUE_M(  IDRES_9_OHM, IDRES_10_OHM)
#define  ID10         ID_VALUE_M( IDRES_10_OHM, IDRES_11_OHM)
#define  ID11         ID_VALUE_M( IDRES_11_OHM, IDRES_12_OHM)
#define  ID12         ID_VALUE_M( IDRES_12_OHM, IDRES_13_OHM)
#define  ID13         ID_VALUE_M( IDRES_13_OHM, IDRES_14_OHM)
#define  ID14         ID_VALUE_M( IDRES_14_OHM, IDRES_15_OHM)
#define  ID15         ID_VALUE_M( IDRES_15_OHM, BFR_OHM )

ADC_POSINPUT secondaryAdc0Channels[3] = {
    SPEED_CMD_ADC_POSINPUT,
    FET_THMSTR2_ADC_POSINPUT,
    SPEED_CMD_ADC_POSINPUT  // blank space available for other channel if needed
};

ADC_POSINPUT secondaryAdc1Channels[3] = {
    BUS_CURRENT_ADC_POSINPUT,
    FET_THMSTR1_ADC_POSINPUT,
    VIN_ADC_POSINPUT
};

uint32_t                adc_0_sum = 0;
uint32_t                adc_1_sum = 0;

uint16_t                calibration_sample_count = 0x0000U;


bool adc_motor_id_error_f = false; //motor id error

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */
//Prototypes
void ADC_CALIB_IBUS_ISR(ADC_STATUS status, uintptr_t context);

/* This ISR calibrates zero crossing point for Phase U and Phase V currents*/
void ADC_CALIB_IPHASE_ISR (ADC_STATUS status, uintptr_t context)
{
    X2CScope_Update();
    calibration_sample_count++;
    if(calibration_sample_count <= 4096) //size of sum? should replace TODO Olivia
    {
        adc_0_sum += ADC0_ConversionResultGet();    
        adc_1_sum += ADC1_ConversionResultGet();
    }
    else
    {
        adc_0_offset = adc_0_sum>>12;
        adc_1_offset = adc_1_sum>>12;

        adc_1_sum = 0;
        //adc_0_sum not used after this

        calibration_sample_count = 0;

        //TODO look into this when there is more time for test, probably each adc should have its own interrupt configured for when a new result is ready
        //we set up the ADC0 to interrupt when a new result is ready. Then, we read both ADC0 and ADC1 measurements for use in the motor control loop
        ADC1_ChannelSelect(BUS_CURRENT_ADC_POSINPUT,ADC_NEGINPUT_GND); // Phase U to ADC1
        ADC0_CallbackRegister((ADC_CALLBACK) ADC_CALIB_IBUS_ISR, (uintptr_t)NULL);
    }
 
}

/* This ISR calibrates zero crossing point for Phase U and Phase V currents*/
void ADC_CALIB_IBUS_ISR (ADC_STATUS status, uintptr_t context)
{
    X2CScope_Update();
    calibration_sample_count++;
    if(calibration_sample_count <= 4096)
    {
        //TODO look into this when there is more time for test, probably each adc should have its own interrupt configured for when a new result is ready
        //we set up the ADC0 to interrupt when a new result is ready. Then, we read both ADC0 and ADC1 measurements for use in the motor control loop
        ADC0_InterruptsClear(ADC_INTFLAG_RESRDY_Msk);
        adc_1_sum += ADC1_ConversionResultGet();
    }
    else
    {
        adc_bus_current_offset = adc_1_sum>>12;

        // Verify offsets are within range
        if ( (adc_0_offset < PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS) ||
             (adc_1_offset < PHASE_CURNT_OFFSET_MIN_THRESHOLD_COUNTS) ||
             (adc_0_offset > PHASE_CURNT_OFFSET_MAX_THRESHOLD_COUNTS) ||
             (adc_1_offset > PHASE_CURNT_OFFSET_MAX_THRESHOLD_COUNTS) ||
             (adc_bus_current_offset < BUS_CURNT_OFFSET_MIN_THRESHOLD_COUNTS) ||
             (adc_bus_current_offset > BUS_CURNT_OFFSET_MAX_THRESHOLD_COUNTS) )
        {
            fh_faultHandler(OPAMP_FAULT);
        }

        ADC0_CallbackRegister((ADC_CALLBACK) fh_ShortedFetISRTasks, (uintptr_t)NULL);
    }
 
}

//BLOCKING FATAL ERROR IF TRAPPED HERE
void ADC_wait_for_cal(void)
{
    while(0 == adc_bus_current_offset);
    //OLIVIA TODO consider adding fault code here
}

void ADC_check_motor_ID(void)
{
    ADC0_REGS->ADC_INTENCLR = ADC_INTFLAG_RESRDY_Msk;
    ADC0_ChannelSelect(MOTOR_ID_ADC_POSINPUT, ADC_NEGINPUT_GND);
    ADC0_ConversionStart();
    // Wait for ADC result to be ready NOTE **BLOCKING** we can't function without this being completed successfully
    // so we are okay with blocking for this to complete (unit is nonfunctional without it)
    while(0 == (ADC0_REGS->ADC_INTFLAG & ADC_INTFLAG_RESRDY_Msk));

    ADC0_REGS->ADC_INTFLAG = ADC_INTFLAG_RESRDY_Msk; //clear result ready interrupt flag (user manual says setting to 1 clears this)

    //discard first result, ADC might have just been enabled so we need to wait for result to stabilize
    //get a new reading and use that to make sure it is accurate
    ADC0_ConversionStart();
    // Wait for ADC result to be ready
    while(0 == (ADC0_REGS->ADC_INTFLAG & ADC_INTFLAG_RESRDY_Msk));

    volatile uint16_t motor_id = ADC0_ConversionResultGet();
    ADC0_REGS->ADC_INTENSET = ADC_INTFLAG_RESRDY_Msk;// Enable ADC interrupt

    //ID Specific - taken from Centerpiece, leaving all IDs separate to add more easily
    if (motor_id < IDLOW_ERROR)
    {
        adc_motor_id_error_f = true;
    }
    else if (motor_id < ID1)     //White
    {
        adc_motor_id_error_f = true;
    }
    else if (motor_id < ID2)     //Black
    {
        adc_motor_id_error_f = true;
    }
    else if (motor_id < ID3)     //Brown
    {
        adc_motor_id_error_f = true;
    }
    else if (motor_id < ID4)     //Orange
    {
        adc_motor_id_error_f = true;
    }
    else if (motor_id < ID5)     //undefined color - Heavy motor
    {
       //Heavy motor
       //set motion params, version number info
       versionNumbers[2] = SPARTAN_HEAVY;
       Motor_ID_num = SPARTAN_HEAVY; //sets parameters
       adc_motor_id_error_f = false;
    }
    else if (motor_id < ID6)     //Green
    {
        adc_motor_id_error_f = true;
    }
    else if (motor_id < ID7)     //Red
    {
        adc_motor_id_error_f = true;
    }
    else if (motor_id < ID8)     //Grey
    {
        adc_motor_id_error_f = true;
    }
    else if (motor_id < ID9)     //Undefined - Lite motor
    {
       //Lite motor
       //set motion params, version number info
       versionNumbers[2] = SPARTAN_LITE;
       Motor_ID_num = SPARTAN_LITE; //sets parameters
       adc_motor_id_error_f = false;
       //set encoder voltage higher for lite
       HALL_Voltage_Select_Set();
    }
    else if (motor_id < ID10)    //Yellow
    {
        adc_motor_id_error_f = true;
    }
    else if (motor_id < ID11)    //Undefined
    {
        adc_motor_id_error_f = true;
    }
    else if (motor_id < ID12)    //Purple
    {
        adc_motor_id_error_f = true;
    }
    else if (motor_id < ID13)    //Undefined
    {
        adc_motor_id_error_f = true;
    }
    else if (motor_id < ID14)    //Blue
    {
        adc_motor_id_error_f = true;
    }
    else if (motor_id < ID15)    //Undefined
    {
        adc_motor_id_error_f = true;
    }
    else
    {
        adc_motor_id_error_f = true;
    }

    //adc_motor_id_error will be checked in board/motor id error check function
}

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */


/* *****************************************************************************
 End of File
 */
