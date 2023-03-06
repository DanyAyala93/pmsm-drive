/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include "mc_app.h"
#include "userparams.h"
#include "mc_Lib.h"
#include "PowerControl.h"
#include "BatteryCharger.h"
#include "microcontroller.h"
#include "board_id.h"
 
// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    delay_10ms.period = DELAY_10MS_COUNT;

    turnOn12VSwRail();  // Start with 12VSW enabled
    turnOn3v3SwRail();  // Start with 3.3VSW enabled

    WDT_Clear();
    mcApp_DriveInitialize();
    WDT_Clear();
    ADC_wait_for_cal(); // Make sure calibration has completed THIS IS BLOCKING, takes a long time
    WDT_Clear();
    fh_shortedFetTest();
    ADC_check_motor_ID();
    WDT_Disable();
    bi_determine_board_id(); //TODO Olivia confirm timing
    WDT_Enable();

// Send version info to main board
    while (1 == Motor_Enable_Get())  // Wait for enable to go high before starting version tx
    {
        WDT_Clear();
    }
    
    delay_us(10000);    // Delay for main board
    
    Drive_Fault_Out_Set();  // Start high
    TC4_TimerCallbackRegister((TC_TIMER_CALLBACK)bi_versionInfo_ISR, (uintptr_t)NULL); //NOSONAR - microchip function 
    TC4_TimerStart(); // Start the timer for version tx

    // Wait for version comm to finish (ISR will disable timer when complete)
    while (TC4_REGS->COUNT16.TC_CTRLA & TC_CTRLA_ENABLE_Msk)
    {
        WDT_Clear();
    }

    TC4_TimerStop();

    TC4_TimerInitialize_FaultCodes(); // Reinitialize for fault code comm
    TC4_TimerStart(); //start timer for fault comms

    WDT_Clear();
    bi_check_id_error(); //check for id error and calls fault handler, must be run after version comms done

// Init pins not used yet
    Dump_Resistor_FET_Control_Clear();

    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
        X2CScope_Communicate();

       if(delay_10ms.count>delay_10ms.period)
       {
            // This runs every 10ms
            delay_10ms.count = 0;
            WDT_Clear();

            if (motorIsMoving_f && (1 == Motor_Enable_Get())) //extra enable checking, just in case - we must stop if disabled
            {
               mcApp_inverterDisable();
            }

            mcApp_DumpResistorControl();// Control overvoltage with dump resistor **includes counter that depends on this function staying here**
            fh_FetTemperatureTest();    // Check FET temperature every 10ms
            fh_BusUnderVoltageTest();   // Test bus voltage
            mcApp_StallDetection();     // Detect a stalled motor
            mcApp_decideLowPowerMode();  // Enter low power mode if conditions are met
            
            mcApp_SpeedRamp();  // Process enable and speedCommand inputs
        }
       else
       {
           
       }
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}

