/* ************************************************************************** */
/** HALL.H

  @Company
    Chamberlain Group Inc

  @File Name
    hall.h

  @Summary
    Header file for hall sensors

 */
/* ************************************************************************** */

#ifndef _HALL_H    /* Guard against multiple inclusion */
#define _HALL_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "userparams.h"
#include "mc_Lib.h"
#include "mc_app.h"

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

#define STT_HALL_ISR_INI 0u
#define STT_HALL_ISR_1ST_JUMP 1u
#define STT_HALL_ISR_ONE_JUMP_LOW_SPEED 2u
#define STT_HALL_ISR_ONE_JUMP_HIGH_SPEED 3u
#define STT_HALL_ISR_SIX_JUMP 4u

#ifdef UNIT_TEST

#ifdef NVIC_EnableIRQ
#undef NVIC_EnableIRQ
#endif
#define NVIC_EnableIRQ(PDEC_OTHER_IRQn)
#ifdef NVIC_DisableIRQ
#undef NVIC_DisableIRQ
#endif
#define NVIC_DisableIRQ(PDEC_OTHER_IRQn)

bool ut_port_pin_read(PORT_PIN p);

#define PORT_PinRead(PORT_PIN)  (ut_port_pin_read(PORT_PIN))

#endif

    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************

tagHALLdata             HALLdata;

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

extern void resetHALLdata(tagHALLdata * ptData);

extern void latchHALLdata(tagHALLdata * ptData);

extern void hall_sensor_init(void);

extern uint8_t CheckIsHallValueValid(void);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _HALL_H */

/* *****************************************************************************
 End of File
 */
