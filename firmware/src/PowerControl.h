/* ************************************************************************** */
/** Power Rail Control

  @Company
    The Chamberlain Group

  @File Name
    PowerRailControl.h

  @Summary
    Header file for PowerRailControl.c

  @Description
    This file provides control of the 3.3VSW and 12VSW switched power rails
 */
/* ************************************************************************** */

#ifndef POWER_CONTROL_H    /* Guard against multiple inclusion */
#define POWER_CONTROL_H

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

void turnOn3v3SwRail();
void turnOff3v3SwRail();
uint8_t getState3v3SwRail();

void turnOn12VSwRail();
void turnOff12VSwRail();
uint8_t getState12VSwRail();

void turnOnBothSwRails();
void turnOffBothSwRails();



#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
