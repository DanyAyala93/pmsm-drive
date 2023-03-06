/* ************************************************************************** */
/** Power Rail Control

  @Company
    The Chamberlain Group

  @File Name
    PowerRailControl.c

  @Summary
    Power rail control functions

  @Description
    This file contains functions for controlling the 3.3VSW and 12VSW switched
    power rails.
 */
/* ************************************************************************** */

#include <stdbool.h>
#include <time.h>

#include "PowerControl.h"
#include "definitions.h"


void turnOn3v3SwRail()
{
   PwrRail_3p3VSW_Enable_Set();
   return;
}

void turnOff3v3SwRail()
{
   PwrRail_3p3VSW_Enable_Clear();
   return;
}

uint8_t getState3v3SwRail()
{
   return PwrRail_3p3VSW_Enable_Get();
}

void turnOn12VSwRail()
{
    PwrRail_12VSW_Enable_Clear();
    return;
}

void turnOff12VSwRail()
{
    PwrRail_12VSW_Enable_Set();
    return;
}

uint8_t getState12VSwRail()
{
    return PwrRail_12VSW_Enable_Get();
}

//void turnOnBothSwRails()
//{
//    PwrRail_3p3VSW_Enable_Set();
//    PwrRail_12VSW_Enable_Set();
//    
//    return;
//}
//
//void turnOffBothSwRails()
//{
//    PwrRail_3p3VSW_Enable_Clear();
//    PwrRail_12VSW_Enable_Clear();
//    
//    return;
//}