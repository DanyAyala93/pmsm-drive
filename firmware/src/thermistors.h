/** @file thermistors.h
 * @brief Header file for thermistors.c
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

#ifndef THERMISTORS_H
#define THERMISTORS_H

#include <stdint.h>

uint16_t thermistorGetFetTemp1(uint16_t AdcVal);
uint16_t thermistorGetFetTemp2(uint16_t AdcVal);


#endif /* THERMISTORS_H */

