/** 
 * @file        pas-co2-serial-rpi.h
 * @brief       PAS CO2 Serial Arduino API
 * @copyright   Copyright (c) 2020-2021 Infineon Technologies AG
 *              
 * SPDX-License-Identifier: MIT
 */

#ifndef XENSIVECO2_H_
#define XENSIVECO2_H_


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h> // Error integer and strerror() function
#include <unistd.h>
#include <termios.h>
#include <linux/types.h>
#include <linux/i2c-dev.h>
#include "xensiv_pasco2.h"


#define RPI_ASSERT(x)   do {                \
                            if(!(x))        \
                            {               \
                                abort();    \
                            }               \
                        } while(false)
#define RPI_ASSERT_RET(x)   if( x != XENSIV_PASCO2_OK ) { return x; }


typedef int32_t Error_t;
typedef xensiv_pasco2_status_t Diag_t;
typedef xensiv_pasco2_boc_cfg_t ABOC_t;

Error_t begin           (bool i2c, bool uart);
Error_t end             (bool i2c, bool uart);
Error_t startMeasure    (int16_t  periodInSec, int16_t alarmTh, void (*cback) (void *));
Error_t stopMeasure     ();
Error_t getCO2          (uint16_t * CO2PPM);
Error_t getDiagnosis    (Diag_t * diagnosis);
Error_t setABOC         (ABOC_t aboc, int16_t abocRef);
Error_t setPressRef     (uint16_t pressRef);
Error_t reset           ();
Error_t getDeviceID     (uint8_t * prodID, uint8_t * revID);


#endif //XENSIVECO2_H_

