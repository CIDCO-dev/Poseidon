/**
 * System Driver Header File
 * 
 * @file system.h
 * 
 * @defgroup systemdriver System Driver
 * 
 * @brief This file contains the API prototype for the System Driver.
 *
 * @version Driver Version 1.0.2
 *
 * @version Package Version 3.1.7
*/
/*
© [2025] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/


#ifndef MCC_H
#define	MCC_H

#ifdef __cplusplus
extern "C" {
#endif

/**
  Section: Included Files
*/
#include "../system/utils/compiler.h"
#include "../system/clock.h"
#include "../system/pins.h"
#include "../system/interrupt.h"
#include "../system/clock.h"
/**
 * @ingroup systemdriver
 * @brief This initializes the system module  This routine is called only once during system initialization, before calling any other API.
 * @param None
 * @return None
*/
void SYSTEM_Initialize(void);

#ifdef __cplusplus
}
#endif
#endif	/* MCC_H */
/**
 End of File
*/