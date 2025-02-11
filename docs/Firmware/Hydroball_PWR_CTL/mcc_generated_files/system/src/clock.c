/**
  * CLKCTRL Generated Driver File
  *
  * @file clkctrl.c
  *
  * @ingroup clkctrl
  *
  * @brief This file contains the driver code for CLKCTRL module.
  *
  * @version CLKCTRL Driver Version 1.0.4
  *
  * @version Package Version 3.2.16
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


#include "../clock.h"

void CLOCK_Initialize(void)
{
    ccp_write_io((void*)&(CLKCTRL.MCLKCTRLA),(0 << CLKCTRL_CLKOUT_bp)   // CLKOUT disabled
            | 0x0   // CLKSEL 20MHz oscillator
            );
    ccp_write_io((void*)&(CLKCTRL.MCLKCTRLB),CLKCTRL_PDIV_6X_gc   // PDIV 6X
            | (1 << CLKCTRL_PEN_bp)   // PEN enabled
            );
    ccp_write_io((void*)&(CLKCTRL.MCLKLOCK),(0 << CLKCTRL_LOCKEN_bp)   // LOCKEN disabled
            );
    ccp_write_io((void*)&(CLKCTRL.OSC20MCTRLA),(0 << CLKCTRL_RUNSTDBY_bp)   // RUNSTDBY disabled
            );
    ccp_write_io((void*)&(CLKCTRL.OSC32KCTRLA),(0 << CLKCTRL_RUNSTDBY_bp)   // RUNSTDBY disabled
            );
    ccp_write_io((void*)&(CLKCTRL.XOSC32KCTRLA),(0 << CLKCTRL_ENABLE_bp)   // ENABLE disabled
            | CLKCTRL_CSUT_1K_gc   // CSUT 1K
            | (0 << CLKCTRL_RUNSTDBY_bp)   // RUNSTDBY disabled
            | (0 << CLKCTRL_SEL_bp)   // SEL disabled
            );
}

/**
 End of File
*/