/**
 * Generated Driver File
 * 
 * @file pins.c
 * 
 * @ingroup  pinsdriver
 * 
 * @brief This is generated driver implementation for pins. 
 *        This file provides implementations for pin APIs for all pins selected in the GUI.
 *
 * @version Driver Version 1.1.0
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

#include "../pins.h"

static void (*Auto_on_InterruptHandler)(void);
static void (*Signal_InterruptHandler)(void);
static void (*Pwr_mon_InterruptHandler)(void);
static void (*Pulse_InterruptHandler)(void);
static void (*Led_InterruptHandler)(void);
static void (*Pwr_ctl_InterruptHandler)(void);
static void (*Pwr_off_InterruptHandler)(void);

void PIN_MANAGER_Initialize()
{

  /* OUT Registers Initialization */
    PORTA.OUT = 0x0;
    PORTB.OUT = 0x0;
    PORTC.OUT = 0x0;

  /* DIR Registers Initialization */
    PORTA.DIR = 0x6;
    PORTB.DIR = 0x1;
    PORTC.DIR = 0x0;

  /* PINxCTRL registers Initialization */
    PORTA.PIN0CTRL = 0x0;
    PORTA.PIN1CTRL = 0x0;
    PORTA.PIN2CTRL = 0x0;
    PORTA.PIN3CTRL = 0x80;
    PORTA.PIN4CTRL = 0x80;
    PORTA.PIN5CTRL = 0x0;
    PORTA.PIN6CTRL = 0x0;
    PORTA.PIN7CTRL = 0x0;
    PORTB.PIN0CTRL = 0x88;
    PORTB.PIN1CTRL = 0x0;
    PORTB.PIN2CTRL = 0x0;
    PORTB.PIN3CTRL = 0x0;
    PORTB.PIN4CTRL = 0x0;
    PORTB.PIN5CTRL = 0x0;
    PORTB.PIN6CTRL = 0x0;
    PORTB.PIN7CTRL = 0x0;
    PORTC.PIN0CTRL = 0x0;
    PORTC.PIN1CTRL = 0x0;
    PORTC.PIN2CTRL = 0x0;
    PORTC.PIN3CTRL = 0x0;
    PORTC.PIN4CTRL = 0x0;
    PORTC.PIN5CTRL = 0x0;
    PORTC.PIN6CTRL = 0x0;
    PORTC.PIN7CTRL = 0x0;

  /* PORTMUX Initialization */
    PORTMUX.CCLROUTEA = 0x0;
    PORTMUX.EVSYSROUTEA = 0x0;
    PORTMUX.SPIROUTEA = 0x0;
    PORTMUX.TCAROUTEA = 0x0;
    PORTMUX.TCBROUTEA = 0x0;
    PORTMUX.USARTROUTEA = 0x0;

  // register default ISC callback functions at runtime; use these methods to register a custom function
    Auto_on_SetInterruptHandler(Auto_on_DefaultInterruptHandler);
    Signal_SetInterruptHandler(Signal_DefaultInterruptHandler);
    Pwr_mon_SetInterruptHandler(Pwr_mon_DefaultInterruptHandler);
    Pulse_SetInterruptHandler(Pulse_DefaultInterruptHandler);
    Led_SetInterruptHandler(Led_DefaultInterruptHandler);
    Pwr_ctl_SetInterruptHandler(Pwr_ctl_DefaultInterruptHandler);
    Pwr_off_SetInterruptHandler(Pwr_off_DefaultInterruptHandler);
}

/**
  Allows selecting an interrupt handler for Auto_on at application runtime
*/
void Auto_on_SetInterruptHandler(void (* interruptHandler)(void)) 
{
    Auto_on_InterruptHandler = interruptHandler;
}

void Auto_on_DefaultInterruptHandler(void)
{
    // add your Auto_on interrupt custom code
    // or set custom function using Auto_on_SetInterruptHandler()
}
/**
  Allows selecting an interrupt handler for Signal at application runtime
*/
void Signal_SetInterruptHandler(void (* interruptHandler)(void)) 
{
    Signal_InterruptHandler = interruptHandler;
}

void Signal_DefaultInterruptHandler(void)
{
    // add your Signal interrupt custom code
    // or set custom function using Signal_SetInterruptHandler()
}
/**
  Allows selecting an interrupt handler for Pwr_mon at application runtime
*/
void Pwr_mon_SetInterruptHandler(void (* interruptHandler)(void)) 
{
    Pwr_mon_InterruptHandler = interruptHandler;
}

void Pwr_mon_DefaultInterruptHandler(void)
{
    // add your Pwr_mon interrupt custom code
    // or set custom function using Pwr_mon_SetInterruptHandler()
}
/**
  Allows selecting an interrupt handler for Pulse at application runtime
*/
void Pulse_SetInterruptHandler(void (* interruptHandler)(void)) 
{
    Pulse_InterruptHandler = interruptHandler;
}

void Pulse_DefaultInterruptHandler(void)
{
    // add your Pulse interrupt custom code
    // or set custom function using Pulse_SetInterruptHandler()
}
/**
  Allows selecting an interrupt handler for Led at application runtime
*/
void Led_SetInterruptHandler(void (* interruptHandler)(void)) 
{
    Led_InterruptHandler = interruptHandler;
}

void Led_DefaultInterruptHandler(void)
{
    // add your Led interrupt custom code
    // or set custom function using Led_SetInterruptHandler()
}
/**
  Allows selecting an interrupt handler for Pwr_ctl at application runtime
*/
void Pwr_ctl_SetInterruptHandler(void (* interruptHandler)(void)) 
{
    Pwr_ctl_InterruptHandler = interruptHandler;
}

void Pwr_ctl_DefaultInterruptHandler(void)
{
    // add your Pwr_ctl interrupt custom code
    // or set custom function using Pwr_ctl_SetInterruptHandler()
}
/**
  Allows selecting an interrupt handler for Pwr_off at application runtime
*/
void Pwr_off_SetInterruptHandler(void (* interruptHandler)(void)) 
{
    Pwr_off_InterruptHandler = interruptHandler;
}

void Pwr_off_DefaultInterruptHandler(void)
{
    // add your Pwr_off interrupt custom code
    // or set custom function using Pwr_off_SetInterruptHandler()
}
ISR(PORTA_PORT_vect)
{ 
    // Call the interrupt handler for the callback registered at runtime
    if(VPORTA.INTFLAGS & PORT_INT3_bm)
    {
       Auto_on_InterruptHandler(); 
    }
    if(VPORTA.INTFLAGS & PORT_INT4_bm)
    {
       Signal_InterruptHandler(); 
    }
    if(VPORTA.INTFLAGS & PORT_INT1_bm)
    {
       Led_InterruptHandler(); 
    }
    if(VPORTA.INTFLAGS & PORT_INT2_bm)
    {
       Pwr_ctl_InterruptHandler(); 
    }
    /* Clear interrupt flags */
    VPORTA.INTFLAGS = 0xff;
}

ISR(PORTB_PORT_vect)
{ 
    // Call the interrupt handler for the callback registered at runtime
    if(VPORTB.INTFLAGS & PORT_INT1_bm)
    {
       Pwr_mon_InterruptHandler(); 
    }
    if(VPORTB.INTFLAGS & PORT_INT2_bm)
    {
       Pulse_InterruptHandler(); 
    }
    if(VPORTB.INTFLAGS & PORT_INT0_bm)
    {
       Pwr_off_InterruptHandler(); 
    }
    /* Clear interrupt flags */
    VPORTB.INTFLAGS = 0xff;
}

ISR(PORTC_PORT_vect)
{ 
    /* Clear interrupt flags */
    VPORTC.INTFLAGS = 0xff;
}

/**
 End of File
*/