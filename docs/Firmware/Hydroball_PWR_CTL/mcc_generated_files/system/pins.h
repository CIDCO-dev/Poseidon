/**
 * Generated Pins header File
 * 
 * @file pins.h
 * 
 * @defgroup  pinsdriver Pins Driver
 * 
 * @brief This is generated driver header for pins. 
 *        This header file provides APIs for all pins selected in the GUI.
 *
 * @version Driver Version  1.1.0
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

#ifndef PINS_H_INCLUDED
#define PINS_H_INCLUDED

#include <avr/io.h>
#include "./port.h"

//get/set Auto_on aliases
#define Auto_on_SetHigh() do { PORTA_OUTSET = 0x8; } while(0)
#define Auto_on_SetLow() do { PORTA_OUTCLR = 0x8; } while(0)
#define Auto_on_Toggle() do { PORTA_OUTTGL = 0x8; } while(0)
#define Auto_on_GetValue() (VPORTA.IN & (0x1 << 3))
#define Auto_on_SetDigitalInput() do { PORTA_DIRCLR = 0x8; } while(0)
#define Auto_on_SetDigitalOutput() do { PORTA_DIRSET = 0x8; } while(0)
#define Auto_on_SetPullUp() do { PORTA_PIN3CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define Auto_on_ResetPullUp() do { PORTA_PIN3CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define Auto_on_SetInverted() do { PORTA_PIN3CTRL  |= PORT_INVEN_bm; } while(0)
#define Auto_on_ResetInverted() do { PORTA_PIN3CTRL  &= ~PORT_INVEN_bm; } while(0)
#define Auto_on_DisableInterruptOnChange() do { PORTA.PIN3CTRL = (PORTA.PIN3CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define Auto_on_EnableInterruptForBothEdges() do { PORTA.PIN3CTRL = (PORTA.PIN3CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define Auto_on_EnableInterruptForRisingEdge() do { PORTA.PIN3CTRL = (PORTA.PIN3CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define Auto_on_EnableInterruptForFallingEdge() do { PORTA.PIN3CTRL = (PORTA.PIN3CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define Auto_on_DisableDigitalInputBuffer() do { PORTA.PIN3CTRL = (PORTA.PIN3CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define Auto_on_EnableInterruptForLowLevelSensing() do { PORTA.PIN3CTRL = (PORTA.PIN3CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)
#define PA3_SetInterruptHandler Auto_on_SetInterruptHandler

//get/set Signal aliases
#define Signal_SetHigh() do { PORTA_OUTSET = 0x10; } while(0)
#define Signal_SetLow() do { PORTA_OUTCLR = 0x10; } while(0)
#define Signal_Toggle() do { PORTA_OUTTGL = 0x10; } while(0)
#define Signal_GetValue() (VPORTA.IN & (0x1 << 4))
#define Signal_SetDigitalInput() do { PORTA_DIRCLR = 0x10; } while(0)
#define Signal_SetDigitalOutput() do { PORTA_DIRSET = 0x10; } while(0)
#define Signal_SetPullUp() do { PORTA_PIN4CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define Signal_ResetPullUp() do { PORTA_PIN4CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define Signal_SetInverted() do { PORTA_PIN4CTRL  |= PORT_INVEN_bm; } while(0)
#define Signal_ResetInverted() do { PORTA_PIN4CTRL  &= ~PORT_INVEN_bm; } while(0)
#define Signal_DisableInterruptOnChange() do { PORTA.PIN4CTRL = (PORTA.PIN4CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define Signal_EnableInterruptForBothEdges() do { PORTA.PIN4CTRL = (PORTA.PIN4CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define Signal_EnableInterruptForRisingEdge() do { PORTA.PIN4CTRL = (PORTA.PIN4CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define Signal_EnableInterruptForFallingEdge() do { PORTA.PIN4CTRL = (PORTA.PIN4CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define Signal_DisableDigitalInputBuffer() do { PORTA.PIN4CTRL = (PORTA.PIN4CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define Signal_EnableInterruptForLowLevelSensing() do { PORTA.PIN4CTRL = (PORTA.PIN4CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)
#define PA4_SetInterruptHandler Signal_SetInterruptHandler

//get/set Pwr_mon aliases
#define Pwr_mon_SetHigh() do { PORTB_OUTSET = 0x2; } while(0)
#define Pwr_mon_SetLow() do { PORTB_OUTCLR = 0x2; } while(0)
#define Pwr_mon_Toggle() do { PORTB_OUTTGL = 0x2; } while(0)
#define Pwr_mon_GetValue() (VPORTB.IN & (0x1 << 1))
#define Pwr_mon_SetDigitalInput() do { PORTB_DIRCLR = 0x2; } while(0)
#define Pwr_mon_SetDigitalOutput() do { PORTB_DIRSET = 0x2; } while(0)
#define Pwr_mon_SetPullUp() do { PORTB_PIN1CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define Pwr_mon_ResetPullUp() do { PORTB_PIN1CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define Pwr_mon_SetInverted() do { PORTB_PIN1CTRL  |= PORT_INVEN_bm; } while(0)
#define Pwr_mon_ResetInverted() do { PORTB_PIN1CTRL  &= ~PORT_INVEN_bm; } while(0)
#define Pwr_mon_DisableInterruptOnChange() do { PORTB.PIN1CTRL = (PORTB.PIN1CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define Pwr_mon_EnableInterruptForBothEdges() do { PORTB.PIN1CTRL = (PORTB.PIN1CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define Pwr_mon_EnableInterruptForRisingEdge() do { PORTB.PIN1CTRL = (PORTB.PIN1CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define Pwr_mon_EnableInterruptForFallingEdge() do { PORTB.PIN1CTRL = (PORTB.PIN1CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define Pwr_mon_DisableDigitalInputBuffer() do { PORTB.PIN1CTRL = (PORTB.PIN1CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define Pwr_mon_EnableInterruptForLowLevelSensing() do { PORTB.PIN1CTRL = (PORTB.PIN1CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)
#define PB1_SetInterruptHandler Pwr_mon_SetInterruptHandler

//get/set Pulse aliases
#define Pulse_SetHigh() do { PORTB_OUTSET = 0x4; } while(0)
#define Pulse_SetLow() do { PORTB_OUTCLR = 0x4; } while(0)
#define Pulse_Toggle() do { PORTB_OUTTGL = 0x4; } while(0)
#define Pulse_GetValue() (VPORTB.IN & (0x1 << 2))
#define Pulse_SetDigitalInput() do { PORTB_DIRCLR = 0x4; } while(0)
#define Pulse_SetDigitalOutput() do { PORTB_DIRSET = 0x4; } while(0)
#define Pulse_SetPullUp() do { PORTB_PIN2CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define Pulse_ResetPullUp() do { PORTB_PIN2CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define Pulse_SetInverted() do { PORTB_PIN2CTRL  |= PORT_INVEN_bm; } while(0)
#define Pulse_ResetInverted() do { PORTB_PIN2CTRL  &= ~PORT_INVEN_bm; } while(0)
#define Pulse_DisableInterruptOnChange() do { PORTB.PIN2CTRL = (PORTB.PIN2CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define Pulse_EnableInterruptForBothEdges() do { PORTB.PIN2CTRL = (PORTB.PIN2CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define Pulse_EnableInterruptForRisingEdge() do { PORTB.PIN2CTRL = (PORTB.PIN2CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define Pulse_EnableInterruptForFallingEdge() do { PORTB.PIN2CTRL = (PORTB.PIN2CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define Pulse_DisableDigitalInputBuffer() do { PORTB.PIN2CTRL = (PORTB.PIN2CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define Pulse_EnableInterruptForLowLevelSensing() do { PORTB.PIN2CTRL = (PORTB.PIN2CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)
#define PB2_SetInterruptHandler Pulse_SetInterruptHandler

//get/set Led aliases
#define Led_SetHigh() do { PORTA_OUTSET = 0x2; } while(0)
#define Led_SetLow() do { PORTA_OUTCLR = 0x2; } while(0)
#define Led_Toggle() do { PORTA_OUTTGL = 0x2; } while(0)
#define Led_GetValue() (VPORTA.IN & (0x1 << 1))
#define Led_SetDigitalInput() do { PORTA_DIRCLR = 0x2; } while(0)
#define Led_SetDigitalOutput() do { PORTA_DIRSET = 0x2; } while(0)
#define Led_SetPullUp() do { PORTA_PIN1CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define Led_ResetPullUp() do { PORTA_PIN1CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define Led_SetInverted() do { PORTA_PIN1CTRL  |= PORT_INVEN_bm; } while(0)
#define Led_ResetInverted() do { PORTA_PIN1CTRL  &= ~PORT_INVEN_bm; } while(0)
#define Led_DisableInterruptOnChange() do { PORTA.PIN1CTRL = (PORTA.PIN1CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define Led_EnableInterruptForBothEdges() do { PORTA.PIN1CTRL = (PORTA.PIN1CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define Led_EnableInterruptForRisingEdge() do { PORTA.PIN1CTRL = (PORTA.PIN1CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define Led_EnableInterruptForFallingEdge() do { PORTA.PIN1CTRL = (PORTA.PIN1CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define Led_DisableDigitalInputBuffer() do { PORTA.PIN1CTRL = (PORTA.PIN1CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define Led_EnableInterruptForLowLevelSensing() do { PORTA.PIN1CTRL = (PORTA.PIN1CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)
#define PA1_SetInterruptHandler Led_SetInterruptHandler

//get/set Pwr_ctl aliases
#define Pwr_ctl_SetHigh() do { PORTA_OUTSET = 0x4; } while(0)
#define Pwr_ctl_SetLow() do { PORTA_OUTCLR = 0x4; } while(0)
#define Pwr_ctl_Toggle() do { PORTA_OUTTGL = 0x4; } while(0)
#define Pwr_ctl_GetValue() (VPORTA.IN & (0x1 << 2))
#define Pwr_ctl_SetDigitalInput() do { PORTA_DIRCLR = 0x4; } while(0)
#define Pwr_ctl_SetDigitalOutput() do { PORTA_DIRSET = 0x4; } while(0)
#define Pwr_ctl_SetPullUp() do { PORTA_PIN2CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define Pwr_ctl_ResetPullUp() do { PORTA_PIN2CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define Pwr_ctl_SetInverted() do { PORTA_PIN2CTRL  |= PORT_INVEN_bm; } while(0)
#define Pwr_ctl_ResetInverted() do { PORTA_PIN2CTRL  &= ~PORT_INVEN_bm; } while(0)
#define Pwr_ctl_DisableInterruptOnChange() do { PORTA.PIN2CTRL = (PORTA.PIN2CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define Pwr_ctl_EnableInterruptForBothEdges() do { PORTA.PIN2CTRL = (PORTA.PIN2CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define Pwr_ctl_EnableInterruptForRisingEdge() do { PORTA.PIN2CTRL = (PORTA.PIN2CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define Pwr_ctl_EnableInterruptForFallingEdge() do { PORTA.PIN2CTRL = (PORTA.PIN2CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define Pwr_ctl_DisableDigitalInputBuffer() do { PORTA.PIN2CTRL = (PORTA.PIN2CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define Pwr_ctl_EnableInterruptForLowLevelSensing() do { PORTA.PIN2CTRL = (PORTA.PIN2CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)
#define PA2_SetInterruptHandler Pwr_ctl_SetInterruptHandler

//get/set Pwr_off aliases
#define Pwr_off_SetHigh() do { PORTB_OUTSET = 0x1; } while(0)
#define Pwr_off_SetLow() do { PORTB_OUTCLR = 0x1; } while(0)
#define Pwr_off_Toggle() do { PORTB_OUTTGL = 0x1; } while(0)
#define Pwr_off_GetValue() (VPORTB.IN & (0x1 << 0))
#define Pwr_off_SetDigitalInput() do { PORTB_DIRCLR = 0x1; } while(0)
#define Pwr_off_SetDigitalOutput() do { PORTB_DIRSET = 0x1; } while(0)
#define Pwr_off_SetPullUp() do { PORTB_PIN0CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define Pwr_off_ResetPullUp() do { PORTB_PIN0CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define Pwr_off_SetInverted() do { PORTB_PIN0CTRL  |= PORT_INVEN_bm; } while(0)
#define Pwr_off_ResetInverted() do { PORTB_PIN0CTRL  &= ~PORT_INVEN_bm; } while(0)
#define Pwr_off_DisableInterruptOnChange() do { PORTB.PIN0CTRL = (PORTB.PIN0CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define Pwr_off_EnableInterruptForBothEdges() do { PORTB.PIN0CTRL = (PORTB.PIN0CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define Pwr_off_EnableInterruptForRisingEdge() do { PORTB.PIN0CTRL = (PORTB.PIN0CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define Pwr_off_EnableInterruptForFallingEdge() do { PORTB.PIN0CTRL = (PORTB.PIN0CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define Pwr_off_DisableDigitalInputBuffer() do { PORTB.PIN0CTRL = (PORTB.PIN0CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define Pwr_off_EnableInterruptForLowLevelSensing() do { PORTB.PIN0CTRL = (PORTB.PIN0CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)
#define PB0_SetInterruptHandler Pwr_off_SetInterruptHandler

/**
 * @ingroup  pinsdriver
 * @brief GPIO and peripheral I/O initialization
 * @param none
 * @return none
 */
void PIN_MANAGER_Initialize();

/**
 * @ingroup  pinsdriver
 * @brief Default Interrupt Handler for Auto_on pin. 
 *        This is a predefined interrupt handler to be used together with the Auto_on_SetInterruptHandler() method.
 *        This handler is called every time the Auto_on ISR is executed. 
 * @pre PIN_MANAGER_Initialize() has been called at least once
 * @param none
 * @return none
 */
void Auto_on_DefaultInterruptHandler(void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt Handler Setter for Auto_on pin input-sense-config functionality.
 *        Allows selecting an interrupt handler for Auto_on at application runtime
 * @pre PIN_MANAGER_Initialize() has been called at least once
 * @param InterruptHandler function pointer.
 * @return none
 */
void Auto_on_SetInterruptHandler(void (* interruptHandler)(void)) ; 

/**
 * @ingroup  pinsdriver
 * @brief Default Interrupt Handler for Signal pin. 
 *        This is a predefined interrupt handler to be used together with the Signal_SetInterruptHandler() method.
 *        This handler is called every time the Signal ISR is executed. 
 * @pre PIN_MANAGER_Initialize() has been called at least once
 * @param none
 * @return none
 */
void Signal_DefaultInterruptHandler(void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt Handler Setter for Signal pin input-sense-config functionality.
 *        Allows selecting an interrupt handler for Signal at application runtime
 * @pre PIN_MANAGER_Initialize() has been called at least once
 * @param InterruptHandler function pointer.
 * @return none
 */
void Signal_SetInterruptHandler(void (* interruptHandler)(void)) ; 

/**
 * @ingroup  pinsdriver
 * @brief Default Interrupt Handler for Pwr_mon pin. 
 *        This is a predefined interrupt handler to be used together with the Pwr_mon_SetInterruptHandler() method.
 *        This handler is called every time the Pwr_mon ISR is executed. 
 * @pre PIN_MANAGER_Initialize() has been called at least once
 * @param none
 * @return none
 */
void Pwr_mon_DefaultInterruptHandler(void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt Handler Setter for Pwr_mon pin input-sense-config functionality.
 *        Allows selecting an interrupt handler for Pwr_mon at application runtime
 * @pre PIN_MANAGER_Initialize() has been called at least once
 * @param InterruptHandler function pointer.
 * @return none
 */
void Pwr_mon_SetInterruptHandler(void (* interruptHandler)(void)) ; 

/**
 * @ingroup  pinsdriver
 * @brief Default Interrupt Handler for Pulse pin. 
 *        This is a predefined interrupt handler to be used together with the Pulse_SetInterruptHandler() method.
 *        This handler is called every time the Pulse ISR is executed. 
 * @pre PIN_MANAGER_Initialize() has been called at least once
 * @param none
 * @return none
 */
void Pulse_DefaultInterruptHandler(void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt Handler Setter for Pulse pin input-sense-config functionality.
 *        Allows selecting an interrupt handler for Pulse at application runtime
 * @pre PIN_MANAGER_Initialize() has been called at least once
 * @param InterruptHandler function pointer.
 * @return none
 */
void Pulse_SetInterruptHandler(void (* interruptHandler)(void)) ; 

/**
 * @ingroup  pinsdriver
 * @brief Default Interrupt Handler for Led pin. 
 *        This is a predefined interrupt handler to be used together with the Led_SetInterruptHandler() method.
 *        This handler is called every time the Led ISR is executed. 
 * @pre PIN_MANAGER_Initialize() has been called at least once
 * @param none
 * @return none
 */
void Led_DefaultInterruptHandler(void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt Handler Setter for Led pin input-sense-config functionality.
 *        Allows selecting an interrupt handler for Led at application runtime
 * @pre PIN_MANAGER_Initialize() has been called at least once
 * @param InterruptHandler function pointer.
 * @return none
 */
void Led_SetInterruptHandler(void (* interruptHandler)(void)) ; 

/**
 * @ingroup  pinsdriver
 * @brief Default Interrupt Handler for Pwr_ctl pin. 
 *        This is a predefined interrupt handler to be used together with the Pwr_ctl_SetInterruptHandler() method.
 *        This handler is called every time the Pwr_ctl ISR is executed. 
 * @pre PIN_MANAGER_Initialize() has been called at least once
 * @param none
 * @return none
 */
void Pwr_ctl_DefaultInterruptHandler(void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt Handler Setter for Pwr_ctl pin input-sense-config functionality.
 *        Allows selecting an interrupt handler for Pwr_ctl at application runtime
 * @pre PIN_MANAGER_Initialize() has been called at least once
 * @param InterruptHandler function pointer.
 * @return none
 */
void Pwr_ctl_SetInterruptHandler(void (* interruptHandler)(void)) ; 

/**
 * @ingroup  pinsdriver
 * @brief Default Interrupt Handler for Pwr_off pin. 
 *        This is a predefined interrupt handler to be used together with the Pwr_off_SetInterruptHandler() method.
 *        This handler is called every time the Pwr_off ISR is executed. 
 * @pre PIN_MANAGER_Initialize() has been called at least once
 * @param none
 * @return none
 */
void Pwr_off_DefaultInterruptHandler(void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt Handler Setter for Pwr_off pin input-sense-config functionality.
 *        Allows selecting an interrupt handler for Pwr_off at application runtime
 * @pre PIN_MANAGER_Initialize() has been called at least once
 * @param InterruptHandler function pointer.
 * @return none
 */
void Pwr_off_SetInterruptHandler(void (* interruptHandler)(void)) ; 
#endif /* PINS_H_INCLUDED */
