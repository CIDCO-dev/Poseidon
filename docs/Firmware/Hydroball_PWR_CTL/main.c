#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/timer/delay.h"
#include "mcc_generated_files/system/pins.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

// Global variable to count time (fictitious: incremented in the loop)
uint32_t g_timeMs = 0; 
uint32_t pwr_timeMs = 0;
bool sign_release = 0;
// New timer to manage the delay before final shutdown
static uint32_t g_shutdownTimerStart = 0;

// Indicates whether we are in the shutdown phase or not
static bool g_isShuttingDown = false;

static inline uint32_t millis(void)
{
    return g_timeMs; 
}

static inline uint32_t pwr_timeout(void)
{
    return pwr_timeMs; 
}

int main(void)
{
    SYSTEM_Initialize();
    Led_SetLow();   
    Pwr_off_SetLow();
    Pwr_ctl_SetHigh();         
             
    bool lastSignal = false;    
    uint32_t signalStartTime = 0;
    bool Last_Pulse = (Pulse_GetValue() != 0); 
    while(1)
    {
        // Read input values
        bool currentSignal = (Signal_GetValue() != 0);
        bool Pwr_ctl_val = (Pwr_ctl_GetValue() != 0);
        bool Pwr_mon_val = (Pwr_mon_GetValue() != 0); 
        bool Pwr_off_val = (Pwr_off_GetValue() != 0); 
        bool Pulse = (Pulse_GetValue() != 0); 
        
        // Reset the timer when the magnet is detected
        if (currentSignal && !lastSignal)
        {
            signalStartTime = millis();
        }
        lastSignal = currentSignal;

        // If the magnet is detected
        if (currentSignal)
        {
            // Calculate how long the signal has been active
            uint32_t elapsed = millis() - signalStartTime;

            if (elapsed < 1000)
            {
                // Less than 1 second => LED ON
                Led_SetHigh();
            }
            else if (elapsed < 10000)
            {
                // If Pwr_ctl is already HIGH, set Pwr_off HIGH (shutdown command)
                if (sign_release)
                {
                    Pwr_off_SetHigh();
                    sign_release = 0;
                }
                else 
                {
                    // Between 1 and 10 seconds => Turn off Pwr_ctl(apply 5v on the rpi)
                    Pwr_ctl_SetLow();
                    sign_release = 0;
                }
            }
            else
            {
                // More than 10 seconds => Reset Pwr_ctl and Pwr_off(cut 5v on the rpi)
                Pwr_off_SetLow();
                Pwr_ctl_SetHigh();
            }
        }
        else
        {
            // Shutdown sequence
            if ((Pwr_off_val)) 
            {
                // Blink LED at 10 Hz if Pwr_ctl=1 and Pwr_mon=0 (During Shutdown)
                uint32_t now = millis();
                if (((now / 100) % 2) == 0)
                {
                    Led_SetHigh();
                }
                else
                {
                    Led_SetLow();
                }
                
                if (!(Pwr_ctl_val))
                {
                    uint32_t now3 = pwr_timeout();
                    // 1) First detection of shutdown request => Start shutdown timer
                    if (!g_isShuttingDown)
                    {
                        g_isShuttingDown = true;
                        g_shutdownTimerStart = now3; 
                    }

                    // 2) If a pulse is detected, reset the timer
                    if (Pulse != Last_Pulse)
                    {
                        Last_Pulse = Pulse_GetValue();
                        g_shutdownTimerStart = now3;
                    }

                    // 3) If shutdown timeout exceeds 35 seconds, cut the power(35000UL = 35 seconds)
                    if ((now3 - g_shutdownTimerStart) >= 35000UL)
                    {
                        Pwr_ctl_SetHigh();
                        Pwr_off_SetLow();
                        sign_release = 0;
                        g_isShuttingDown = false;  // Exit shutdown phase (power cut)
                    }
                }
            }         
            // Startup sequence
            else if ((!(Pwr_ctl_val)) && (Pwr_mon_val))
            {
                // Blink LED at 0.5 Hz during startup
                uint32_t now = millis();
                if (((now / 500) % 2) == 0)
                {
                    Led_SetHigh();
                }
                else
                {
                    Led_SetLow();
                }
            }
            // Reset variables when in power-off state
            else if ((!(Pwr_ctl_val)) && (!Pwr_mon_val))
            {
                sign_release = 1;
                Led_SetLow();
            } 
            else
            {
                // Otherwise, keep the LED off
                Led_SetLow();
            }
        }

        // Increment time counters and add a small delay
        g_timeMs++;
        pwr_timeMs++;
        DELAY_milliseconds(1);
    }
}
