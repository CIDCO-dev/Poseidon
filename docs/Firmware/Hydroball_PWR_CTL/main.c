#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/timer/delay.h"
#include "mcc_generated_files/system/pins.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

// Variable globale pour compter le temps (fictive : incr�ment�e dans la boucle)
uint32_t g_timeMs = 0; 
uint32_t pwr_timeMs = 0;
bool sign_release = 0;
// Nouveau timer pour g�rer la temporisation avant extinction d�finitive
static uint32_t g_shutdownTimerStart = 0;

// Indique si l?on est en phase de � shutdown � ou non
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
             

    //DELAY_milliseconds(10000);
    
    bool lastSignal = false;    
    uint32_t signalStartTime = 0;
    bool Last_Pulse = (Pulse_GetValue() != 0); 
    while(1)
    {
        // Lecture de l'entr�e Signal
        bool currentSignal = (Signal_GetValue() != 0);
        bool Pwr_ctl_val = (Pwr_ctl_GetValue() != 0);
        bool Pwr_mon_val = (Pwr_mon_GetValue() != 0); 
        bool Pwr_off_val = (Pwr_off_GetValue() != 0); 
        
        bool Pulse = (Pulse_GetValue() != 0); 
        //bool rpi_pwr = 0;
        
        
        // Mise a z�ro du timer pour la longeur de pr�riode de la pr�sence de l'aiment
        if (currentSignal && !lastSignal)
        {
            signalStartTime = millis();
            
        }
        lastSignal = currentSignal;

        
        //si l.aiment est pr�sente devant le capteur magn�tique
        if (currentSignal)
        {
            // Calcul de la dur�e depuis laquelle le signal est actif
            uint32_t elapsed = millis() - signalStartTime;

            if (elapsed < 1000)
            {
                // Moins de 1 seconde => LED allum�e
                Led_SetHigh();
            }
            else if (elapsed < 10000)
            {
                

                // Si Pwr_ctl est d�j� 1, on appelle Pwr_off_SetHigh()
                if (sign_release)
                {
                    Pwr_off_SetLow();
                    
                   
                    sign_release = 0;
                    
                }
                else 
                {
                // Entre 1 et 10 secondes
                Pwr_ctl_SetLow();
                
                sign_release = 0;
                
                }
            }
            else
            {
                // Plus de 10 secondes => on remet Pwr_ctl et Pwr_off � Low
                Pwr_off_SetLow();
                Pwr_ctl_SetHigh();
                //Led_SetLow(); 
                // (� adapter selon votre besoin)
            }
        }
        else
        {
            // s�quence d'arret
            if ((Pwr_off_val)) 
            {
                // Clignotement 1 Hz si Pwr_ctl=1 ET Pwr_mon=0
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
                    // 1) Premi�re fois que l?on d�tecte l?appui => on d�marre la fen�tre de shutdown
                    if (!g_isShuttingDown)
                    {
                        g_isShuttingDown = true;
                        //pwr_timeMs = 1;
                        g_shutdownTimerStart = now3; 
                    }

                    // 2) Si on d�tecte un pulse, on r�initialise le timer
                    //    (on suppose que Pulse_GetValue() retourne true quand le pulse est pr�sent)
                    
                    if (Pulse != Last_Pulse)
                    {
                        Last_Pulse = Pulse_GetValue();
                        g_shutdownTimerStart = now3;
                    }

                    // 3) V�rifier si on a d�pass� la dur�e d?attente souhait�e (ex: 60000ms = 60s)
                    if ((now3 - g_shutdownTimerStart) >= 35000UL)
                    {
                        // Alors on coupe r�ellement l?alim :
                        Pwr_ctl_SetHigh();
                        Pwr_off_SetLow();
                        sign_release = 0;
                        g_isShuttingDown = false;  // On sort de la phase d?arr�t (car alim coup�e)
                    }
                
                }
            }         
            //s�quence de d�mmarage
            else if ((!(Pwr_ctl_val)) && (Pwr_mon_val))
            {
                // Dans ce bloc, on consid�re qu'on est en phase d'arr�t ou de pr�paration � l'arr�t.
                // Ex: Clignotement LED (ici 1 Hz, juste en exemple)
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

            else if ((!(Pwr_ctl_val)) && (!Pwr_mon_val))
            {
                sign_release = 1;
                Led_SetLow();
                
            } 
            
            else
            {
                // Sinon, LED �teinte
                Led_SetLow();
                
            }
            
           
     
            
        }

        // Incr�mente le compteur de temps et petite pause
        g_timeMs++;
        pwr_timeMs++;
        DELAY_milliseconds(1);
    }
}
