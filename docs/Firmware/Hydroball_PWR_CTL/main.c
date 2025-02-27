#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/timer/delay.h"
#include "mcc_generated_files/system/pins.h"

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

// Variable globale pour compter le temps (fictive : incrémentée dans la boucle)
uint32_t g_timeMs = 0; 
uint32_t pwr_timeMs = 0;
bool sign_release = 0;
// Nouveau timer pour gérer la temporisation avant extinction définitive
static uint32_t g_shutdownTimerStart = 0;

// Indique si l?on est en phase de « shutdown » ou non
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
        // Lecture de l'entrée Signal
        bool currentSignal = (Signal_GetValue() != 0);
        bool Pwr_ctl_val = (Pwr_ctl_GetValue() != 0);
        bool Pwr_mon_val = (Pwr_mon_GetValue() != 0); 
        bool Pwr_off_val = (Pwr_off_GetValue() != 0); 
        
        bool Pulse = (Pulse_GetValue() != 0); 
        //bool rpi_pwr = 0;
        
        
        // Mise a zéro du timer pour la longeur de prériode de la présence de l'aiment
        if (currentSignal && !lastSignal)
        {
            signalStartTime = millis();
            
        }
        lastSignal = currentSignal;

        
        //si l.aiment est présente devant le capteur magnétique
        if (currentSignal)
        {
            // Calcul de la durée depuis laquelle le signal est actif
            uint32_t elapsed = millis() - signalStartTime;

            if (elapsed < 1000)
            {
                // Moins de 1 seconde => LED allumée
                Led_SetHigh();
            }
            else if (elapsed < 10000)
            {
                

                // Si Pwr_ctl est déjà 1, on appelle Pwr_off_SetHigh()
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
                // Plus de 10 secondes => on remet Pwr_ctl et Pwr_off à Low
                Pwr_off_SetLow();
                Pwr_ctl_SetHigh();
                //Led_SetLow(); 
                // (À adapter selon votre besoin)
            }
        }
        else
        {
            // séquence d'arret
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
                    // 1) Première fois que l?on détecte l?appui => on démarre la fenêtre de shutdown
                    if (!g_isShuttingDown)
                    {
                        g_isShuttingDown = true;
                        //pwr_timeMs = 1;
                        g_shutdownTimerStart = now3; 
                    }

                    // 2) Si on détecte un pulse, on réinitialise le timer
                    //    (on suppose que Pulse_GetValue() retourne true quand le pulse est présent)
                    
                    if (Pulse != Last_Pulse)
                    {
                        Last_Pulse = Pulse_GetValue();
                        g_shutdownTimerStart = now3;
                    }

                    // 3) Vérifier si on a dépassé la durée d?attente souhaitée (ex: 60000ms = 60s)
                    if ((now3 - g_shutdownTimerStart) >= 35000UL)
                    {
                        // Alors on coupe réellement l?alim :
                        Pwr_ctl_SetHigh();
                        Pwr_off_SetLow();
                        sign_release = 0;
                        g_isShuttingDown = false;  // On sort de la phase d?arrêt (car alim coupée)
                    }
                
                }
            }         
            //séquence de démmarage
            else if ((!(Pwr_ctl_val)) && (Pwr_mon_val))
            {
                // Dans ce bloc, on considère qu'on est en phase d'arrêt ou de préparation à l'arrêt.
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
                // Sinon, LED éteinte
                Led_SetLow();
                
            }
            
           
     
            
        }

        // Incrémente le compteur de temps et petite pause
        g_timeMs++;
        pwr_timeMs++;
        DELAY_milliseconds(1);
    }
}
