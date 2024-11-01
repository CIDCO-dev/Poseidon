import smbus2
import time

# Adresse I2C pour le PCA9533
I2C_ADDR = 0x62

# Registres du PCA9533
PSC0 = 0x01  # Prescaler 0
PWM0 = 0x02  # PWM 0
PSC1 = 0x03  # Prescaler 1
PWM1 = 0x04  # PWM 1
LS0 = 0x05   # LED selector

# Configuration initiale du PCA9533
bus = smbus2.SMBus(1)

# Réglage des fréquence
bus.write_byte_data(I2C_ADDR, PSC0, 151) # Période de clignotement de 1 seconde 1Hz
bus.write_byte_data(I2C_ADDR, PWM0, 128) # Rapport cyclique à 50%
bus.write_byte_data(I2C_ADDR, PSC1, 15)  # Période de clignotement de 0.1 seconde 10Hz
bus.write_byte_data(I2C_ADDR, PWM1, 128) # Rapport cyclique à 50%

#Fonction pour convertir l'état des led
def set_led_states(led0, led1, led2):
    """
    Définit l'état de chaque LED sur le PCA9533.
    
    Les paramètres led0 à led3 peuvent avoir les valeurs suivantes :
    0 pour haute impédance (LED off),
    1 pour état bas (LED on),
    2 pour clignotement au taux de 1hz,
    3 pour clignotement au taux de 10hz.
    """
    # Convertir les états en une valeur de 8 bits
    states = (0 << 6) | (led2 << 4) | (led1 << 2) | led0
    bus.write_byte_data(I2C_ADDR, LS0, states)

# Fonction pour régler la couleur rouge
def set_red(LS):
    set_led_states(LS, 0, 0)

# Fonction pour régler la couleur verte
def set_green(LS):
    set_led_states(0, LS, 0)

# Fonction pour régler la couleur bleue
def set_blue(LS):
    set_led_states(0, 0, LS)

# Fonction pour régler la couleur jaune (rouge + vert)
def set_yellow(LS):
    set_led_states(LS, LS, 0)

# Fonction pour régler la couleur mauve (rouge + bleu)
def set_mauve(LS):
    set_led_states(LS, 0, LS)

# Fonction pour régler la couleur cyan (vert + bleu)
def set_cyan(LS):
    set_led_states(0, LS, LS)

# Fonction pour régler la couleur blanc
def set_blanc(LS):
    set_led_states(LS, LS, LS)

# Fonction pour éteindre les led
def set_off():
    set_led_states(0, 0, 0)




# Exemple d'utilisation
set_red(1)
time.sleep(5)
set_green(2)
time.sleep(5)
set_blue(3)
time.sleep(5)
set_yellow(1)
time.sleep(5)
set_mauve(1)
time.sleep(5)
set_cyan(1)
