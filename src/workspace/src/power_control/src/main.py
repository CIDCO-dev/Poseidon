#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import subprocess
import time

# Numéros de GPIO en BCM
GPIO_OUT = 16   # La broche à mettre à 1 au démarrage
GPIO_IN  = 22   # La broche à surveiller pour le shutdown
GPIO_Pulse = 23 # Broche pour la pulse 

def shutdown_callback(channel):
    """
    Callback appelée quand GPIO_IN passe à l'état HAUT (détection front montant).
    On déclenche un shutdown du Raspberry Pi.
    """
    rospy.logwarn("GPIO {} est passé à 1 : déclenchement du shutdown...".format(GPIO_IN))
    #GPIO.output(GPIO_OUT, GPIO.LOW)
    #GPIO.cleanup()
    # Lancer la commande de shutdown
    subprocess.call(["shutdown", "now"])

def on_shutdown():
    """
    Cette fonction s'exécute quand ROS arrête ce nœud (SIGINT ou autre).
    On repasse la broche GPIO_OUT à 0 avant de terminer.
    """
    rospy.loginfo("Arrêt du nœud : on repasse GPIO {} à 0.".format(GPIO_OUT))
    GPIO.output(GPIO_OUT, GPIO.LOW)
    GPIO.cleanup()

def main():
    # Initialisation du nœud ROS
    rospy.init_node('gpio_control_node', anonymous=True)

    # Configuration des GPIO
    GPIO.setmode(GPIO.BCM)
    
    # Sortie
    GPIO.setup(GPIO_OUT, GPIO.OUT)
    GPIO.output(GPIO_OUT, GPIO.HIGH)  # Met la broche 16 à l'état haut

    # Entrée
    GPIO.setup(GPIO_IN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # Détecter le front montant sur GPIO_IN et appeler shutdown_callback
    GPIO.add_event_detect(GPIO_IN, GPIO.FALLING, callback=shutdown_callback, bouncetime=300)

    #Pulse
    GPIO.setup(GPIO_Pulse, GPIO.OUT)

    # Définir la fonction à exécuter au shutdown du nœud 
    rospy.on_shutdown(on_shutdown)

    rospy.loginfo("Nœud GPIO démarré : GPIO {}=HIGH. Surveillance GPIO {}.".format(GPIO_OUT, GPIO_IN))

    # Boucle pour garder le nœud ROS actif
    # (on dort ici pour libérer le CPU, ROS gère l'événement d'interruption pour la broche 22)
    rate = rospy.Rate(1)  # 10 Hz
    pulse = False
    while not rospy.is_shutdown():
        rate.sleep()
        pulse = not pulse; 
        GPIO.output(GPIO_Pulse, pulse)
        

if __name__ == "__main__":
    main()
