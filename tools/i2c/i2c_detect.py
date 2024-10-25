import smbus2

# Fonction pour vérifier la présence d'un appareil à une adresse donnée
def check_device_presence(i2c_bus, address):
    try:
        bus = smbus2.SMBus(i2c_bus)
        bus.read_byte(address)  # Essayer de lire un byte à cette adresse
        return True  # Si aucune erreur n'est générée, l'appareil est présent
    except Exception as e:  # Attraper l'exception si l'appareil ne répond pas
        print(f"Erreur détectée : {e}")
        return False  # L'appareil n'est pas présent ou il y a un problème de communication
    finally:
        bus.close()  # Assurez-vous de fermer le bus I2C

# Utilisation de la fonction
i2c_bus = 1  # Bus I2C 1 est généralement le bus par défaut sur un Raspberry Pi
address = 0x62  # Adresse I2C du PCA9533
is_device_present = check_device_presence(i2c_bus, address)

print(f"Le dispositif à l'adresse 0x{address:02x} est {'présent' if is_device_present else 'absent'}.")
