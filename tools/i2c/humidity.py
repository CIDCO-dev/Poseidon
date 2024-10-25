import smbus
import time

# Créer une instance de la classe SMBus pour accéder à l'I2C bus
bus = smbus.SMBus(1)  # Utilisez 0 pour les anciens modèles Raspberry Pi

# Adresse I2C du capteur HIH8130 (peut varier, vérifiez votre documentation)
SENSOR_ADDRESS = 0x27

def read_hih8130():
    # Envoyer une requête de mesure au capteur
    bus.write_quick(SENSOR_ADDRESS)
    time.sleep(0.1)  # Attendre la fin de la mesure

    # Lire 4 octets de données; 2 pour l'humidité, 2 pour la température
    data = bus.read_i2c_block_data(SENSOR_ADDRESS, 0, 4)

    # Convertir les données en valeur d'humidité
    humidity_raw = ((data[0] & 0x3F) << 8) + data[1]
    humidity = (humidity_raw / (2**14 - 2)) * 100.0

    # Convertir les données en valeur de température
    temp_raw = (data[2] << 6) + (data[3] >> 2)
    temperature = ((temp_raw / (2**14 - 2))* 165)  - 40.0

    return humidity, temperature

# Lire et afficher l'humidité et la température
humidity, temperature = read_hih8130()
print(f"Humidité: {humidity:.2f}%")
print(f"Température: {temperature:.2f}°C")
