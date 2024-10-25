import smbus
import time

# Créez une instance de l'objet SMBus pour accéder à l'I2C bus.
bus = smbus.SMBus(4)

# Adresse I2C de votre INA238
ina238_address = 0x40  # Remplacez par l'adresse I2C réelle de votre INA238

# Adresses des registres (à remplacer par les vraies adresses de votre composant)
registre_de_tension = 0x05
registre_shunt = 0x04  # Registre de la chute de tension sur la résistance shunt
registre_temperature = 0x06  # Registre de la température

try:
    # Lecture de la tension
    data = bus.read_i2c_block_data(ina238_address, registre_de_tension, 2)
    tension = (data[0] * 256 + data[1]) * 3.125 / 1000
    print(f"Tension: {tension} V")

    # Lecture de la chute de tension sur la résistance shunt
    data = bus.read_i2c_block_data(ina238_address, registre_shunt, 2)
    chute_shunt = (data[0] * 256 + data[1]) * 5 / 1000  # Ajustez la formule selon la fiche technique
    print(f"Chute de tension sur shunt: {chute_shunt} mV")


    # Lecture de la température
    data = bus.read_i2c_block_data(ina238_address, registre_temperature, 2)
    raw_temperature = (data[0] * 256 + data[1]) >> 4  # Décale de 4 bits pour ignorer les bits réservés

    # Conversion de la valeur brute en complément à deux si nécessaire
    if raw_temperature & 0x800:  # Teste si le bit le plus significatif est à 1
        temperature = -((~raw_temperature & 0xFFF) + 1)  # Convertit en complément à deux
    else:
        temperature = raw_temperature

    temperature = temperature * 125.0 / 1000  # Convertit en °C en utilisant le facteur 125 m°C/LSB

    print(f"Température: {temperature} °C")

except Exception as e:
    print(e)
