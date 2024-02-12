import serial
import time


# Validation des messages NMEA sur le port ttyAMA0 à 38400 bps
print("Validation des messages NMEA sur le port ttyAMA0 à 38400 bps...")
with serial.Serial("/dev/ttyAMA0", 38400, timeout=1) as ser_nmea:
    start_time = time.time()
    while time.time() - start_time < 1:  # Validation pendant une seconde
        message1 = ser_nmea.readline().strip()  # Lecture en tant que bytes
    print(f"NMEA Reçu: {message1}")  # Décodage en UTF-8 avec gestion d'erreur
    ser_nmea.close()

# Validation des trames u-blox sur le port /dev/gnss à 115200 bps
print("Validation des trames u-blox sur le port /dev/gnss à 115200 bps...")
with serial.Serial("/dev/gnss", 460800, timeout=1) as ser_ubx:
    start_time = time.time()
    while time.time() - start_time < 1:  # Validation pendant une seconde
        message2 = ser_ubx.read(1000)
    print(f"UBX Reçu: {message2}")
    ser_ubx.close()
