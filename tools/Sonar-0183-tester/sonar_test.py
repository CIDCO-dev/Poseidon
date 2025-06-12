import serial
import time
import re

# baudrates to test
baudrates = [4800, 9600, 19200, 38400, 57600, 115200]
serial_port = "/dev/sonar"  

# Expression to be detected
nmea_regex = re.compile(r'^\$(..)(DBT|DPT|ADS),.*\*[0-9A-F]{2}')

# checksum check
def validate_checksum(nmea_str):
    if not nmea_str.startswith('$'):
        return False
    try:
        data, checksum = nmea_str[1:].split('*')
    except ValueError:
        return False
    calc_checksum = 0
    for c in data:
        calc_checksum ^= ord(c)
    return calc_checksum == int(checksum, 16)


def test_baudrate(baud):
    try:
        with serial.Serial(serial_port, baudrate=baud, timeout=1) as ser:
            print(f"Testing baudrate {baud}...")
            start = time.time()
            while time.time() - start < 5:  # essaie pendant 5 secondes
                line = ser.readline().decode(errors='ignore').strip()
                if nmea_regex.match(line) and validate_checksum(line):
                    print(f"Baudrate {baud} detected with data : {line}")
                    return True
    except Exception as e:
        print(f"Error for {baud} baud : {e}")
    return False


for baud in baudrates:
    if test_baudrate(baud):
        break
else:
    print("No data detected on default baudrate.")