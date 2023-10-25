
import os.path
import platform
import subprocess
import distro as distro
import psutil
import os
import inquirer

def check_ubuntu_distribution():
    if platform.system() == 'Linux':
        try:
            with open('/etc/os-release', 'r') as os_release:
                for line in os_release:
                    if line.startswith('ID=ubuntu'):
                        return True
        except FileNotFoundError:
            pass
    return False

system = platform.system()
print("[!] Environment system:", system)

isubuntu = check_ubuntu_distribution()
if isubuntu:
    print("[!] Distribution supported!")
else:
    print("[X] Distribution unsupported please use Ubuntu")
    exit()

version = int(float(distro.version()))
print("[!] Ubuntu Version:", version)

processor = platform.processor()
print("[!] CPU Architecture:", processor)

print("--------------------------------------------")

ram = psutil.virtual_memory().total / 1000000000
print("[!] Total RAM:", ram, "Gb")

swap = psutil.swap_memory().total / 1000000000
print("[!] Total Swap:", swap, "Gb")



def install():
    if system != "Linux":
        return print("Not linux")
    elif os.geteuid() != 0:
        return print("You are not the root user.")
    elif ram < 1 or (ram + swap < 2):
        return print("Not enough RAM.")

    if version == 18 and processor == "x86_64":
        os.system('/opt/Poseidon/install/x64-melodic.sh')
    elif version == 18 and processor.lower().startswith("armv"):
        os.system('/opt/Poseidon/install/rpi4-noetic.sh')

    if version == 20 and processor == "x86_64":
        os.system('/opt/Poseidon/install/x64-noetic.sh')
    elif version == 20 and processor.lower().startswith("armv"):
        os.system('/opt/Poseidon/install/rpi4-noetic.sh')


def check_install():
    path = '/opt/Poseidon/src/workspace/devel'
    exists = os.path.exists(path)
    if exists is True:
        print('[!] Previous installation detected!')
        return 
    else:
        print('[!] New installation.')

print("--------------------------------------------")

print("HotSpot")

hotspot_if = inquirer.list_input("Select the wifi interface for creating the hotspot?",
                              choices=['wlan0', 'wlan1'])

hotspot_ssid = inquirer.text(message="Enter the HotSpot SSID")
hotspot_password = inquirer.text(message="Enter the HotSpot Password")



check_install()
#install()
