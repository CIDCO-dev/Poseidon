import os.path
import platform
import subprocess
import distro as distro
# from whichcraft import which
import psutil

system = platform.system()

print("system:", system)

processor = platform.processor()

print("processor:", processor)

platformName = platform.platform()

print("platform:", platformName)

machine = platform.machine()

print("machine:", machine)

release = platform.release()

print("release:", release)

version = platform.version()

print("version:", version)

architecture = platform.architecture()

print("architecture:", architecture)

version = int(float(distro.version()))

print("version:", version)

ram = psutil.virtual_memory().total / 1000000000

print("RAM:", ram)

swap = psutil.swap_memory().total / 1000000000
print("Swap:", swap)
print(psutil.swap_memory())


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
    path = '/opt/Poseidon'
    exists = os.path.exists(path)
    if exists is True:
        return print('Previous installation detected!')
    else:
        print('New installation.')


check_install()
install()
