
import os.path
import platform
import subprocess
import distro as distro
import psutil
import os
import inquirer
import subprocess


system = "notset"
isubuntu = False
version = "notset"
processor = "notset"
ram = "notset"
swap = "notset"


def loadvariable()
    global system
    global isubuntu
    global version
    global processor
    global ram
    global swap

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

    print("--------------------------------------------")

    print("HotSpot")

    hotspot_if = inquirer.list_input("Select the wifi interface for creating the hotspot?",
                                choices=['wlan0', 'wlan1'])

    hotspot_ssid = inquirer.text(message="Enter the HotSpot SSID")
    hotspot_password = inquirer.text(message="Enter the HotSpot Password")

    wifi_enable = inquirer.list_input("Do you want to configure wifi internet access?",
                                choices=['Yes', 'No'])

    wifi_if = inquirer.list_input("Select the wifi interface for creating the WIFI connection?",
                                choices=['wlan0', 'wlan1'])

    wifi_ssid = inquirer.text(message="Enter the WIFI SSID")
    wifi_password = inquirer.text(message="Enter the WIFI Password")

    wirred_type = inquirer.list_input("What type of IP configuration do you want for the wired network interface?",
                                choices=['DHCP', 'Static'])

    rtc = inquirer.list_input("Do you want to activate physical RTC for your raspberry?",
                                choices=['Yes', 'No'])

    swap_status = inquirer.list_input("Do you want to desactivate the SWAP",
                                choices=['Yes', 'No'])

    serialnumber = inquirer.text(message="Enter the Serial Number")

def check_install():
    path = '/opt/Poseidon/src/workspace/devel'
    exists = os.path.exists(path)
    if exists is True:
        print('[!] Previous installation detected!')
        return 
    else:
        print('[!] New installation.')

def install_package(package_name):
    try:
        log_file = open("install.log", "a") 
        subprocess.run(["sudo", "apt-get", "install", package_name], check=True, stdout=log_file, stderr=log_file)
        log_file.close()
        print(f"[!] {package_name} have been installed successfully.")
    except subprocess.CalledProcessError:
        print(f"[X] Error during the installation of {package_name}.")



def update_packager():
    try:
        log_file = open("install.log", "a") 
        subprocess.run(["apt-get", "update"], check=True, stdout=log_file, stderr=log_file)
        log_file.close()
        print(f"[!] Successful update of the package list.")
    except subprocess.CalledProcessError: 
        print(f"[X] Error during the update of the package list.")
    try:
        log_file = open("install.log", "a")
        subprocess.run(["apt-get", "upgrade", "-y"], check=True, stdout=log_file, stderr=log_file)
        log_file.close()
        print(f"[!] Success for the package update.")
    except subprocess.CalledProcessError:
        print(f"[X] Error during the package update .")

def install_ros_melodic():
    exit()

def install_ros_noetic():
    try:
        log_file = open("install.log", "a") 
        command = ['/bin/sh', '-c', 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list']
        subprocess.run(command, check=True, stdout=log_file, stderr=log_file)
        log_file.close()
        print(f"[!] Successful append ROS sources.")
    except subprocess.CalledProcessError: 
        print(f"[X] Error during the ROS sources append.")
    try:
        log_file = open("install.log", "a") 
        curl_command = ['/usr/bin/curl', '-s', 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc']
        apt_key_command = ['/usr/bin/apt-key', 'add', '-']
        curl_process = subprocess.Popen(curl_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        apt_key_process = subprocess.Popen(apt_key_command, stdin=curl_process.stdout, stdout=log_file, stderr=log_file)
        curl_process.stdout.close()
        apt_key_process.communicate()
        log_file.close()        
        print(f"[!] The key have been installed.")
    except subprocess.CalledProcessError: 
        print(f"[X] Error during key installation.")
    update_packager()
    install_package("ros-noetic-ros-base")
    install_package("ros-noetic-tf2-geometry-msgs")
    install_package("g++")
    install_package("python3-rosdep")
    install_package("ros-noetic-mavros")
    install_package("ros-noetic-sbg-driver")
    install_package("python3-rosinstall")
    install_package("python3-rosinstall-generator")
    install_package("python3-wstool")
    install_package("build-essential")
    #rosdep init
    #gpsd client

def install_toolchain():
    update_packager()
    install_package("gcc")
    install_package("python3-dev")
    install_package("python3-pip")
    install_package("python-setuptools")
    install_package("git")
    install_package("curl")
    install_package("zip")

def install():
    global system
    global isubuntu
    global version
    global processor
    global ram
    global swap
    if system != "Linux" and isubuntu:
        return print("[!] Please use Ubuntu as OS!")
    elif os.geteuid() != 0:
        return print("The installation must be executed by the root user!")
    elif (ram + swap < 2):
        return print("Not enough RAM and SWAP.")
        # implement swap gestion

    install_toolchain()
    if version == 18:
        install_ros_melodic()


    if version == 20:
        install_ros_noetic()

loadvariable()
check_install()
install()
