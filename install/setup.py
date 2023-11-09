
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


def loadvariable():
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
    #validation version os
    #validation architeture cpu

    ram = psutil.virtual_memory().total / 1000000000
    print("[!] Total RAM:", ram, "Gb")

    swap = psutil.swap_memory().total / 1000000000
    print("[!] Total Swap:", swap, "Gb")

    print("--------------------------------------------")
    #validation memoire
    #validation espace disque

    print("HotSpot")
    #affichage des interface résçau disponible pour la sélection du hotspot
    #liste de choix dynamique
    hotspot_if = inquirer.list_input("Select the wifi interface for creating the hotspot?",
                                choices=['wlan0', 'wlan1'])

    hotspot_ssid = inquirer.text(message="Enter the HotSpot SSID")
    hotspot_password = inquirer.text(message="Enter the HotSpot Password")

    wifi_enable = inquirer.list_input("Do you want to configure wifi internet access?",
                                choices=['Yes', 'No'])
    #affichage des interface wifi disponible pour la connection internet
    #liste de choix dynamique
    wifi_if = inquirer.list_input("Select the wifi interface for creating the WIFI connection?",
                                choices=['wlan0', 'wlan1'])

    wifi_ssid = inquirer.text(message="Enter the WIFI SSID")
    wifi_password = inquirer.text(message="Enter the WIFI Password")

    wirred_type = inquirer.list_input("What type of IP configuration do you want for the wired network interface?",
                                choices=['DHCP', 'Static'])
    #si static demander configuration ip
    rtc = inquirer.list_input("Do you want to activate physical RTC for your raspberry?",
                                choices=['Yes', 'No'])

    swap_status = inquirer.list_input("Do you want to desactivate the SWAP",
                                choices=['Yes', 'No'])

    serialnumber = inquirer.text(message="Enter the Serial Number")

    #selection des nodes a installer
        #slection du node GNSS
        #selection du node IMU
        #selection du node Sonar
        #Selection du type de logger
        # selection du lidar
    
    # creation du launch file

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
        print(f"[V] {package_name} have been installed successfully.")
    except subprocess.CalledProcessError:
        print(f"[X] Error during the installation of {package_name}.")



def update_packager():
    try:
        log_file = open("install.log", "a") 
        subprocess.run(["apt-get", "update"], check=True, stdout=log_file, stderr=log_file)
        log_file.close()
        print(f"[V] Successful update of the package list.")
    except subprocess.CalledProcessError: 
        print(f"[X] Error during the update of the package list.")
    try:
        log_file = open("install.log", "a")
        subprocess.run(["apt-get", "upgrade", "-y"], check=True, stdout=log_file, stderr=log_file)
        log_file.close()
        print(f"[V] Success for the package update.")
    except subprocess.CalledProcessError:
        print(f"[X] Error during the package update .")

def install_ros_melodic():
    print("[X] Not implemented yet!")

def install_ros_noetic():
    print(f"[!] ROS Noetic Installation.")
    try:
        log_file = open("install.log", "a") 
        command = ['/bin/sh', '-c', 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list']
        subprocess.run(command, check=True, stdout=log_file, stderr=log_file)
        log_file.close()
        print(f"[V] Successful append ROS sources.")
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
        print(f"[V] The key have been installed.")
    except subprocess.CalledProcessError: 
        print(f"[X] Error during key installation.")
    
    install_package("ros-noetic-ros-base")
    install_package("ros-noetic-tf2-geometry-msgs")
    install_package("ros-noetic-mavros")
    install_package("ros-noetic-mavros-extras")
    install_package("ros-noetic-mavros-msgs")
    install_package("ros-noetic-sbg-driver")
    install_package("ros-noetic-control-toolbox")
    
    
    install_package("ros-noetic-velodyne")
    #rosdep init
    

def install_toolchain():
    print(f"[!] Toolchain Installation.")
    install_package("gcc")
    install_package("g++")
    install_package("build-essential")
    install_package("python3-dev")
    install_package("python3-pip")
    install_package("python3-rosdep")
    install_package("python3-rosinstall")
    install_package("python3-rosinstall-generator")
    install_package("python3-wstool")
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
    #dois etre vérifier avant les questions
    #remplacer les variable global par des fonction
    if system != "Linux" and isubuntu:
        return print("[X] Please use Ubuntu as OS!")
    elif os.geteuid() != 0:
        return print("[X] The installation must be executed by the root user!")
    elif (ram + swap < 2):
        return print("[X] Not enough RAM and SWAP.")
        # implement swap gestion

    update_packager()
    install_toolchain()
    if version == 18:
        install_ros_melodic()


    if version == 20:
        install_ros_noetic()
        #gpsd client
        #wifi hotspot 29
        install_package("network-manager")
        #install web server 3
        install_package("lighttpd")
        service_enable("lighttpd")
        install_package("libwebsocketpp-dev")
        install_package("rapidjson-dev")
        make_record()
        fix_lighttpd_conf()
        service_start("lighttpd")
        #install gpsD 4
        install_package("gpsd")
        install_package("gpsd-clients") 
        install_package("libgps-dev")
        fix_gpsd_conf
        install_package("pps-tools")

        #install opencv 5
        #install libexiv2 5
        #install inertial sence sdlk5 
        #install Chrony 5
        # install rtklib
        #architecture detection 
            #x64
            #arm
                #fix uboot for rpi3 2 rpi
                #install wiringpi to be confirmed
                #uart configuration dtoverlay
                #install pps
                #install i2c
                #config uart cmdline.txt
                # RTC
                #enable spi
        #device remap        
        #build
        #service ros        
        #Zed-f9p configuration
        #bno055 calibration



loadvariable()
check_install()
install()
