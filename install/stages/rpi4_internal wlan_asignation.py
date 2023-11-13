import subprocess
import re

# Execute the 'nmcli' command to get information about network interfaces
nmcli_output = subprocess.check_output(['nmcli', 'device', 'show']).decode('utf-8')

# Search for the interface managed by the brcmfmac driver
brcmfmac_interface_match = re.search(r'(brcmfmac[0-9]+).*?HARDWARE ADDRESSES: ([0-9a-fA-F:]+)', nmcli_output)

if brcmfmac_interface_match:
    brcmfmac_interface = brcmfmac_interface_match.group(1)
    brcmfmac_mac_address = brcmfmac_interface_match.group(2)
    print(f"Interface managed by brcmfmac detected: {brcmfmac_interface}")
    print(f"MAC address of the interface managed by brcmfmac: {brcmfmac_mac_address}")

    # Create the udev rule based on the MAC address
    udev_rule = f'SUBSYSTEM=="net", ACTION=="add", ATTR{{address}}=="{brcmfmac_mac_address}", NAME="wlan0"'

    # Write the udev rule to a temporary file
    with open('/tmp/70-persistent-net-custom.rules', 'w') as udev_file:
        udev_file.write(udev_rule)

    # Copy the temporary file to the udev rules directory
    subprocess.call(['sudo', 'cp', '/tmp/70-persistent-net-custom.rules', '/etc/udev/rules.d/'])

    # Reload udev rules
    subprocess.call(['sudo', 'udevadm', 'control', '--reload-rules'])
    subprocess.call(['sudo', 'udevadm', 'trigger'])

    print(f"Udev rule added to assign the name wlan0 to the interface with MAC address {brcmfmac_mac_address}.")
else:
    print("No interface managed by brcmfmac was found in the nmcli output.")
