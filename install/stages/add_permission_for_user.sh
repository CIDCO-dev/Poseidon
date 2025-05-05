#!/bin/bash

# Create a temporary sudoers file in /etc/sudoers.d/
echo "$USER ALL=(ALL) NOPASSWD: /bin/systemctl stop ros" | sudo tee /etc/sudoers.d/special_permission > /dev/null
echo "$USER ALL=(ALL) NOPASSWD: /usr/sbin/shutdown" | sudo tee /etc/sudoers.d/special_permission > /dev/null

# Set correct permissions (read-only for root)
sudo chmod 440 /etc/sudoers.d/special_permission

echo "Sudo rule added for $USER to stop the ROS service and shutdown without password." | tee -a log.txt
