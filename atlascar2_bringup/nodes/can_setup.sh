#!/bin/bash

ifconfig can0
# sudo ip link set can0 up type can bitrate 500000 
SUDO_PASSWORD="your_personal_password"
echo $SUDO_PASSWORD | sudo -S /sbin/ip link set can0 down
echo $SUDO_PASSWORD | sudo -S /sbin/ip link set can0 up type can bitrate 500000