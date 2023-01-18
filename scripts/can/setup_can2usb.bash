#!/bin/bash

# enable kernel module: gs_usb
echo mrob@148 | sudo -S modprobe gs_usb

# bring up can interface
echo mrob@148 | sudo -S ip link set can0 up type can bitrate 500000

# install can utils
echo mrob@148 | sudo -S apt install -y can-utils