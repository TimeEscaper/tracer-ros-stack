#!/bin/bash

# bring up can interface
echo mrob@148 | sudo -S ip link set can0 up type can bitrate 1000000