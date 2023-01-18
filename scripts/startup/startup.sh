#!/bin/sh

# canbus
./tracer-ros-stack/scripts/can/bringup_can2usb_500k.bash

# docker container
docker run --rm --net=host --pid=host tracer

