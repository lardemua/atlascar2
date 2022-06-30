#!/bin/bash

ifconfig can0
sudo ip link set can0 up type can bitrate 500000