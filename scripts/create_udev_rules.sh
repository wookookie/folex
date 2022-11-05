#!/bin/bash

echo ""
echo "This script copies the rules to '/etc/udev/rules.d/' and applies."
echo "ROBOTIS U2D2 SYMLINK: /dev/U2D2"
echo "USB latency: 1"

sudo cp 99-u2d2-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ""
echo "USB rules reloaded."
