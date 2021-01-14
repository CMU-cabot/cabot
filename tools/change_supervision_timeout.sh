#!/bin/sh

echo 200 | sudo tee /sys/kernel/debug/bluetooth/hci0/supervision_timeout
echo 200 | sudo tee /sys/kernel/debug/bluetooth/hci1/supervision_timeout
