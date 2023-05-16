#!/bin/bash

if [[ -z $1 ]]; then
    size=$((64*1024*1024))
else
    size=$(($1*1024*1024))
fi
echo "size=$size"
sudo sysctl -w net.core.rmem_max=$size net.core.rmem_default=$size
sudo sysctl -w net.core.wmem_max=$size net.core.wmem_default=$size
