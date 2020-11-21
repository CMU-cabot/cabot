#!/bin/bash
set -e


#colcon build
#source install/setup.bash
#./install/groot/bin/Groot --mode monitor

cd /home/developer/src/Groot
mkdir -p build
cd build
cmake ..
make
./Groot --mode monitor

