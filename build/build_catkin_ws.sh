#!/bin/bash

echo "Building and all that..."
cd ~/catkin_ws/
catkin_make
sudo chown root:root /path/to/UWB_scan_node/or/desired/folder
sudo chmod a+rx /path/to/UWB_scan_node/or/desired/folder
sudo chmod u+s /path/to/UWB_scan_node/or/desired/folder
