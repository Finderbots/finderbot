#!/bin/bash

echo "Building and all that..."
cd ~/catkin_ws/
catkin_make
sudo chown root:root ~/catkin_ws/devel/lib/finderbot/UWB_scan_node
sudo chmod a+rx ~/catkin_ws/devel/lib/finderbot/UWB_scan_node
sudo chmod u+s ~/catkin_ws/devel/lib/finderbot/UWB_scan_node
