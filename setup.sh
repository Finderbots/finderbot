path_dst=~/.gazebo/models/
path_lidar_src=~/catkin_ws/src/finderbot/FinderBot_Gazebo/hokuyo_utm30lx
path_robot_src=~/catkin_ws/src/finderbot/FinderBot_Gazebo/finderbot_model
cp -r $path_lidar_src $path_dst
cp -r $path_robot_src $path_dst