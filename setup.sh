path_dst=~/.gazebo/models/
path_src_dir=~/catkin_ws/src/finderbot/FinderBot_Gazebo/
path_lidar_src=${path_src_dir}/hokuyo_utm30lx
path_robot_src=${path_src_dir}/finderbot_model
path_complete_src=${path_src_dir}/Finderbot_Lidar
path_imu_src=${path_src_dir}/noisy_imu
cp -r $path_lidar_src $path_dst
cp -r $path_robot_src $path_dst
cp -r $path_complete_src $path_dst
cp -r $path_imu_src $path_dst