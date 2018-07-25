roslaunch pepper_interaction pepper.launch 

roslaunch pepper_interaction gmapping_pointCloud.launch 
# or
roslaunch pepper_interaction gmapping_laser.launch
# or
roslaunch pepper_interaction gmapping_octomap.launch

rosrun map_server map_saver /home/luka/catkin_ws/src/pepper_interaction/maps/map.yaml

roslaunch pepper_interaction navigation.launch

roslaunch pepper_interaction servers.launch 

