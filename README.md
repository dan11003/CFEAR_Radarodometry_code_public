CFEAR Radarodometry

Install prerequisites
*Google Ceres solver  http://ceres-solver.org/installation.html
*ROS Melodic or later


The odometry an be launched in two modes.
*Online: Standard rostopic interface. radar_driver_node and odometry_keyframes. The driver subscribes to "/Navtech/Polar" 
#NOTE that rosbag play <bagfile> needs to be executed manually



*Offline: Uses the Rosbag API and attempt to estimate odometry at the maximum possible rate.
cd /radar_mapping/launch/oxford/eval/
./run_sequence


