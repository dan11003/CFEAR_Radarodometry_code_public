


# CFEAR Radarodometry
This guide describes how to download data and estimate radar odometry using CFEAR_Radarodometry
  
## prerequisites
  * Google Ceres solver  http://ceres-solver.org/installation.html
  * ROS Melodic or later


Ask questions [here](https://github.com/dan11003/CFEAR_Radarodometry_code_public/issues).

## How to build with catkin

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/dan11003/CFEAR_Radarodometry_code_public.git
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release 
$ source ~/catkin_ws/devel/setup.bash
```
## Downloading data (Oxford Radar Robotcar)
Currently, only rosbags that contain sensor_msgs/Image are supported.
We prepared a rosbag from the Oxford Radar Robotcar dataset on our [google drive](https://drive.google.com/drive/folders/12YNIvHQqSO5Et3UIzKD1z3XQACpoGZ1L?usp=sharing)
Download the file Oxford (processed rosbag)/2019-01-10-12-32-52-radar-oxford-10k.bag to the folder created below:
```
mkdir -p /home/${USER}/Documents/oxford-eval-sequences/2019-01-10-12-32-52-radar-oxford-10k/
cd /home/${USER}/Documents/oxford-eval-sequences/2019-01-10-12-32-52-radar-oxford-10k/
```

## Running
The odometry an be launched in two modes.
* Offline: (used in this guide) runs at the maximum possible rate.
* Online: Standard rostopic interface. radar_driver_node and odometry_keyframes. The radar_driver_node subscribes to "/Navtech/Polar" 

```
roscd cfear_radarodometry/launch
./oxford_demo
```


## Troubleshooting

There is a known competability issue related to incompatible versions of LZ4. See [this link](https://github.com/ethz-asl/lidar_align/issues/16) for a dirty fix.

## Other files
For a detailed guide on how to download and format radar data into rosbags, visit our [Oxford converter readme file](https://docs.google.com/document/d/1ij8E4PMpCpBwWYbRAdU9rnScocOaMB4Sqz4XS48XWoo/edit?usp=sharing)

## Future

* Prepare demo for MulRan, VolvoCE, Kvarntorp and Orkla

## Developing

Feel free to issue pull requests with updates.



