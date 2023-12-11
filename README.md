# Lidar-Level Localization With Radar? The CFEAR Approach to Accurate, Fast, and Robust Large-Scale Radar Odometry in Diverse Environments
This repository implements the most recent version of CFEAR Radar odometry.

This guide describes how to download data and estimate radar odometry using CFEAR Radar odometry

## News (April 2023): Article is now published in Transactions on Robotics

Paper: [T-RO](https://ieeexplore.ieee.org/document/9969174) or on
[arXiv](https://arxiv.org/abs/2211.02445)
<details>
<summary>Bibtex</summary>
 
```
@ARTICLE{9969174,
  author={Adolfsson, Daniel and Magnusson, Martin and Alhashimi, Anas and Lilienthal, Achim J. and Andreasson, Henrik},
  journal={IEEE Transactions on Robotics}, 
  title={Lidar-Level Localization With Radar? The CFEAR Approach to Accurate, Fast, and Robust Large-Scale Radar Odometry in Diverse Environments}, 
  year={2023},
  volume={39},
  number={2},
  pages={1476-1495},
  doi={10.1109/TRO.2022.3221302}}
  
```
</details>  



<img src="cfear_3_animation.gif" width="500" height="500">

  
## Prerequisites
  * Install the Google Ceres solver  http://ceres-solver.org/installation.html
  * ROS [Melodic](http://wiki.ros.org/melodic) or later, tested with ubuntu 16.04, 18.04 and 20.04
  * The master branch contains the code used in the T-RO publication.

## How to build with catkin

```
$ mkdir -p ~/catkin_ws/src/
$ cd ~/catkin_ws/src/
$ git clone https://github.com/dan11003/CFEAR_Radarodometry_code_public.git
$ cd ~/catkin_ws
$ catkin_make -DCMAKE_BUILD_TYPE=Release 
$ source ~/catkin_ws/devel/setup.bash
```
## Downloading data (Oxford Radar RobotCar)
Currently, only rosbags that contain sensor_msgs/Image are supported.
We prepared a rosbag from the Oxford Radar Robotcar dataset on our [google drive](https://drive.google.com/drive/folders/12YNIvHQqSO5Et3UIzKD1z3XQACpoGZ1L?usp=sharing).
Download the Oxford bag file from (processed rosbag)/2019-01-10-12-32-52-radar-oxford-10k.bag to the folder created below:
```
$ mkdir -p /home/${USER}/Documents/oxford-eval-sequences/2019-01-10-12-32-52-radar-oxford-10k/
$ cd /home/${USER}/Documents/oxford-eval-sequences/2019-01-10-12-32-52-radar-oxford-10k/
$ mv ~/Downloads/2019-01-10-12-32-52-radar-oxford-10k.bag .
```

## Running
The odometry can be launched in two modes.
* Offline: (used in this guide) runs at the maximum possible rate.
* Online (not tested): Standard rostopic interface. radar_driver_node and odometry_keyframes. The radar_driver_node subscribes to "/Navtech/Polar" 

```
roscd cfear_radarodometry/launch
./oxford_demo # (Note: oxford trajectory is mirrored)
```

## Optional -  Evaluate odometry quality

```
cd ~/catkin_ws/src/
git clone https://github.com/dan11003/radar_kitti_benchmark 
cd radar_kitti_benchmark/python
./oxford_demo_eval.sh # Note that every run is evaluated
```

This will compute odometry error metrics, draw graphs and figures, and save these to the folder:
```
/home/${USER}/Documents/oxford-eval-sequences/2019-01-10-12-32-52-radar-oxford-10k/eval/<yyyy-mm-dd_HH:mm>,`
```
where the folder name <yyyy-mm-dd_HH:mm>  is set automatically according to time.

Some of the evaluation output is:
 * The raw trajectory (/est/01.txt)
 * Trajectory visualized (/est/plot_path/sequence_01_orig.pdf)  -  similar to Fig.15a in the article
 * Error metrics (/est/result.txt). -  Tab.III (row "CFEAR-3", column "10-12-32") in the article.
 * Parameters  (pars.txt)  - Tab.I (column "CFEAR-3")  in the article
 
 
NOTE: Results are slightly better compared to the article.


## Troubleshooting

If the fixes below does not help. Ask questions [here](https://github.com/dan11003/CFEAR_Radarodometry/issues).

### Malformed bagfile
```console
# Problem
user@computer:~$ terminate called after throwing an instance of 'rosbag::BagFormatException' what():  Required 'op' field missing
# Solution
rosbag reindex 2019-01-10-12-32-52-radar-oxford-10k.bag
```
### Compilation issue due to incompatible versions of LZ4 in ROS Melodic
During compilation, this error can occur on some installations of Ubuntu 18.04 with ROS Melodic:
```console
/usr/include/flann/ext/lz4.h:196:57: error: conflicting declaration ‘typedef struct LZ4_stream_t LZ4_stream_t’
 typedef struct { long long table[LZ4_STREAMSIZE_U64]; } LZ4_stream_t;
...
/usr/include/lz4.h:196:57: note: previous declaration as ‘typedef struct LZ4_stream_t LZ4_stream_t’
 typedef struct { long long table[LZ4_STREAMSIZE_U64]; } LZ4_stream_t;
```
and
```console
/usr/include/flann/ext/lz4.h:249:72: error: conflicting declaration ‘typedef struct LZ4_streamDecode_t LZ4_streamDecode_t’
 typedef struct { unsigned long long table[LZ4_STREAMDECODESIZE_U64]; } LZ4_streamDecode_t;
...
/usr/include/lz4.h:249:72: note: previous declaration as ‘typedef struct LZ4_streamDecode_t LZ4_streamDecode_t’
 typedef struct { unsigned long long table[LZ4_STREAMDECODESIZE_U64]; } LZ4_streamDecode_t;
```
This error is caused by two conflicting versions of LZ4, one used for ROS serialization and the other one by flann kdtree in PCL. As this [GitHub issue](https://github.com/ethz-asl/lidar_align/issues/16) suggests, there are a few ways to fix it, yet none of them can be really considered clean. We suggest using simlinks to point to a single version of the conficting header files, since this modification can be easily reverted:
```console
sudo mv /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4.h.bak
sudo mv /usr/include/flann/ext/lz4hc.h /usr/include/flann/ext/lz4.h.bak


sudo ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h
sudo ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h
```
## Future

* Prepare demo for MulRan, VolvoCE, Kvarntorp and Orkla.
* Integrate reading of the raw data format adopted in the [Oxford dataset](https://github.com/ori-mrg/robotcar-dataset-sdk).

## Developing

Feel free to issue pull requests with updates.


## Other files
For a detailed guide on how to download and format Oxford RobotCar radar data into rosbags, visit our [Oxford converter readme file](https://docs.google.com/document/d/1ij8E4PMpCpBwWYbRAdU9rnScocOaMB4Sqz4XS48XWoo/edit?usp=sharing).
Data from the MulRan dataset can be processed using the [file_player_tool](https://github.com/irapkaist/file_player_mulran).
Don't forget to specify the sensor resolution and the dataset parameter if you use something other than Oxford data.

## Updates
Follow our updates [here](https://github.com/dan11003/CFEAR_Radarodometry).

## CFEAR use cases

### April 2023 - TBV Radar SLAM 
In the article TBV Radar SLAM by Adolfsson et al. CFEAR was integrated with robust loop closure into a fully-fledged system for large-scale, real-time, radar-only SLAM.
The method generalizes across diverse environments without the need for returning parameters. The [code is released here](https://github.com/dan11003/tbv_slam_public).


### October 2023 - RadaRays
[RadaRays](https://arxiv.org/pdf/2310.03505.pdf) is a method for simulating radar data, released as a gazebo plugin. Mock et al. were able to simulate radar scans from a prior lidar map, and accurately estimate radar odometry in the MulRan dataset. The estimated trajectories closely resemble the odometry estimated from real radar data.






