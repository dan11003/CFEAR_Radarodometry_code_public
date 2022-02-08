#pragma once
#include "pcl/io/pcd_io.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/publisher.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/point_types.h"
#include <time.h>
#include <fstream>
#include <cstdio>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include <Eigen/Eigen>
#include "tuple"
#include "list"



namespace CFEAR_Radarodometry {

using std::cout;
using std::endl;

void IntensityFilter(pcl::PointCloud<pcl::PointXYZI>& cld_in, double th=50, double min = 0, double max = 150);

void FilterReflections(pcl::PointCloud<pcl::PointXYZI>& cld_in, int n_strongest = 25, bool closest=false); // prefare closest rather than strongest

//add some variance on z
void AddNoise(pcl::PointCloud<pcl::PointXYZI>& cld_in, double varz=0.1);

void Convert(pcl::PointCloud<pcl::PointXYZI>& cld_in,pcl::PointCloud<pcl::PointXYZ>& cld_out);

pcl::PointXYZ XYZI_to_XYZ(const pcl::PointXYZI &pin);

pcl::PointXYZI XYZ_to_XYZI(const pcl::PointXYZ &pin, double intensity = 1);



}



