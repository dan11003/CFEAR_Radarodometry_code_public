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
#include "ceres/autodiff_cost_function.h"
#include "ceres/local_parameterization.h"
#include "ceres/ceres.h"
#include "angles/angles.h"



namespace CFEAR_Radarodometry {

using std::cout;
using std::endl;

inline double GetRelTimeStamp(const double x, const double y, const bool ccw){
  double a = atan2(y, x);
  double d = ((a > 0.00001   ? a : (2*M_PI + a))  / (2*M_PI));
  return ccw ? -(d-0.5) : (d-0.5);
}

void IntensityFilter(pcl::PointCloud<pcl::PointXYZI>& cld_in, double th=50, double min = 0, double max = 150);

void FilterReflections(pcl::PointCloud<pcl::PointXYZI>& cld_in, int n_strongest = 25, bool closest=false); // prefare closest rather than strongest

//add some variance on z
void AddNoise(pcl::PointCloud<pcl::PointXYZI>& cld_in, double varz=0.1);

void Convert(pcl::PointCloud<pcl::PointXYZI>& cld_in,pcl::PointCloud<pcl::PointXYZ>& cld_out);

pcl::PointXYZ XYZI_to_XYZ(const pcl::PointXYZI &pin);

pcl::PointXYZI XYZ_to_XYZI(const pcl::PointXYZ &pin, double intensity = 1);

void Compensate(pcl::PointCloud<pcl::PointXYZI>& cloud, const std::vector<double>& mot, bool ccw);

void Compensate(pcl::PointCloud<pcl::PointXYZI>& cloud, const Eigen::Affine3d& Tmotion, bool ccw);

void Affine3dToVectorXYeZ(const Eigen::Affine3d& T, std::vector<double>& par);

void Affine3dToEigVectorXYeZ(const Eigen::Affine3d& T, Eigen::Vector3d& par);

Eigen::Vector3d getScaledTranslationVector(const std::vector<double>& parameters, double factor);

Eigen::Matrix3d getScaledRotationMatrix(const std::vector<double>& parameters, double factor);

template<typename T>
void printVector(const T& t) {
    std::copy(t.cbegin(), t.cend(), std::ostream_iterator<typename T::value_type>(std::cout, ", "));
}



}



