#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
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
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/common/centroid.h"
#include "pcl/point_types.h"

#include "visualization_msgs/MarkerArray.h"
#include "cfear_radarodometry/intensity_utils.h"
#include "ceres/autodiff_cost_function.h"
#include "ceres/local_parameterization.h"
#include "ceres/ceres.h"
#include "angles/angles.h"
#include "string.h"
#include "cfear_radarodometry/pointnormal.h"
#include "boost/shared_ptr.hpp"




namespace CFEAR_Radarodometry {

class Registration;

typedef std::vector<std::pair< Eigen::Affine3d, pcl::PointCloud<pcl::PointXYZI> > > reference_scan;
typedef std::vector<int,int> associations;
typedef std::pair<int,int> int_pair;
typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef boost::shared_ptr<CFEAR_Radarodometry::Registration> regPtr;

const Matrix6d Identity66 = Matrix6d::Identity();


class Registration
{
public:

EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Registration();

  //virtual bool Register(MapNormalPtr target, MapNormalPtr src, Eigen::Affine3d& Tsrc, Eigen::MatrixXd& reg_cov)=0;

  virtual bool Register(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc, std::vector<Matrix6d>& reg_cov, bool soft_constraints = false)=0;

//  virtual bool Register(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc, std::vector<Eigen::Matrix3d>& reg_cov, bool soft_constraints = false)=0;

  //virtual bool Register(MapNormalPtr target, MapNormalPtr src, Eigen::Affine3d& Tsrc, Eigen::Matrix3d& reg_cov) =0;

  virtual double getScore();

  virtual std::string GetParameterString();

  void SetFixedBlocks(std::vector<bool>& fixedBlock){fixedBlock_ = fixedBlock;}



 std::map<int_pair,std::vector<int_pair> >  scan_associations_;

 ceres::Solver::Summary summary_;

protected:

  std::vector<MapNormalPtr> scans_; // 0 = target, 1 = source
  double radius_ = 2.0;
  std::vector<std::vector<double> > parameters;
  std::vector<bool> fixedBlock_;
  ros::NodeHandle nh_;
  boost::shared_ptr<ceres::Problem> problem_;

  ceres::Solver::Options options_;
  ros::Publisher pub_association;
  double score_;

};


template <typename T>
Eigen::Matrix<T,3,3> getRotationMatrix(const T* parameters)
{
  Eigen::Matrix<T,3,3> rotation_matrix;
  const T s_1 = ceres::sin(parameters[2]);
  const T c_1 = ceres::cos(parameters[2]);
  rotation_matrix <<
      c_1,     -s_1,     T(0),
      s_1,     c_1,      T(0),
      T(0),    T(0),     T(1);
  return rotation_matrix;
}
template <typename T>
Eigen::Matrix<T,3,1> getTranslationVector(const T* parameters)
{
  return Eigen::Matrix<T,3,1>(parameters[0], parameters[1], T(0));
}


Eigen::Matrix3d getScaledRotationMatrix(const std::vector<double>& parameters, double factor);

Eigen::Vector3d getScaledTranslationVector(const std::vector<double>& parameters, double factor);

inline geometry_msgs::Point Pntgeom(Eigen::Vector3d& u);

void normalizeEulerAngles(Eigen::Vector3d &euler);

inline geometry_msgs::Point Pntgeom(Eigen::Vector3d& u);

Eigen::Affine3d vectorToAffine3d(double x, double y, double z, double ex, double ey, double ez);

Eigen::Affine2d vectorToAffine2d(double x, double y, double ez);

void Affine3dToEigVectorXYeZ(const Eigen::Affine3d& T, Eigen::Vector3d& par);

void Affine3dToVectorXYeZ(const Eigen::Affine3d& T, std::vector<double>& par) ;

Eigen::Matrix3d Cov6to3(const Matrix6d &C);

}

