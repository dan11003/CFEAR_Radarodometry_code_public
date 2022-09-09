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
#include "cfear_radarodometry/utils.h"
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

typedef enum reg_mode{incremental_last_to_previous, many_to_many_refinement} regmode;

typedef enum weight_options{Uniform = 0, Sim_N = 1, Sim_direciton = 2, Sim_scale = 3, Combined_weights = 4}weightoption;

const Matrix6d Identity66 = Matrix6d::Identity();

/* cost metric */
typedef enum costmetric{P2P, P2L, P2D}cost_metric;

cost_metric Str2Cost(const std::string& str);

/* robust loss function */
typedef enum losstype{None, Huber, Cauchy, SoftLOne, Combined, Tukey}loss_type;

std::string loss2str(const loss_type& loss);

loss_type Str2loss(const std::string& loss);



class Registration
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Registration();

  virtual bool Register(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc, std::vector<Matrix6d>& reg_cov, bool soft_constraints = false)=0;

  virtual double getScore();

  virtual std::string GetParameterString();

  //void SetFixedBlocks(std::vector<bool>& fixedBlock){fixedBlock_ = fixedBlock;}

  void InitFixedBlocks(const size_t& nsize);

  void SetMode(const regmode mode){mode_ = mode;}

  class Weights
  {
  public:

    Weights(double N1, double N2, double sim_dir, double plan1, double plan2) : N1_(N1), N2_(N2), sim_dir_(sim_dir), plan1_(plan1), plan2_(plan2){}

    double GetWeight(const weightoption opt);

    double Similarity(const double x, const double y){return 2*std::min(x,y)/(x+y);}

    double N1_, N2_;
    double sim_dir_;
    double plan1_, plan2_;
  };

  weightoption weight_opt_ = weightoption::Uniform;
  std::map<int_pair, std::vector<Weights> >  weight_associations_;

  std::map<int_pair, std::vector<int_pair> > scan_associations_;
  size_t itr_ = 0;


  ceres::Solver::Summary summary_;

protected:


  ceres::LossFunction* GetLoss();

  cost_metric cost_ = P2L;
  loss_type loss_ = Huber;
  double loss_limit_ = 0.1;

  std::vector<MapNormalPtr> scans_; // 0 = target, 1 = source
  double radius_ = 2.0;
  std::vector<std::vector<double> > parameters;
  std::vector<bool> fixedBlock_;
  ros::NodeHandle nh_;
  boost::shared_ptr<ceres::Problem> problem_;
  regmode mode_ = incremental_last_to_previous;

  ceres::Solver::Options options_;
  ros::Publisher pub_association;
  double score_;

};


Eigen::Matrix3d getScaledRotationMatrix(const std::vector<double>& parameters, double factor);

Eigen::Vector3d getScaledTranslationVector(const std::vector<double>& parameters, double factor);

inline geometry_msgs::Point Pntgeom(Eigen::Vector3d& u);

void normalizeEulerAngles(Eigen::Vector3d &euler);

inline geometry_msgs::Point Pntgeom(Eigen::Vector3d& u);

Eigen::Affine3d vectorToAffine3d(double x, double y, double z, double ex, double ey, double ez);

Eigen::Affine3d vectorToAffine3d(const std::vector<double>& vek);

Eigen::Affine2d vectorToAffine2d(double x, double y, double ez);

void Affine3dToEigVectorXYeZ(const Eigen::Affine3d& T, Eigen::Vector3d& par);

void Affine3dToVectorXYeZ(const Eigen::Affine3d& T, std::vector<double>& par) ;

Eigen::Matrix3d Cov6to3(const Matrix6d &C);

}

