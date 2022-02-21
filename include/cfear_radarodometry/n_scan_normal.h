#pragma once
#include "pcl/io/pcd_io.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/publisher.h"
#include "sensor_msgs/PointCloud2.h"
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
#include "cfear_radarodometry/registration.h"
#include "ceres/loss_function.h"

namespace CFEAR_Radarodometry{

/* cost metric */
typedef enum costmetric{P2P, P2L, P2D}cost_metric;

cost_metric Str2Cost(const std::string& str);

/* robust loss function */
typedef enum losstype{None, Huber, Cauchy, SoftLOne, Combined, Tukey}loss_type;

std::string loss2str(const loss_type& loss);

loss_type Str2loss(const std::string& loss);


/* Registration type */
class n_scan_normal_reg : public Registration{

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  n_scan_normal_reg();

  n_scan_normal_reg(const cost_metric& cost, loss_type loss=Huber, double loss_limit=0.1);

  bool Register(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc, std::vector<Matrix6d>& reg_cov, bool soft_constraints = false);

  bool GetCost(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc, double& score, std::vector<double>& residuals);

  void GetSurface(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc, std::vector<Matrix6d> &reg_cov, bool soft_constraints, Eigen::MatrixXd& surface, double res, int width);

  //bool Register(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc, std::vector<Eigen::Matrix3d>& reg_cov, bool soft_constraints = false);

  double getScore(){return score_;}

  double SetD2dPar(const double cov_scale,const double regularization){cov_scale_ = cov_scale; regularization_ = regularization;}

  void AddScanPairCost(MapNormalPtr& target_local, MapNormalPtr& src_local, const Eigen::Affine2d& Ttar, const Eigen::Affine2d& Tsrc, const size_t scan_idx_tar, const size_t scan_idx_src);


private:

  bool BuildOptimizationProblem(std::vector<MapNormalPtr>& scans, const Eigen::MatrixXd& cov = Eigen::Matrix<double,3,3>::Identity(), const Eigen::Vector3d &guess = Eigen::Vector3d::Identity(), bool soft_constraint = false);

  bool SolveOptimizationProblem();

  bool GetCovariance(Matrix6d& Cov);

  void EvaluateResiduals(std::vector<MapNormalPtr>& scans);

  cost_metric cost_ = P2L;
  loss_type loss_ = Huber;
  double loss_limit_ = 0.1;
  double cov_scale_ = 1;
  double regularization_ = 0.01;
  const double score_tolerance = 0.00001;
  const double max_itr = 8, min_itr = 2;
  const bool efficient_implementation = true;

};

class RegistrationCost{
  protected:

  RegistrationCost () {}

  template <typename T>
  static Eigen::Matrix<T, 2, 2> GetRotMatrix2D(const T* par){
    T cos_yaw = ceres::cos(par[2]);
    T sin_yaw = ceres::sin(par[2]);
    Eigen::Matrix<T, 2, 2> rotation;
    rotation << cos_yaw,  -sin_yaw,
        sin_yaw,  cos_yaw;
    return rotation;
  }

  template <typename T>
  static Eigen::Matrix<T, 2, 1> GetTransMatrix2D(const T*  par) {
    Eigen::Matrix<T, 2, 1> trans;
    trans << par[0], par[1];
    return trans;
  }
};

class P2LCost : public RegistrationCost{
public:

  P2LCost (const Eigen::Vector2d& target_mean, const Eigen::Vector2d& target_normal, const Eigen::Vector2d& src_mean, const double weight) :
    tar_mean_(target_mean),
    tar_normal_(target_normal),
    src_mean_(src_mean),
    weight_(weight) {}

  template <typename T>
  bool operator()(const T*  Ta,
                  const T*  Tb,
                  T* residuals_ptr) const {
    const Eigen::Matrix<T,2,1> trans_mat_tar = GetTransMatrix2D(Ta);
    const Eigen::Matrix<T,2,2> rot_mat_tar = GetRotMatrix2D(Ta);

    const Eigen::Matrix<T,2,1> trans_mat_src = GetTransMatrix2D(Tb);
    const Eigen::Matrix<T,2,2> rot_mat_src = GetRotMatrix2D(Tb);

    const Eigen::Matrix<T,2,1> transformed_mean_tar = (rot_mat_tar * (tar_mean_).cast<T>()) + trans_mat_tar;
    const Eigen::Matrix<T,2,1> transformed_mean_src = (rot_mat_src * (src_mean_).cast<T>()) + trans_mat_src;

    const Eigen::Matrix<T,2,1> transformed_normal_tar = (rot_mat_tar * (tar_normal_).cast<T>());

    //T cost = T(0.0);
    const Eigen::Matrix<T,2,1> v = transformed_mean_src - transformed_mean_tar;

    const Eigen::Matrix<T,2,1> n = transformed_normal_tar;

    residuals_ptr[0] = v.dot(n);
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector2d& target_mean, const Eigen::Vector2d& target_normal, const Eigen::Vector2d& src_mean, double scale_similarity) {
    return new ceres::AutoDiffCostFunction<P2LCost ,1, 3, 3>(new P2LCost (target_mean, target_normal, src_mean, scale_similarity));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:

  const Eigen::Vector2d tar_mean_;
  const Eigen::Vector2d tar_normal_;
  const Eigen::Vector2d src_mean_;
  const double weight_;
};


// reference precomputed
class P2LEfficientCost : public RegistrationCost{
public:

  P2LEfficientCost (const Eigen::Vector2d& transformed_mean_tar, const Eigen::Vector2d& transformed_normal_tar, const Eigen::Vector2d& src_mean, double weight) :
    transformed_mean_tar_(transformed_mean_tar),
    transformed_normal_tar_(transformed_normal_tar),
    src_mean_(src_mean),
    weight_(weight)  {}

  template <typename T>
  bool operator()(const T*  Tb,
                  T* residuals_ptr) const {

    const Eigen::Matrix<T,2,1> trans_mat_src = GetTransMatrix2D(Tb);
    const Eigen::Matrix<T,2,2> rot_mat_src = GetRotMatrix2D(Tb);

    const Eigen::Matrix<T,2,1> transformed_mean_src = (rot_mat_src * (src_mean_).cast<T>()) + trans_mat_src;
    const Eigen::Matrix<T,2,1> v = transformed_mean_src - transformed_mean_tar_.cast<T>();
    const Eigen::Matrix<T,2,1> n = transformed_normal_tar_.cast<T>();
    residuals_ptr[0] = v.dot(n);
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector2d& transformed_mean_tar, const Eigen::Vector2d& transformed_normal_tar, const Eigen::Vector2d& src_mean, double weight) {
    return new ceres::AutoDiffCostFunction<P2LEfficientCost ,1, 3>(new P2LEfficientCost (transformed_mean_tar, transformed_normal_tar, src_mean, weight));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:

  const Eigen::Vector2d transformed_mean_tar_;
  const Eigen::Vector2d transformed_normal_tar_;
  const Eigen::Vector2d src_mean_;
  const double weight_;
};


class P2DCost : public RegistrationCost{
public:

  P2DCost (const Eigen::Vector2d& target_mean, const Eigen::Matrix2d& tar_sqrt_information, const Eigen::Vector2d& src_mean) :
    tar_mean_(target_mean),
    tar_sqrt_information_(tar_sqrt_information),
    src_mean_(src_mean) {}

  template <typename T>
  bool operator()(const T*  Ta,
                  const T*  Tb,
                  T* residuals_ptr) const {
    const Eigen::Matrix<T,2,1> trans_mat_tar = GetTransMatrix2D(Ta);
    const Eigen::Matrix<T,2,2> rot_mat_tar = GetRotMatrix2D(Ta);

    const Eigen::Matrix<T,2,1> trans_mat_src = GetTransMatrix2D(Tb);
    const Eigen::Matrix<T,2,2> rot_mat_src = GetRotMatrix2D(Tb);

    const Eigen::Matrix<T,2,1> transformed_mean_tar = (rot_mat_tar * (tar_mean_).cast<T>()) + trans_mat_tar;
    const Eigen::Matrix<T,2,1> transformed_mean_src = (rot_mat_src * (src_mean_).cast<T>()) + trans_mat_src;


    //residuals_ptr[0] = /*T(scale_similarity_)**/ v.dot(n);   // if scale similarity is different, then simply consider it less as the data association is inaccurate
    Eigen::Map<Eigen::Matrix<T, 2, 1> > residuals(residuals_ptr);
    residuals.template block<2, 1>(0, 0) = (transformed_mean_src - transformed_mean_tar);
    residuals.applyOnTheLeft(tar_sqrt_information_.template cast<T>());
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector2d& target_mean, const Eigen::Matrix2d& tar_sqrt_information, const Eigen::Vector2d& src_mean) {
    return new ceres::AutoDiffCostFunction<P2DCost ,2, 3, 3>(new P2DCost (target_mean, tar_sqrt_information, src_mean));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:

  const Eigen::Vector2d tar_mean_;
  const Eigen::Matrix2d tar_sqrt_information_;
  const Eigen::Vector2d src_mean_;
};



class mahalanobisDistanceError {
public:

  mahalanobisDistanceError (const Eigen::Vector3d& target_mean, const Eigen::Matrix3d& tar_sqrt_information, const double alpha) : tar_mean_(target_mean), tar_sqrt_information_(tar_sqrt_information), alpha_(alpha) {}

  template <typename T>
  static Eigen::Matrix<T, 3, 1> GetVector(const T*  par) {
    Eigen::Matrix<T, 3, 1> trans;
    trans << par[0], par[1], par[2];
    return trans;
  }

  template <typename T>
  bool operator()(const T*  par,
                  T* residuals_ptr) const {

    Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals(residuals_ptr);
    residuals.template block<3, 1>(0, 0) = T(alpha_)*(tar_mean_.template cast<T>() - GetVector(par));
    residuals.applyOnTheLeft(tar_sqrt_information_.template cast<T>());
    return true;
  }
  static ceres::CostFunction* Create(
      const Eigen::Vector3d& target_mean, const Eigen::Matrix3d& tar_sqrt_information, double alpha) {
    return new ceres::AutoDiffCostFunction<mahalanobisDistanceError,1, 3>(new mahalanobisDistanceError (target_mean, tar_sqrt_information, alpha));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:

  Eigen::Vector3d tar_mean_;
  Eigen::Matrix3d tar_sqrt_information_;
  double alpha_;
};


class P2PCost : public RegistrationCost{
public:

  P2PCost (const Eigen::Vector2d& target_mean, const Eigen::Vector2d& src_mean) : tar_mean_(target_mean), src_mean_(src_mean) {}

  template <typename T>
  bool operator()(const T*  Ta,
                  const T*  Tb,
                  T* residuals_ptr) const {
    const Eigen::Matrix<T,2,1> trans_mat_tar = GetTransMatrix2D(Ta);
    const Eigen::Matrix<T,2,2> rot_mat_tar = GetRotMatrix2D(Ta);

    const Eigen::Matrix<T,2,1> trans_mat_src = GetTransMatrix2D(Tb);
    const Eigen::Matrix<T,2,2> rot_mat_src = GetRotMatrix2D(Tb);

    const Eigen::Matrix<T,2,1> transformed_mean_tar = (rot_mat_tar * (tar_mean_).cast<T>()) + trans_mat_tar;
    const Eigen::Matrix<T,2,1> transformed_mean_src = (rot_mat_src * (src_mean_).cast<T>()) + trans_mat_src;

    //residuals_ptr[0] = /*T(scale_similarity_)**/ v.dot(n);   // if scale similarity is different, then simply consider it less as the data association is inaccurate
    residuals_ptr[0] = transformed_mean_tar(0) - transformed_mean_src(0);
    residuals_ptr[1] = transformed_mean_tar(1) - transformed_mean_src(1);
    //residuals_ptr[0] = T(scale_similarity_)*(transformed_mean_src-transformed_mean_tar).norm();
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector2d& target_mean, const Eigen::Vector2d& src_mean) {
    return new ceres::AutoDiffCostFunction<P2PCost ,2, 3, 3>(new P2PCost (target_mean, src_mean));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:

  const Eigen::Vector2d tar_mean_, src_mean_;

};


class P2PEfficientCost : public RegistrationCost{
public:

  P2PEfficientCost (const Eigen::Vector2d& tar_transformed, const Eigen::Vector2d& src_mean) : tar_transformed_(tar_transformed), src_mean_(src_mean) {}

  template <typename T>
  bool operator()(const T*  Tb,
                  T* residuals_ptr) const {

    const Eigen::Matrix<T,2,1> trans_mat_src = GetTransMatrix2D(Tb);
    const Eigen::Matrix<T,2,2> rot_mat_src = GetRotMatrix2D(Tb);

    const Eigen::Matrix<T,2,1> transformed_mean_tar = tar_transformed_.cast<T>();
    const Eigen::Matrix<T,2,1> transformed_mean_src = (rot_mat_src * (src_mean_).cast<T>()) + trans_mat_src;

    //residuals_ptr[0] = /*T(scale_similarity_)**/ v.dot(n);   // if scale similarity is different, then simply consider it less as the data association is inaccurate
    residuals_ptr[0] = transformed_mean_tar(0) - transformed_mean_src(0);
    residuals_ptr[1] = transformed_mean_tar(1) - transformed_mean_src(1);
    //residuals_ptr[0] = T(scale_similarity_)*(transformed_mean_src-transformed_mean_tar).norm();
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector2d& target_mean, const Eigen::Vector2d& src_mean) {
    return new ceres::AutoDiffCostFunction<P2PEfficientCost,2, 3>(new P2PEfficientCost (target_mean, src_mean));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:

  const Eigen::Vector2d tar_transformed_, src_mean_;
};

}
