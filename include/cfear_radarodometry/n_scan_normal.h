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
#include <algorithm>
#include "ceres/normal_prior.h"

namespace CFEAR_Radarodometry{

/* Registration type */
class n_scan_normal_reg : public Registration{

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  n_scan_normal_reg();

  n_scan_normal_reg(const cost_metric& cost, loss_type loss = Huber, double loss_limit = 0.1, const weightoption opt = weightoption::Uniform);

  bool Register(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc, std::vector<Matrix6d>& reg_cov, bool soft_constraints = false);

  bool RegisterTimeContinuous(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc, std::vector<Matrix6d>& reg_cov,  Eigen::Affine3d& Tprev, bool soft_constraints = false, bool ccw = false);

  bool GetCost(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc, double& score, std::vector<double>& residuals);

  void GetSurface(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc, std::vector<Matrix6d> &reg_cov, bool soft_constraints, Eigen::MatrixXd& surface, double res, int width);

  //bool Register(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc, std::vector<Eigen::Matrix3d>& reg_cov, bool soft_constraints = false);

  double getScore(){return score_;}

  void getScore(double& score, int& num_residuals){score = score_; num_residuals = problem_->NumResiduals();}

  void SetD2dPar(const double cov_scale,const double regularization){cov_scale_ = cov_scale; regularization_ = regularization;}

  void AddScanPairCost(MapNormalPtr& target_local, MapNormalPtr& src_local, const Eigen::Affine2d& Ttar, const Eigen::Affine2d& Tsrc, const size_t scan_idx_tar, const size_t scan_idx_src);




private:

  bool BuildOptimizationProblem(std::vector<MapNormalPtr>& scans, const Eigen::MatrixXd& cov = Eigen::Matrix<double,3,3>::Identity(), const Eigen::Vector3d &guess = Eigen::Vector3d::Identity(), bool soft_constraint = false);

  bool SolveOptimizationProblem();

  bool GetCovariance(Matrix6d& Cov);

  void EvaluateResiduals(std::vector<MapNormalPtr>& scans);

  double cov_scale_ = 1;
  double regularization_ = 0.01;
  const double score_tolerance = 0.00001;
  const double max_itr = 8, min_itr = 3;
  const bool efficient_implementation = true;

  bool time_continuous_ = false;
  std::vector<double> vel_parameters_;
  bool ccw_ = false;

  //std::vector<std::tuple<Eigen::Vector2d,Eigen::Vector2d,double,int> > vis_residuals;

};

class RegistrationCost{
protected:

  RegistrationCost () {}

  template <typename T>
  static Eigen::Matrix<T, 2, 2> GetRotMatrix2D(const T* par){
    const T cos_yaw = ceres::cos(par[2]);
    const T sin_yaw = ceres::sin(par[2]);
    Eigen::Matrix<T, 2, 2> rotation;
    rotation << cos_yaw,  -sin_yaw,
        sin_yaw,  cos_yaw;
    return rotation;
  }

  template <typename T>
  static Eigen::Matrix<T, 2, 1> GetTransMatrix2D(const T*  par) {
    Eigen::Matrix<T, 2, 1> trans {par[0], par[1]};
    return trans;
  }
  template <typename T>
  static Eigen::Matrix<T, 2, 2> GetScaledRotMatrix2D(const T* par, const T scale){
    const T scaled_angle = scale*par[2];
    const T cos_yaw = ceres::cos(scaled_angle);
    const T sin_yaw = ceres::sin(scaled_angle);
    Eigen::Matrix<T, 2, 2> rotation;
    rotation << cos_yaw,  -sin_yaw,
        sin_yaw,  cos_yaw;
    return rotation;
  }
  template <typename T>
  static Eigen::Matrix<T, 2, 2> GetRotMatrix2D(const T theta){
    const T cos_yaw = ceres::cos(theta);
    const T sin_yaw = ceres::sin(theta);
    Eigen::Matrix<T, 2, 2> rotation;
    rotation << cos_yaw,  -sin_yaw,
        sin_yaw,  cos_yaw;
    return rotation;
  }

  template <typename T>
  static Eigen::Matrix<T, 2, 1> GetScaledTransMatrix2D(const T*  par, const T scale) {
    Eigen::Matrix<T, 2, 1> trans(scale*par[0], scale*par[1]);
    return trans;
  }
};

class P2LCost : public RegistrationCost{
public:

  P2LCost (const Eigen::Vector2d& target_mean, const Eigen::Vector2d& target_normal, const Eigen::Vector2d& src_mean) :
    tar_mean_(target_mean),
    tar_normal_(target_normal),
    src_mean_(src_mean){}

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
      const Eigen::Vector2d& target_mean, const Eigen::Vector2d& target_normal, const Eigen::Vector2d& src_mean) {
    return new ceres::AutoDiffCostFunction<P2LCost ,1, 3, 3>(new P2LCost (target_mean, target_normal, src_mean));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:

  const Eigen::Vector2d tar_mean_;
  const Eigen::Vector2d tar_normal_;
  const Eigen::Vector2d src_mean_;
};


// reference precomputed
class P2LEfficientCost : public RegistrationCost{
public:

  P2LEfficientCost (const Eigen::Vector2d& transformed_mean_tar, const Eigen::Vector2d& transformed_normal_tar, const Eigen::Vector2d& src_mean) :
    transformed_mean_tar_(transformed_mean_tar),
    transformed_normal_tar_(transformed_normal_tar),
    src_mean_(src_mean)
    {}

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
      const Eigen::Vector2d& transformed_mean_tar, const Eigen::Vector2d& transformed_normal_tar, const Eigen::Vector2d& src_mean) {
    return new ceres::AutoDiffCostFunction<P2LEfficientCost ,1, 3>(new P2LEfficientCost (transformed_mean_tar, transformed_normal_tar, src_mean));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:

    const Eigen::Vector2d transformed_mean_tar_;
  const Eigen::Vector2d transformed_normal_tar_;
  const Eigen::Vector2d src_mean_;
};


class P2DEfficientCost : public RegistrationCost{
public:

  P2DEfficientCost (const Eigen::Vector2d& target_mean, const Eigen::Matrix2d& tar_sqrt_information, const Eigen::Vector2d& src_mean) :
    tar_mean_(target_mean),
    tar_sqrt_information_(tar_sqrt_information),
    src_mean_(src_mean) {}

  template <typename T>
  bool operator()(const T*  Ta,
                  T* residuals_ptr) const {
    //const Eigen::Matrix<T,2,1> trans_mat_tar = GetTransMatrix2D(Ta);
    //const Eigen::Matrix<T,2,2> rot_mat_tar = GetRotMatrix2D(Ta);

    const Eigen::Matrix<T,2,1> trans_mat_src = GetTransMatrix2D(Ta);
    const Eigen::Matrix<T,2,2> rot_mat_src = GetRotMatrix2D(Ta);

    //const Eigen::Matrix<T,2,1> transformed_mean_tar = (rot_mat_tar * (tar_mean_).cast<T>()) + trans_mat_tar;
    const Eigen::Matrix<T,2,1> transformed_mean_tar = tar_mean_.cast<T>();
    const Eigen::Matrix<T,2,1> transformed_mean_src = (rot_mat_src * (src_mean_).cast<T>()) + trans_mat_src;


    //residuals_ptr[0] = /*T(scale_similarity_)**/ v.dot(n);   // if scale similarity is different, then simply consider it less as the data association is inaccurate
    Eigen::Map<Eigen::Matrix<T, 2, 1> > residuals(residuals_ptr);
    residuals.template block<2, 1>(0, 0) = (transformed_mean_src - transformed_mean_tar);
    residuals.applyOnTheLeft(tar_sqrt_information_.template cast<T>());
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector2d& target_mean, const Eigen::Matrix2d& tar_sqrt_information, const Eigen::Vector2d& src_mean) {
    return new ceres::AutoDiffCostFunction<P2DEfficientCost ,2, 3>(new P2DEfficientCost (target_mean, tar_sqrt_information, src_mean));
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

class P2PEfficientContinuousCost : public RegistrationCost{
public:

  P2PEfficientContinuousCost (const Eigen::Vector2d& tar_transformed, const Eigen::Vector2d& src_mean, const double time_factor, const std::vector<double>& velocity) : tar_transformed_(tar_transformed), src_mean_(src_mean), time_factor_(time_factor), velocity_(velocity){}

  template <typename T>
  bool operator()(const T*  Tpose,
                  T* residuals_ptr) const {

    const T ts(time_factor_);
    const T vtheta( ts*(velocity_[2]) ); // rotation velocity
    //const Eigen::Matrix<T,2,1> trans_vel{ ts*(current_pose_[0] - prev_pose_[0]), ts*(Tpose[1] - current_pose_[1])}; // translation velocity

    const Eigen::Matrix<T,2,1> trans_vel{ ts*velocity_[0], ts*velocity_[1] }; // translation velocityconst Eigen::Matrix<T,2,2> rot_vel = GetRotMatrix2D(vtheta);

    //const T vtheta(0.0); // rotation velocity
    //const Eigen::Matrix<T,2,1> trans_vel( 0, 0); // translation velocity
    const Eigen::Matrix<T,2,2> rot_vel = GetRotMatrix2D(vtheta);
    const Eigen::Matrix<T,2,1> motion_corrected_src = (rot_vel * (src_mean_).cast<T>()) + trans_vel;

    const Eigen::Matrix<T,2,1> trans_mat_src = GetTransMatrix2D(Tpose);  // transform with pose
    const Eigen::Matrix<T,2,2> rot_mat_src = GetRotMatrix2D(Tpose);
    const Eigen::Matrix<T,2,1> transformed_mean_src = (rot_mat_src * motion_corrected_src) + trans_mat_src;

    const Eigen::Matrix<T,2,1> transformed_mean_tar = tar_transformed_.cast<T>();
    residuals_ptr[0] = transformed_mean_tar(0) - transformed_mean_src(0);
    residuals_ptr[1] = transformed_mean_tar(1) - transformed_mean_src(1);
    //residuals_ptr[0] = T(scale_similarity_)*(transformed_mean_src-transformed_mean_tar).norm();
    return true;
  }

  static ceres::CostFunction* Create(
      const Eigen::Vector2d& target_mean, const Eigen::Vector2d& src_mean, const double time_factor, const std::vector<double>& velocity) {
    return new ceres::AutoDiffCostFunction<P2PEfficientContinuousCost,2, 3>(new P2PEfficientContinuousCost (target_mean, src_mean, time_factor, velocity));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:

  const Eigen::Vector2d tar_transformed_, src_mean_;
  const double time_factor_;
  const std::vector<double> velocity_;
};



}
