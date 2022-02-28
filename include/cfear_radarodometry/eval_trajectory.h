#pragma once


#include <iostream>
#include <string>

#include <ros/ros.h>


#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "boost/filesystem.hpp"
#include <sensor_msgs/Image.h>
#include "ros/time.h"
#include "vector"
#include "eigen3/Eigen/Eigen"
#include "eigen3/Eigen/StdVector"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "eigen3/Eigen/SVD"
#include "tf/transform_broadcaster.h"
#include "eigen_conversions/eigen_msg.h"
#include "tf_conversions/tf_eigen.h"
#include "numeric"
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/common/transforms.h"
#include "nav_msgs/Path.h"
#include "ros/publisher.h"
#include "geometry_msgs/PoseStamped.h"
#include "pcl/2d/convolution.h"
#include "pcl/filters/random_sample.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/filters/radius_outlier_removal.h"

namespace CFEAR_Radarodometry {


using std::string;
using std::cout;
using std::cerr;
using std::endl;

using namespace message_filters;
using namespace sensor_msgs;


//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::matrix<double,4,4>)

//typedef std::pair<Eigen::Affine3d, ros::Time> poseStamped;
typedef std::pair<Eigen::Affine3d, ros::Time> poseStamped;
typedef std::vector<poseStamped, Eigen::aligned_allocator<poseStamped>> poseStampedVector;
typedef sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> double_odom;




class EvalTrajectory
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  class Parameters
  {
  public:
    Parameters() {}
    std::string est_output_dir = "", gt_output_dir = "";
    std::string sequence = "", method = "";
    std::string odom_est_topic = "", odom_gt_topic = "";
    int job_nr = -1;
    bool save_pcd = false;

    void GetParametersFromRos( ros::NodeHandle& param_nh){
      param_nh.param<std::string>("est_topic", odom_est_topic, "/lidar_odom");
      param_nh.param<std::string>("gt_topic",  odom_gt_topic, "/gt");
      param_nh.param<std::string>("est_output_dir", est_output_dir, "");
      param_nh.param<std::string>("gt_otuput_dir",  gt_output_dir, "");
      param_nh.param<std::string>("bag_name",  sequence, "");
      param_nh.param<bool>("save_pcd",  save_pcd, false);
    }

    std::string ToString(){
      std::ostringstream stringStream;
      //stringStream << "EvaluateTrajectory::Parameters:"<<endl;
      stringStream << "odom_est_topic, "<<odom_est_topic<<endl;
      stringStream << "gt_topic, "<<odom_gt_topic<<endl;
      stringStream << "est_output_dir, "<<est_output_dir<<endl;
      stringStream << "gt_output_dir, "<<gt_output_dir<<endl;
      stringStream << "sequence, "<<sequence<<endl;
      stringStream << "job nr, "<<job_nr<<endl;
      stringStream << "save pcd, "<<save_pcd<<endl;
      stringStream << "method, "<<method<<endl;
      return stringStream.str();
    }
  };




  EvalTrajectory(const EvalTrajectory::Parameters& pars, bool disable_callback = false);

  //~EvalTrajectory(){cout<<"destruct eval"<<endl;}

  void CallbackSynchronized(const nav_msgs::Odometry::ConstPtr& msg_est, const nav_msgs::Odometry::ConstPtr& msg_gt);

  void CallbackEigen(const poseStamped& Test, const poseStamped& Tgt);

  std::string DatasetToSequence(const std::string& dataset);

  void CallbackGT(const nav_msgs::Odometry::ConstPtr &msg);

  void CallbackEst(const nav_msgs::Odometry::ConstPtr &msg);

  void Save();

  void CallbackGTEigen(const poseStamped& Tgt);

  void CallbackESTEigen(const poseStamped& Test);

  void CallbackESTEigen(const poseStamped& Test, const pcl::PointCloud<pcl::PointXYZI>& cld);
  
  size_t GetSize(){return std::min(gt_vek.size(),est_vek.size());}

  void AlignTrajectories();


private:

  void SavePCD(const std::string& folder);

  void One2OneCorrespondance();

  bool SearchByTime(const ros::Time& t, poseStampedVector::iterator& itr);

  bool Interpolate(const ros::Time& t, poseStamped& Tinterpolated, poseStampedVector::iterator& itr);

  Eigen::Affine3d pose_interp(double t, double t1, double t2,Eigen::Affine3d const& aff1, Eigen::Affine3d const& aff2);

  void PublishTrajectory(poseStampedVector& vek, ros::Publisher& pub);

  Eigen::Matrix4d best_fit_transform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);

  void RemoveExtras();

  void Write(const std::string& path, const poseStampedVector& v);

  void PrintStatus();

  poseStampedVector::iterator FindElement(const ros::Time& t);

  Parameters par;

  ros::NodeHandle nh_;
  ros::Subscriber sub_est, sub_gt;
  ros::Publisher pub_est, pub_gt, pub_cloud;
  tf::TransformBroadcaster br;
  Subscriber<nav_msgs::Odometry> *pose_sub_gt = NULL, *pose_sub_est = NULL;
  Synchronizer<double_odom> *sync = NULL;

  poseStampedVector est_vek, gt_vek;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> clouds;
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled;



};


}
