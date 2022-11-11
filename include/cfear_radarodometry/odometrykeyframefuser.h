#pragma once
#include <ros/ros.h>


#include <Eigen/Eigen>
#include "eigen_conversions/eigen_msg.h"
#include <tf_conversions/tf_eigen.h>


#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"

#include <fstream>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_listener.h>

#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"

#include <time.h>
#include <cstdio>

//VK: random for playing with the perturbation
#include <random>
#include <algorithm>

#include <pcl_ros/transforms.h>
#include "pcl_ros/publisher.h"

//s#include "fuzzy_msgs/Registration.h"
#include <geometry_msgs/Transform.h>
#include "cfear_radarodometry/utils.h"

#include "cfear_radarodometry/pointnormal.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Header.h"
#include "std_msgs/Time.h"
#include "cfear_radarodometry/n_scan_normal.h"
#include "cfear_radarodometry/statistics.h"
#include "std_msgs/ColorRGBA.h"
#include "boost/shared_ptr.hpp"
#include "cfear_radarodometry/types.h"
#include "cfear_radarodometry/eval_trajectory.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using std::string;
using std::cout;
using std::cerr;
using std::endl;





namespace CFEAR_Radarodometry {


visualization_msgs::Marker GetDefault();

typedef std::vector<RadarScan> PoseScanVector;

class OdometryKeyframeFuser {

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
  class Parameters {

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  public:


    Parameters() {}
    std::string scan_registered_latest_topic = "radar_registered";
    std::string scan_registered_keyframe_topic = "radar_registered_keyframe";
    std::string odom_latest_topic = "radar_odom";
    std::string odom_keyframe_topic = "radar_odom_keyframe";
    std::string odometry_link_id = "world";
    std::string input_points_topic = "/Navtech/Filtered";
    std::string cost_type = "P2L";
    weightoption weight_opt = weightoption::Uniform;


    bool visualize = true;
    int submap_scan_size = 3;
    bool weight_intensity_ = false;

    bool use_guess = true, disable_registration = false, soft_constraint = false;
    bool compensate = true, radar_ccw = false;
    bool use_keyframe = true, enable_filter = false, use_raw_pointcloud = false;
    double res = 3.5;
    double min_keyframe_dist_ = 1.5, min_keyframe_rot_deg_ = 5;
    std::string loss_type_ = "Huber";
    double loss_limit_ = 0.1;
    double covar_scale_ = 1.0;
    double regularization_ = 0.0;

    bool estimate_cov_by_sampling = true;
    bool cov_samples_to_file_as_well = false; // Will save in the desired folder
    std::string cov_sampling_file_directory = "/tmp/cfear_out";
    double cov_sampling_xy_range = 0.4;  // Will sample from -0.2 to +0.2
    double cov_sampling_yaw_range = 0.0043625; //Will sample from -half to +half of this range as well
    unsigned int cov_sampling_samples_per_axis = 5; //Will do 5^3 in the end
    double cov_sampling_covariance_scaler = 1.0;


    bool publish_tf_ = true;
    bool store_graph = false;



    void GetParametersFromRos( ros::NodeHandle& param_nh){
      param_nh.param<std::string>("input_points_topic", input_points_topic, "/Navtech/Filtered");
      param_nh.param<std::string>("scan_registered_latest_topic", scan_registered_latest_topic, "/radar_registered");
      param_nh.param<std::string>("scan_registered_keyframe_topic", scan_registered_keyframe_topic, "/radar_registered_keyframe");
      param_nh.param<std::string>("odom_latest_topic", odom_latest_topic, "/radar_odom");
      param_nh.param<std::string>("odom_keyframe_topic", odom_keyframe_topic, "/radar_odom_keyframe");


      param_nh.param<std::string>("odometry_link_id", odometry_link_id, "world");
      param_nh.param("visualize", visualize, true);

      param_nh.param<bool>("use_raw_pointcloud", use_raw_pointcloud, false);
      param_nh.param<int>("submap_scan_size", submap_scan_size, 3);

      param_nh.param<double>("res", res, 3.0);
      double d_factor;
      param_nh.param<double>("downsample_factor", d_factor, 1);
      MapPointNormal::downsample_factor = d_factor;

      param_nh.param<double>("registered_min_keyframe_dist", min_keyframe_dist_, 1.5);
      param_nh.param<double>("min_keyframe_rot_deg_", min_keyframe_rot_deg_, 5);
      param_nh.param<bool>("use_keyframe", use_keyframe, true);
      param_nh.param<bool>("use_guess", use_guess, true);
      param_nh.param<bool>("radar_ccw", radar_ccw, false);
      param_nh.param<bool>("disable_registration", disable_registration, false);
      param_nh.param<bool>("soft_constraint", soft_constraint, false);
      param_nh.param<bool>("compensate", compensate, true);
      param_nh.param<bool>("store_graph", store_graph, false);
      param_nh.param<std::string>("cost_type", cost_type, "P2L");


      param_nh.param<std::string>("loss_type", loss_type_, "Huber");
      param_nh.param<double>("loss_limit", loss_limit_, 0.1);
      param_nh.param<double>("covar_scale", covar_scale_, 1);
      param_nh.param<double>("regularization", regularization_, 0);
      param_nh.param<bool>("weight_intensity", weight_intensity_, false);
      param_nh.param<bool>("publish_tf", publish_tf_, false);
    }

    std::string ToString(){
      std::ostringstream stringStream;
      //stringStream << "OdometryKeyframeFuser::Parameters"<<endl;
      stringStream << "input_points_topic, "<<input_points_topic<<endl;
      stringStream << "scan_registered_latest_topic, "<<scan_registered_latest_topic<<endl;
      stringStream << "scan_registered_keyframe_topic, "<<scan_registered_keyframe_topic<<endl;
      stringStream << "odom_latest_topic, "<<odom_latest_topic<<endl;
      stringStream << "odom_keyframe_topic, "<<odom_keyframe_topic<<endl;
      stringStream << "use raw pointcloud, "<<std::boolalpha<<use_raw_pointcloud<<endl;
      stringStream << "submap keyframes, "<<submap_scan_size<<endl;
      stringStream << "resolution r,"<<res<<endl;
      stringStream << "resample factor f, "<<MapPointNormal::downsample_factor<<endl;
      stringStream << "min. sensor distance [m], "<<min_keyframe_dist_<<endl;
      stringStream << "min. sensor rot. [deg], "<<min_keyframe_rot_deg_<<endl;
      stringStream << "use keyframe, "<<std::boolalpha<<use_keyframe<<endl;
      stringStream << "use initial guess, "<<std::boolalpha<<use_guess<<endl;
      stringStream << "radar reversed, "<<std::boolalpha<<radar_ccw<<endl;
      stringStream << "disable registration, "<<std::boolalpha<<disable_registration<<endl;
      stringStream << "soft velocity constraint, "<<std::boolalpha<<soft_constraint<<endl;
      stringStream << "compensate, "<<std::boolalpha<<compensate<<endl;
      stringStream << "cost type, "<<cost_type<<endl;
      stringStream << "loss type, "<<loss_type_<<endl;
      stringStream << "loss limit, "<<std::to_string(loss_limit_)<<endl;
      stringStream << "covar scale, "<<std::to_string(covar_scale_)<<endl;
      stringStream << "regularization, "<<std::to_string(regularization_)<<endl;
      stringStream << "weight intensity, "<<std::boolalpha<<weight_intensity_<<endl;
      stringStream << "publish_tf, "<<std::boolalpha<<publish_tf_<<endl;
      stringStream << "store graph, "<<store_graph<<endl;
      stringStream << "Weight, "<<weight_opt<<endl;
      return stringStream.str();
    }
  };

  RadarScan scan_;
  bool updated = false;


protected:
  Eigen::Affine3d Tprev_fused, T_prev, Tmot;
  Eigen::Affine3d Tcurrent;
  Covariance cov_current;

  // Components mat publishing
  boost::shared_ptr<n_scan_normal_reg> radar_reg = NULL;
  PoseScanVector keyframes_;
  simple_graph graph_;

  unsigned int frame_nr_ = 0, nr_callbacks_ = 0;
  double distance_traveled = 0.0;
  const double Tsensor = 1.0/4.0;


  Parameters par;
  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_callback;
  ros::Publisher pose_current_publisher, pose_keyframe_publisher, pubsrc_cloud_latest, pub_cloud_keyframe;;
  tf::TransformBroadcaster Tbr;

  //VK: Tools for perturbing the odometry
  std::default_random_engine generator;



public:

  OdometryKeyframeFuser(const Parameters& pars, bool disable_callback = false);

  //~OdometryKeyframeFuser();

  void pointcloudCallback(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered,  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered_peaks,  Eigen::Affine3d &Tcurr, const ros::Time& t);

  void pointcloudCallback(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered,  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered_peaks,  Eigen::Affine3d &Tcurr, const ros::Time& t, Covariance &cov_curr);

  std::string GetStatus(){return "Distance traveled: "+std::to_string(distance_traveled)+", nr sensor readings: "+std::to_string(frame_nr_);}

  void PrintSurface(const std::string& path, const Eigen::MatrixXd& surface);

  void SaveGraph(const std::string& path);

  void AddGroundTruth(poseStampedVector& gt_vek);

private: 

  bool AccelerationVelocitySanityCheck(const Eigen::Affine3d& Tmot_prev, const Eigen::Affine3d& Tmot_curr);

  bool KeyFrameBasedFuse(const Eigen::Affine3d& diff, bool use_keyframe, double min_keyframe_dist, double min_keyframe_rot_deg);

  void AddToGraph(PoseScanVector& reference, RadarScan& scan,  const Eigen::Matrix<double,6,6>& Cov);

  Eigen::Affine3d Interpolate(const Eigen::Affine3d &T2, double factor, const Eigen::Affine3d &T1 = Eigen::Affine3d::Identity());

  pcl::PointXYZI Transform(const Eigen::Affine3d& T, pcl::PointXYZI& p);

  nav_msgs::Odometry FormatOdomMsg(const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot, const ros::Time& t, Matrix6d &Cov);

  pcl::PointCloud<pcl::PointXYZI> FormatScanMsg(pcl::PointCloud<pcl::PointXYZI>& cloud_in, Eigen::Affine3d& T);

  void processFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_peaks, const ros::Time& t);

  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_filtered);





};

void AddToReference(PoseScanVector& reference, RadarScan& scan, size_t submap_scan_size);

void FormatScans(const PoseScanVector& reference,
                 const MapNormalPtr& Pcurrent,
                 const Eigen::Affine3d& Tcurrent,
                 std::vector<Matrix6d>& cov_vek,
                 std::vector<MapNormalPtr>& scans_vek,
                 std::vector<Eigen::Affine3d>& T_vek
                 );

template<typename T> std::vector<double> linspace(T start_in, T end_in, int num_in);

}


