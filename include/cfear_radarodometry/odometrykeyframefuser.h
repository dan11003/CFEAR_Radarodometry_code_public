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

using std::string;
using std::cout;
using std::cerr;
using std::endl;




namespace CFEAR_Radarodometry {


visualization_msgs::Marker GetDefault();

typedef std::vector<std::pair< Eigen::Affine3d, MapNormalPtr> > PoseScanVector;

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

    bool publish_tf_ = true;

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
      stringStream << "Weight, "<<weight_opt<<endl;
      return stringStream.str();
    }
  };


protected:
  Eigen::Affine3d Tcurrent, Tprev_fused, T_prev, Tmot;
  // Components for publishing
  boost::shared_ptr<n_scan_normal_reg> radar_reg = NULL;
  PoseScanVector keyframes_;

  unsigned int frame_nr_ = 0, nr_callbacks_ = 0;
  double distance_traveled = 0.0;
  const double Tsensor = 1.0/4.0;


  Parameters par;
  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_callback;
  ros::Publisher pose_current_publisher, pose_keyframe_publisher, pubsrc_cloud_latest, pub_cloud_keyframe;;
  tf::TransformBroadcaster Tbr;



public:

  OdometryKeyframeFuser(const Parameters& pars, bool disable_callback = false);

  //~OdometryKeyframeFuser();

  void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZI>::Ptr& msg_in, Eigen::Affine3d& Tcurr);

  std::string GetStatus(){return "Distance traveled: "+std::to_string(distance_traveled)+", nr sensor readings: "+std::to_string(frame_nr_);}

  void PrintSurface(const std::string& path, const Eigen::MatrixXd& surface);

private: 

  bool AccelerationVelocitySanityCheck(const Eigen::Affine3d& Tmot_prev, const Eigen::Affine3d& Tmot_curr);

  bool KeyFrameBasedFuse(const Eigen::Affine3d& diff, bool use_keyframe, double min_keyframe_dist, double min_keyframe_rot_deg);



  Eigen::Affine3d Interpolate(const Eigen::Affine3d &T2, double factor, const Eigen::Affine3d &T1 = Eigen::Affine3d::Identity());

  pcl::PointXYZI Transform(const Eigen::Affine3d& T, pcl::PointXYZI& p);

  nav_msgs::Odometry FormatOdomMsg(const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot, const ros::Time& t, Matrix6d &Cov);

  pcl::PointCloud<pcl::PointXYZI> FormatScanMsg(pcl::PointCloud<pcl::PointXYZI>& cloud_in, Eigen::Affine3d& T);


  void processFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in);




};

void AddToReference(PoseScanVector& reference, MapNormalPtr cloud,  const Eigen::Affine3d& T,  size_t submap_scan_size);

void FormatScans(const PoseScanVector& reference,
                   const MapNormalPtr& Pcurrent,
                   const Eigen::Affine3d& Tcurrent,
                   std::vector<Matrix6d>& cov_vek,
                   std::vector<MapNormalPtr>& scans_vek,
                   std::vector<Eigen::Affine3d>& T_vek
                   );

}
