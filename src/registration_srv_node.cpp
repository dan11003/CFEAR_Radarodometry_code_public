
//#include "graph_map/graph_map_fuser.h"
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include <Eigen/Eigen>
#include "eigen_conversions/eigen_msg.h"
#include <tf_conversions/tf_eigen.h>

#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"

#include <fstream>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include <time.h>
#include <fstream>
#include <cstdio>

#include <pcl_ros/transforms.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/publisher.h"
#include "robust_mapping_custom_msgs/registration.h"
#include "cfear_radarodometry/pointnormal.h"
#include "cfear_radarodometry/normal_registration.h"
#include "cfear_radarodometry/n_scan_normal.h"
#include "cfear_radarodometry/intensity_utils.h"
#include "cfear_radarodometry/n_scan_normal.h"
#include "robust_mapping_custom_msgs/n_registration.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
//#include "robust_mapping/serialization.h"

/** \brief A ROS node which implements p2p/p2l or p2d scan registration
 * This is not tested
 * \author Daniel adolfsson
 */

using std::string;
using std::cout;
using std::cerr;
using std::endl;
using CFEAR_Radarodometry::MapPointNormal;
using CFEAR_Radarodometry::Matrix6d;

const Matrix6d cov_Identity = 100000*Matrix6d::Identity(6,6);
class RegistrationService{
public:
  RegistrationService():nh_("~"){

    nh_.param<std::string>("registration_service_topic", srv_topic_, "registration");
    nh_.param<std::string>("n_scan_registration_service_topic", srv_n_reg_topic_, "/n_registration");
    nh_.param<std::string>("world_frame", world_id_, "world");
    nh_.param<double>("resolution", resolution_, 4.0);
    nh_.param<double>("downsample_factor_map_normal", MapPointNormal::downsample_factor, 2.0);

    nh_.param<bool>("disable_registration", disable_registration_, false);
    nh_.param<bool>("visualize", visualize, false);
    nh_.param<double>("use_consistency", consistency_max_distance, 6.0); // meters
    nh_.param<double>("use_consistency", consistency_max_angle, 20.0); // degrees
    std::string reg_type;
    nh_.param<std::string>("cost_type", reg_type, "P2L");

    CFEAR_Radarodometry::cost_metric cost = CFEAR_Radarodometry::Str2Cost(reg_type);


    reg = new CFEAR_Radarodometry::n_scan_normal_reg(cost);

    pub_src_init = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("src_init", 10);
    pub_target = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("target", 10);
    pub_src_registered = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("src_registered", 10);

    pub_init_intensity = nh_.advertise<pcl::PointCloud<pcl::PointXYZI> >("init_intensity", 10);
    pub_registered_intensity = nh_.advertise<pcl::PointCloud<pcl::PointXYZI> >("registered_intensity", 10);

    cout<<"register service on topic: "<<srv_topic_<<endl;
    service = nh_.advertiseService(srv_topic_, &RegistrationService::ServiceCallback, this);
    n_reg_service = nh_.advertiseService(srv_n_reg_topic_, &RegistrationService::nScanServiceCallbackLocal, this);

  }
  void Publish(ros::Publisher &pub, pcl::PointCloud<pcl::PointXYZ> &cloud, std::string frame_id, const ros::Time &t){
    cloud.header.frame_id = frame_id;
    pcl_conversions::toPCL(t, cloud.header.stamp);
    pub.publish(cloud);
  }
  void PublishALL(const Eigen::Affine3d& Tsrc_init, const Eigen::Affine3d& Ttar, const Eigen::Affine3d& Tsrc_registered , pcl::PointCloud<pcl::PointXYZ>& src, pcl::PointCloud<pcl::PointXYZ>&  target){
    tf::Transform tfsrc_init, tftar, tf_src_registered;
    tf::poseEigenToTF(Tsrc_init, tfsrc_init);
    tf::poseEigenToTF(Ttar, tftar);
    tf::poseEigenToTF(Tsrc_registered, tf_src_registered);
    ros::Time t = ros::Time::now();
    br.sendTransform( tf::StampedTransform(tfsrc_init, t, world_id_, "/reg_src"));
    br.sendTransform( tf::StampedTransform(tftar, t, world_id_, "/reg_tar"));
    br.sendTransform( tf::StampedTransform(tf_src_registered, t, world_id_, "/reg_src_registered"));
    Publish(pub_src_init, src, "/reg_src", t);
    Publish(pub_target, target, "/reg_tar", t);
    Publish(pub_src_registered, src, "/reg_src_registered", t);
  }

  void Convert(pcl::PointCloud<pcl::PointXYZ>& cld_in, pcl::PointCloud<pcl::PointXYZI>& cld_out){
    for(auto && p : cld_in.points){
      pcl::PointXYZI p_i;
      p_i.x = p.x;
      p_i.y = p.y;
      p_i.z = p.z;
      p_i.intensity = 100;
      cld_out.push_back(p_i);
    }
    cld_out.header = cld_in.header;
  }
  bool IsConsistent(const Eigen::Affine3d& Tdiscrepancy){
    const double distance = Tdiscrepancy.translation().norm();
    Eigen::Vector3d euler = Tdiscrepancy.rotation().eulerAngles(0,1,2);
    CFEAR_Radarodometry::normalizeEulerAngles(euler);
    const double angle = 180.0/M_PI*euler.norm();
    cout<<" distance :"<<distance <<", consistency_max_distance: "<<consistency_max_distance<<", angle: "<<angle<<", consistency_max_angle: "<<consistency_max_angle<<endl;
    if(distance >consistency_max_distance || angle > consistency_max_angle){
      cout<<"INCONSISTENT: distance :"<<distance <<", consistency_max_distance: "<<consistency_max_distance<<", angle: "<<angle<<", consistency_max_angle: "<<consistency_max_angle<<endl;
      return false;
    }
    else return true;
  }
  bool ServiceCallback(robust_mapping_custom_msgs::registration::Request& request, robust_mapping_custom_msgs::registration::Response& response)
  {
    cout<<"service callback"<<endl;
    ros::Time t = ros::Time::now();
    response.quality = 1;
    pcl::PointCloud<pcl::PointXYZ> src_local, target_local;
    std::cerr<<"Cld src: "<<request.cloudsrc.width*request.cloudsrc.height<<", cld tar: "<<request.cloudtarget.width*request.cloudtarget.height<<endl;

    Eigen::Affine3d Tsrc, Ttar;
    tf::poseMsgToEigen(request.Tsrc, Tsrc);
    tf::poseMsgToEigen(request.Ttarget, Ttar);
    if(!request.cloud_local_Frame){
      pcl::PointCloud<pcl::PointXYZ> src, target;
      pcl::fromROSMsg(request.cloudsrc, src);
      pcl::fromROSMsg(request.cloudtarget, target);
      pcl::transformPointCloud(src, src_local, Tsrc.inverse());
      pcl::transformPointCloud(target, target_local, Ttar.inverse());
    }
    else{
      pcl::fromROSMsg(request.cloudsrc, src_local);
      pcl::fromROSMsg(request.cloudtarget, target_local);
    }

    if(src_local.empty() || target_local.empty()){
      std::cerr<<"Error empty clouds"<<endl;
      return false;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr src_local_intensity(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr tar_local_intensity(new pcl::PointCloud<pcl::PointXYZI>());
    Convert(src_local, *src_local_intensity);
    Convert(target_local, *tar_local_intensity);
    std::cerr<<"reg_serv_node: Cld src: "<<src_local_intensity->size()<<", cld tar: "<<tar_local_intensity->size()<<endl;


    ros::Time t2 = ros::Time::now();

    CFEAR_Radarodometry::MapNormalPtr Nsrc = CFEAR_Radarodometry::MapNormalPtr ( new CFEAR_Radarodometry::MapPointNormal(src_local_intensity,    resolution_));
    CFEAR_Radarodometry::MapNormalPtr Ntar = CFEAR_Radarodometry::MapNormalPtr ( new CFEAR_Radarodometry::MapPointNormal(tar_local_intensity, resolution_));
    std::vector<CFEAR_Radarodometry::MapNormalPtr> scans{Ntar, Nsrc};
    const Eigen::Affine3d Tinit = Ttar.inverse()*Tsrc;
    std::vector<Eigen::Affine3d> Tscans{Eigen::Affine3d::Identity(), Tinit};
    std::vector<Matrix6d> reg_cov{cov_Identity, cov_Identity};
    std::vector<bool> fixedBlock{true, false};


    ros::Time t3 = ros::Time::now();
    bool success =  disable_registration_ ? true : reg->Register(scans, Tscans, reg_cov);
    double score = disable_registration_  ? 1.0  : reg->getScore();

    if(!success){
      cout<<"reg_serv_node: registration failure"<<endl;
      return false;
    }

    Eigen::Affine3d Tregistered = Tscans[0].inverse()*Tscans[1];
    Eigen::Affine3d Tdiscrepancy = Tinit.inverse()*Tregistered;
    if(IsConsistent(Tdiscrepancy)){
      cout<<"reg_serv_node: Tregistered: "<<Tregistered.translation().transpose()<<", score: "<<score<<", Tconvert="<<t2-t<<", Treg="<<ros::Time::now()-t3<<endl;
      tf::poseEigenToMsg(Tregistered, response.pose.pose);

      //Eigen::Matrix<double,1,36> cov_vec = 100*Eigen::Map<Eigen::Matrix<double,1,36> >(reg_cov.back().data());
      Eigen::Matrix<double,6,1> ones;
      ones <<1, 1, 1, 1, 1, 1;
      Eigen::Matrix<double,6,6> cov = ones.asDiagonal();


      Eigen::Matrix<double,1,36> cov_vec = 100*Eigen::Map<Eigen::Matrix<double,1,36> >(cov.data());
      for(int i=0;i<36;i++)
        response.pose.covariance[i] = cov_vec(0,i);

      PublishALL(Tsrc, Ttar, Ttar*Tregistered, src_local, target_local); // to be done, change fran

      return true;
    }
    else return false;
  }

  void VisualizeNscan(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >& scans, const std::vector<Eigen::Affine3d>&T, const ros::Publisher& pub){
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged(new pcl::PointCloud<pcl::PointXYZI>());
    for(int i=0;i<scans.size();i++){
      pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::transformPointCloud(*scans[i], *tmp, T[i]);
      *merged+=*tmp;
    }
    ros::Time t = ros::Time::now();
    pcl_conversions::toPCL(t, merged->header.stamp);
    merged->header.frame_id = world_id_;
    pub.publish(*merged);


    for(int i=0;i<T.size();i++){
      tf::Transform T_tf;
      tf::poseEigenToTF(T[i], T_tf);
      ros::Time t = ros::Time::now();
      br.sendTransform( tf::StampedTransform(T_tf, t, world_id_, "T_"+std::to_string(i)));
    }
  }
  bool nScanServiceCallbackLocal(robust_mapping_custom_msgs::n_registration::Request& request, robust_mapping_custom_msgs::n_registration::Response& response){

    if( request.scans.size()!=request.Tinit.size() || request.Tinit.size()==0)
      return false;
    int n_scans = request.scans.size();

    Eigen::Matrix<double,6,1> unit_diag;
    unit_diag<<0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
    std::vector<Matrix6d> covs(n_scans, unit_diag.asDiagonal());
    std::vector<Eigen::Affine3d> Tpose(n_scans,Eigen::Affine3d::Identity());
    std::vector<pcl::PointCloud<pcl::PointXYZ> > scans(n_scans);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > scans_intensity;
    for(int i=0;i<n_scans;i++)
      scans_intensity.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()));

    static int j = 0;
    for(int i=0 ; i<n_scans ; i++){
      pcl::fromROSMsg(request.scans[i], scans[i]);
      Convert(scans[i], *scans_intensity[i]);
    }
    for(int i=0 ; i<n_scans ; i++){
      tf::poseMsgToEigen(request.Tinit[i].pose, Tpose[i]);
    }

    ros::Time t1 = ros::Time::now();
    std::vector<CFEAR_Radarodometry::MapNormalPtr> normal_scans_local(n_scans, NULL);
    if(visualize)
      VisualizeNscan(scans_intensity, Tpose, pub_init_intensity);
#pragma omp parallel for
    for(int i=0 ; i<n_scans ; i++)
      normal_scans_local[i] = CFEAR_Radarodometry::MapNormalPtr(new MapPointNormal(scans_intensity[i], resolution_));

    ros::Time t2 = ros::Time::now();
    CFEAR_Radarodometry::n_scan_normal_reg n_reg;
    bool success = true;

    if(disable_registration_)
      cerr<<"Registration disabled"<<endl;
    else
      success = n_reg.Register(normal_scans_local, Tpose, covs);


    ros::Time t3 = ros::Time::now();
    //cout<<"time req: create: "<<t2-t1<<", complete registration: "<<t3-t2<<endl;

    if(visualize){
      /*for(int i=0;i<n_scans;i++){
        MapPointNormal::PublishMap("/proj_after_"+std::to_string(i), normal_scans_local[i], Tpose[i], "world", 255/10.0*(i+1));
      }*/

      VisualizeNscan(scans_intensity, Tpose, pub_registered_intensity);
    }


    if(success){

      //response.Treg.resize(n_scans);
      for(int i=0; i <n_scans ; i++){
        geometry_msgs::PoseWithCovariance Tc;
        tf::poseEigenToMsg(Tpose[i], Tc.pose); // TEST SHOULD NOT BE TINIT HERE
        Matrix6d c = covs[i];
        c.resize(1,36);
        for(int j=0;j<36;j++)
          Tc.covariance[j] = c.data()[j];
        response.Treg.push_back( Tc);
      }
      return true;
    }
    else
      return false;

  }

  ros::NodeHandle nh_;
  ros::ServiceServer service, n_reg_service;
  bool visualize;
  ros::Publisher pub_src_init, pub_target, pub_src_registered, pub_registered_intensity, pub_init_intensity;
  CFEAR_Radarodometry::Registration *reg;
  CFEAR_Radarodometry::Registration *n_reg;
  tf::TransformBroadcaster br;
  double resolution_;

  std::string reg_type_, map_type_;
  std::string srv_topic_, srv_n_reg_topic_;
  std::string world_id_;
  bool disable_registration_;
  double consistency_max_distance, consistency_max_angle;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reg_service_node_2d");
  RegistrationService srv;
  ros::spin();
  return 0;
}



