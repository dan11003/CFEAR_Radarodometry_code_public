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
#include "list"
//#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/common/centroid.h"
#include "pcl/point_types.h"

#include "visualization_msgs/MarkerArray.h"
#include "tuple"
#include "pcl/common/transforms.h"
#include <pcl/search/kdtree.h>
#include "boost/shared_ptr.hpp"
#include "tgmath.h"
#include "cfear_radarodometry/statistics.h"

namespace CFEAR_Radarodometry {
using std::cout;
using std::endl;
using std::cerr;
class cell;
typedef boost::shared_ptr<cell> cellptr;



class cell
{
public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  cell(const Eigen::Vector3d& u,   const Eigen::Matrix3d& cov, const Eigen::Vector3d& origin = Eigen::Vector3d(0,0));

  cell(const Eigen::Vector3d& u, const Eigen::Matrix3d& cov, const Eigen::Vector3d& normal, const double scale);

  cell TransformCopy(const Eigen::Affine3d& T);

  double GetAngle();

  static cell GetDefault();

  std::pair<Eigen::Vector3d,Eigen::Vector3d> TransformMeanNormal(const Eigen::Affine3d& T);

  Eigen::Vector3d u_;
  Eigen::Matrix3d cov_;
  double scale_;
  Eigen::Vector3d snormal_;

  //static void ToNDTMsg(std::vector<cellptr>& cells, ndt_map::NDTMapMsg& msg);

};

class MapPointNormal;
typedef boost::shared_ptr<MapPointNormal> MapNormalPtr;

class MapPointNormal
{


public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MapPointNormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cld, float radius, const Eigen::Vector3d& origin = Eigen::Vector3d(0,0,0), bool raw = false);

  MapPointNormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cld, float radius, std::vector<cell>& cell_orig, const Eigen::Affine3d& T);


  MapPointNormal(){
    input_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    downsampled_ = pcl::PointCloud<pcl::PointXY>::Ptr(new pcl::PointCloud<pcl::PointXY>());

  }



  std::vector<cell> GetCells(){return cells;}

  //std::vector<cell*> GetClosest(cell* c, double d);

  std::vector<cell*> GetClosest(Eigen::Vector3d& p, double d);

  std::vector<int> GetClosestIdx(const Eigen::Vector3d&  p, double d);

  std::vector<int> GetClosestIdx(const Eigen::Vector2d&  p, double d);

  ros::Time GetTime(){ros::Time t; pcl_conversions::fromPCL(input_->header.stamp, t); return t;}

  //pcl::PointCloud<pcl::PointXYZINormal>::Ptr GetCloud();

  std::vector<cell> TransformCells(const Eigen::Affine3d& T);

  void Transform(const Eigen::Affine3d& T, Eigen::MatrixXd& means, Eigen::MatrixXd& normals);

  void Transform2d(const Eigen::Affine3d& T, Eigen::MatrixXd& means, Eigen::MatrixXd& normals);

  Eigen::MatrixXd GetNormals();

  Eigen::MatrixXd GetNormals2d();

  std::vector<Eigen::Matrix3d> GetCovs();

  Eigen::MatrixXd GetMeans();

  Eigen::Vector2d GetMean2d(const size_t  i){ return cells[i].u_.block<2,1>(0,0);}

  Eigen::Matrix2d GetCov2d(const size_t i){ return cells[i].cov_.block<2,2>(0,0);}

  Eigen::Vector2d GetNormal2d(const size_t i){ return cells[i].snormal_.block<2,1>(0,0);}

  Eigen::MatrixXd GetMeans2d();

  Eigen::MatrixXd GetCloudTransformed(const Eigen::Affine3d& Toffset);

  Eigen::MatrixXd GetScales();

  //void Set(pcl::PointCloud<pcl::PointXYZI>& input, std::vector<cellptr>& cells );

  MapNormalPtr TransformMap(const Eigen::Affine3d& T);

  pcl::PointCloud<pcl::PointXYZI>::Ptr GetScan(){return input_;}

  size_t GetSize(){return cells.size();}



private:



  MapPointNormal(pcl::PointCloud<pcl::PointXYZI>& cld);

  void ComputeSearchTreeFromCells();

  void ComputeNormals(const Eigen::Vector3d &origin);

  bool ComputeMeanAndNormals(const std::vector<int>& pointIdxNKNSearch,  Eigen::Vector3d& u, Eigen::Matrix3d& Cov);

  inline double Gausian(const double x, const double u, const double sigma);

  bool WeightedCovariance(Eigen::MatrixXd& w, Eigen::MatrixXd& x, Eigen::Matrix3d& cov);


  // MEMBER VARIABLES
  std::vector<cell> cells;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_ = NULL;
  pcl::PointCloud<pcl::PointXY>::Ptr downsampled_ = NULL;
  pcl::KdTreeFLANN<pcl::PointXY> kd_cells;
  float radius_;

public:


  static void PublishMap(const std::string& topic, MapNormalPtr map, Eigen::Affine3d& T, const std::string& frame_id, const int value=0);

  static std::map<std::string, ros::Publisher> pubs;

  static double downsample_factor;

};



visualization_msgs::Marker DefaultMarker( const ros::Time& time, const std::string& frame);

visualization_msgs::MarkerArray Cells2Markers(std::vector<cell>& cells, const ros::Time& time, const std::string& frame, const int val=0 );

void intToRGB(int value,float& red,float& green, float& blue);

inline pcl::PointXYZI Pnt(Eigen::Vector3d& u);

inline pcl::PointXYZ PntXYZ(Eigen::Vector3d& u);

inline geometry_msgs::Point Pntgeom(Eigen::Vector3d& u);




}


