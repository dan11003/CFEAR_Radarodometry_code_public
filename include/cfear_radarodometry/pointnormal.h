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
#include "cfear_radarodometry/utils.h"
//#include "ndt_map/NDTMapMsg.h"
//#include "ndt_map/ndt_map.h"
//#include "ndt_map/ndt_conversions.h"


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

  //cell(const Eigen::Vector2d& u,   const Eigen::Matrix2d& cov, const double intensity, const int N, const Eigen::Vector2d& origin = Eigen::Vector2d(0,0));

  //cell(const Eigen::Vector2d& u, const Eigen::Matrix2d& cov, const double intensity, const int N, const Eigen::Vector2d& normal, const double scale);
  cell(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, const std::vector<int>& pointIdxNKNSearch, const bool weight_intensity, const Eigen::Vector2d& origin = Eigen::Vector2d(0,0));

  cell TransformCopy(const Eigen::Affine2d& T);

  double GetAngle();

  double GetPlanarity(){return 1.0 - lambda_min/(lambda_min + lambda_max);}

  static cell GetIdentityCell(const Eigen::Vector2d& u, const double intensity) { return cell(u,intensity); } // Use only for raw data

  //static void ToNDTMsg(std::vector<cell>& cells, ndt_map::NDTMapMsg& msg);

  Eigen::Vector2d u_ = Eigen::Vector2d(0,0);
  Eigen::Matrix2d cov_ = Eigen::Matrix2d::Identity()*0.1;
  double scale_;
  Eigen::Vector2d snormal_, orth_normal;
  double lambda_min, lambda_max;
  double sum_intensity_, avg_intensity_;
  size_t Nsamples_;
  bool valid_;

private:

  cell(const Eigen::Vector2d& u, const double intensity) : // Only for raw data
    u_(u), scale_(1.0), snormal_(1,0), orth_normal(0,1), lambda_min(1),
    lambda_max(1), sum_intensity_(1.0), avg_intensity_(1.0), Nsamples_(1), valid_(true)
  {}

  bool ComputeNormal(const Eigen::Vector2d& origin);

};

class MapPointNormal;
typedef boost::shared_ptr<MapPointNormal> MapNormalPtr;

class MapPointNormal
{


public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MapPointNormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cld, float radius, const Eigen::Vector2d& origin = Eigen::Vector2d(0,0), const bool weight_intensity = false, const bool raw = false);

  MapPointNormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cld, float radius, std::vector<cell>& cell_orig, const Eigen::Affine3d& T);


  MapPointNormal(){
    input_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    downsampled_ = pcl::PointCloud<pcl::PointXY>::Ptr(new pcl::PointCloud<pcl::PointXY>());

  }

  std::vector<cell> GetCells(){return cells;}

  cell& GetCell(const size_t i){cells[i];}

  //std::vector<cell*> GetClosest(cell* c, double d);

  std::vector<cell*> GetClosest(Eigen::Vector2d& p, double d);

  std::vector<int> GetClosestIdx(const Eigen::Vector2d&  p, double d);

  ros::Time GetTime(){ros::Time t; pcl_conversions::fromPCL(input_->header.stamp, t); return t;}

  double GetCellRelTimeStamp(const size_t index, const bool ccw);

  //pcl::PointCloud<pcl::PointXYZINormal>::Ptr GetCloud();

  void Compensate(const Eigen::Affine3d& Tmot, const bool ccw);

  std::vector<cell> TransformCells(const Eigen::Affine3d& T);

  void Transform(const Eigen::Affine3d& T, Eigen::MatrixXd& means, Eigen::MatrixXd& normals);

  void Transform2d(const Eigen::Affine3d& T, Eigen::MatrixXd& means, Eigen::MatrixXd& normals);

  //Eigen::MatrixXd GetNormals();

  Eigen::MatrixXd GetNormals2d();

  std::vector<Eigen::Matrix2d> GetCovs();

  //Eigen::MatrixXd GetMeans();

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

  void ComputeNormals(const Eigen::Vector2d &origin);

  //bool ComputeMeanAndNormals(const std::vector<int>& pointIdxNKNSearch,  Eigen::Vector2d& u, Eigen::Matrix2d& Cov, double& avg_intensity, int &Nsamples);

  inline double Gausian(const double x, const double u, const double sigma);

  //bool WeightedCovariance(Eigen::MatrixXd& w, Eigen::MatrixXd& x, Eigen::Matrix2d& cov);


  // MEMBER VARIABLES
  std::vector<cell> cells;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_ = NULL;
  pcl::PointCloud<pcl::PointXY>::Ptr downsampled_ = NULL;
  pcl::KdTreeFLANN<pcl::PointXY> kd_cells;
  float radius_;
  bool weight_intensity_;

public:


  static void PublishMap(const std::string& topic, MapNormalPtr map, Eigen::Affine3d& T, const std::string& frame_id, const int value=0);

  static std::map<std::string, ros::Publisher> pubs;

  static double downsample_factor;

};



visualization_msgs::Marker DefaultMarker( const ros::Time& time, const std::string& frame);

visualization_msgs::MarkerArray Cells2Markers(std::vector<cell>& cells, const ros::Time& time, const std::string& frame, const int val=0 );

void intToRGB(int value,float& red,float& green, float& blue);

inline pcl::PointXYZI Pnt(Eigen::Vector2d& u);

inline pcl::PointXYZ PntXYZ(Eigen::Vector2d& u);

inline geometry_msgs::Point Pntgeom(Eigen::Vector2d& u);




}


