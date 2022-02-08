#pragma once
#include "tuple"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>


#include <sensor_msgs/Image.h>
#include "vision_msgs/Detection2D.h"
#include "sensor_msgs/CameraInfo.h"
#include "ros/time.h"
#include "map"
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/map.hpp>
#include "unordered_map"
#include "tuple"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/publisher.h"
#include "pcl/common/centroid.h"
#include "pcl/point_types.h"
#include <pcl_conversions/pcl_conversions.h>

namespace CFEAR_Radarodometry {

using std::cout;
using std::endl;
using std::cerr;

typedef std::tuple<int,int> idx_grid;

int GetX(idx_grid& idx);

int GetY(idx_grid& idx);

int GetZ(idx_grid& idx);

class voxel
{
public:

  voxel();

  voxel(const Eigen::Vector2d& p);

  void AddPoint(const Eigen::Vector2d& p);

  void ComputeMeanCov(const Eigen::Vector2d& p);

  unsigned int N_;
  Eigen::Matrix2d Cov_;
  Eigen::Vector2d u_;
  bool has_gausian_;
  Eigen::MatrixXd pnts_;

};


class Voxelgrid
{
public:

  Voxelgrid(double resolution = 0.01):resolution_(resolution) {}

  idx_grid GetIndex(const double x, const double y);

  idx_grid GetIndex(const pcl::PointXYZI& p);

  void IdxToCenter(const idx_grid& idx, Eigen::Vector2d& center);

  Eigen::Vector2d IdxToCenterReturn(const idx_grid& idx);

  void InsertElement(const double x, const double y,const voxel& obj);

  void InsertElement(idx_grid idx, const voxel& obj);

  voxel* GetElement(const idx_grid& idx);

  std::vector<idx_grid> GetNeighbours( idx_grid& idx, int n);

  const voxel* GetElement(const double x, const double y);

  bool ElementExists(const idx_grid& idx);

  size_t Size();

  std::vector<idx_grid> GetIndecies();

  std::vector<Eigen::Vector2d> GetCenters();

  std::vector<voxel> GetCells();

  double GetGridProb();

  void SetGridProb(double prob);

  void Clear();

  double GetResolution();


  double resolution_;
  std::map<idx_grid, voxel> map_;

};


class NDTGrid
{
public:

  NDTGrid(float resolution);

  NDTGrid(const pcl::PointCloud<pcl::PointXYZI>& cld, float resolution);

private:
  Voxelgrid map_;

};


}
