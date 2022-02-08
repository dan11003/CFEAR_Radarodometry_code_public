#pragma once

// STL
#include <iostream>
#include <cmath>
#include <vector>
#include <functional>
#include <numeric>

// OpenCv
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// PCL
#include <boost/make_shared.hpp>
#include <pcl/common/common_headers.h>
#include "pcl_conversions/pcl_conversions.h"
#include "unordered_map"
#include "statistics.h"

namespace CFEAR_Radarodometry
{



// Template class
class RadarFilter
{
public:

  virtual void getFilteredPointCloud(const cv_bridge::CvImagePtr &radar_image, pcl::PointCloud<pcl::PointXYZI>::Ptr &output_pointcloud, const double &range_resolution);

  virtual void getFilteredImage(const cv_bridge::CvImagePtr &radar_image, cv_bridge::CvImagePtr &output_image){output_image = nullptr;}

  virtual ~RadarFilter(){}

protected:
  void toPointCloud(const cv_bridge::CvImagePtr &radar_image, const pcl::PointCloud<pcl::PointXYZI>::Ptr &output_pointcloud, const double &range_resolution) const;
};


// Implementation (1) - insert and restort
void InsertStrongestK(std::vector<pcl::PointXYZI>& pnts_sorted, const pcl::PointXYZI& p, int k_strongest);

void k_strongest_filter(cv_bridge::CvImagePtr &cv_polar_image, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int k_strongest, double z_min, double range_res, double min_distance );

// Implementation (2) use a lookup table
class kstrongLookup : public RadarFilter
{
  typedef std::unordered_map<int,std::vector<int>> azi_range_map;

public:

  kstrongLookup(int k_strongest, double z_min, double range_res, double min_distance) : zmin(z_min), min_distance_(min_distance), range_res_(range_res), kstrongest_(k_strongest) {}

  void getFilteredPointCloud(const cv_bridge::CvImagePtr &radar_image, pcl::PointCloud<pcl::PointXYZI>::Ptr &output_pointcloud);

private:

  void SortBinsByIntensity(const cv_bridge::CvImagePtr &radar_image);

  const int kstrongest_;
  const double range_res_;
  const double min_distance_;
  const int zmin;
  // key: intensity, values vector of all bins that has intensity level== key
  int nb_ranges_;
  int nb_azimuths_;
  azi_range_map filtered; // filtered[azimuth][ranges]
  size_t filtered_size = 0;


};


//Implementation (3) insert sorted


class StructuredKStrongest
{
  public:

  typedef std::pair<uchar,int> intensity_range;

  StructuredKStrongest(const cv_bridge::CvImagePtr &radar_image, const int z_min, const int k_strongest, const double min_distance, const double range_res);

  void getPeaksFilteredPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& output_pointcloud, bool peaks = false);

protected:

  void getPeaksFilteredPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& output_pointcloud, const std::vector<std::vector<intensity_range>>& vek);

  void FilterKstrongest();

  void AxialNonMaxSupress();

  inline void InsertStrongestK(std::vector<intensity_range>& sorted, const intensity_range& p);

  int z_min_, k_strongest_;
  double min_distance_, range_res_;
  const int nb_ranges;
  const int nb_azimuths;

  int nr_filtered_ = 0, nr_peaks_ = 0;
  cv_bridge::CvImagePtr raw_input_;

  cv_bridge::CvImagePtr sparse_filrered_;
  std::vector<std::vector<intensity_range>> dense_filtered_;
  std::vector<std::vector<intensity_range>> dense_filtered_peaks_;
};


}
