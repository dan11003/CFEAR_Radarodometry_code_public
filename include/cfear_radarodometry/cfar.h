#pragma once
#include "cfear_radarodometry/radar_filters.h"


//////////////////////////////////// Old interface ////////////////////////////////////////

class CFARFilter
{
protected:
  const double false_alarm_rate_;
  double scaling_factor_;
  const double range_resolution_;
  const double static_threshold_;
  const double min_distance_;
  const double max_distance_;
  const bool square_law_ = false;

public:
  CFARFilter(const double &false_alarm_rate = 0.001, const double &range_resolution = 0.0432,
             const double &static_threshold = 20.0, const double &min_distance = 2.5, const double &max_distance = 150.0);

protected:
  double getCAScalingFactor(const int &window_size) const;

  double getIntensity(uchar intensity, bool square_law) const;
};

class AzimuthCACFAR : public CFARFilter
{
private:
  int window_size_;
  int nb_guard_cells_;

public:
  AzimuthCACFAR(const int &window_size = 40, const double &false_alarm_rate = 0.01, const int &nb_guard_cells = 5,
                const double &range_resolution = 0.0438, const double &static_threshold = 60.0, const double &min_distance = 2.5, const double &max_distance = 130.0);

  void getFilteredPointCloud(const cv_bridge::CvImagePtr &radar_image, pcl::PointCloud<pcl::PointXYZI>::Ptr &output_pointcloud) const;

private:
  double getMean(const cv::Mat &azimuth, const int &start_idx, const int &end_idx) const;
};

