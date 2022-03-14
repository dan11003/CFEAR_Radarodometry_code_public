#include "cfear_radarodometry/cfar.h"


/////////////////////////////////// CFAR Filter ////////////////////////////////////////

CFARFilter::CFARFilter(const double &false_alarm_rate,  const double &range_resolution,
                       const double &static_threshold, const double &min_distance, const double &max_distance) :
  false_alarm_rate_(false_alarm_rate), range_resolution_(range_resolution), static_threshold_(static_threshold),
  min_distance_(min_distance), max_distance_(max_distance)
{}

double CFARFilter::getCAScalingFactor(const int &window_size) const
{
  const double N = window_size;
  return  N * (std::pow(false_alarm_rate_, -1./N) - 1.);
}

double CFARFilter::getIntensity(uchar intensity, bool square_law) const
{
  if(square_law)
    return std::pow(double(intensity), 2.);

  return double(intensity);
}

//////////////////////////////////// Azimuth CA-CFAR ////////////////////////////////////

AzimuthCACFAR::AzimuthCACFAR(const int &window_size, const double &false_alarm_rate, const int &nb_guard_cells,
                             const double &range_resolution, const double &static_threshold, const double &min_distance, const double &max_distance) :
  CFARFilter(false_alarm_rate, range_resolution, static_threshold, min_distance, max_distance), window_size_(window_size), nb_guard_cells_(nb_guard_cells)
{
  this->scaling_factor_ = getCAScalingFactor(window_size_ * 2);
}

void AzimuthCACFAR::getFilteredPointCloud(const cv_bridge::CvImagePtr &radar_image, pcl::PointCloud<pcl::PointXYZI>::Ptr &output_pointcloud) const
{
  for(int azimuth_nb = 0; azimuth_nb < radar_image->image.rows; azimuth_nb++)
  {
    cv::Mat azimuth = radar_image->image.row(azimuth_nb);
    const double theta = (double(azimuth_nb + 1) / radar_image->image.rows) * 2. * M_PI;
    for(int range_bin = 0; range_bin < azimuth.cols; range_bin++)
    {
      const double range = range_resolution_ * double(range_bin);
      const double intensity = double(azimuth.at<uchar>(range_bin));
      if(range > min_distance_ && range < max_distance_ && intensity > static_threshold_)
      {
        //Scaling factor should be updated with actual window size.
        const int trailing_window_start = std::max(0, range_bin - nb_guard_cells_ - window_size_);
        const int trailing_window_end = range_bin - nb_guard_cells_;
        const double trailing_mean = getMean(azimuth, trailing_window_start, trailing_window_end);

        const int forwarding_window_start = range_bin + nb_guard_cells_;
        const int forwarding_window_end = std::min(azimuth.cols, range_bin + nb_guard_cells_ + window_size_);
        const double forwarding_mean = getMean(azimuth, forwarding_window_start, forwarding_window_end);

        const double mean = std::max(trailing_mean, forwarding_mean);
        const double threshold = scaling_factor_ * mean;
        const double squared_intensity = std::pow(intensity, 2.);
        if(squared_intensity > threshold)
        {
          pcl::PointXYZI p;
          p.x = range * std::cos(theta);
          p.y = range * std::sin(theta);
          p.intensity = intensity;
          output_pointcloud->push_back(p);
        }
      }
    }
  }
}

double AzimuthCACFAR::getMean(const cv::Mat &azimuth, const int &start_idx, const int &end_idx) const
{
  double sum = 0.;
  double N = 0.;
  for(size_t i = start_idx; i < end_idx; i++)
  {
    sum += std::pow(double(azimuth.at<uchar>(i)), 2.);
    N += 1.;
  }
  return sum / N;
}

