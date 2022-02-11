#include "cfear_radarodometry/radar_filters.h"

namespace CFEAR_Radarodometry
{

struct GreaterPXYZI //For sotring pnts XYZI
{
  inline bool operator() (const pcl::PointXYZI& p1, const pcl::PointXYZI& p2)
  {
    return (p1.intensity > p2.intensity);
  }

};



bool sortIntensityPair(const std::pair<double,int>& p1, const std::pair<double,int>& p2)
{
  return (p1.first > p2.first);
}


//////////////////////////////////// Radar Filter ////////////////////////////////////////

void InsertStrongestK(std::vector<pcl::PointXYZI>& pnts_sorted, const pcl::PointXYZI& p, int k_strongest)
{
  if( pnts_sorted.empty() ){
    pnts_sorted.push_back(p);
    return;
  }
  else if(p.intensity <= pnts_sorted.back().intensity)
    return;
  else {
    pnts_sorted.push_back(p);
    std::sort(pnts_sorted.begin(), pnts_sorted.end(), GreaterPXYZI());
    if(pnts_sorted.size()>k_strongest)
      pnts_sorted.pop_back();
  }
}
void k_strongest_filter(cv_bridge::CvImagePtr &cv_polar_image, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int k_strongest, double z_min, double range_res, double min_distance ){
  if(cloud==NULL)
    cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>());
  pcl_conversions::toPCL(cv_polar_image->header.stamp, cloud->header.stamp);


  const  double min_distance_sqrd = min_distance*min_distance;
  sensor_msgs::ImagePtr msg = cv_polar_image->toImageMsg();
  float theta;
  /*if(cv_polar_image->image.rows!=400 || cv_polar_image->image.cols!=3768){
    std::cout<<"Size error rows: "<<cv_polar_image->image.rows<<", cols:"<<cv_polar_image->image.cols<<std::endl;
    exit(0);
  }*/
  for (int bearing = 0; bearing < cv_polar_image->image.rows; bearing++){
    theta = ((float)(bearing+1) / cv_polar_image->image.rows) * 2 * M_PI;
    std::vector<pcl::PointXYZI> pnts_sorted;
    for (size_t i = 0; i < cv_polar_image->image.cols; i++){
      int ind = i+0; //Unsure about this one!
      double d = range_res*(ind);

      if(cv_polar_image->image.at<uchar>(bearing, ind) < z_min ){
        continue;
      }
      pcl::PointXYZI p;
      p.x = range_res * ind * cos(theta);
      p.y = range_res * ind * sin(theta);
      p.intensity = cv_polar_image->image.at<uchar>(bearing, ind);
      p.z = 0;
      InsertStrongestK(pnts_sorted, p, k_strongest);
    }
    for(auto && p :pnts_sorted){
      if (p.x*p.x+p.y*p.y>min_distance_sqrd)
        cloud->push_back(p);
    }
  }

  cloud->width = (int)cloud->points.size();
  cloud->height = 1;
}

void RadarFilter::getFilteredPointCloud(const cv_bridge::CvImagePtr &radar_image, pcl::PointCloud<pcl::PointXYZI>::Ptr &output_pointcloud, const double &range_resolution)
{
  cv_bridge::CvImagePtr result_image = boost::make_shared<cv_bridge::CvImage>();
  getFilteredImage(radar_image, result_image);
  toPointCloud(result_image, output_pointcloud, range_resolution);
}

void RadarFilter::toPointCloud(const cv_bridge::CvImagePtr &radar_image, const pcl::PointCloud<pcl::PointXYZI>::Ptr &output_pointcloud, const double &range_resolution) const
{
  const int nb_ranges = radar_image->image.cols;
  const int nb_azimuths = radar_image->image.rows;
  for(int azimuth_nb = 0; azimuth_nb < nb_azimuths; azimuth_nb++)
  {
    const double theta = (double(azimuth_nb + 1) / radar_image->image.rows) * 2. * M_PI;
    for(int range_bin = 0; range_bin < nb_ranges; range_bin++)
    {
      const double range = range_resolution * double(range_bin);
      const double intensity = double(radar_image->image.at<uchar>(azimuth_nb, range_bin));
      if(intensity > 0.)
      {
        pcl::PointXYZI p;
        p.x = range * std::cos(theta);
        p.y = range * std::sin(theta);
        p.intensity = intensity;
        p.z = 0;
        output_pointcloud->push_back(p);
      }
    }
  }
}

void kstrongLookup::SortBinsByIntensity(const cv_bridge::CvImagePtr &radar_image){

  const int min_range_bin = 0; // min_distance_/range_res_;
  filtered_size = 0;
  for(int azimuth_nb = 0; azimuth_nb < nb_azimuths_; azimuth_nb++)
  {
    int zmax = 0;
    //cout<<"zmin: "<<(double)zmin<<endl;
    std::unordered_map<uchar,std::vector<int>> th_ranges;
    int count = 0;
    for(int range_bin = min_range_bin ; range_bin < nb_ranges_ ; range_bin++)
    {
      const int intensity = int(radar_image->image.at<uchar>(azimuth_nb, range_bin));
      if(intensity >= zmin)
      {
        th_ranges[intensity].push_back(range_bin);
        count++;
        zmax = std::max(zmax,intensity);
      }
      /*else {
        cout<<"intensity: "<<intensity<<", vs "<<", zmin: "<<zmin<<endl;
      }*/
    }
    //cout<<"consider: "<<count<<" bins"<<endl;

    int k_selected = 0;
    bool all_bins_found = false;
    for( int intensity = zmax ; intensity >= zmin && k_selected < kstrongest_ ; intensity-- ) //start at maximum observed intensity and continue to lower intensities.
    {
      if( th_ranges.find(intensity)==th_ranges.end() ) // no data here
      {
        continue;
      }

      //cout<<"bins: "<<th_ranges[intensity].size()<<endl;
      for(auto && range :th_ranges[intensity]) // iterate through all range bins
      {
        filtered[azimuth_nb].push_back(range); //Add to filtered set
        filtered_size++;
        if(++k_selected == kstrongest_){
          break;
        }
      }

    }
  }// Azimuth bins
  //cout<<"filtered_size: "<<filtered_size<<endl;
}

void kstrongLookup::getFilteredPointCloud(const cv_bridge::CvImagePtr &radar_image, pcl::PointCloud<pcl::PointXYZI>::Ptr &output_pointcloud)
{
  if(output_pointcloud==NULL)
    output_pointcloud = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>());
  pcl_conversions::toPCL(radar_image->header.stamp, output_pointcloud->header.stamp);
  nb_ranges_ = radar_image->image.cols;
  nb_azimuths_ = radar_image->image.rows;
  cout<<"---: "<<nb_ranges_<<" x "<<nb_azimuths_<<endl;
  SortBinsByIntensity(radar_image);
  output_pointcloud->resize(filtered_size);

  int index = 0, azimuths = 0;
  for (auto && it=filtered.begin() ; it!=filtered.end() ; ++it)
  {
    azimuths++;
    if(it->second.empty())
      continue;
    else
    {
      const int azimuth = it->first;
      const double theta = (double(azimuth + 1) / nb_azimuths_) * 2. * M_PI;
      const double cos_t = cos(theta);
      const double sin_t = sin(theta);

      for( auto && range : it->second)
      {
        output_pointcloud->points[index].x = (range * range_res_) * cos_t;
        output_pointcloud->points[index].y = (range * range_res_) * sin_t;
        output_pointcloud->points[index].z = 0.0;
        output_pointcloud->points[index].intensity = double(radar_image->image.at<uchar>(azimuth, range)) - ((double)zmin);
        index++;
      }
    }
  }
  output_pointcloud->width = output_pointcloud->size();
  output_pointcloud->height = 1;
}

StructuredKStrongest::StructuredKStrongest(const cv_bridge::CvImagePtr &radar_image, const int z_min, const int k_strongest, const double min_distance, const double range_res)
  :z_min_(z_min), k_strongest_(k_strongest), min_distance_(min_distance), range_res_(range_res), nb_ranges(radar_image->image.cols), nb_azimuths(radar_image->image.rows){
  raw_input_ = radar_image;
  sparse_filrered_ = boost::make_shared<cv_bridge::CvImage>();
  sparse_filrered_->header = radar_image->header;
  sparse_filrered_->image = cv::Mat::zeros(nb_azimuths, nb_ranges, CV_8UC1); //CV_32F //CV_8UC1
  sparse_filrered_->encoding = sensor_msgs::image_encodings::TYPE_8SC1;
  FilterKstrongest();
  //std::cout<<"filtered: "<<nr_filtered_<<"peaks: "<<nr_peaks_<<std::endl;

}
void StructuredKStrongest::FilterKstrongest(){

  assert(raw_input_ != nullptr);
  uchar u_zmin = uchar(z_min_);
  dense_filtered_.resize(raw_input_->image.rows,std::vector<intensity_range>());
  for (size_t bearing = 0; bearing < nb_azimuths; bearing++){
    for (size_t range = 0; range < nb_ranges; range++){
      const uchar intensity = raw_input_->image.at<uchar>(bearing, range);
      if(intensity < u_zmin )
        continue;

      if( dense_filtered_[bearing].empty() ){

        dense_filtered_[bearing].push_back(std::make_pair(intensity,range));
      }else{
        const intensity_range p = std::make_pair(intensity,range);
        auto it = std::lower_bound(dense_filtered_[bearing].cbegin(), dense_filtered_[bearing].cend(), p); //1
        dense_filtered_[bearing].insert(it, p);
        if(dense_filtered_[bearing].size()>k_strongest_)
          dense_filtered_[bearing].erase(dense_filtered_[bearing].begin());
      }
    }
    for (auto && ir : dense_filtered_[bearing]){
      const int range = ir.second;
      sparse_filrered_->image.at<uchar>(bearing,range) = raw_input_->image.at<uchar>(bearing,range);
      nr_filtered_++;
    }
  }
}
void StructuredKStrongest::AxialNonMaxSupress(){

  const int window_size = 3;
  uint16_t limit = ( window_size*2 + 1)*z_min_;
  dense_filtered_peaks_.resize(raw_input_->image.rows,std::vector<intensity_range>());

  for ( size_t bearing = 0; bearing < nb_azimuths ; bearing++ ){
    std::unordered_map<int,uint16_t> score;
    //Compute score of all points

    for ( auto && ir : dense_filtered_[bearing] ){
      const int masked_range = ir.second; // a range bin in the masked image

      if(masked_range<window_size || masked_range>= nb_ranges-window_size) // outside boundries of polar image
        continue;
      for(int r_n = masked_range-window_size ; r_n <= masked_range+window_size ; r_n++) //Enumerate all ranges that are neighbour to masked points, this make sure all neigbours exists during lookup in the next step
      {
        if(score.find(r_n)==score.end()) // if not already computed score for this bin
        {
          score[r_n] = 0;
          for(int r_nn = r_n - window_size ; r_nn <= r_n + window_size ; r_nn++) // enumerate all neibours
          { //Sum nearby intensities to "Smoothen the curve"
            score[r_n] += (uint16_t)raw_input_->image.at<uchar>(bearing,r_nn); // score of neighbour "r_n" to "masked point" is sum of r_n's neighbours (sum r_nn)
          }
        }
      }

    }
    //Based on the score, keep all masked points with local maxima in score
    for (auto && ir : dense_filtered_[bearing])
    {
      const int masked_range = ir.second;
      bool largest = true;
      uint16_t pthis =  score[masked_range];
      //if(pthis<limit) // this is the first requirement, it should be a "strong" peak
      //  continue;
      for(uint16_t i = (uint16_t)1 ; i <= window_size ; i++){
        uint16_t pnext =  score[masked_range + i];
        uint16_t pprev =  score[masked_range - i];
        /*if(pprev == pthis || pthis == pnext){ //for now this is fine
          //std::cout<<"tie, pprev :"<<pprev<<", pthis :"<<pthis<<", pnext:"<<pnext<<std::endl;
          //largest = false;
          //break;
        }*/
        if(pprev > pthis || pthis < pnext){
          largest = false;
          break;
        }
      }
      if(largest == true)
      {
        //for(uint16_t r = masked_range - window_size; r <= masked_range + window_size ; r++)
        //  std::cout<<score[r]<<",";
        nr_peaks_++;
        const uchar intensity = raw_input_->image.at<uchar>(bearing,masked_range);
        dense_filtered_peaks_[bearing].push_back(std::make_pair(intensity,masked_range));
      }
      //Enumerate all ranges that are neighbour to masked points
    }
  }
}

void StructuredKStrongest::getPeaksFilteredPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& output_pointcloud, bool peaks){
  if(peaks){
    AxialNonMaxSupress();
    getPeaksFilteredPointCloud(output_pointcloud,dense_filtered_peaks_);
  }
  else
    getPeaksFilteredPointCloud(output_pointcloud,dense_filtered_);
}

void StructuredKStrongest::getPeaksFilteredPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& output_pointcloud, const std::vector<std::vector<intensity_range>>& vek){
  if(output_pointcloud == nullptr)
    output_pointcloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  assert(output_pointcloud != nullptr);
  assert(nb_azimuths == vek.size() && nb_azimuths > 0);

  const int min_range_bin = (int)ceil(min_distance_/range_res_);
  for (size_t bearing = 0; bearing < nb_azimuths; bearing++){
    const double theta = (double(bearing + 1) / nb_azimuths) * 2. * M_PI;

    if(vek[bearing].empty())
      continue;

    const double cos_t = cos(theta);
    const double sin_t = sin(theta);
    const double range_res_half = range_res_/2.0;
    for (auto && ir : vek[bearing]){
      const int range = ir.second;
      if(range > min_range_bin){
        pcl::PointXYZI p;
        p.x = (range_res_half + range_res_ * range)* cos_t;
        p.y = (range_res_half + range_res_ * range)* sin_t;
        p.intensity = ir.first;
        p.z = 0; //(p.intensity-z_min_)/10.0;
        output_pointcloud->push_back(p);
      }
    }
  }
}
}
