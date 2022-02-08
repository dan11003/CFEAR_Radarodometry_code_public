#include "cfear_radarodometry/radar_driver.h"

namespace CFEAR_Radarodometry {


radarDriver::radarDriver(const Parameters& pars, bool disable_callback):par(pars),nh_("~"), it(nh_) {


  min_distance_sqrd = par.min_distance*par.min_distance;
  FilteredPublisher = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>(par.topic_filtered, 1000);
  if(!disable_callback){
    if(par.dataset=="oxford")
      sub = nh_.subscribe<sensor_msgs::Image>("/Navtech/Polar", 1000, &radarDriver::CallbackOxford, this);
    else
      sub = nh_.subscribe<sensor_msgs::Image>("/Navtech/Polar", 1000, &radarDriver::Callback, this);
  }
}

void radarDriver::InitAngles(){
  sin_values.resize(par.azimuths);
  cos_values.resize(par.azimuths);
  for (int i=0;i<par.azimuths;i++){
    float theta=(float)(i+1)*2*M_PI/par.azimuths;
    sin_values[i]=std::sin(theta);
    cos_values[i]=std::cos(theta);
  }
}

void radarDriver::Callback(const sensor_msgs::ImageConstPtr &radar_image_polar){
    if(radar_image_polar==NULL){
        cerr<<"Radar image NULL"<<endl;
        exit(0);
    }
  ros::Time t0 = ros::Time::now();
  cv_bridge::CvImagePtr cv_polar_image;
  cv_polar_image = cv_bridge::toCvCopy(radar_image_polar, sensor_msgs::image_encodings::MONO8);
  cv_polar_image->header.stamp=radar_image_polar->header.stamp;
  cv::resize(cv_polar_image->image,cv_polar_image->image,cv::Size(), 1, 1, cv::INTER_NEAREST);
  //range_res=range_res*2;
  uint16_t bins = cv_polar_image->image.rows;
  //uint16_t azimuths = cv_polar_image->image.cols;
  rotate(cv_polar_image->image, cv_polar_image->image, cv::ROTATE_90_COUNTERCLOCKWISE);

  //cv::normalize(cv_polar_image->image, cv_polar_image->image, 0, 255, cv::NORM_MINMAX, CV_8UC1);

  cloud_filtered = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  CFEAR_Radarodometry::k_strongest_filter(cv_polar_image, cloud_filtered, par.k_strongest, par.z_min, par.range_res, par.min_distance);
  cloud_filtered->header.frame_id = par.radar_frameid;
  pcl_conversions::toPCL(cv_polar_image->header.stamp, cloud_filtered->header.stamp);//pcl_conversions::toPCL(cv_polar_image->header.stamp,cloud->header.stamp);
  FilteredPublisher.publish(cloud_filtered);
  ros::Time t2 = ros::Time::now();
  CFEAR_Radarodometry::timing.Document("Filtering",CFEAR_Radarodometry::ToMs(t2-t0));


}

/*Assumptions:
       *
       * TYPE_8UC1
       * Rows azimuth
       * Cols Range, from left (rmin) to right (rmax)
       *
       * */
void radarDriver::CallbackOxford(const sensor_msgs::ImageConstPtr &radar_image_polar)
{
    if(radar_image_polar==NULL){
        cerr<<"Radar image NULL"<<endl;
        exit(0);
    }
  ros::Time t0 = ros::Time::now();
  cv_bridge::CvImagePtr cv_polar_image;
  cv_polar_image = cv_bridge::toCvCopy(radar_image_polar, sensor_msgs::image_encodings::TYPE_8UC1);
  ros::Time t1 = ros::Time::now();

  cv_polar_image->header.stamp = radar_image_polar->header.stamp;
  ros::Time t2 = ros::Time::now();
  //ros::Time t2 = ros::Time::now();
  //CFEAR_Radarodometry::timing.Document("Filtering",CFEAR_Radarodometry::ToMs(t2-t0));
  cloud_filtered = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  // Implementation (1)
  //CFEAR_Radarodometry::kstrongLookup filter(par.k_strongest, par.z_min,par.range_res,par.min_distance);
  //filter.getFilteredPointCloud(cv_polar_image, cloud_filtered);

  //Implementation (2)
  //CFEAR_Radarodometry::k_strongest_filter(cv_polar_image, cloud_filtered, par.k_strongest, par.z_min, par.range_res, par.min_distance);

  //Implementation (3)
  StructuredKStrongest filt(cv_polar_image, par.z_min, par.k_strongest, par.min_distance, par.range_res);
  filt.getPeaksFilteredPointCloud(cloud_filtered, false);


  cloud_filtered->header.frame_id = par.radar_frameid;
  FilteredPublisher.publish(cloud_filtered);
  ros::Time t3 = ros::Time::now();
  CFEAR_Radarodometry::timing.Document("time-Filtering",CFEAR_Radarodometry::ToMs(t3-t0));
}

void radarDriver::CallbackOffline(const sensor_msgs::ImageConstPtr& radar_image_polar,  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud){

  if(par.dataset=="oxford")
    CallbackOxford(radar_image_polar);
  else
    Callback(radar_image_polar);

  *cloud = *cloud_filtered;
  cloud->header = cloud_filtered->header;

}

}
