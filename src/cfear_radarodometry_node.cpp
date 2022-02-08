

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include <Eigen/Eigen>
#include "eigen_conversions/eigen_msg.h"
#include <tf_conversions/tf_eigen.h>

#include "sensor_msgs/PointCloud2.h"

#include "cfear_radarodometry/odometrykeyframefuser.h"
#include "cfear_radarodometry/n_scan_normal.h"

/** \brief A ROS node which implements CFEAR Radarodometry within a ros node. Subscribes to filtered radar data and publishes odometry and point cloud
 * \author Daniel adolfsson
 */


using namespace CFEAR_Radarodometry;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cfear_radarodometry_node");
  OdometryKeyframeFuser::Parameters pars;
  ros::NodeHandle nh("~");
  pars.GetParametersFromRos(nh);
  std::cout<<pars.ToString()<<std::endl;
  OdometryKeyframeFuser t(pars);
  ros::spin();
  timing.PresentStatistics();

  return 0;
}



