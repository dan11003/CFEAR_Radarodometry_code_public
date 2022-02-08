
#include <ros/ros.h>
#include "cfear_radarodometry/radar_driver.h"

/** \brief A ROS node which implements radar filtering based on k-strongest
 * \author Daniel adolfsson
 */

using CFEAR_Radarodometry::radarDriver;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "radar_filtering_node");
  ros::NodeHandle nh("~");
  radarDriver::Parameters par;
  par.GetParametersFromRos(nh);
  radarDriver driver(par);
  ros::spin();
  return 0;
}
