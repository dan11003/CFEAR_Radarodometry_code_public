#include "ros/ros.h"
#include "cfear_radarodometry/eval_trajectory.h"
#include "stdio.h"

using CFEAR_Radarodometry::EvalTrajectory;

/** \brief Ros node for evaluating odometry quality. Subscribes to ground truth and estimated odometry and export trajectories as .csv files.
 * \author Daniel adolfsson
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluation_node");
  ros::NodeHandle nh("~");
  EvalTrajectory::Parameters par;
  par.GetParametersFromRos(nh);
  EvalTrajectory e(par);
  ros::spin();
  e.Save();

  return 0;
}



