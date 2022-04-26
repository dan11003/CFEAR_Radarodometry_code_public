  #pragma once

#include <time.h>
#include <ros/ros.h>
#include "string.h"
#include "map"
#include "vector"
#include "unordered_map"
#include "tuple"
#include <numeric>
namespace CFEAR_Radarodometry {

using std::endl;
using std::cout;
using std::cerr;
typedef std::unordered_map<std::string, std::vector<double>> executionTimes;
typedef std::tuple<std::string, double, double, int> reports; //name, mean, variance, N samples

class statistics{
public:

  statistics();

  void Document(const std::string& name, const double& value, bool report = false);
  
  void PresentStatistics();
  
  std::string GetStatistics();
  
  private:
  
   void ComputeStatistics(std::vector<reports>& rep);
 
   executionTimes t;
};


extern statistics timing;

double ToMs(const ros::Duration& dur);

double ToMsClock(const double& t);



}

