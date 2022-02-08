#include "cfear_radarodometry/intensity_utils.h"
namespace CFEAR_Radarodometry {
void IntensityFilter(pcl::PointCloud<pcl::PointXYZI>& cld_in, double th, double min, double max){
  pcl::PointCloud<pcl::PointXYZI> filtered;
  double min_sqr = min*min;
  double max_sqr = max*max;
  for (int var = 0; var < cld_in.points.size(); ++var) {
    if(cld_in.points[var].intensity >= th ){
      double x = cld_in.points[var].x;
      double y = cld_in.points[var].y;
      double z = cld_in.points[var].z;
      double d = x*x+y*y+z*z;
      if(d<max_sqr && d>min_sqr)
        filtered.push_back(cld_in.points[var]);
    }
  }
  cld_in.points.clear();
  cld_in+=filtered;
}
typedef std::tuple<double, double, pcl::PointXYZI> PointAIXYZ;
bool sortGreater(const PointAIXYZ& a,
                 const PointAIXYZ& b)
{
  return (std::get<0>(a) > std::get<0>(b));
}
void FilterReflections(pcl::PointCloud<pcl::PointXYZI>& cld_in, int n_strongest, bool closest){
  pcl::PointCloud<pcl::PointXYZI> filtered;
  double eps = 0.0001;
  std::vector<std::vector<PointAIXYZ> > pnts;
  for (int var = 0; var < cld_in.points.size(); ++var) {

    double alpha = atan2(cld_in.points[var].y, cld_in.points[var].x);
    bool found = false;
    for(int i =0;i<pnts.size();i++){
      double a2 = std::get<1>(pnts[i].front());
      if(fabs(alpha-a2)<eps || fabs( (std::min(alpha,a2)+2*M_PI) - std::max(alpha,a2) )<eps ){ // if there exist point in same bin
        double x = cld_in.points[var].x;
        double y = cld_in.points[var].y;
        double d = x*x+y*y;
        if(!closest)
          pnts[i].push_back( std::make_tuple(cld_in.points[var].intensity, alpha, cld_in.points[var]));
        else
          pnts[i].push_back( std::make_tuple(-d, alpha, cld_in.points[var]));
        found = true;
        break;
      }
    }
    if(!found){
      std::vector<std::tuple<double, double, pcl::PointXYZI>> vek;
      vek.push_back( std::make_tuple(cld_in.points[var].intensity, alpha, cld_in.points[var]));
      pnts.push_back(vek);
    }
  }
  cld_in.clear();


  int count = 0;
  for(int i=0;i<pnts.size();i++){
    //cout<<"bin: "<<i<<" - a="<<pnts[i].front().first<<", count: "<<pnts[i].size()<<endl;
    std::sort(pnts[i].begin(), pnts[i].end(), sortGreater);
    for( int j=0 ; j<pnts[i].size() && j<n_strongest; j++){
      cld_in.push_back(std::get<2>(pnts[i][j]));
    }

  }
  //cout<<"Filter from : "<<count<<" to "<<cld_in.size()<<endl;
  //cout<<"tot: "<<count<<", "<<", actual: "<<cld_in.size()<<endl;

}

void AddNoise(pcl::PointCloud<pcl::PointXYZI>& cld_in, double varz){
  for(int i=0; i<cld_in.points.size(); i++)
    cld_in.points[i].z += varz*((double)rand())/(double)INT_MAX;
}

pcl::PointXYZ XYZI_to_XYZ(const pcl::PointXYZI& pin){
  return pcl::PointXYZ(pin.x, pin.y, pin.z);
}
pcl::PointXYZI XYZ_to_XYZI(const pcl::PointXYZ& pin, double intensity){
  pcl::PointXYZI pnt;
  pnt.x = pin.x;
  pnt.y = pin.y;
  pnt.z = pin.z;
  pnt.intensity = intensity;
  return pnt;
}
void Convert(pcl::PointCloud<pcl::PointXYZI>& cld_in,pcl::PointCloud<pcl::PointXYZ>& cld_out){
  for(auto & p_in: cld_in.points){
    pcl::PointXYZ p = XYZI_to_XYZ(p_in);
    cld_out.push_back(p);
  }
  cld_out.header = cld_in.header;
}



}
