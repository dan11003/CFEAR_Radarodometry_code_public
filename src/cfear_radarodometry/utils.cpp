#include "cfear_radarodometry/utils.h"
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


void Compensate(pcl::PointCloud<pcl::PointXYZI>& cloud, const std::vector<double>& mot, bool ccw){
  for ( int i=0; i<cloud.size();i++) {
    pcl::PointXYZI p = cloud.points[i];
    double d = GetRelTimeStamp(p.x, p.y, ccw);
    Eigen::Vector2d peig(p.x,p.y);
    Eigen::Matrix2d R = getScaledRotationMatrix(mot, d).block<2,2>(0,0);
    Eigen::Vector2d t = getScaledTranslationVector(mot, d).block<2,1>(0,0);
    Eigen::Vector2d peig_transformed = R*peig + t;
    cloud.points[i].x = peig_transformed(0,0);
    cloud.points[i].y = peig_transformed(1,0);
  }
}

void Compensate(pcl::PointCloud<pcl::PointXYZI>& cloud, const Eigen::Affine3d& Tmotion, bool ccw){
  std::vector<double> mot;
  CFEAR_Radarodometry::Affine3dToVectorXYeZ(Tmotion, mot);
  Compensate(cloud, mot, ccw);
}

void Affine3dToVectorXYeZ(const Eigen::Affine3d& T, std::vector<double>& par) {
  if(par.size()!=3)
    par.resize(3,0);
  par[0] = T.translation()(0);
  par[1] = T.translation()(1);
  Eigen::Vector3d eul = T.linear().eulerAngles(0,1,2);
  par[2] = eul(2);
}

void Affine3dToEigVectorXYeZ(const Eigen::Affine3d& T, Eigen::Vector3d& par) {
  Eigen::Vector3d eul = T.linear().eulerAngles(0,1,2);
  par << T.translation()(0), T.translation()(1), eul(2);
}


Eigen::Matrix3d getScaledRotationMatrix(const std::vector<double>& parameters, double factor)
{
  Eigen::Matrix3d rotation_matrix;
  const double s_1 = ceres::sin(factor*parameters[2]);
  const double c_1 = ceres::cos(factor*parameters[2]);
  rotation_matrix <<
      c_1,     -s_1,     0.0,
      s_1,     c_1,      0.0,
      0.0,     0.0,      1.0;
  return rotation_matrix;
}

Eigen::Vector3d getScaledTranslationVector(const std::vector<double>& parameters, double factor){
  Eigen::Vector3d vek;
  vek << factor*parameters[0], factor*parameters[1], 0.0;
  return vek;
}
void VectorXYeZtoAffine2d(const Eigen::Affine3d& T, Eigen::Vector3d& par){

}


}
