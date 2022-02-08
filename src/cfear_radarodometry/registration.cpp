#include "cfear_radarodometry/registration.h"
namespace CFEAR_Radarodometry {

Registration::Registration():nh_("~"){
  this->options_.line_search_direction_type = ceres::LineSearchDirectionType::BFGS;
  this->problem_ = nullptr;
  pub_association = nh_.advertise<visualization_msgs::MarkerArray>("associations",100);
}
double Registration::getScore(){
  return score_;
}
std::string Registration::GetParameterString(){
  std::string par_string="";
  for( size_t i=0 ; i<parameters.size() ; i++)
    par_string += std::to_string(parameters[i][0])+" "+std::to_string(parameters[i][1])+" "+std::to_string(parameters[i][2])+"\n";
  return par_string;
}

void normalizeEulerAngles(Eigen::Vector3d &euler) {
  euler[0] = angles::normalize_angle(euler[0]);
  euler[1] = angles::normalize_angle(euler[1]);
  euler[2] = angles::normalize_angle(euler[2]);

  if (fabs(euler[0]) > M_PI / 2) {
    euler[0] += M_PI;
    euler[1] = -euler[1] + M_PI;
    euler[2] += M_PI;

    euler[0] = angles::normalize_angle(euler[0]);
    euler[1] = angles::normalize_angle(euler[1]);
    euler[2] = angles::normalize_angle(euler[2]);
  }
}
inline geometry_msgs::Point Pntgeom(Eigen::Vector3d& u){
  geometry_msgs::Point p;
  p.x = u(0);
  p.y = u(1);
  p.z = u(2);
  return p;
}
Eigen::Matrix3d Cov6to3(const Matrix6d& C){
  Eigen::Matrix3d cov3;
  cov3 << C(0,0), C(0,1), C(0,5),
      C(1,0), C(1,1), C(1,5),
      C(5,0), C(5,1), C(5,5);
  return cov3;
}

Eigen::Affine3d   vectorToAffine3d(double x, double y, double z, double ex, double ey, double ez) {

  return Eigen::Translation<double, 3>(x, y, z) *
      Eigen::AngleAxis<double>(ex, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxis<double>(ey, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxis<double>(ez, Eigen::Vector3d::UnitZ());
}

Eigen::Affine2d  vectorToAffine2d(double x, double y, double ez) {
  Eigen::Affine3d T = vectorToAffine3d(x, y, 0, 0, 0, ez);
  return Eigen::Translation2d(T.translation().topRows<2>()) * T.linear().topLeftCorner<2,2>();

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
Eigen::Matrix3d Cov6dTo3d(const Matrix6d& cov){
  Eigen::Matrix3d cov2d;
  cov2d.block<2,2>(0,0) = cov.block<2,2>(0,0);
  cov2d(2,0) = cov(5,0);
  cov2d(0,2) = cov(0,5);
  cov2d(2,2) = cov(5,5);
  return cov2d;
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



}
