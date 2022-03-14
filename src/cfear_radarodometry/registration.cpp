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
void Registration::InitFixedBlocks(const size_t& nsize){
  if(mode_ == incremental_last_to_previous){
    fixedBlock_ = std::vector<bool>(nsize,true);
    fixedBlock_.back() = false;
  }
  else{
    std::cerr<<"Not implemented"<<endl;
    exit(0);
  }
}


cost_metric Str2Cost(const std::string& str){
  if (str=="P2L")
    return cost_metric::P2L;
  else if (str=="P2D")
    return cost_metric::P2D;
  else //(str=="P2P")
    return cost_metric::P2P;
}

std::string Loss2str(const loss_type& loss){
  switch (loss){
  case Huber: return "Huber";
  case Cauchy: return "Cauchy";
  case SoftLOne: return "SoftLOne";
  case Combined: return "Combined";
  case Tukey: return "Tukey";
  case None: return "None";
  default: return "Huber";
  }
}
loss_type Str2loss(const std::string& loss){
  if (loss=="Huber")
    return loss_type::Huber;
  else if (loss=="Cauchy")
    return loss_type::Cauchy;
  else if (loss=="SoftLOne")
    return loss_type::SoftLOne;
  else if (loss=="Combined")
    return loss_type::Combined;
  else if (loss=="Tukey")
    return loss_type::Tukey;
  else if (loss=="None")
    return loss_type::None;
  else
    return loss_type::Huber;
}

ceres::LossFunction* Registration::GetLoss(){
  ceres::LossFunction* ceres_loss = nullptr;
  if(loss_ == Huber)
    ceres_loss = new ceres::HuberLoss(loss_limit_);
  else if( loss_ == Cauchy)
    ceres_loss = new ceres::CauchyLoss(loss_limit_);
  else if( loss_ == SoftLOne)
    ceres_loss = new ceres::SoftLOneLoss(loss_limit_);
  else if( loss_ == Tukey)
    ceres_loss = new ceres::TukeyLoss(loss_limit_);
  else if(loss_ == Combined){
    ceres::LossFunction* f = new ceres::HuberLoss(1);
    ceres::LossFunction* g = new ceres::CauchyLoss(1);
    ceres_loss = new ceres::ComposedLoss(f, ceres::DO_NOT_TAKE_OWNERSHIP, g, ceres::DO_NOT_TAKE_OWNERSHIP);
  }
  else
    ceres_loss = nullptr;
  return ceres_loss;
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
Eigen::Affine3d vectorToAffine3d(const std::vector<double>& vek){
  assert(vek.size() == 3);
  return Eigen::Translation<double, 3>(vek[0], vek[1], 0) *
      Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxis<double>(vek[2], Eigen::Vector3d::UnitZ());
}

Eigen::Affine3d vectorToAffine3d(double x, double y, double z, double ex, double ey, double ez) {

  return Eigen::Translation<double, 3>(x, y, z) *
      Eigen::AngleAxis<double>(ex, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxis<double>(ey, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxis<double>(ez, Eigen::Vector3d::UnitZ());
}

Eigen::Affine2d  vectorToAffine2d(double x, double y, double ez) {
  Eigen::Affine3d T = vectorToAffine3d(x, y, 0, 0, 0, ez);
  return Eigen::Translation2d(T.translation().topRows<2>()) * T.linear().topLeftCorner<2,2>();

}


Eigen::Matrix3d Cov6dTo3d(const Matrix6d& cov){
  Eigen::Matrix3d cov2d;
  cov2d.block<2,2>(0,0) = cov.block<2,2>(0,0);
  cov2d(2,0) = cov(5,0);
  cov2d(0,2) = cov(0,5);
  cov2d(2,2) = cov(5,5);
  return cov2d;
}



}
