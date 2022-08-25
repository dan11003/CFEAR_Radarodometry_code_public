#include "cfear_radarodometry/types.h"
namespace CFEAR_Radarodometry {


Pose3d::Pose3d(const Eigen::Affine3d& T){

  Eigen::Matrix3d m = T.linear();
  this->q = Eigen::Quaterniond(m);
  this->q.normalize();
  this->p = T.translation();
}

Pose3d Pose3d::Inverse() {
  Pose3d pinv;
  pinv.p = - p;
  pinv.q = q.inverse();
  return pinv;
}

Eigen::Affine3d Pose3d::GetPose(){
  return(PoseCeresToEig(*this));
}


Pose3d PoseEigToCeres(const Eigen::Affine3d &pose_eig){
  Pose3d pose_ceres;
  Eigen::Matrix3d m = pose_eig.linear();
  pose_ceres.q = Eigen::Quaterniond(m);
  pose_ceres.q.normalize();
  pose_ceres.p = pose_eig.translation();
  return pose_ceres;
}
Eigen::Affine3d PoseCeresToEig(const Pose3d &pose_ceres){
  Eigen::Affine3d pose_eig;
  pose_eig.translation() = pose_ceres.p;
  pose_eig.linear() = pose_ceres.q.toRotationMatrix();
  return pose_eig;
}
tf::Transform PoseCeresToTf(const Pose3d &pose_ceres){
  tf::Transform T;
  tf::Vector3 v;
  tf::Quaternion q;
  tf::vectorEigenToTF(pose_ceres.p, v);
  tf::quaternionEigenToTF(pose_ceres.q, q);
  T.setOrigin(v);
  T.setRotation(q);

  return T;
}

Pose3d PoseToCeres(double x, double y, double z, double ex, double ey, double ez){

  Eigen::Quaterniond q;

  q = Eigen::AngleAxisd(ex, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(ey, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(ez, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d p(x,y,z);
  Pose3d pose_ceres;
  pose_ceres.q = q;
  pose_ceres.p = p;
  return pose_ceres;
}
std::string MatToString(Eigen::MatrixXd& m){
  std::stringstream ss;
  ss << std::fixed << std::showpoint;
  assert(m.rows()== 4 && m.cols()==4);

  ss<< m(0,0) <<" "<< m(0,1) <<" "<< m(0,2) <<" "<< m(0,3) <<" "<<
             m(1,0) <<" "<< m(1,1) <<" "<< m(1,2) <<" "<< m(1,3) <<" "<<
             m(2,0) <<" "<< m(2,1) <<" "<< m(2,2) <<" "<< m(2,3);
  return ss.str();
}
unsigned int RadarScan::counter = 0;

RadarScan::RadarScan(const Eigen::Affine3d& pose_scan, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_peaks, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_nopeaks, const CFEAR_Radarodometry::MapNormalPtr& cloud_normal):
  T(pose_scan), idx_(RadarScan::counter++), cloud_peaks_(cloud_peaks), cloud_nopeaks_(cloud_nopeaks), cloud_normal_(cloud_normal)
{
  stamp_ = cloud_nopeaks->header.stamp;
}
RadarScan::RadarScan(const Eigen::Affine3d& pose_scan, const Eigen::Affine3d& Tmotion, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_peaks, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_nopeaks, const CFEAR_Radarodometry::MapNormalPtr& cloud_normal, const ros::Time& t):
  RadarScan( pose_scan, cloud_peaks, cloud_nopeaks, cloud_normal)
{
  motion_ = Tmotion;
  stamp_ = t.toNSec();
}

RadarScan::RadarScan(const Eigen::Affine3d& pose_scan):
  T(pose_scan),idx_(RadarScan::counter++)
{

}
const std::string RadarScan::ToString(){

Eigen::MatrixXd m(T.GetPose().matrix());
assert(m.rows()== 4 && m.cols()==4);
std::stringstream ss;
ss << std::fixed << std::showpoint;

ss <<MatToString(m) << " " << std::to_string(stamp_) << std::endl;
return ss.str();
}
void SaveSimpleGraph(const std::string& path, simple_graph &graph){
  try {
    std::cout << "Save simple graph to: " << path<<endl;
    std::cout << "Size: " << graph.size() << std::endl;
    std::ofstream ofs(path);
    boost::archive::binary_oarchive oa(ofs);
    oa << graph;
    ofs.close();
  }catch (std::exception &e) {
  }
}

bool LoadSimpleGraph(const std::string& path, simple_graph &graph){

  try {
    std::ifstream ifs(path);
    boost::archive::binary_iarchive ia(ifs);
    ia >> graph;
    ifs.close();
    cout << "Simple graph succesfully loaded from: " << path << endl;
    cout << graph.size() << std::endl;

    return true;
  }catch (std::exception &e) {
    std::cerr<<"Graph could not be loaded from: "<<path<<std::endl;
    return false;
  }
}

ConstraintsHandler::ConstraintsHandler()
{
  constraints_ = { {ConstraintType::odometry,        Constraints() },
                   {ConstraintType::loop_appearance, Constraints() },
                   {ConstraintType::candidate,       Constraints() }
                 };
}
std::string ConstraintsHandler::ToString(){
  return "odom constraints: " + std::to_string(size(ConstraintType::odometry)) + ", loop constraints: " + std::to_string(size(ConstraintType::loop_appearance)) + "\n";;
}
std::string Constraint2String(const ConstraintType& c){
  if (c==odometry)
    return "odometry";
  else if (c==loop_appearance)
    return "loop_apperance";
  else
    return  "loop_candidate";
}

void ConstraintsHandler::Add(const Constraint3d& c){
  if( constraints_.find(c.type)==constraints_.end() ){
    throw std::invalid_argument("constraint type not defined - out of range; ConstraintsHandler::Add");
  }
  auto key = MakeKey(c.id_begin,c.id_end);
  constraints_[c.type][key] = c;
  if(c.type == odometry)
    dist_trav += c.t_be.p.norm();
}
void ConstraintsHandler::Add(const RadarScan& from, const RadarScan& to, const Eigen::Affine3d& Tdiff, const Covariance& Cov, ConstraintType ct ){
  Add({from.idx_, to.idx_, PoseEigToCeres(Tdiff), Cov, ct});
}

ConstraintsHandler::ConstraintKey ConstraintsHandler::MakeKey(unsigned int a, unsigned int b){
  if(a==b)
    throw std::invalid_argument("a == b error, self-constraint not allowed");
  else
    return std::make_pair(std::min(a,b),std::max(a,b)); //
}

bool ConstraintsHandler::HasConstraintType(int id, ConstraintType type){
  for(auto c : constraints_[type]){
    if( (c.second.id_begin == id || c.second.id_end== id))
      return true;
  }
  return false;
}

bool ConstraintsHandler::FindConstraint( unsigned int id1, unsigned int id2,  Constraint3d& it, const ConstraintType type ){
  auto key = MakeKey(id1,id2);
  if(constraints_.find(type) != constraints_.end() && //Orderly search
     constraints_[type].find(key) != constraints_[type].end() ){
    it = constraints_[type][key];

    return true;
  }
  else
    return false;
}
bool ConstraintsHandler::ConstraintExists( unsigned int id1, unsigned int id2, const ConstraintType type){
  Constraint3d c;
  return FindConstraint(id1,id2,c,type);
}


/*RadarScan& RadarScanHandler::operator[] (int index){
  if(RadarScans_.find(index)!=RadarScans_.end()){
    return RadarScans_.find(index)->second;
  }
  else{
    throw std::invalid_argument("RadarScan& RadarScanHandler::operator[] (int index) - index out of range");

  }
}*/
Eigen::Affine3d ConstraintsHandler::RelativeMotion( unsigned int id1, unsigned int id2,  const ConstraintType type){
  Constraint3d Constraint;
  if(FindConstraint(id1,id2, Constraint, type))
    return PoseCeresToEig(Constraint.t_be);
  else {
    std::cerr << "Constraint does not exist" << endl;
    return Eigen::Affine3d::Identity();
  }

}

void RadarScanHandler::Add(const RadarScan& scan){
  RadarScans_[scan.idx_] = scan;
}
double RadarScanHandler::GetDistance(unsigned int from, unsigned int to)
{
  double d = 0;
  const unsigned idx_from = std::min(from,to);
  const unsigned idx_to   = std::max(from,to);
  for(int i = idx_from ; i < idx_to - 1 ; i++)
    d += (RadarScans_[i].T.p-RadarScans_[i+1].T.p).norm();

  return d;
}

}
