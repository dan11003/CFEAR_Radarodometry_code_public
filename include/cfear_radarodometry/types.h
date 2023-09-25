
#pragma once
#include <istream>
#include <map>
#include <string>
#include <vector>
#include "Eigen/Core"
#include "tf/tf.h"
#include "tf_conversions/tf_eigen.h"
#include "eigen_conversions/eigen_msg.h"
#include "Eigen/Geometry"
#include "cfear_radarodometry/serialization.h"
#include "cfear_radarodometry/pointnormal.h"
#include "iterator"
#include "boost/serialization/map.hpp"
#include "boost/serialization/shared_ptr.hpp"
#include "boost/serialization/throw_exception.hpp"
#include "boost/tuple/tuple.hpp"
#include "boost/archive/binary_iarchive.hpp"
#include "boost/archive/binary_oarchive.hpp"
#include <iostream>     // std::cout
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace CFEAR_Radarodometry {

typedef unsigned long stamp;
// Representing a pose
class Pose3d;

typedef Eigen::Matrix<double, 6, 6> Covariance;

/////// CONVERSION
Pose3d PoseEigToCeres(const Eigen::Affine3d &pose_eig);

Eigen::Affine3d PoseCeresToEig(const Pose3d &pose_ceres);

Pose3d PoseToCeres(double x, double y, double z, double ex, double ey, double ez);

tf::Transform PoseCeresToTf(const Pose3d &pose_ceres);

std::string MatToString(Eigen::MatrixXd& m);


class Pose3d {
public:
  Pose3d(){}
  Pose3d(Eigen::Vector3d transl, Eigen::Quaterniond quat):p(transl),q(quat){}

  Pose3d(const Eigen::Affine3d& T);

  Eigen::Vector3d p;
  Eigen::Quaterniond q;
  // The name of the data type in the g2o file format.
  static std::string name() { return "VERTEX_SE3:QUAT";}

  static Pose3d Identity() {
    Pose3d pidentity;
    pidentity.p<<0,0,0;
    pidentity.q.setIdentity();
    return pidentity;
  }
  Pose3d Inverse();

  Eigen::Affine3d GetPose();

  Pose3d operator*(const Pose3d& other)const // copy assignment
  {
    return(Pose3d(p + q*other.p, q*other.q));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & p;
    ar & q;
  }
};

inline std::istream& operator>>(std::istream& input, Pose3d& pose) {
  input >> pose.p.x() >> pose.p.y() >> pose.p.z() >> pose.q.x() >>
                         pose.q.y() >> pose.q.z() >> pose.q.w();
  // Normalize the quaternion to account for precision loss due to
  // serialization.
  pose.q.normalize();
  return input;
}


class RadarScan //Rename to RadarTrajectory ?
{

public:
  RadarScan(const Eigen::Affine3d& pose_scan, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_peaks, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_nopeaks, const CFEAR_Radarodometry::MapNormalPtr& cloud_normal);

  RadarScan(const Eigen::Affine3d& pose_scan, const Eigen::Affine3d& Tmotion, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_peaks, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_nopeaks, const CFEAR_Radarodometry::MapNormalPtr& cloud_normal, const ros::Time& t);

  RadarScan(const Eigen::Affine3d& pose_scan);

  RadarScan(){}

  const Eigen::Affine3d GetPose() const {return PoseCeresToEig(T); }

  const std::string ToString();

  double GetDistance(int from, int to);

  static unsigned int counter;

  Pose3d T;
  Pose3d Tgt;
  bool has_Tgt_ = false;
  unsigned int idx_;
  stamp stamp_;

  Eigen::Affine3d motion_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_peaks_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_nopeaks_;
  CFEAR_Radarodometry::MapNormalPtr cloud_normal_;


private:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & T;
    ar & Tgt;
    ar & has_Tgt_;
    ar & idx_;
    ar & stamp_;
    ar & motion_;
    ar & cloud_peaks_;
    ar & cloud_nopeaks_;
    ar & cloud_normal_;
  }

};



// The 5 between two vertices in the pose graph. The constraint is the
// transformation from vertex id_begin to vertex id_end.

typedef enum Constrainttype{odometry=0, loop_appearance, mini_loop, candidate} ConstraintType;


std::string Constraint2String(const ConstraintType& c);

struct Constraint3d {

  unsigned long id_begin;

  unsigned long id_end;

  // The transformation that represents the pose of the end frame E w.r.t. the
  // begin frame B. In other words, it transforms a vector in the E frame to
  // the B frame.
  Pose3d t_be;
  // The inverse of the covariance matrix for the measurement. The order of the
  // entries are x, y, z, delta orientation.
  Covariance information;
  // The name of the data type in the g2o file format.

  ConstraintType type;

  std::map<std::string,double> quality;

  std::string info;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & id_begin;
    ar & id_end;
    ar & t_be;
    ar & information;
    ar & type;
    ar & quality;
    ar & info;

  }
};

typedef std::vector< std::pair<RadarScan, std::vector<Constraint3d>> > simple_graph;

void SaveSimpleGraph(const std::string& path, simple_graph &graph);

bool LoadSimpleGraph(const std::string& path, simple_graph &graph);

/*inline std::istream& operator>>(std::istream& input, Constraint3d& constraint) {
  Pose3d& t_be = constraint.t_be;
  input >> constraint.id_begin >> constraint.id_end >> t_be;
  for (int i = 0; i < 6 && input.good(); ++i) {
    for (int j = i; j < 6 && input.good(); ++j) {
      input >> constraint.information(i, j);
      if (i != j) {
        constraint.information(j, i) = constraint.information(i, j);
      }
    }
  }
  return input;
}*/

class ConstraintsHandler{
public:

  typedef std::tuple<unsigned int,unsigned int> ConstraintKey;

  typedef std::map<ConstraintKey, Constraint3d> Constraints;

  ConstraintsHandler();

  void Add(const Constraint3d& c);

  void Add(const RadarScan& from, const RadarScan& to, const Eigen::Affine3d& Tdiff, const Covariance& Cov, ConstraintType ct = ConstraintType::odometry);

  bool HasConstraintType(int id, ConstraintType type);

  bool FindConstraint( unsigned int id1, unsigned int id2, Constraint3d& it, const ConstraintType type = ConstraintType::odometry );

  bool ConstraintExists( unsigned int id1, unsigned int id2, const ConstraintType type = ConstraintType::odometry );

  Constraints::iterator begin(ConstraintType ct = ConstraintType::odometry){ return constraints_[ct].begin(); }

  Constraints::iterator end(ConstraintType ct = ConstraintType::odometry){ return constraints_[ct].end(); }

  size_t size(ConstraintType ct = ConstraintType::odometry){ return constraints_.find(ct)!=constraints_.end() ? std::distance(begin(ct),end(ct)) : 0; }

  double DistanceTraveled(){return dist_trav/(0.1+constraints_[odometry].size());}

  Eigen::Affine3d RelativeMotion( unsigned int id1, unsigned int id2,  const ConstraintType type = ConstraintType::odometry );

  double RelativeDistance( unsigned int id1, unsigned int id2,  const ConstraintType type = ConstraintType::odometry );

  std::string ToString();

  void Statistics(const ConstraintType type = ConstraintType::odometry);

private:

  ConstraintKey MakeKey(unsigned int a, unsigned int b);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & constraints_;
  }

  void CheckValid();

  private:
  double dist_trav = 0;



  std::map<ConstraintType,Constraints> constraints_;
};



typedef std::map<int, RadarScan, std::less<int>,Eigen::aligned_allocator<std::pair<const int, RadarScan> > > RadarScans;

class RadarScanHandler{
public:

  RadarScanHandler(){}

  //RadarScan& operator[] (int index);
  void Add(const RadarScan& scan);

  RadarScan& GetScan(unsigned int id){return RadarScans_[id];}

  double GetDistance(unsigned int from, unsigned int to);

  size_t size()const{return RadarScans_.size();}

  bool NodeExists(unsigned int id){return RadarScans_.find(id)!=RadarScans_.end();}

  RadarScans::iterator Get(unsigned int id){ return RadarScans_.find(id);}

  RadarScans::iterator begin(){return RadarScans_.begin();}

  RadarScans::iterator end(){return RadarScans_.end();}

  RadarScans::reverse_iterator rbegin(){return RadarScans_.rbegin();}

  RadarScans::reverse_iterator rend(){return RadarScans_.rend();}

private:


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & RadarScans_;
    ar & traveled_distance;
  }


  RadarScans RadarScans_;
  double traveled_distance = 0;

};









}  // namespace examples




