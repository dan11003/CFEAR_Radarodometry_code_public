
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
#include <std_msgs/Header.h>

#include <boost/serialization/split_free.hpp>
#include "pcl/io/pcd_io.h"
#include <pcl/point_cloud.h>
#include <boost/serialization/array.hpp>
#include "serialization.h"
#include <pcl/PointIndices.h>
#include <pcl/PCLHeader.h>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include "tuple"
#include "opencv2/core/core.hpp"

#include "boost/serialization/serialization.hpp"
#include "boost/serialization/binary_object.hpp"
namespace boost {
namespace serialization {
class access;
// Serialization of Eigen::Vector2d
/* Already exists in semrob package semrob_generic as a typedef to pose2d */
template<typename Archive>
void serialize(Archive& ar, Eigen::Vector3d& o, const unsigned int version) {
  ar & o[0] & o[1] & o[2];
}
template<typename Archive>
void serialize(Archive& ar, Eigen::Vector2d& o, const unsigned int version) {
  ar & o[0] & o[1];
}

template<typename Archive>
void serialize(Archive & ar, Eigen::Quaterniond& m, const unsigned int /*version*/) {
  ar & m.x();
  ar & m.y();
  ar & m.z();
  ar & m.w();
}

template <class Archive, typename Derived>
 void serialize( Archive & ar, Eigen::EigenBase<Derived> & g, const unsigned int version){
     ar & boost::serialization::make_array(g.derived().data(), g.size());
 }

template<typename Archive>
void serialize(Archive & ar, Eigen::Matrix<double, 3, 3> & m, const unsigned int /*version*/) {
  ar & boost::serialization::make_array(m.data(), 3 * 3);
}
template<typename Archive>
void serialize(Archive & ar, Eigen::Matrix<double, 2, 2> & m, const unsigned int /*version*/) {
  ar & boost::serialization::make_array(m.data(), 2 * 2);
}
template<typename Archive>
void serialize(Archive & ar, Eigen::Matrix<double, 6, 6> & m, const unsigned int /*version*/) {
  ar & boost::serialization::make_array(m.data(), 6 * 6);
}

template<typename Archive>
void serialize(Archive& ar, std::tuple<unsigned int, unsigned int>& t, const unsigned int /*version*/) {
  ar & std::get<0>(t);
  ar & std::get<1>(t);
}

template<class Archive , typename T1 , typename T2 >
void serialize(Archive & ar, std::tuple<T1,T2>& t, const unsigned int){
  ar & std::get<0>(t);
  ar & std::get<1>(t);

}
#ifdef MATRIX_D31
#define MATRIX_D31
template<typename Archive>
void serialize(Archive & ar, Eigen::Matrix<double, 3, 1> & m, const unsigned int /*version*/) {
  ar & boost::serialization::make_array(m.data(), 3 * 1);
}
#endif
/*
template<typename Archive>
void serialize(Archive& ar, std::vector<pcl::PointXYZ,Eigen::aligned_allocator<pcl::PointXYZ> > points, const unsigned version) {

  for(int i=0; i<points.size();i++)
    ar & points[i];
}*/
/*template<class Archive>
  void serialize(Archive & ar, pcl::PCLHeader & g, const unsigned int version)
  {
    ar & g.seq;
    ar & g.stamp;
    ar & g.frame_id;
  }*/

template<typename Archive,typename T>
void serialize(Archive& ar,  pcl::PointCloud<T> &points, const unsigned version) {
  ar & points.header.stamp;
  ar & points.header.seq;
  ar & points.header.frame_id;
  ar & points.height;
  ar & points.width;
  points.resize(points.height*points.width);
  for(int i=0; i<points.size();i++){
    ar & points[i];
  }
}

template<typename Archive>
void serialize(Archive& ar, pcl::PointXYZ &point, const unsigned version) {
  ar & point.data;
}
template<typename Archive>
void serialize(Archive& ar, pcl::PointXY &point, const unsigned version) {
  ar & point.x;
  ar & point.y;
}
template<typename Archive>
void serialize(Archive& ar, pcl::PointXYZI &point, const unsigned version) {
  ar & point.data;
  ar & point._PointXYZI::intensity;
}


template<typename Archive>
void serialize(Archive& ar, Eigen::Affine3d &o, const unsigned version) {
  for (int i = 0; i < 16; i++) {
    ar & o.data()[i];
  }
}



/**
 * Serialisation for the OpenCV cv::Mat class.
 *
 * Based on the answer from: http://stackoverflow.com/questions/4170745/serializing-opencv-mat-vec3f
 * And the blog post on: http://cheind.wordpress.com/2011/12/06/serialization-of-cvmat-objects-using-boost/
 */


template<class Archive>
void serialize(Archive& ar, cv::Mat& mat, const unsigned int /*version*/)
{
  int rows, cols, type;
  bool continuous;

  if (Archive::is_saving::value) {
    rows = mat.rows;
    cols = mat.cols;
    type = mat.type();
    continuous = mat.isContinuous();
  }

  ar & BOOST_SERIALIZATION_NVP(rows) & BOOST_SERIALIZATION_NVP(cols) & BOOST_SERIALIZATION_NVP(type) & BOOST_SERIALIZATION_NVP(continuous);

  if (Archive::is_loading::value)
    mat.create(rows, cols, type);

  if (continuous) {
    const int data_size = rows * cols * static_cast<int>(mat.elemSize());
    boost::serialization::binary_object mat_data(mat.data, data_size);
    ar & BOOST_SERIALIZATION_NVP(mat_data);
  }
  else {
    const int row_size = cols * static_cast<int>(mat.elemSize());
    for (int i = 0; i < rows; i++) {
      boost::serialization::binary_object row_data(mat.ptr(i), row_size);
      std::string row_name("mat_data_row_" + std::to_string(i));
      ar & make_nvp(row_name.c_str(), row_data);
    }
  }
};


}
}






