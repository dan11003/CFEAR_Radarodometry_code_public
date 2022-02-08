#include "cfear_radarodometry/pointnormal.h"
namespace CFEAR_Radarodometry {

std::map<std::string,ros::Publisher> MapPointNormal::pubs;
double MapPointNormal::downsample_factor = 1;


MapPointNormal::MapPointNormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cld, float radius, const Eigen::Vector3d& origin, bool raw)  {

  input_ = cld;
  downsampled_ = pcl::PointCloud<pcl::PointXY>::Ptr(new pcl::PointCloud<pcl::PointXY>());

  radius_ = radius;
  //cout<<"create map: "<<cld.size()<<endl;
  if(input_->size()==0){
    cout<<"error, cloud empty"<<endl;
    exit(0);
  }
  if(raw){
    for(auto && p : input_->points){
      Eigen::Vector3d u;
      u << p.x, p.y, 0;
      Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
      cells.push_back(cell(u, cov, origin));
      //cout<<"compute mean"<<endl;
    }
  }
  else
    ComputeNormals(origin);

  ComputeSearchTreeFromCells();
    CFEAR_Radarodometry::timing.Document("Surface points",cells.size());
  //cout<<"created normal: input: "<<cld.size()<<" -> "<<cells.size()<<endl;

}
MapPointNormal::MapPointNormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cld, float radius, std::vector<cell>& cell_orig, const Eigen::Affine3d& T){

  input_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  downsampled_ = pcl::PointCloud<pcl::PointXY>::Ptr(new pcl::PointCloud<pcl::PointXY>());
  radius_ = radius;
  pcl::transformPointCloud(*cld, *input_, T); // transform to new reference frame

  for(auto&& c : cell_orig)
    cells.push_back( c.TransformCopy(T) );

  for(auto && c : cells){
    pcl::PointXY pxy;
    pxy.x = c.u_(0);
    pxy.y = c.u_(1);
    downsampled_->push_back(pxy);
  }

  kd_cells.setInputCloud(downsampled_);
}


MapNormalPtr MapPointNormal::TransformMap(const Eigen::Affine3d& T){
  return MapNormalPtr( new MapPointNormal(input_, radius_, cells, T) );
}


inline double MapPointNormal::Gausian(const double x, const double u, const double sigma){
  double sqr = (u-x)*(u-x);
  return exp(-sqr/(2*sigma*sigma));
}

void MapPointNormal::ComputeSearchTreeFromCells(){
  downsampled_->clear();
  if(cells.empty()){
      cells.push_back(cell::GetDefault());
  }
  for(auto && c : cells){
    pcl::PointXY pxy;
    pxy.x = c.u_(0);
    pxy.y = c.u_(1);
    downsampled_->push_back(pxy);
  }

  kd_cells.setSortedResults(true);
  kd_cells.setInputCloud(downsampled_);
}
bool MapPointNormal::ComputeMeanAndNormals(const std::vector<int>& pointIdxNKNSearch, Eigen::Vector3d& u, Eigen::Matrix3d& Cov){

  //double sigma = 50;
  Eigen::Vector2d tot(0,0);
  Eigen::MatrixXd w(pointIdxNKNSearch.size(), 1);
  Eigen::MatrixXd x(pointIdxNKNSearch.size(), 2);

  double max_intensity = 0;
  for(int i=0 ; i<pointIdxNKNSearch.size() ; i++){ // build sample and weight block
    x.block<1,2>(i,0) = Eigen::Vector2d(input_->points[pointIdxNKNSearch[i]].x, input_->points[pointIdxNKNSearch[i]].y);
    //cout<<"x: "<<x.block<1,2>(i,0)<<endl;
    //double intensity = cld->points[pointIdxNKNSearch[i]].intensity;
    //w(i,0) = input_intensity_->points[pointIdxNKNSearch[i]].intensity;

    //if(intensity>max_intensity)
    //  max_intensity = intensity;
  }
  //cout<<"w: "<<w<<endl;

  //for(int i=0;i<pointIdxNKNSearch.size();i++)
  //  w(i,0) = Gausian(w(i,0), max_intensity, sigma); //reassign weight by intensity difference from max
  //w = w/w.sum();    //normalize weights
  u = Eigen::Vector3d(0,0,0);
  //for(int i=0;i<x.rows();i++) // calculate mean by weighting
  // u+= x.block<1,2>(i,0)*w(i,0);


  for(int i=0;i<x.rows();i++) // calculate mean by weighting
    u.block<2,1>(0,0) += x.block<1,2>(i,0).transpose();
  u = u/pointIdxNKNSearch.size();
  //cout<<"u: "<<u<<endl;

  for(int i=0;i<pointIdxNKNSearch.size();i++) // subtract mean
    x.block<1,2>(i,0) = x.block<1,2>(i,0) - u.block<2,1>(0,0).transpose();


  return WeightedCovariance(w, x, Cov);
  //cout<<"u2: "<<u<<endl;
  //cout<<"w: "<<w<<endl;
  //cout<<"cov: "<<Cov<<endl;

}

bool MapPointNormal::WeightedCovariance(Eigen::MatrixXd& w, Eigen::MatrixXd& x, Eigen::Matrix3d& cov){ //mean already subtracted from x
  Eigen::Matrix2d covSum = x.transpose()*x;
  float n = x.rows();
  cov.block<2,2>(0,0) = covSum*1.0/(n-1.0);
  cov(2,2) = 1;
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov.block<2,2>(0,0));
  double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
  //cout<<"cond "<<cond<<endl;
  return cond < 100000;
}

/*std::vector<cell*> MapPointNormal::GetClosest(ce c, double d){
  if(cell!=NULL)
    return GetClosest(cell->u_, d);
  else return std::vector<cell*>();
}*/
std::vector<int> MapPointNormal::GetClosestIdx(const Eigen::Vector2d&  p, double d){
  Eigen::Vector3d p3d;
  p3d<<p(0),p(1),0;
  return GetClosestIdx(p3d,d);
}

std::vector<int> MapPointNormal::GetClosestIdx(const Eigen::Vector3d&  p, double d){

  pcl::PointXY pnt;
  pnt.x = p(0);
  pnt.y = p(1);

  std::vector<int> pointIdxNKNSearchs;
  std::vector<int> sub_index;
  std::vector<float> pointNKNSquaredDistances;
  kd_cells.setSortedResults(true);
  kd_cells.radiusSearchT (pnt, d, pointIdxNKNSearchs, pointNKNSquaredDistances);
  /*
  kd_cells.nearestKSearch(pnt, 1, pointIdxNKNSearchs, pointNKNSquaredDistances);
  if(!pointIdxNKNSearchs.empty() && pointNKNSquaredDistances[0]<d*d )
    return pointIdxNKNSearchs;
  else
    return std::vector<int>();*/


  return pointIdxNKNSearchs;
}

std::vector<cell*> MapPointNormal::GetClosest(Eigen::Vector3d&  p, double d){
  std::vector<cell*> nearby;
  std::vector<int> close_idx = GetClosestIdx(p,d);
  for(auto idx : close_idx)
    nearby.push_back(&cells[idx]);
  return nearby;
}


void MapPointNormal::ComputeNormals(const Eigen::Vector3d& origin){
  if(input_->size()==0)
    cerr<<"cloud size error"<<endl;

  pcl::search::KdTree<pcl::PointXYZI> kdt_input;
  pcl::PointCloud<pcl::PointXYZI> cell_sample_means;

  input_->width = input_->points.size();
  input_->height = 1;
  kdt_input.setInputCloud(input_);

  //cout<<"downsample"<<endl;
  pcl::VoxelGrid<pcl::PointXYZI> sor; //downsample
  sor.setInputCloud (input_);
  sor.setLeafSize (radius_/downsample_factor, radius_/downsample_factor, radius_/downsample_factor);
  sor.filter (cell_sample_means);
  if(cell_sample_means.size()<3)
    std::cerr<<"Error, downsampeld cloud empty"<<endl;

  //file_input.open ("/home/daniel/Documents/input.txt");
  //file_downsampled.open("/home/daniel/Documents/downsampled.txt");

  //pcl::io::savePCDFileASCII("/home/daniel/Documents/input.pcd", *input_);
  //pcl::io::savePCDFileASCII("/home/daniel/Documents/downsampled.pcd", cell_sample_means);

  //cout<<"input:"<<input_->size()<<endl;

  //cout<<"loop"<<endl;
  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;
  for(int i=0;i<cell_sample_means.size();i++){


    pointIdxNKNSearch.clear();
    pointNKNSquaredDistance.clear();
    //cout<<"cleared arrs"<<endl;
    pcl::PointXYZI pnt = cell_sample_means.points[i];
    //double d = sqrt( pnt.x*pnt.x+pnt.y*pnt.y);
    //cout<<"got pnt"<<endl;
    //cout<<pnt.x<<" "<<pnt.y<<" "<<pnt.z<<endl;

    //cout<<"i="<<i<<", "<<pnt.x<<" "<<pnt.y<<" "<<pnt.z<<endl;
    //cout<<i<<": "<<pnt.x<<","<<pnt.y<<","<<pnt.z<<","<<pnt.intensity<<endl;
    //cout<<"search"<<endl;
    //  double r = 2+5*(d/150.0); // 5+0.3m at at 150m
    if ( kdt_input.radiusSearchT (pnt, radius_, pointIdxNKNSearch, pointNKNSquaredDistance) >=6 ){
      Eigen::Vector3d u;
      Eigen::Matrix3d cov;
      //cout<<"compute mean"<<endl;
      if(ComputeMeanAndNormals(pointIdxNKNSearch, u, cov))
        cells.push_back(cell(u, cov, origin));
    }
  }
  //cout<<"end of loop"<<endl;
  //cerr<<"done normals"<<endl;
}

Eigen::MatrixXd MapPointNormal::GetNormals(){
  Eigen::MatrixXd normals(3,cells.size());

  for(int i=0;i<cells.size();i++)
    normals.block<3,1>(0,i) = cells[i].snormal_.block<3,1>(0,0);
  return normals;
}

Eigen::MatrixXd MapPointNormal::GetNormals2d(){
  Eigen::MatrixXd normals(2,cells.size());

  for(int i=0;i<cells.size();i++)
    normals.block<2,1>(0,i) = cells[i].snormal_.block<2,1>(0,0);
  return normals;
}

std::vector<Eigen::Matrix3d> MapPointNormal::GetCovs(){
  std::vector<Eigen::Matrix3d> covs(cells.size());

  for(int i=0;i<cells.size();i++)
    covs[i] = cells[i].cov_;
  return covs;
}
Eigen::MatrixXd MapPointNormal::GetMeans(){
  Eigen::MatrixXd means(3,cells.size());

  for(int i=0;i<cells.size();i++)
    means.block<3,1>(0,i) = cells[i].u_.block<3,1>(0,0);
  return means;
}

Eigen::MatrixXd MapPointNormal::GetMeans2d(){
  Eigen::MatrixXd means(2,cells.size());

  for(int i=0;i<cells.size();i++)
    means.block<2,1>(0,i) = cells[i].u_.block<2,1>(0,0);
  return means;
}



Eigen::MatrixXd MapPointNormal::GetCloudTransformed(const Eigen::Affine3d& Toffset){
  Eigen::MatrixXd pnts(3,input_->size());
  for(int i=0; i<input_->size();i++){
    Eigen::Vector3d v;
    v<<input_->points[i].x, input_->points[i].y, input_->points[i].z;
    pnts.block<3,1>(0,i) = Toffset*v;
  }
  return pnts;
}
Eigen::MatrixXd MapPointNormal::GetScales(){
  Eigen::MatrixXd scales(1,cells.size());
  for(int i=0;i<cells.size();i++)
    scales(0,i) = cells[i].scale_;
  return scales;
}
std::vector<cell> MapPointNormal::TransformCells(const Eigen::Affine3d& T){
  std::vector<cell> transformed;
  for(auto && c : cells){
    //cout<<"before: "<<c.u_.transpose()<<endl;
    cell ct = c.TransformCopy(T);
    //cout<<"after: "<<ct.u_.transpose()<<endl;
    transformed.push_back(ct);
  }
  return transformed;
}
void MapPointNormal::Transform(const Eigen::Affine3d& T, Eigen::MatrixXd& means, Eigen::MatrixXd& normals){
  means = GetMeans();
  normals = GetNormals();
  for(int i=0;i<means.cols();i++)
    means.block<3,1>(0,i) = T.linear()*means.block<3,1>(0,i)+T.translation();
  normals = T.linear()*normals;
}

void MapPointNormal::Transform2d(const Eigen::Affine3d& T, Eigen::MatrixXd& means, Eigen::MatrixXd& normals){
  means = GetMeans2d();
  normals = GetNormals2d();
  for(int i=0;i<means.cols();i++)
    means.block<2,1>(0,i) = T.linear().block<2,2>(0,0)*means.block<2,1>(0,i)+T.translation().block<2,1>(0,0);
  normals = T.linear().block<2,2>(0,0)*normals;
}

inline pcl::PointXYZI Pnt(Eigen::Vector3d& u){
  pcl::PointXYZI p;
  p.x = u(0);
  p.y = u(1);
  p.z = 0;
  p.intensity = 0;
  return p;
}

inline pcl::PointXYZ PntXYZ(Eigen::Vector3d& u){
  pcl::PointXYZ p;
  p.x = u(0);
  p.y = u(1);
  p.z = u(2);
  return p;
}



visualization_msgs::Marker DefaultMarker( const ros::Time& time, const std::string& frame){
  visualization_msgs::Marker marker;

  marker.header.frame_id = frame;
  marker.header.stamp = time;

  marker.ns = "point_cloud";
  marker.id = 0;

  marker.type = visualization_msgs::Marker::ARROW;
  marker.lifetime = ros::Duration(0.0);

  marker.action = visualization_msgs::Marker::ADD;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.2;

  return marker;
}
void intToRGB(const int value,float& red,float& green, float& blue) {
  //credit to https://stackoverflow.com/a/2262117/2737978 for the idea of how to implement
  //blue = std::floor(value % 256);
  //green = std::floor(value / 256 % 256);
  //red = std::floor(value / 256 / 256 % 256);
  uint8_t color = value%255;

  // so you take 2 bits and multiply by 64 to possibly have intensities: 0, 64, 128, 192
  red = (((color & 0xC0) >> 6) * 64)/255.0;
  green = (((color & 0x30) >> 4) * 64)/255.0;
  blue = (((color & 0x0C) >> 2) * 64)/255.0;

  //cout<< "rgb(" << red << "," << green << "," << blue << ")";
}


visualization_msgs::MarkerArray Cells2Markers(std::vector<cell>& cells, const ros::Time& time, const std::string& frame, int val){
  visualization_msgs::MarkerArray marr;
  visualization_msgs::Marker m = DefaultMarker(time, frame);

  for(int i=0 ; i<cells.size() ; i++){
    m.points.clear();
    if(val==-1){
      m.color.r = 1;
      m.color.g = 0;
      m.color.b = 0;
      m.color.a = 1;
    }
    else
      intToRGB(val, m.color.r, m.color.g, m.color.b);
    Eigen::Vector3d u = cells[i].u_;
    double d = cells[i].scale_;
    Eigen::Vector3d end = u + 3*cells[i].snormal_;//*d;//+ cells[i].snormal_*log(d/2);
    geometry_msgs::Point p;
    p.x = u(0);
    p.y = u(1);
    p.z = u(2);
    m.points.push_back(p);
    p.x = end(0);
    p.y = end(1);
    p.z = end(2);
    m.points.push_back(p);
    m.id = i;
    m.ns = "";
    //m.pose.orientation.w = 1;
    //m.pose.orientation.z = 1;
    //m.pose.orientation.y = 0;
    //m.pose.orientation.x = 0;
    marr.markers.push_back(m);
  }
  return marr;
}

cell::cell(const Eigen::Vector3d& u, const Eigen::Matrix3d& cov, const Eigen::Vector3d& origin ) {

  Eigen::Matrix2d c = cov.block<2,2>(0,0);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(c);
  es.compute(c);
  snormal_ = Eigen::Vector3d(0,0,0);
  snormal_.block<2,1>(0,0) = es.eigenvectors().col(0);
  double ratio = es.eigenvalues()[1]/(es.eigenvalues()[0]+0.000001); //regularization shoudn't be needed here if condition number is previously controlled for
  scale_ = log(ratio/2);
  //u_(2) = GetAngle();
  u_ = u;
  u_(2) = 0;
  cov_ = cov;
  Eigen::Vector3d Po_u = origin - u_;
  if(snormal_.dot(Po_u)<0)
    snormal_ = - snormal_;
}
cell::cell(const Eigen::Vector3d& u, const Eigen::Matrix3d& cov, const Eigen::Vector3d& normal, const double scale){
  u_ = u;
  cov_ = cov;
  snormal_ = normal;
  scale_ = scale; // scale already set?
}

cell cell::TransformCopy(const Eigen::Affine3d& T){
  Eigen::Matrix3d R = T.linear();
  Eigen::Matrix3d C = R*T*cov_*R.transpose();
  Eigen::Vector3d N = R*snormal_;
  Eigen::Vector3d U = T*u_;
  //U(2) = std::cos(GetAngle());
  return cell(U, C, N, scale_);
}
double cell::GetAngle(){
  cout<<"normal: "<<snormal_<<endl;
  double alpha = atan2(snormal_(1), snormal_(0));
  return alpha;
}
cell cell::GetDefault(){
    Eigen::Vector3d normal;
    normal<<1,0.1,0;
    Eigen::Matrix3d cov = normal.asDiagonal();
    cell c(Eigen::Vector3d::Identity(), cov);
    return c;
}
std::pair<Eigen::Vector3d,Eigen::Vector3d> cell::TransformMeanNormal(const Eigen::Affine3d& T){
  return std::make_pair(T*u_,T*snormal_);
}

void MapPointNormal::PublishMap(const std::string& topic, MapNormalPtr map, Eigen::Affine3d& T, const std::string& frame_id, const int value){
  if(map==NULL)
    return;

  std::map<std::string, ros::Publisher>::iterator it = MapPointNormal::pubs.find(topic);
  if (it == pubs.end()){
    ros::NodeHandle nh("~");
    pubs[topic] =  nh.advertise<visualization_msgs::MarkerArray>(topic,100);
    it = MapPointNormal::pubs.find(topic);
  }
  //cout<<"publish to "<<topic<<endl;
  visualization_msgs::Marker m = DefaultMarker(ros::Time::now(), frame_id);
  m.action = visualization_msgs::Marker::DELETEALL;
  visualization_msgs::MarkerArray marr_delete;
  marr_delete.markers.push_back(m);
  it->second.publish(marr_delete);
  std::vector<cell> cells = map->TransformCells(T);
  visualization_msgs::MarkerArray marr = Cells2Markers(cells, ros::Time::now(), frame_id, value);
  it->second.publish(marr);
}

/*void cell::ToNDTMsg(std::vector<cellptr>& cells, ndt_map::NDTMapMsg& msg){

  std::vector<perception_oru::NDTCell*> ndt_cells;
  for(auto && c : cells){
    perception_oru::NDTCell* ndt_cell = new perception_oru::NDTCell();
    Eigen::Matrix3d Cov;
    Cov.block<3,3>(0,0) = c->cov_;
    ndt_cell->setCov(Cov);
    Eigen::Vector3d u(c->u_(0),c->u_(1), c->u_(2) );
    ndt_cell->setCenter(pcl::PointXYZ(u(0), u(1), u(2)));
    ndt_cell->setMean(c->u_);
    ndt_cell->hasGaussian_ = true;
    ndt_cells.push_back(ndt_cell);
  }
  perception_oru::toMessage(ndt_cells, msg, "navtech");
}*/

}
