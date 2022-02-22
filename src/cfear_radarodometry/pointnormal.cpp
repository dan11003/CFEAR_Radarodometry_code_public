#include "cfear_radarodometry/pointnormal.h"
namespace CFEAR_Radarodometry {

std::map<std::string,ros::Publisher> MapPointNormal::pubs;
double MapPointNormal::downsample_factor = 1;

cell::cell(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, const std::vector<int>& pointIdxNKNSearch, const bool weight_intensity, const Eigen::Vector2d& origin) : Nsamples_(pointIdxNKNSearch.size()) {
  // Compute Covariance
  Eigen::Vector2d tot(0, 0);
  Eigen::MatrixXd w(Nsamples_, 1);
  Eigen::MatrixXd x(Nsamples_, 2);

  for(size_t i=0 ; i<Nsamples_ ; i++){ // build sample and weight block
    x.block<1,2>(i,0) = Eigen::Vector2d(input->points[pointIdxNKNSearch[i]].x, input->points[pointIdxNKNSearch[i]].y);
    w(i,0) = weight_intensity ? std::max(input->points[pointIdxNKNSearch[i]].intensity-60.0,0.0) : 1.0;
  }

  sum_intensity_ = w.sum();
  avg_intensity_ = sum_intensity_/Nsamples_;

  w = w/sum_intensity_; // normalize sum = 1.0

  for(Eigen::Index i=0 ; i<Nsamples_ ; i++) // calculate mean by weighting, w sums to one, no need to normalize after
    u_.block<2,1>(0,0) += w(i,0)*x.block<1,2>(i,0).transpose();

  for(Eigen::Index i=0 ; i<Nsamples_ ; i++) // subtract mean
    x.block<1,2>(i,0) = x.block<1,2>(i,0) - u_.block<2,1>(0,0).transpose(); //

  Eigen::MatrixXd x_weighted(x.rows(),x.cols()); // Weighted
  for(int i = 0; i < x.rows() ; i++)
    x_weighted.block<1,2>(i,0) = w(i,0)*x.block<1,2>(i,0);

  cov_ = x.transpose()*x_weighted; // weighted

  valid_ = ComputeNormal(origin);
}
bool cell::ComputeNormal(const Eigen::Vector2d& origin)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov_);
  es.compute(cov_);

  snormal_ = es.eigenvectors().col(0);
  orth_normal = es.eigenvectors().col(1);
  lambda_min = es.eigenvalues()[0];
  lambda_max = es.eigenvalues()[1];
  const double condition_number = lambda_max/lambda_min;
  const double determinant = lambda_max*lambda_min;
  const double det_tolerance = 0.00001;
  const bool cov_reasonable = (condition_number <= 10000) && (determinant > det_tolerance); // one side is not unproportionally larger than the other and matrix is positive semidefinite
  scale_ = log(condition_number/2.0) ; //Used for visualization - ranges from 0.1

  Eigen::Vector2d Po_u = origin - u_;
  if(snormal_.dot(Po_u)<0)
    snormal_ = - snormal_;
  return cov_reasonable;
}

MapPointNormal::MapPointNormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cld, float radius, const Eigen::Vector2d& origin, const bool weight_intensity, const bool raw) : weight_intensity_(weight_intensity)  {

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
      Eigen::Vector2d u(p.x,p.y);
      Eigen::Matrix2d cov = Eigen::Matrix2d::Identity();
      cells.push_back(cell::GetIdentityCell(u,p.intensity));
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

  const Eigen::Affine2d T2d = Eigen::Translation2d(T.translation().topRows<2>()) *
      T.linear().topLeftCorner<2,2>();
  for(auto&& c : cell_orig)
    cells.push_back( c.TransformCopy(T2d) );

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

double MapPointNormal::GetCellRelTimeStamp(const size_t index, const bool ccw){
  const double x = cells[index].u_(0);
  const double y = cells[index].u_(1);
  return CFEAR_Radarodometry::GetRelTimeStamp(x, y, ccw);
}


inline double MapPointNormal::Gausian(const double x, const double u, const double sigma){
  double sqr = (u-x)*(u-x);
  return exp(-sqr/(2*sigma*sigma));
}

void MapPointNormal::ComputeSearchTreeFromCells(){
  downsampled_->clear();
  for(auto && c : cells){
    pcl::PointXY pxy;
    pxy.x = c.u_(0);
    pxy.y = c.u_(1);
    downsampled_->push_back(pxy);
  }

  kd_cells.setSortedResults(true);
  kd_cells.setInputCloud(downsampled_);
}
//Return true if succesfull
/*bool MapPointNormal::ComputeMeanAndNormals(const std::vector<int>& pointIdxNKNSearch, Eigen::Vector2d& u, Eigen::Matrix2d& Cov, double& avg_intensity, int &Nsamples ){

  Nsamples = pointIdxNKNSearch.size();
  Eigen::Vector2d tot(0, 0);
  Eigen::MatrixXd w(Nsamples, 1);
  Eigen::MatrixXd x(Nsamples, 2);


  double max_intensity = 0;
  for(int i=0 ; i<Nsamples ; i++){ // build sample and weight block
    x.block<1,2>(i,0) = Eigen::Vector2d(input_->points[pointIdxNKNSearch[i]].x, input_->points[pointIdxNKNSearch[i]].y);
    //w(i,0) = 1.0/x.rows();
    w(i,0) = std::max(input_->points[pointIdxNKNSearch[i]].intensity-60.0,10.0);
  }

  const double w_sum = w.sum();
  avg_intensity = w_sum/Nsamples;
  w = w/w_sum; // Manhattan
  //const double w_sum = x.rows();
  //w = w/x.rows(); // Manhattan

  //w.normalize();   // Euclidian
  //cout<<"w: "<<w<<endl;

  u = Eigen::Vector2d(0,0);
  for(int i=0 ; i<Nsamples ; i++){ // calculate mean by weighting, w sums to one, no need to normalize after
    u.block<2,1>(0,0) += w(i,0)*x.block<1,2>(i,0).transpose();
    //u.block<2,1>(0,0) += x.block<1,2>(i,0).transpose();
  }
  //u = u/w_sum;

  for(int i=0 ; i<avg_intensity ; i++){ // subtract mean
    //x.block<1,2>(i,0) = x.block<1,2>(i,0) - u.block<2,1>(0,0).transpose();
    x.block<1,2>(i,0) = x.block<1,2>(i,0) - u.block<2,1>(0,0).transpose();
  }
  return WeightedCovariance(w, x, Cov);
}*/
/*
bool MapPointNormal:: WeightedCovariance(Eigen::MatrixXd& w, Eigen::MatrixXd& x, Eigen::Matrix2d& cov){ //mean already subtracted from x

  Eigen::MatrixXd x_weighted(x.rows(),x.cols()); // Weighted
  for(int i = 0; i < x.rows() ; i++){
    x_weighted.block<1,2>(i,0) = w(i,0)*x.block<1,2>(i,0);
    //x_weighted.block<1,2>(i,1) = w(i,0)*x_weighted.block<1,2>(i,1);
  }
  //const double unbiased_factor = 1.0/(x.rows()-1.0);
  const double unbiased_factor = 1.0; //uniform//weighted

  Eigen::Matrix2d covSum = unbiased_factor*x.transpose()*x_weighted; // weighted
  //Eigen::Matrix2d covSum = unbiased_factor*x.transpose()*x;
  //float n = x.rows();
  cov = Eigen::Matrix2d::Identity()*0.1;
  cov.block<2,2>(0,0) = covSum;
  //cov(2,2) = 1;
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov.block<2,2>(0,0));
  const double cond = svd.singularValues()(0)/svd.singularValues()(1);
  const double det = covSum.determinant();
  const double det_tolerance = 0.00001;
  //cout<<"cond "<<cond<<endl;
  const bool cov_reasonable = (cond <= 10000) && (det > det_tolerance); // one side is not unproportionally larger than the other and matrix is positive semidefinite
  //if(!cov_reasonable)
  //  cout<<"problem: cond: "<<cond<<", det: "<<det<<endl;
  //else
  //  cout<<"ok: cond: "<<cond<<", det: "<<det<<endl;
  return cov_reasonable;
}
*/
/*std::vector<cell*> MapPointNormal::GetClosest(ce c, double d){
  if(cell!=NULL)
    return GetClosest(cell->u_, d);
  else return std::vector<cell*>();
}*/


std::vector<int> MapPointNormal::GetClosestIdx(const Eigen::Vector2d&  p, double d){

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

std::vector<cell*> MapPointNormal::GetClosest(Eigen::Vector2d&  p, double d){
  std::vector<cell*> nearby;
  std::vector<int> close_idx = GetClosestIdx(p,d);
  for(auto idx : close_idx)
    nearby.push_back(&cells[idx]);
  return nearby;
}


void MapPointNormal::ComputeNormals(const Eigen::Vector2d& origin){
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

  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;
  for(int i=0;i<cell_sample_means.size();i++){
    pointIdxNKNSearch.clear();
    pointNKNSquaredDistance.clear();
    pcl::PointXYZI pnt = cell_sample_means.points[i];

    if ( kdt_input.radiusSearchT (pnt, radius_, pointIdxNKNSearch, pointNKNSquaredDistance) >=6 ){
      cell c = cell(input_, pointIdxNKNSearch, weight_intensity_, origin);
      if(c.valid_)
        cells.push_back(c);
    }
  }
}

/*Eigen::MatrixXd MapPointNormal::GetNormals(){
  Eigen::MatrixXd normals(2,cells.size());

  for(int i=0;i<cells.size();i++)
    normals.block<2,1>(0,i) = cells[i].snormal_.block<2,1>(0,0);
  return normals;
}*/

Eigen::MatrixXd MapPointNormal::GetNormals2d(){
  Eigen::MatrixXd normals(2,cells.size());

  for(int i=0;i<cells.size();i++)
    normals.block<2,1>(0,i) = cells[i].snormal_.block<2,1>(0,0);
  return normals;
}

std::vector<Eigen::Matrix2d> MapPointNormal::GetCovs(){
  std::vector<Eigen::Matrix2d> covs(cells.size());

  for(int i=0;i<cells.size();i++)
    covs[i] = cells[i].cov_;
  return covs;
}
/*Eigen::MatrixXd MapPointNormal::GetMeans(){
  Eigen::MatrixXd means(3,cells.size());

  for(int i=0;i<cells.size();i++)
    means.block<3,1>(0,i) = cells[i].u_.block<3,1>(0,0);
  return means;
}*/

Eigen::MatrixXd MapPointNormal::GetMeans2d(){
  Eigen::MatrixXd means(2,cells.size());

  for(int i=0;i<cells.size();i++)
    means.block<2,1>(0,i) = cells[i].u_.block<2,1>(0,0);
  return means;
}

Eigen::MatrixXd MapPointNormal::GetCloudTransformed(const Eigen::Affine3d& Toffset){
  Eigen::MatrixXd pnts(3,input_->size());
  for(int i=0; i<input_->size();i++){
    Eigen::Vector3d v(input_->points[i].x, input_->points[i].y, input_->points[i].z);
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
  const Eigen::Affine2d T2d = Eigen::Translation2d(T.translation().topRows<2>()) *
      T.linear().topLeftCorner<2,2>();
  for(auto && c : cells){
    cell ct = c.TransformCopy(T2d);
    transformed.push_back(ct);
  }
  return transformed;
}
void MapPointNormal::Transform(const Eigen::Affine3d& T, Eigen::MatrixXd& means, Eigen::MatrixXd& normals){
  means = GetMeans2d();
  normals = GetNormals2d();
  const Eigen::Affine2d T2d = Eigen::Translation2d(T.translation().topRows<2>()) *
      T.linear().topLeftCorner<2,2>();
  for(int i=0;i<means.cols();i++)
    means.block<2,1>(0,i) = T2d.linear()*means.block<2,1>(0,i)+T2d.translation();
  normals = T.linear()*normals;
}

void MapPointNormal::Transform2d(const Eigen::Affine3d& T, Eigen::MatrixXd& means, Eigen::MatrixXd& normals){
  means = GetMeans2d();
  normals = GetNormals2d();
  for(int i=0;i<means.cols();i++)
    means.block<2,1>(0,i) = T.linear().block<2,2>(0,0)*means.block<2,1>(0,i)+T.translation().block<2,1>(0,0);
  normals = T.linear().block<2,2>(0,0)*normals;
}

inline pcl::PointXYZI Pnt(Eigen::Vector2d& u){
  pcl::PointXYZI p;
  p.x = u(0);
  p.y = u(1);
  p.z = 0;
  p.intensity = 0;
  return p;
}

inline pcl::PointXYZ PntXYZ(Eigen::Vector2d& u){
  pcl::PointXYZ p;
  p.x = u(0);
  p.y = u(1);
  p.z = 0;
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
    Eigen::Vector2d u = cells[i].u_;
    double d = cells[i].scale_;
    //Eigen::Vector2d end = u + 3*cells[i].snormal_;//*d;//+ cells[i].Affine3d*log(d/2);
    Eigen::Vector2d end = u + cells[i].snormal_*d;
    geometry_msgs::Point p;
    p.x = u(0);
    p.y = u(1);
    p.z = 0;
    m.points.push_back(p);
    p.x = end(0);
    p.y = end(1);
    p.z = 0;
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

/*cell::cell(const Eigen::Vector2d& u, const Eigen::Matrix2d& cov, const Eigen::Vector2d& origin ) {

  Eigen::Matrix2d c = cov.block<2,2>(0,0);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(c);
  es.compute(c);
  snormal_ = Eigen::Vector2d(0,0);
  snormal_.block<2,1>(0,0) = es.eigenvectors().col(0);
  double ratio = es.eigenvalues()[1]/(es.eigenvalues()[0]+0.000001); //regularization shoudn't be needed here if condition number is previously controlled for
  scale_ = log(ratio/2);
  //u_(2) = GetAngle();
  u_ = u;
  u_(2) = 0;
  cov_ = cov;
  Eigen::Vector2d Po_u = origin - u_;
  if(snormal_.dot(Po_u)<0)
    snormal_ = - snormal_;
}*/


cell cell::TransformCopy(const Eigen::Affine2d& T){
  Eigen::Matrix2d R = T.linear();
  Eigen::Matrix2d C = R*T*cov_*R.transpose();
  Eigen::Vector2d N = R*snormal_;
  Eigen::Vector2d N_orth = R*orth_normal;
  Eigen::Vector2d U = T*u_;
  //U(2) = std::cos(GetAngle());
  cell c(*this);
  c.u_ = U;
  c.cov_ = C;
  c.snormal_ = N;
  c.orth_normal = N_orth;

  return c;
}
double cell::GetAngle(){
  cout<<"normal: "<<snormal_<<endl;
  double alpha = atan2(snormal_(1), snormal_(0));
  return alpha;
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
  for(auto && c : cells){
    //cout<<"u: "<<c.u_<<
  }
  visualization_msgs::MarkerArray marr = Cells2Markers(cells, ros::Time::now(), frame_id, value);
  it->second.publish(marr);
}

/*void cell::ToNDTMsg(std::vector<cell>& cells, ndt_map::NDTMapMsg& msg){

  std::vector<perception_oru::NDTCell*> ndt_cells;
  for(auto && c : cells){
    perception_oru::NDTCell* ndt_cell = new perception_oru::NDTCell();
    Eigen::Matrix3d Cov = Eigen::Matrix3d::Identity();
    Cov.block<2,2>(0,0) = c.cov_;;
    ndt_cell->setCov(Cov);
    Eigen::Vector3d u(c.u_(0),c.u_(1), 0);
    ndt_cell->setCenter(pcl::PointXYZ(u(0), u(1), 0));
    ndt_cell->setMean(u);
    ndt_cell->hasGaussian_ = true;
    ndt_cells.push_back(ndt_cell);
  }
  cout<<endl;
  perception_oru::toMessage(ndt_cells, msg, "sensor_est");
}*/

}
