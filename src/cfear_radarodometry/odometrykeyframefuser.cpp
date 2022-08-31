#include "cfear_radarodometry/odometrykeyframefuser.h"
namespace CFEAR_Radarodometry {

visualization_msgs::Marker GetDefault(){
  visualization_msgs::Marker m;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 1;
  m.color.a = 1;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.id = 0;
  m.scale.x = 0.1;
  m.action = visualization_msgs::Marker::ADD;
  m.lifetime = ros::Duration(0);
  m.header.frame_id ="world";
  m.header.stamp = ros::Time::now();
  return m;
}

OdometryKeyframeFuser::OdometryKeyframeFuser(const Parameters& pars, bool disable_callback) : par(pars), nh_("~"){
  assert (!par.input_points_topic.empty() && !par.scan_registered_latest_topic.empty() && !par.scan_registered_keyframe_topic.empty() && !par.odom_latest_topic.empty() && !par.odom_keyframe_topic.empty() );
  assert(par.res>0.05 && par.submap_scan_size>=1 );
  //radar_reg =
  radar_reg = boost::shared_ptr<CFEAR_Radarodometry::n_scan_normal_reg>(new n_scan_normal_reg(Str2Cost(par.cost_type),
                                                                                              Str2loss(par.loss_type_),
                                                                                              par.loss_limit_,
                                                                                              par.weight_opt));

  radar_reg->SetD2dPar(par.covar_scale_,par.regularization_);

  Tprev_fused = Eigen::Affine3d::Identity();
  Tcurrent = Eigen::Affine3d::Identity();
  cov_current = Covariance::Identity();
  T_prev = Eigen::Affine3d::Identity();
  Tmot = Eigen::Affine3d::Identity();

  pose_current_publisher = nh_.advertise<nav_msgs::Odometry>(par.odom_latest_topic,50);
  pose_keyframe_publisher = nh_.advertise<nav_msgs::Odometry>(par.odom_keyframe_topic,50);
  pubsrc_cloud_latest = nh_.advertise<pcl::PointCloud<pcl::PointXYZI> >(par.scan_registered_latest_topic, 1000);
  pub_cloud_keyframe = nh_.advertise<pcl::PointCloud<pcl::PointXYZI> >(par.scan_registered_keyframe_topic, 1000);

  if(!disable_callback) {
    cout<<"subscribe<sensor_msgs::PointCloud2>("<<par.input_points_topic<<")"<<endl;
    pointcloud_callback = nh_.subscribe<sensor_msgs::PointCloud2>(par.input_points_topic, 1000,
                                                                  &OdometryKeyframeFuser::pointcloudCallback, this,
                                                                  ros::TransportHints().tcpNoDelay(true));
  }
  else
    cout<<"callback disabled"<<endl;
}
/*OdometryKeyframeFuser::~OdometryKeyframeFuser(){
  //cout<<"destruct fuser"<<endl;
  keyframes_.clear();
  ///  cout<<"destructed fuser"<<endl;
}*/



bool OdometryKeyframeFuser::KeyFrameBasedFuse(const Eigen::Affine3d& diff, bool use_keyframe, double min_keyframe_dist, double min_keyframe_rot_deg){

  if(!use_keyframe)
    return true;
  Eigen::Vector3d Tmotion_euler = diff.rotation().eulerAngles(0,1,2);
  CFEAR_Radarodometry::normalizeEulerAngles(Tmotion_euler);
  bool fuse_frame = false;
  //cout<<"diff (trans[m]/rot[deg])=("<<diff.translation().norm()<<"/"<<diff.rotation().eulerAngles(0,1,2).norm()*180.0/M_PI<<") limit=("<<min_keyframe_dist_<<"/"<<min_keyframe_rot_deg_<<")"<<endl;
  if(diff.translation().norm()>min_keyframe_dist || Tmotion_euler.norm()>(min_keyframe_rot_deg*M_PI/180.0))
    fuse_frame = true;
  return fuse_frame;
}


bool OdometryKeyframeFuser::AccelerationVelocitySanityCheck(const Eigen::Affine3d& Tmot_prev, const Eigen::Affine3d& Tmot_curr) {
  const double dt = 0.25 ; // 4Hz
  const double vel_limit = 200; // m/s
  const double acc_limit = 200; // m/s
  const double vel = (Tmot_curr.translation()/dt).norm();
  const double acc = ((Tmot_curr.translation() - Tmot_prev.translation())/(dt*dt)).norm();

  //cout<<"vel: "<<vel<<", acc: "<<acc<<endl;
  if(acc>acc_limit){
    cout<<"acceleration exceeds limit"<<acc<<" > "<<acc_limit<<endl;
    return false;
  }
  else if(vel>vel_limit){
    cout<<"velocity exceeds limit"<<vel<<" > "<<vel_limit<<endl;
    return false;
  }
  else return true;

}


Eigen::Affine3d OdometryKeyframeFuser::Interpolate(const Eigen::Affine3d &T2, double factor, const Eigen::Affine3d &T1) {
  // std::cout<<"factor="<<factor<<std::endl;
  Eigen::Affine3d pose;
  Eigen::Quaterniond q1(T1.linear());
  Eigen::Quaterniond q2(T2.linear());
  pose.linear() = (q1.slerp(factor, q2)).toRotationMatrix();
  Eigen::Vector3d tdiff = T2.translation() - T1.translation();
  pose.translation() = T1.translation() + tdiff * factor;
  return pose;
}
pcl::PointXYZI OdometryKeyframeFuser::Transform(const Eigen::Affine3d& T, pcl::PointXYZI& p){
  Eigen::Vector3d v(p.x,p.y,p.z);
  Eigen::Vector3d v2 = T*v;
  pcl::PointXYZI p2;
  p2.x = v2(0);
  p2.y = v2(1);
  p2.z = v2(2);
  p2.intensity = p.intensity;
  return p2;
}
nav_msgs::Odometry OdometryKeyframeFuser::FormatOdomMsg(const Eigen::Affine3d& T, const Eigen::Affine3d& Tmot, const ros::Time& t, Matrix6d& Cov){
  nav_msgs::Odometry odom_msg;

  //double d = Tmot.translation().norm();
  Eigen::MatrixXd cov_1_36(Cov);
  //reg_cov(0,0) = reg_cov(1,1) = (d*0.02)*(d*0.02);
  //reg_cov(5,5) = (d*0.005)*(d*0.005);
  cov_1_36.resize(1,36);
  for(int i=0;i<36;i++){
    odom_msg.pose.covariance[i] = cov_1_36.data()[i];
  }

  odom_msg.header.stamp = t;
  odom_msg.header.frame_id = par.odometry_link_id;
  odom_msg.child_frame_id = "sensor";
  tf::poseEigenToMsg( T, odom_msg.pose.pose);
  return odom_msg;
}
pcl::PointCloud<pcl::PointXYZI> OdometryKeyframeFuser::FormatScanMsg(pcl::PointCloud<pcl::PointXYZI>& cloud_in, Eigen::Affine3d& T){
  pcl::PointCloud<pcl::PointXYZI> cloud_out;
  pcl::transformPointCloud(cloud_in, cloud_out, T);
  cloud_out.header.frame_id = par.odometry_link_id;
  cloud_out.header.stamp = cloud_in.header.stamp;
  return cloud_out;
}

void OdometryKeyframeFuser::processFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_peaks, const ros::Time& t) {


  ros::Time t0 = ros::Time::now();
  Eigen::Affine3d TprevMot(Tmot);
  if(par.compensate){
    Compensate(*cloud, TprevMot, par.radar_ccw);
    Compensate(*cloud_peaks, TprevMot, par.radar_ccw);
  }


  std::vector<Matrix6d> cov_vek;
  std::vector<CFEAR_Radarodometry::MapNormalPtr> scans_vek;
  std::vector<Eigen::Affine3d> T_vek;
  ros::Time t1 = ros::Time::now();
  CFEAR_Radarodometry::MapNormalPtr Pcurrent = CFEAR_Radarodometry::MapNormalPtr(new MapPointNormal(cloud, par.res, Eigen::Vector2d(0,0), par.weight_intensity_, par.use_raw_pointcloud));


  ros::Time t2 = ros::Time::now();
  Eigen::Affine3d Tguess;
  if(par.use_guess)
    Tguess = T_prev*TprevMot;
  else
    Tguess = T_prev;

  if(keyframes_.empty()){
    scan_ = RadarScan(Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity(), cloud_peaks, cloud, Pcurrent, t);
    AddToGraph(keyframes_, scan_, Eigen::Matrix<double,6,6>::Identity());
    updated = true;
    AddToReference(keyframes_, scan_, par.submap_scan_size);
    return;
  }
  else
    FormatScans(keyframes_, Pcurrent, Tguess, cov_vek, scans_vek, T_vek);

  //Only for generating plots


  bool success = true;
  if(!par.disable_registration)
    bool success = radar_reg->Register(scans_vek, T_vek, cov_vek, par.soft_constraint);

  ros::Time t3 = ros::Time::now();

  if(success==false){
    cout<<"registration failure"<<radar_reg->summary_.FullReport()<<endl;
    exit(0);
  }

  Tcurrent = T_vek.back();
  cov_current = cov_vek.back();
  Eigen::Affine3d Tmot_current = T_prev.inverse()*Tcurrent;
  if(!AccelerationVelocitySanityCheck(Tmot, Tmot_current))
    Tcurrent = Tguess; //Insane acceleration and speed, lets use the guess.
  Tmot = T_prev.inverse()*Tcurrent;


  MapPointNormal::PublishMap("/current_normals", Pcurrent, Tcurrent, par.odometry_link_id,-1,0.5);
  pcl::PointCloud<pcl::PointXYZI> cld_latest = FormatScanMsg(*cloud, Tcurrent);
  nav_msgs::Odometry msg_current = FormatOdomMsg(Tcurrent, Tmot, t, cov_current);
  pubsrc_cloud_latest.publish(cld_latest);
  pose_current_publisher.publish(msg_current);
  if(par.publish_tf_){
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = msg_current.header.stamp;
    transformStamped.header.frame_id = msg_current.header.frame_id;

    tf::Transform Tf;
    std::vector<tf::StampedTransform> trans_vek;
    tf::transformEigenToTF(Tcurrent, Tf);
    trans_vek.push_back(tf::StampedTransform(Tf, t, par.odometry_link_id, "radar_link"));
    Tbr.sendTransform(trans_vek);
  }

  const Eigen::Affine3d Tkeydiff = keyframes_.back().GetPose().inverse()*Tcurrent;
  bool fuse = KeyFrameBasedFuse(Tkeydiff, par.use_keyframe, par.min_keyframe_dist_, par.min_keyframe_rot_deg_);


  CFEAR_Radarodometry::timing.Document("velocity", Tmot.translation().norm()/Tsensor);


  if(success && fuse){
    cout << "fuse" <<endl;
    distance_traveled += Tkeydiff.translation().norm();
    Tprev_fused = Tcurrent;
    pcl::PointCloud<pcl::PointXYZI> cld_keyframe = FormatScanMsg(*cloud, Tcurrent);
    nav_msgs::Odometry msg_keyframe = FormatOdomMsg(Tcurrent, Tkeydiff, t, cov_vek.back());
    pub_cloud_keyframe.publish(cld_keyframe);
    pose_keyframe_publisher.publish(msg_keyframe);

    frame_nr_++;
    scan_ = RadarScan(Tcurrent, TprevMot, cloud_peaks, cloud, Pcurrent, t);
    AddToGraph(keyframes_, scan_, Eigen::Matrix<double,6,6>::Identity());
    AddToReference(keyframes_,scan_, par.submap_scan_size);
    updated = true;

  }
  else
    cout << "no fuse" <<endl;
  ros::Time t4 = ros::Time::now();
  CFEAR_Radarodometry::timing.Document("compensate", ToMs(t1-t0));
  CFEAR_Radarodometry::timing.Document("build_normals", ToMs(t2-t1));
  CFEAR_Radarodometry::timing.Document("register", ToMs(t3-t2));
  CFEAR_Radarodometry::timing.Document("publish_etc", ToMs(t4-t3));
  T_prev = Tcurrent;

}

void OdometryKeyframeFuser::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in){
  updated = false;
  //  cout<<"callback"<<endl;
  ros::Time t1 = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_peaks(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg (*msg_in, *cloud);
  pcl_conversions::toPCL(msg_in->header.stamp, cloud->header.stamp);
  this->processFrame(cloud, cloud_peaks, msg_in->header.stamp);
  nr_callbacks_++;
  ros::Time t2 = ros::Time::now();
  CFEAR_Radarodometry::timing.Document("Registration-full",CFEAR_Radarodometry::ToMs(t2-t1));
}


void OdometryKeyframeFuser::pointcloudCallback(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered,  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered_peaks,  Eigen::Affine3d &Tcurr, const ros::Time& t){
  ros::Time t1 = ros::Time::now();
  updated = false;
  this->processFrame(cloud_filtered, cloud_filtered_peaks, t);
  nr_callbacks_++;
  Tcurr = Tcurrent;
  ros::Time t2 = ros::Time::now();
  CFEAR_Radarodometry::timing.Document("Registration",CFEAR_Radarodometry::ToMs(t2-t));

}

void OdometryKeyframeFuser::pointcloudCallback(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered,  pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered_peaks,  Eigen::Affine3d &Tcurr, const ros::Time& t, Covariance &cov_curr){
    pointcloudCallback(cloud_filtered, cloud_filtered_peaks, Tcurr, t);
    cov_curr = cov_current;
}


void OdometryKeyframeFuser::PrintSurface(const std::string& path, const Eigen::MatrixXd& surface){
  std::ofstream myfile;
  myfile.open (path);
  for(int i=0;i<surface.rows();i++){
    for(int j=0;j<surface.cols();j++){
      if(j==surface.cols()-1)
        myfile<<surface(i,j)<<std::endl;
      else
        myfile<<surface(i,j)<<" ";
    }
  }
  myfile.close();
}

void OdometryKeyframeFuser::AddToGraph(PoseScanVector& reference, RadarScan& scan,  const Eigen::Matrix<double,6,6>& Cov){
  if(frame_nr_ == 0){
    graph_.push_back(std::make_pair(scan,std::vector<Constraint3d>()));
  }
  else{
    std::vector<Constraint3d> constraints;
    for(auto itr = reference.rbegin() ; itr!=reference.rend() ; itr++){
      const Eigen::Affine3d Tfrom = scan.GetPose();
      const Eigen::Affine3d Tto = itr->GetPose();
      Eigen::Affine3d Tdiff = Tfrom.inverse()*Tto;
      Eigen::Matrix<double,6,6> C = Cov;
      C.block<3,3>(0,0) = Tfrom.rotation()*Cov.block<3,3>(0,0)*Tfrom.rotation().transpose(); //Change frame to Tprev
      constraints.push_back({scan.idx_, itr->idx_, PoseEigToCeres(Tdiff), C.inverse(), ConstraintType::odometry});
      break;
    }
    graph_.push_back(std::make_pair(scan, constraints));
  }
}
void OdometryKeyframeFuser::AddGroundTruth(poseStampedVector& gt_vek){
  std::map<unsigned long,Pose3d> stamp_map;
  for (auto && gt : gt_vek ){ // construct map of ground truth poses
    stamp_map[std::get<2>(gt).toNSec()] = PoseEigToCeres(std::get<0>(gt));
    //cout << "gt : " << gt.second.toNSec() << endl;
  }

  for (auto itr = graph_.begin() ; itr != graph_.end() ; itr++) { // for each pose, add to ground truth
    //cout << "est: " << itr->first.stamp_ << endl;
    if(stamp_map.find( itr->first.stamp_) != stamp_map.end()){ // if estimated exist in ground truth map
      itr->first.Tgt = stamp_map[itr->first.stamp_]; // add ground truth pose;
      itr->first.has_Tgt_ = true;
    }
    /*else
      //std::cerr << "cannot find pose " << itr->first.stamp_<< " in simple graph" << std::endl;
    */
  }
}

void OdometryKeyframeFuser::SaveGraph(const std::string& path){
  if(par.store_graph)
    CFEAR_Radarodometry::SaveSimpleGraph(path, graph_);
}

void AddToReference(PoseScanVector& reference, RadarScan& scan, size_t submap_scan_size){

  reference.push_back(scan);
  if(reference.size() > submap_scan_size){
    reference.erase(reference.begin());
  }
}

void FormatScans(const PoseScanVector& reference,
                 const CFEAR_Radarodometry::MapNormalPtr& Pcurrent,
                 const Eigen::Affine3d& Tcurrent,
                 std::vector<Matrix6d>& cov_vek,
                 std::vector<MapNormalPtr>& scans_vek,
                 std::vector<Eigen::Affine3d>& T_vek
                 ){

  for (int i=0;i<reference.size();i++) {
    cov_vek.push_back(Identity66);
    scans_vek.push_back(reference[i].cloud_normal_);
    T_vek.push_back(reference[i].GetPose());
  }
  cov_vek.push_back(Identity66);
  scans_vek.push_back(Pcurrent);
  T_vek.push_back(Tcurrent);
}


}
