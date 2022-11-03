#include "cfear_radarodometry/eval_trajectory.h"
#include <unsupported/Eigen/MatrixFunctions>

namespace CFEAR_Radarodometry {


EvalTrajectory::EvalTrajectory(const EvalTrajectory::Parameters& pars, bool disable_callback) :par(pars), nh_("~"){

  if(!disable_callback){
    assert(!par.odom_gt_topic.empty() && !par.odom_est_topic.empty());
    if(par.synced_callback){
      pose_sub_gt  = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, par.odom_gt_topic, 100);
      pose_sub_est  = new message_filters::Subscriber<nav_msgs::Odometry>(nh_, par.odom_est_topic, 100);
      sync = new Synchronizer<double_odom>(double_odom(100), *pose_sub_gt, *pose_sub_est);
      sync->registerCallback(boost::bind(&EvalTrajectory::CallbackSynchronized,this, _1, _2));
    }
    else{
      sub_est = nh_.subscribe(par.odom_est_topic, 1000, &EvalTrajectory::CallbackEst, this);
      sub_gt = nh_.subscribe(par.odom_gt_topic, 1000, &EvalTrajectory::CallbackGT, this);
    }
  }
  pub_est = nh_.advertise<nav_msgs::Path>("path_est", 10);
  pub_gt = nh_.advertise<nav_msgs::Path>("path_gt", 10);

}


void EvalTrajectory::CallbackSynchronized(const nav_msgs::Odometry::ConstPtr& msg_est, const nav_msgs::Odometry::ConstPtr& msg_gt){
  cout<<"callback"<<endl;
  Eigen::Affine3d T;
  Covariance cov;

  cov = Eigen::Map<const Covariance>(msg_est->pose.covariance.data());       // the assignment makes a copy - safe
  tf::poseMsgToEigen(msg_est->pose.pose, T);
  gt_vek.push_back(poseStamped(T, cov, msg_est->header.stamp));

  cov = Eigen::Map<const Covariance>(msg_gt->pose.covariance.data());
  tf::poseMsgToEigen(msg_gt->pose.pose, T);
  est_vek.push_back(poseStamped(T, cov, msg_gt->header.stamp));
}
void EvalTrajectory::CallbackEigen(const poseStamped& Test, const poseStamped& Tgt){
  gt_vek.push_back(Tgt);
  est_vek.push_back(Test);
}
void EvalTrajectory::CallbackGTEigen(const poseStamped& Tgt){
  gt_vek.push_back(Tgt);
}

void EvalTrajectory::CallbackESTEigen(const poseStamped& Test){
  est_vek.push_back(Test);
}


void EvalTrajectory::CallbackGT(const nav_msgs::Odometry::ConstPtr &msg){
  Eigen::Affine3d T;
  Covariance cov;
  ros::Time t = msg->header.stamp;
  tf::poseMsgToEigen(msg->pose.pose, T);
  cov = Eigen::Map<const Covariance>(msg->pose.covariance.data());
  gt_vek.push_back(poseStamped(T, cov, t));
}


void EvalTrajectory::CallbackEst(const nav_msgs::Odometry::ConstPtr &msg){
  Eigen::Affine3d T;
  Covariance cov;
  tf::poseMsgToEigen(msg->pose.pose, T);
  ros::Time t = msg->header.stamp;
  cov = Eigen::Map<const Covariance>(msg->pose.covariance.data());
  est_vek.push_back(poseStamped(T, cov, t));
}

std::string EvalTrajectory::DatasetToSequence(const std::string& dataset){
  if(dataset=="2019-01-10-11-46-21-radar-oxford-10k")
    return "00.txt";
  else if(dataset=="2019-01-10-12-32-52-radar-oxford-10k")
    return "01.txt";
  else if(dataset=="2019-01-10-14-02-34-radar-oxford-10k")
    return "02.txt";
  else if(dataset=="2019-01-10-14-36-48-radar-oxford-10k-partial")
    return "03.txt";
  else if(dataset=="2019-01-10-14-50-05-radar-oxford-10k")
    return "04.txt";
  else if(dataset=="2019-01-10-15-19-41-radar-oxford-10k")
    return "05.txt";
  else if(dataset=="2019-01-11-12-26-55-radar-oxford-10k")
    return "06.txt";
  else if(dataset=="2019-01-11-13-24-51-radar-oxford-10k")
    return "07.txt";
  else if(dataset=="2019-01-11-14-02-26-radar-oxford-10k")
    return "08.txt";
  else if(dataset=="2019-01-11-14-37-14-radar-oxford-10k")
    return "09.txt";
  else if(dataset=="2019-01-14-12-05-52-radar-oxford-10k")
    return "10.txt";
  else if(dataset=="2019-01-14-12-41-28-radar-oxford-10k")
    return "11.txt";
  else if(dataset=="2019-01-14-13-38-21-radar-oxford-10k")
    return "12.txt";
  else if(dataset=="2019-01-14-14-15-12-radar-oxford-10k")
    return "13.txt";
  else if(dataset=="2019-01-14-14-48-55-radar-oxford-10k")
    return "14.txt";
  else if(dataset=="2019-01-15-12-01-32-radar-oxford-10k")
    return "15.txt";
  else if(dataset=="2019-01-15-12-52-32-radar-oxford-10k-partial")
    return "16.txt";
  else if(dataset=="2019-01-15-13-06-37-radar-oxford-10k")
    return "17.txt";
  else if(dataset=="2019-01-15-13-53-14-radar-oxford-10k")
    return "18.txt";
  else if(dataset=="2019-01-15-14-24-38-radar-oxford-10k")
    return "19.txt";
  else if(dataset=="2019-01-16-11-53-11-radar-oxford-10k")
    return "20.txt";
  else if(dataset=="2019-01-16-13-09-37-radar-oxford-10k")
    return "21.txt";
  else if(dataset=="2019-01-16-13-42-28-radar-oxford-10k")
    return "22.txt";
  else if(dataset=="2019-01-16-14-15-33-radar-oxford-10k")
    return "23.txt";
  else if(dataset=="2019-01-17-11-46-31-radar-oxford-10k")
    return "24.txt";
  else if(dataset=="2019-01-17-12-48-25-radar-oxford-10k")
    return "25.txt";
  else if(dataset=="2019-01-17-13-26-39-radar-oxford-10k")
    return "26.txt";
  else if(dataset=="2019-01-17-14-03-00-radar-oxford-10k")
    return "27.txt";
  else if(dataset=="2019-01-18-12-42-34-radar-oxford-10k")
    return "28.txt";
  else if(dataset=="2019-01-18-14-14-42-radar-oxford-10k")
    return "29.txt";
  else if(dataset=="2019-01-18-14-46-59-radar-oxford-10k")
    return "30.txt";
  else if(dataset=="2019-01-18-15-20-12-radar-oxford-10k")
    return "31.txt";
  else if(dataset=="2019-01-18-14-46-59-radar-oxford-10k")
    return "32.txt";
  else return "01.txt";

}
void EvalTrajectory::RemoveExtras(){
  //cerr<<"Before remove. est_vek.size()="<<est_vek.size()<<", gt_vek.size()="<<gt_vek.size()<<"\n";

  while( !est_vek.empty() || !gt_vek.empty() ){

    if( fabs( ros::Duration(est_vek.front().t - gt_vek.front().t).toSec() ) > 0.0001  ){
      if(est_vek.front().t < gt_vek.front().t){
        est_vek.erase(est_vek.begin());
      }
      else
        gt_vek.erase(gt_vek.begin());
    }
    else if( fabs( ros::Duration(est_vek.back().t - gt_vek.back().t).toSec() ) > 0.0001  ){
      if(est_vek.back().t > gt_vek.back().t){
        est_vek.pop_back();
      }
      else
        gt_vek.pop_back();
    }
    else {
      break;
    }
  }
  return;
}
void EvalTrajectory::Write(const std::string& path, const poseStampedVector& v){
  std::ofstream evalfile;
  cout<<"Saving: "<<v.size()<<" poses to file: "<<path<<endl;
  evalfile.open(path);
  for(size_t i=0;i<v.size();i++){
    // get the Affine3d matrix from the tuple
    Eigen::MatrixXd m(v[i].pose.matrix());
    // print to the file
    evalfile << std::fixed << std::showpoint;
    assert(m.rows()== 4 && m.cols()==4);
    evalfile << MatToString(m) << endl;
  }
  evalfile.close();
  return;
}

void EvalTrajectory::WriteTUM(const std::string& path, const poseStampedVector& v){
    std::ofstream evalfile;
    cout<<"Saving: "<<v.size()<<" poses to file: "<<path<<endl;
    evalfile.open(path);

    for(size_t i=0;i<v.size();i++){
        // print the timestamp
        evalfile << v[i].t.sec << "." << std::setfill('0') << std::setw(9) << v[i].t.nsec << " " << std::setw(0);

        // get the Affine3d pose and quaternion
        // print the pose to the file
        evalfile << std::fixed << std::setprecision(4);
        evalfile << v[i].pose.translation().x() << " " << v[i].pose.translation().y() << " " << v[i].pose.translation().z() << " ";
        Eigen::Quaterniond quat(v[i].pose.rotation());  // print as a quaternion
        evalfile << std::defaultfloat;
        evalfile << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w(); // << " ";

        // get the covariance and print it inline
        //Eigen::IOFormat Inline_matrix_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ",
        //                              " ", "", "", "", "");
        //evalfile << v[i].cov.format(Inline_matrix_format) << " ";

        evalfile << std::endl;
    }
    evalfile.close();
    return;
}


void EvalTrajectory::WriteCov(const std::string& path, const poseStampedVector& v){
    std::ofstream evalfile;
    cout<<"Saving: "<<v.size()<<" covariances to file: "<<path<<endl;
    evalfile.open(path);

    for(size_t i=0;i<v.size();i++){
        // print the timestamp
        evalfile << v[i].t.sec << "." << std::setfill('0') << std::setw(9) << v[i].t.nsec << " " << std::setw(0);

        // get the covariance and print it inline
        Eigen::IOFormat Inline_matrix_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ",
                                      " ", "", "", "", "");
        evalfile << v[i].cov.format(Inline_matrix_format);

        evalfile << std::endl;
    }
    evalfile.close();
    return;
}


void Plot(){

}
void EvalTrajectory::PublishTrajectory(poseStampedVector& vek, ros::Publisher& pub){
  nav_msgs::Path path;
  path.header.frame_id="world";
  path.header.stamp = ros::Time::now();

  std::vector<tf::StampedTransform> trans_vek;
  for (int i=0;i<vek.size();i++) {
    Eigen::Affine3d T = vek[i].pose;
    geometry_msgs::PoseStamped Tstamped;
    tf::poseEigenToMsg(T,Tstamped.pose);
    path.poses.push_back(Tstamped);
  }
  pub.publish(path);
}
void EvalTrajectory::PrintStatus(){
  if(gt_vek.size()!=est_vek.size()){
    //    std::cerr<<"SIZE ERROR. est_vek.size()="<<est_vek.size()<<", gt_vek.size()="<<gt_vek.size() <<std::endl;
  }
  cout<<"Est first: "<<est_vek.front().pose.translation().transpose()<<" - time: "<<est_vek.front().t<<endl;
  cout<<"GT: first: "<<gt_vek.front().pose.translation().transpose()<<" - time: "<<gt_vek.front().t<<endl;

  cout<<"Est Last: "<<est_vek.back().pose.translation().transpose()<<" - time: "<<est_vek.back().t<<endl;
  cout<<"GT: Last: "<<gt_vek.back().pose.translation().transpose()<<" - time: "<<gt_vek.back().t<<endl;
  return;
}


void EvalTrajectory::Save(){
  cout << "Saving, outpout: " << par.est_output_dir << std::endl;

  if(est_vek.empty() && gt_vek.empty()){
    cout<<"Nothing to evaluate"<<endl;
    cerr<<"array size error. est_vek.size()="<<est_vek.size()<<", gt_vek.size()="<<gt_vek.size()<<endl;
    exit(0);
  }
  else if(gt_vek.empty()){
    cout<<"No ground truth? No problem! without gps the need of radar based localization is even larger"<<endl;
    boost::filesystem::create_directories(par.est_output_dir);
    std::string est_path = par.est_output_dir+DatasetToSequence(par.sequence);
    std::string est_path_tum = par.est_output_dir+ "tum_" +DatasetToSequence(par.sequence);
    std::string est_path_cov = par.est_output_dir+ "cov_" +DatasetToSequence(par.sequence);
    cout<<"Saving estimate poses only, no ground truth, total: "<<est_vek.size()<<" poses"<<endl;
    cout<<"To path: "<<est_path<<endl;
    Write(est_path,  est_vek);
    WriteTUM(est_path_tum, est_vek);
    WriteCov(est_path_cov, est_vek);
    cout<<"Trajectory saved"<<endl;
    return;
    
  }else{
    //PrintStatus();
    //RemoveExtras();
    //PrintStatus();
    One2OneCorrespondance();
  }


  cout << "create: " << par.gt_output_dir << std::endl;
  boost::filesystem::create_directories(par.gt_output_dir);
  cout << "create: " << par.est_output_dir << std::endl;
  boost::filesystem::create_directories(par.est_output_dir);
  std::string gt_path  = par.gt_output_dir +DatasetToSequence(par.sequence);
  std::string gt_path_tum  = par.gt_output_dir+ "tum_" +DatasetToSequence(par.sequence);
  std::string est_path = par.est_output_dir+DatasetToSequence(par.sequence);
  std::string est_path_tum = par.est_output_dir + "tum_" + DatasetToSequence(par.sequence);
  std::string est_path_cov = par.est_output_dir + "cov_" + DatasetToSequence(par.sequence);
  cout<<"Saving est_vek.size()="<<est_vek.size()<<", gt_vek.size()="<<gt_vek.size()<<endl;
  cout<<"To path: \n\" "<<gt_path<<"\""<<"\n\""<<est_path<<"\""<<endl;

  Write(gt_path, gt_vek);
  WriteTUM(gt_path_tum, gt_vek);
  Write(est_path,  est_vek);
  WriteTUM(est_path_tum,  est_vek);
  WriteCov(est_path_cov,  est_vek);
  cout<<"Trajectories saved"<<endl;

  return;
}

/*void EvalTrajectory::AlignTrajectories(){
  cout<<"align"<<endl;
  if(est_vek.size()!=gt_vek.size())
    return;
  size_t n = est_vek.size();
  Eigen::MatrixXd traj_est(n,3), traj_gt(n,3);
  for(size_t i=0 ; i<n ; i++){
    traj_est.block<1,3>(i,0) = est_vek[i].first.translation().transpose();
    traj_gt.block<1,3>(i,0) = gt_vek[i].first.translation().transpose();
  }
  Eigen::Matrix4d transform = best_fit_transform(traj_gt,traj_est);
  Eigen::Affine3d T(transform);
  double sum = 0;


  for(size_t i=0 ; i<n ; i++){
    est_vek[i].first = T.inverse()*est_vek[i].first;
    double d = (est_vek[i].first.translation() - gt_vek[i].first.translation()).norm();
    sum +=d*d;
  }

  const double ATE = std::sqrt(sum/n);
  cout<<"ATE: "<<ATE<<endl;
  cout<<"result: "<<transform<<endl;

}*/
Eigen::Affine3d best_fit_transform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B){

  //  Notice:
  //  1/ JacobiSVD return U,S,V, S as a vector, "use U*S*Vt" to get original Matrix;
  //  2/ matrix type 'MatrixXd' or 'MatrixXf' matters.
  //

  Eigen::Matrix4d T = Eigen::MatrixXd::Identity(4,4);
  Eigen::Vector3d centroid_A(0,0,0);
  Eigen::Vector3d centroid_B(0,0,0);
  Eigen::MatrixXd AA = A;
  Eigen::MatrixXd BB = B;
  int row = A.rows();

  for(int i=0; i<row; i++){
    centroid_A += A.block<1,3>(i,0).transpose();
    centroid_B += B.block<1,3>(i,0).transpose();
  }
  centroid_A /= std::max(row, 1);
  centroid_B /= std::max(row, 1);
  for(int i=0; i<row; i++){
    AA.block<1,3>(i,0) = A.block<1,3>(i,0) - centroid_A.transpose();
    BB.block<1,3>(i,0) = B.block<1,3>(i,0) - centroid_B.transpose();
  }

  Eigen::MatrixXd H = AA.transpose()*BB;
  Eigen::MatrixXd U;
  Eigen::VectorXd S;
  Eigen::MatrixXd V;
  Eigen::MatrixXd Vt;
  Eigen::Matrix3d R;
  Eigen::Vector3d t;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  U = svd.matrixU();
  S = svd.singularValues();
  V = svd.matrixV();
  Vt = V.transpose();

  R = Vt.transpose()*U.transpose();

  if (R.determinant() < 0 ){
    Vt.block<1,3>(2,0) *= -1;
    R = Vt.transpose()*U.transpose();
  }

  t = centroid_B - R*centroid_A;

  T.block<3,3>(0,0) = R;
  T.block<3,1>(0,3) = t;
  return Eigen::Affine3d(T);

}




void EvalTrajectory::One2OneCorrespondance(){

  cout<<"force one to one. est: "<<est_vek.size()<<", gt: "<<gt_vek.size()<<endl;
  poseStampedVector revised_est, revised_gt;
  poseStampedVector::iterator init_guess = gt_vek.begin();
  for(size_t i=0 ; i<est_vek.size() ; i++){
    poseStamped interp_corr;
    if(Interpolate(est_vek[i].t, interp_corr, init_guess)){
      revised_est.push_back(est_vek[i]);
      revised_gt.push_back(interp_corr);
    }
    else{
      init_guess = gt_vek.begin();//reset guess
    }
  }
  est_vek.clear();
  gt_vek.clear();
  est_vek = revised_est;
  gt_vek = revised_gt;
  cout<<"forced one to one. est: "<<est_vek.size()<<", gt: "<<gt_vek.size()<<endl;
  //for(int i=0;i<est_vek.size() && i<gt_vek.size();i+=1){
  //  cout<<"est: "<<est_vek[i].first.translation().transpose()<<" gt: "<<gt_vek[i].first.translation().transpose()<<endl;
  //}
}
bool EvalTrajectory::SearchByTime(const ros::Time& t, poseStampedVector::iterator& itr){
  const double tsearch = t.toSec();
  if(gt_vek.size() <=2){
    itr = gt_vek.begin();
    return false;
  }
  poseStampedVector::iterator last = std::prev(gt_vek.end());

  for(auto &&it = itr ; itr != last; it++){
    double tprev = it->t.toSec();
    double tnext = std::next(it)->t.toSec();
    if(tprev <= tsearch && tsearch <= tnext ) {
      itr = it;
      //cout<<"min:"<<it->second<<" search: "<<t<<"max: "<<std::next(it)->second<<endl;
      return true;
    }
  }
  return false;
}
bool EvalTrajectory::Interpolate(const ros::Time& t, poseStamped& Tinterpolated, poseStampedVector::iterator& itr){
  //cout<<"search: "<<t<<endl;
  bool found = SearchByTime(t,itr);
  if(found){
    poseStamped Tbefore = *itr;
    poseStamped Tafter = *std::next(itr);

      Tinterpolated.pose = pose_interp(t.toSec(), Tbefore.t.toSec(), Tafter.t.toSec(),
                                               Tbefore.pose, Tafter.pose);
      Tinterpolated.t = t;
      //TODO: This should be a geometric mean instead to do the interpolation correctly: https://www.lyndonduong.com/psd-cone/
      //However, we don't have GT's covariance anyway, so we would be computing all the stuff (3x matrix sqrt) for nothing
      Tinterpolated.cov = Tbefore.cov;
    return true;
  }
  else return false;

}
Eigen::Affine3d EvalTrajectory::pose_interp(double t, double t1, double t2, Eigen::Affine3d const& aff1, Eigen::Affine3d const& aff2) {
  // assume here t1 <= t <= t2

  double alpha = 0.0;
  if (t2 != t1)
    alpha = (t - t1) / (t2 - t1);

  Eigen::Quaternion<double> rot1(aff1.linear());
  Eigen::Quaternion<double> rot2(aff2.linear());

  Eigen::Vector3d trans1 = aff1.translation();
  Eigen::Vector3d trans2 = aff2.translation();
  //cout<<"slerp: alpha:"<<alpha<<trans1.transpose()<<" - "<<trans2.transpose()<<endl;
  Eigen::Affine3d result;
  result.translation() = (1.0 - alpha) * trans1 + alpha * trans2;
  Eigen::Matrix3d R = rot1.slerp(alpha, rot2).toRotationMatrix();
  /*double yaw = R.eulerAngles(0,1,2)(2);


  Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
*/

  //Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
  //result.linear()         = R;//q.matrix();;
  result.linear() = R;
  result.translation()(2) = 0;

  return result;
}
/*
void ReadPosesFromFile(const std::string &filepath, std::map<unsigned int,Eigen::Affine3d>& poses){

  cout<<"Opening file: "<<filepath<<endl;
  string line;
  int index =0;
  std::ifstream myfile (filepath);
  if (myfile.is_open()){
    while ( getline (myfile,line) ){
      cout <<"line: " << line << endl;
      std::vector<double> pose_compoments;
      std::vector<std::string> tokens;
      boost::split( tokens, line, boost::is_any_of(" ") );
      for(int i=0;i<tokens.size()-1;i++){
        pose_compoments.push_back(std::stod(tokens[i].c_str()));
      }

      const unsigned int stamp = std::stoul (tokens.back(), nullptr, 0);
      Eigen::MatrixXd input = Eigen::Map<Eigen::Matrix<double, 3, 4> >(pose_compoments.data());
      Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
      m.block<3,4>(0,0) = input;
      Eigen::Affine3d T(m);
      poses[stamp] = T;
      cout  << T.matrix()<< endl;
    }
    myfile.close();
  }
  else{
    std::cout<<"couldn't open file"<<endl;
    exit(0);
  }*/


}
