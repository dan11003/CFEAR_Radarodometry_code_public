#include "cfear_radarodometry/n_scan_normal.h"
namespace CFEAR_Radarodometry {




n_scan_normal_reg::n_scan_normal_reg(){
  this->problem_ = nullptr;
  this->options_.max_num_iterations = 20;

  //These guys are used by default
  //this->options_.minimizer_type = ceres::TRUST_REGION;
  //this->options_.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

}

n_scan_normal_reg::n_scan_normal_reg(const cost_metric &cost,loss_type loss, double loss_limit, const weightoption opt) : n_scan_normal_reg()
{
  cost_ = cost;
  loss_ = loss;
  loss_limit_ = loss_limit;
  weight_opt_ = opt;
}
void n_scan_normal_reg::GetSurface(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc, std::vector<Matrix6d> &reg_cov, bool soft_constraints, Eigen::MatrixXd& surface, double res, int width){
  const size_t n_scans = scans.size();
  assert(Tsrc.size()==n_scans && reg_cov.size()==n_scans);
  InitFixedBlocks(n_scans);
  parameters.resize(n_scans,std::vector<double>());
  for(size_t i = 0 ; i<n_scans ; i++){
    Affine3dToVectorXYeZ(Tsrc[i], parameters[i]);
    assert(scans[i]!=nullptr);
  }

  Eigen::Vector3d guess;
  Affine3dToEigVectorXYeZ(Tsrc.back(), guess);

  scan_associations_.clear();
  weight_associations_.clear();
  bool success = BuildOptimizationProblem(scans, reg_cov.back(), guess, soft_constraints);
  int i = 0, j = 0;
  int pixels = std::ceil(2.0*width/res)+1;
  cout<<"est: "<<parameters[n_scans-1][0]<<", "<<parameters[n_scans-1][1]<<endl;
  double x_0 = parameters[n_scans-1][0];
  double y_0 = parameters[n_scans-1][1];

  surface.resize(pixels,pixels);
  for(double x=x_0-width;x<=x_0+width;x=x+res){
    j=0;
    for(double y=y_0-width;y<=y_0+width;y=y+res){
      parameters[n_scans-1][0] = x;
      parameters[n_scans-1][1] = y;
      ceres::Problem::EvaluateOptions opt;
      double cost = 0;
      problem_->Evaluate(opt, &cost, nullptr, nullptr, nullptr);
      surface(i,j) = cost;
      j++;
    }
    i++;
  }
}

bool n_scan_normal_reg::RegisterTimeContinuous(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc, std::vector<Matrix6d>& reg_cov,  Eigen::Affine3d& Tvel, bool soft_constraints, bool ccw)
{

  Eigen::Affine3d Tid = Eigen::Affine3d::Identity();
  Affine3dToVectorXYeZ(Tvel, vel_parameters_);
  //Affine3dToVectorXYeZ(Tprev, prev_parameters_);
  //Affine3dToVectorXYeZ(Tsrc.back(), current_parameters_);
  time_continuous_ = true;
  ccw_ = ccw;
  bool status = Register(scans, Tsrc, reg_cov, soft_constraints);
  time_continuous_ = false;
  return status;
  //cout<<"vel: "<<prev_parameters_[0]<<", "<<prev_parameters_[1]<<", "<<prev_parameters_[2]<<endl;
}

bool n_scan_normal_reg::Register(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc, std::vector<Matrix6d> &reg_cov, bool soft_constraints){
  const size_t n_scans = scans.size();
  assert(Tsrc.size()==n_scans && reg_cov.size()==n_scans);
  InitFixedBlocks(n_scans);


  parameters.resize(n_scans,std::vector<double>());
  for(size_t i = 0 ; i<n_scans ; i++){
    Affine3dToVectorXYeZ(Tsrc[i], parameters[i]);
    assert(scans[i]!=nullptr);
  }

  Eigen::Vector3d guess;
  Affine3dToEigVectorXYeZ(Tsrc.back(), guess);

  bool success = true;


  std::vector<double> prev_par = parameters.back();
  double prev_score = DBL_MAX;
  for(itr_ = 1 ; itr_<=8 && success; itr_++){
    scan_associations_.clear();
    weight_associations_.clear();
    success = BuildOptimizationProblem(scans, reg_cov.back(), guess, soft_constraints);
    /*MapPointNormal::PublishMap("/before_matching_t", scans.back(), Tsrc.back(), "world",-4,0.5);
    for(int i=0;i<Tsrc.size()-1;i++){
      MapPointNormal::PublishMap("/before_matching_k"+std::to_string(i), scans[i], Tsrc[i], "world",-i-1,0.5);
    }
    MapPointNormal::PublishDataAssociationsMap("associations_residuals", vis_residuals);*/
    //vis_residuals.clear();


    if(!success)
      break;

    success = SolveOptimizationProblem();
    //  FOr visualization only
    if(success)
      for(size_t i = 0 ; i<n_scans ; i++)
        Tsrc[i] = vectorToAffine3d(parameters[i]);
    double current_score = summary_.final_cost;
    const double rel_improvement = (prev_score-current_score)/prev_score;
    const Eigen::Vector2d trans_diff(parameters.back()[0] - prev_par[0],parameters.back()[1] - prev_par[1]);
    const double rot_diff = fabs(parameters.back()[2] - prev_par[2])*180/M_PI;

    //cout<<"trans diff: "<<trans_diff.norm()<<", rot diff: "<<rot_diff<<endl;
    /*if(itr > 1 && (trans_diff.norm() < 0.001 && rot_diff <0.01) ){ // this movement is so small, no need to waste time on details of this level. Could be that the sensor is stationary
      //CFEAR_Radarodometry::timing.Document("no-param-change", 1);
      break;
    }*/

    if( itr_ > min_itr){
      if(prev_score < current_score) // potential problem, recover to prev iteration
      {
        //CFEAR_Radarodometry::timing.Document("prev-better", 1);
        parameters.back() = prev_par;
        break;
      }
      else if(rel_improvement < score_tolerance ){
        //CFEAR_Radarodometry::timing.Document("rel-outter-improvement", 1);
        break;
      }
      else if(summary_.iterations.back().relative_decrease < score_tolerance  || summary_.iterations.size()==1){ // this is a sign
        //CFEAR_Radarodometry::timing.Document("rel-inner-improvement", 1);
        break;
      }
    }
    prev_score = current_score;
    prev_par = parameters.back();
    /*MapPointNormal::PublishMap("/before_matching_t", scans.back(), Tsrc.back(), "world",-4, 0.5);
    for(int i=0;i<Tsrc.size()-1;i++){
      MapPointNormal::PublishMap("/before_matching_k"+std::to_string(i), scans[i], Tsrc[i], "world",-i-1,0.5);
    }
    MapPointNormal::PublishDataAssociationsMap("associations_residuals", vis_residuals);*/
    //cout<<"build: "<<t2-t1<<", solve: "<<t3-t2<<endl;
//return success;
  }
  //cout<<"itrs: "<<itr<<endl;
  CFEAR_Radarodometry::timing.Document("itrs", (double)itr_);
  //cout<<"itrs: "<<itr<<endl;

  if(success){
    //cout<<"after: "<<":\n"<<GetParameterString()<<endl;
    score_ = this->summary_.final_cost/this->summary_.num_residuals;
    //cout<<"sol: "<<GetParameterString()<<", score: "<<score_<<endl;
    //cout<<"score: "<<score_<<endl;
    //cout<<this->summary_.FullReport()<<endl;
    //cout<<GetParameterString()<<endl;
    Eigen::Matrix<double,6,1> m;

    m<< 0.1*0.1, 0.1*0.1, 0, 0, 0, 0.01*0.01;
    for(size_t i=0;i<n_scans;i++)
      reg_cov[i] = m.asDiagonal();

    for(size_t i = 0 ; i<n_scans ; i++)
      Tsrc[i] = vectorToAffine3d(parameters[i]);

    success = GetCovariance(reg_cov.back());
    //cout<<summary_.FullReport()<<endl;

    return success;
  }
  else return false;

}
bool n_scan_normal_reg::GetCost(std::vector<MapNormalPtr>& scans, std::vector<Eigen::Affine3d>& Tsrc, double& score, std::vector<double>& residuals){
  size_t n_scans = scans.size();
  assert( Tsrc.size()==n_scans && n_scans >= 2);
  fixedBlock_ = std::vector<bool>(n_scans,true);
  fixedBlock_.back() = false;

  parameters.resize(n_scans,std::vector<double>());
  for(size_t i = 0 ; i<n_scans ; i++){
    Affine3dToVectorXYeZ(Tsrc[i], parameters[i]);
    assert(scans[i]!=nullptr);
  }
  scan_associations_.clear();
  weight_associations_.clear();

  if(!BuildOptimizationProblem(scans))
    return false;
  CHECK(problem_ != nullptr);
  if(problem_->NumResiduals()<=1){
    cout<<"too few residuals: "<<problem_->NumResiduals()<<endl;
    return false;
  }
  ceres::Problem::EvaluateOptions opt;
  bool success = problem_->Evaluate(opt, &score, &residuals, nullptr, nullptr);
  score_ = score/( std::max((int)residuals.size(),1) );
  return success;
}

void n_scan_normal_reg::AddScanPairCost(MapNormalPtr& target_local, MapNormalPtr& src_local, const Eigen::Affine2d& Ttar, const Eigen::Affine2d& Tsrc, const size_t scan_idx_tar, const size_t scan_idx_src){



  double angle_outlier = std::cos(M_PI/6.0);
  int_pair scan_pair = std::make_pair(scan_idx_tar, scan_idx_src);
  std::unordered_map<size_t,double> stamps;
  double curr_radius = (itr_ == 1) ? 2*radius_ : radius_; // course to fine strategy

  Eigen::Affine2d Tsrctotar = Ttar.inverse()*Tsrc;    // Associate in global reference frame based normals and center
  //cout<<"tar: "<<scan_idx_tar<<", src: "<<scan_idx_src<<", par: "<<parameters.size()<<endl;
  for(size_t src_idx=0 ; src_idx<src_local->GetSize() ; src_idx++){
    if(time_continuous_){ // Actually doesn't improve the results
      if (scan_idx_src != parameters.size()-1) {
        cout<<"error index, "<<scan_idx_src<<","<<parameters.size()-1<<endl;
        exit(0);
      }

      double tfactor = src_local->GetCellRelTimeStamp(src_idx, ccw_); // not used ATM
      stamps[src_idx] = tfactor;
      Eigen::Affine2d  Tcomp = vectorToAffine2d(tfactor*vel_parameters_[0], tfactor*vel_parameters_[1], tfactor*vel_parameters_[2]);
      Tsrctotar = Ttar.inverse()*Tsrc*Tcomp; //  Target frame <- odom frame <- corrected distortion <- observation in src frame
    }


    const Eigen::Vector2d src_trans_mean  = Tsrctotar*src_local->GetMean2d(src_idx); //src_means_proj.block<2,1>(0,src_idx);
    std::vector<int> tar_idx_nearby = target_local->GetClosestIdx(src_trans_mean, curr_radius);
    int max_n_terms = 1, n_terms = 0;
    for(auto &&tar_idx : tar_idx_nearby){
      Eigen::Vector2d src_normal_trans = Tsrctotar.linear()* src_local->GetNormal2d(src_idx);// src.block<2,1>(0,src_idx);
      Eigen::Vector2d tar_normal = target_local->GetNormal2d(tar_idx);  //.block<2,1>(0,tar_idx);
      const double direction_similarity = std::max(src_normal_trans.dot(tar_normal), 0.0);
      if(direction_similarity > angle_outlier){ // Gives slightly better accuracy and runtime-performance

        const double n_src = src_local->GetCell(src_idx).Nsamples_;
        const double n_tar = target_local->GetCell(tar_idx).Nsamples_;

        const double plan_src = src_local->GetCell(src_idx).GetPlanarity();
        const double plan_tar = target_local->GetCell(tar_idx).GetPlanarity();

        weight_associations_[scan_pair].push_back(Weights(n_src, n_tar, direction_similarity, plan_src, plan_tar));
        scan_associations_[scan_pair].push_back(std::make_pair(tar_idx,src_idx));

        if(++n_terms==max_n_terms)
          break;

      }
    }
  }

  // For now this only interates over one correspondance
  for(size_t i=0 ; i<scan_associations_[scan_pair].size() ; i++){
    const size_t ass_tar_idx = scan_associations_[scan_pair][i].first;
    const size_t ass_src_idx = scan_associations_[scan_pair][i].second;
    const double time_scale = time_continuous_ ? stamps.find(ass_src_idx)->second : 0;
    const Eigen::Vector2d tar_mean = target_local->GetMean2d(ass_tar_idx);
    const Eigen::Vector2d src_mean = src_local->GetMean2d(ass_src_idx);
    const Eigen::Vector2d tar_mean_world = Ttar*tar_mean;
    const Eigen::Vector2d src_mean_world = Tsrc*src_mean;
    const double weight_after_loss = weight_associations_[scan_pair][i].GetWeight(weight_opt_);
    //vis_residuals.push_back(std::make_tuple(src_mean_world,tar_mean_world,weight_after_loss,scan_idx_tar));
    //cout << "weight_after_loss: " << weight_after_loss <<", weight_opt_" <<weight_opt_<< endl;
    ceres::LossFunction* ceres_loss = new ceres::ScaledLoss(GetLoss(), weight_after_loss, ceres::TAKE_OWNERSHIP); // Important, loss should be applied after Mr. Huber
    ceres::CostFunction* cost_function = nullptr;
    if(cost_ == cost_metric::P2L){
      //const Eigen::Vector2d src_normal = src_local->GetNormal2d(ass_src_idx);
      //Eigen::Vector2d src_normal_trans = Tsrctotar.linear()* src_local->GetNormal2d(ass_src_idx);// src.block<2,1>(0,src_idx);
      const Eigen::Vector2d tar_normal = target_local->GetNormal2d(ass_tar_idx);
      //double similarity_weight = fabs(src_normal_trans.dot(tar_normal));
      if(efficient_implementation)
        cost_function = P2LEfficientCost::Create(Ttar*tar_mean, Ttar.linear()*tar_normal, src_mean);
      else {
        cost_function = P2LCost::Create(tar_mean, tar_normal, src_mean);
      }
    }
    else if( cost_ == cost_metric::P2D){

      Eigen::Matrix2d reg_mat;
      reg_mat<<regularization_, 0, 0, regularization_; //pow(10,-6), 0, 0, pow(10,-6);
      const Eigen::Matrix2d tar_cov = (reg_mat + Ttar.linear()*target_local->GetCov2d(ass_tar_idx)*Ttar.linear().transpose())*cov_scale_ ;
      //cout<<"regularization_: "<<regularization_<<", cov_scale_ :"<<cov_scale_ <<endl;
      //<<"tar_cov: "<<tar_cov<<endl;
      const Eigen::Matrix2d sqrt_information = tar_cov.inverse().llt().matrixL();
      //cost_function = ceres::NormalPrior(sqrt_information,Ttar.translation());
      cost_function = P2DEfficientCost::Create(Ttar*tar_mean, sqrt_information , src_mean);
    }
    else{
      //double scale_similarity = 1-fabs(src_scales(0,ass_src_idx)-tar_scales(0,ass_tar_idx))/(src_scales(0,ass_src_idx)+tar_scales(0,ass_tar_idx));

      if(time_continuous_)
        cost_function = P2PEfficientContinuousCost::Create(Ttar*tar_mean, src_mean, time_scale, vel_parameters_);
      else if(efficient_implementation)
        cost_function = P2PEfficientCost::Create(Ttar*tar_mean, src_mean);
      else
        cost_function = P2PCost::Create(tar_mean, src_mean);


    }

    if(time_continuous_)
      problem_->AddResidualBlock(cost_function, ceres_loss, parameters[scan_idx_src].data());
    else if(efficient_implementation)
      problem_->AddResidualBlock(cost_function, ceres_loss, parameters[scan_idx_src].data());
    else
      problem_->AddResidualBlock(cost_function, ceres_loss, parameters[scan_idx_tar].data(), parameters[scan_idx_src].data());
  }
  if(time_continuous_){
    //problem_->SetParameterBlockConstant(vel_parameters_.data());
  }


}
/*std::vector<double> residuals;
        for(int i=0 ; i<associations.size() ; i++){
          int ass_tar_idx = associations[i].first;
          int ass_src_idx = associations[i].second;
          Eigen::Vector2d tar_mean = tar_means_local.block<2,1>(0,   ass_tar_idx);
          Eigen::Vector2d src_mean = src_means_local.block<2,1>(0,   ass_src_idx);
          Eigen::Vector2d tar_normal = tar_normals_local.block<2,1>(0, ass_tar_idx);

          residuals.push_back(scan_pair_2dnorm_error::Evalp2l(parameters[scan_idx_tar].data(), parameters[scan_idx_src].data(),tar_mean,tar_normal,src_mean));

          //ceres::CostFunction* cost_function = scan_pair_2dnorm_error::Create(tar_mean, tar_normal, src_mean, src_normal, scale_similarity);
          //problem_->AddResidualBlock(cost_function, loss, parameters[scan_idx_tar].data(), parameters[scan_idx_src].data());
        }*/
//Eigen::Vector2d src_normal = src_normals.block<2,1>(0, src_idx);



bool n_scan_normal_reg:: BuildOptimizationProblem(std::vector<MapNormalPtr>& scans, const Eigen::MatrixXd& cov, const Eigen::Vector3d& guess, bool soft_constraints){


  std::vector<Eigen::Affine2d> Tvek(scans.size());
  problem_ = boost::shared_ptr<ceres::Problem>(new ceres::Problem());
  for(size_t i=0 ; i<scans.size() ; i++){ // project scans [targets] into world frame using transformation parameters, src is always given in the local reference frame.
    problem_->AddParameterBlock(parameters[i].data(), 3);
    Eigen::Affine3d T = vectorToAffine3d(parameters[i]);
    Tvek[i] = Eigen::Translation2d(T.translation().topRows<2>()) * T.linear().topLeftCorner<2,2>();
  }


  CHECK(problem_ != nullptr);

  //  ros::Time t3 = ros::Time::now();
  for(size_t i=0 ; i<scans.size() ; i++)
    for(size_t j=0 ; j<scans.size() ; j++)
      if( !(fixedBlock_[j] && fixedBlock_[i]) && i!=j){ // if not both fixed, and i!=j
        if( (mode_ == incremental_last_to_previous && j > i && !fixedBlock_[j])
            || mode_ == many_to_many_refinement){ // only refine last parameter

          AddScanPairCost(scans[i], scans[j], Tvek[i], Tvek[j], i, j);
        }
      }


  if(problem_->NumResiduals()<=1)
    return false;

  if(soft_constraints){
    Eigen::Matrix3d guess_inf_sqrt = Cov6to3(cov).inverse().llt().matrixL();
    ceres::CostFunction* cost_function = mahalanobisDistanceError::Create(guess, guess_inf_sqrt, sqrt(scans.back()->GetSize()));
    problem_->AddResidualBlock(cost_function, nullptr, parameters.back().data());
  }

  if(fixedBlock_.size()==0)
    problem_->SetParameterBlockConstant(parameters[0].data());
  else if(fixedBlock_.size() == parameters.size()){
    for (size_t i=0 ; i<fixedBlock_.size() ; i++)
      if(fixedBlock_[i])
        problem_->SetParameterBlockConstant(parameters[i].data());
  }
  else {
    cerr<<"Fixed Block size error"<<endl;
    exit(0);
  }
  return true;
}
bool n_scan_normal_reg::GetCovariance(Matrix6d &Cov){

  // then normal solve your problem
  // After the solve is completed, please call the code

  ceres::Covariance::Options opt;
  ceres::Covariance covariance(opt);
  std::vector<std::pair<const double*, const double*> > covariance_blocks;
  double* v = parameters[parameters.size()-1].data();
  covariance_blocks.push_back(std::make_pair(v, v));
  bool cov_deficient = !covariance.Compute(covariance_blocks, problem_.get());
  //double score;
  ////std::vector<double> residuals;
  //ceres::CRSMatrix jac;
  //ceres::Problem::EvaluateOptions opti;

  //bool success = problem_->Evaluate(opti, &score, &residuals, nullptr, &jac);
  //cout<<jac.rows.size()<<" x "<<jac.cols.size()<<endl;
  //cout<<"residuals: "<<" x "<<residuals.size()<<endl;
  if(cov_deficient)
    return false;
  else{
    double covariance_xx[3 * 3];
    covariance.GetCovarianceBlock(v, v, covariance_xx);
    Eigen::MatrixXd cmat = 10*Eigen::Map<Eigen::Matrix<double,3,3> >(covariance_xx);
    Cov = Eigen::Matrix<double,6,6>::Identity();
    Cov.block<2,2>(0,0) = cmat.block<2,2>(0,0);
    Cov(5,5) = cmat(2,2);
    Cov(0,5) = cmat(0,2);
    Cov(5,0) = cmat(2,0);
    return true;
  }
}
bool n_scan_normal_reg::SolveOptimizationProblem(){

  CHECK(problem_ != nullptr);
  if(problem_->NumResiduals()<=1){
    cout<<"too few residuals: "<<problem_->NumResiduals()<<endl;
    return false;
  }
  ceres::Solve(options_, problem_.get(), &summary_);
  return summary_.IsSolutionUsable();
}


}
