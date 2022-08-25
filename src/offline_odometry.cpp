
#include "ros/ros.h"
#include "ros/node_handle.h"
#include "vector"
#include "stdio.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Image.h"

#include "boost/foreach.hpp"
#include "rosbag/view.h"
#include "rosbag/bag.h"

#include "cfear_radarodometry/radar_driver.h"
#include "cfear_radarodometry/odometrykeyframefuser.h"
#include <mutex>
#include <condition_variable>
#include "ros/time.h"
#include "eigen_conversions/eigen_msg.h"
#include "cfear_radarodometry/eval_trajectory.h"
#include <boost/program_options.hpp>
#include <iostream>

#define foreach BOOST_FOREACH
#define MAX_SIZE 3
using namespace CFEAR_Radarodometry;
namespace po = boost::program_options;

/** \brief Offline version of CFEAR Radarodometry, used in the evaluation
 * This allows estimating odometry at the maximum possible rate
 * \author Daniel adolfsson
 */

typedef struct eval_parameters_{
  std::string bag_file_path ="";
  bool save_pcds = false;
  bool save_radar_img = false;
}eval_parameters;


class radarReader
{

private:

  ros::NodeHandle nh_;
  ros::Publisher pub_odom;
  EvalTrajectory eval;
  std::string output_dir;
  radarDriver driver;
  OdometryKeyframeFuser fuser;
  Eigen::Affine3d Toffset = Eigen::Affine3d::Identity();


public:


  radarReader(const OdometryKeyframeFuser::Parameters& odom_pars,
              const radarDriver::Parameters& rad_pars,
              const EvalTrajectory::Parameters& eval_par,
              const eval_parameters& p) : nh_("~"), driver(rad_pars,true), fuser(odom_pars, true), eval(eval_par,true),output_dir(eval_par.est_output_dir) {

    pub_odom = nh_.advertise<nav_msgs::Odometry>("/gt", 1000);
    cout<<"Loading bag from: "<<p.bag_file_path<<endl;
    rosbag::Bag bag;
    bag.open(p.bag_file_path, rosbag::bagmode::Read);

    std::vector<std::string> topics = {"/Navtech/Polar","/gt"};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int frame = 0;


    foreach(rosbag::MessageInstance const m, view)
    {

      if(!ros::ok())
        break;

      nav_msgs::Odometry::ConstPtr odom_msg = m.instantiate<nav_msgs::Odometry>();
      if (odom_msg != NULL){
        nav_msgs::Odometry msg_odom = *odom_msg;
        poseStamped stamped_gt_pose = std::make_pair(Eigen::Affine3d::Identity(), odom_msg->header.stamp);
        tf::poseMsgToEigen(msg_odom.pose.pose, stamped_gt_pose.first);
        stamped_gt_pose.first = stamped_gt_pose.first;//transform into sensor frame
        Eigen::Matrix4d m = stamped_gt_pose.first.matrix();
        m(0,2) = 0; m(2,0) = 0; m(2,1) = 0; m(1,2) = 0; m(2,2) = 1; // 3d -> 2d
        stamped_gt_pose.first = Eigen::Affine3d(m);
        static Eigen::Affine3d Tfirst_i = stamped_gt_pose.first.inverse();
        stamped_gt_pose.first = Tfirst_i*stamped_gt_pose.first;
        eval.CallbackGTEigen(stamped_gt_pose);

        msg_odom.header.stamp = ros::Time::now();
        msg_odom.header.frame_id = "world";
        pub_odom.publish(msg_odom);
        //cout << "gt: " << odom_msg->header.stamp.toNSec() << endl;
        continue;
      }
      sensor_msgs::ImageConstPtr image_msg = m.instantiate<sensor_msgs::Image>();
      if(image_msg != NULL) {
        ros::Time tinit = ros::Time::now();
        //if(frame==0)
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered, cloud_filtered_peaks;
        driver.CallbackOffline(image_msg, cloud_filtered, cloud_filtered_peaks);
        CFEAR_Radarodometry::timing.Document("Filtered points",cloud_filtered->size());
        Eigen::Affine3d Tcurrent;

        fuser.pointcloudCallback(cloud_filtered, cloud_filtered_peaks, Tcurrent, image_msg->header.stamp);
        if(fuser.updated && p.save_radar_img){
          std::string path = eval_par.est_output_dir + "/" + std::to_string(image_msg->header.stamp.toNSec())+".png";
          cv::imwrite(path, driver.cv_polar_image->image);
        }




        eval.CallbackESTEigen(std::make_pair(Tcurrent, image_msg->header.stamp));
        ros::Time tnow = ros::Time::now();
        ros::Duration d = ros::Duration(tnow-tinit);
        static ros::Duration tot(0);
        tot +=d;
        //usleep(100*1000);


        cout<<"Frame: "<<frame<<", dur: "<<d<<", avg: "<<++frame/tot.toSec()<<endl;
      }
    }
    cout<<fuser.GetStatus()<<endl;
    bag.close();
    CFEAR_Radarodometry::timing.PresentStatistics();
    return;
  }

  void Save(){
    eval.Save(); // must be called before ground
    poseStampedVector& vek(eval.GetGtVek());
    fuser.AddGroundTruth(vek);

    fuser.SaveGraph(output_dir+"simple_graph.sgh");
    return;
  }

  ~radarReader(){
    return;
  }
  size_t GetSize(){return eval.GetSize();}
};


void ReadOptions(const int argc, char**argv, OdometryKeyframeFuser::Parameters& par, radarDriver::Parameters& rad_par, CFEAR_Radarodometry::EvalTrajectory::Parameters& eval_par, eval_parameters& p){

    po::options_description desc{"Options"};
    desc.add_options()
        ("help,h", "Help screen")
        ("res", po::value<double>()->default_value(3.5), "res")
        ("range-res", po::value<double>()->default_value(0.0438), "range resolution")
        ("min_distance", po::value<double>()->default_value(2.5), "min sensor distance")
        ("max_distance", po::value<double>()->default_value(200), "mib sensor distance ")
        ("submap_scan_size", po::value<int>()->default_value(3), "submap_scan_size")
        ("weight_intensity", po::value<bool>()->default_value(true),"weight_intensity")
        ("k_strongest", po::value<int>()->default_value(12), "kstrongest points filtering")
        ("job_nr", po::value<int>()->default_value(-1), "jobnr")
        ("registered_min_keyframe_dist", po::value<double>()->default_value(1.5), "registered_min_keyframe_dist")
        ("z-min", po::value<double>()->default_value(65), "zmin intensity, expected noise level")
        ("radar_ccw", po::value<bool>()->default_value(false),"radar_ccw")
        ("soft_constraint", po::value<bool>()->default_value(false),"soft_constraint")
        ("savepcd", "save_pcd_files")
        ("disable_compensate", po::value<bool>()->default_value(false),"disable_compensate")
        ("cost_type", po::value<std::string>()->default_value("P2L"), "P2L")
        ("loss_type", po::value<std::string>()->default_value("Huber"), "robust loss function eg. Huber Caunchy, None")
        ("loss_limit", po::value<double>()->default_value(0.1), "loss limit")
        ("covar_scale", po::value<double>()->default_value(1), "covar scale")// Please fix combined parameter
        ("regularization", po::value<double>()->default_value(1), "regularization")
        ("est_directory", po::value<std::string>()->default_value(""), "output folder of estimated trajectory")
        ("gt_directory", po::value<std::string>()->default_value(""), "output folder of ground truth trajectory")
        ("sequence", po::value<std::string>()->default_value("2019-01-10-12-32-52-radar-oxford-10k"), "sequence contrained in \"bagfile\" to evaluate e.g. 2019-01-10-12-32-52-radar-oxford-10k")
        ("dataset", po::value<std::string>()->default_value("oxford"), "name of dataset, take special actions depending on radar file format etc")
        ("filter-type", po::value<std::string>()->default_value("kstrong"), "filter type")
        ("method_name", po::value<std::string>()->default_value("method"), "method name")
        ("weight_option", po::value<int>()->default_value(0), "how to weight residuals")
        ("false-alarm-rate", po::value<float>()->default_value(0.01), "CA-CFAR false alarm rate")
        ("nb-guard-cells", po::value<int>()->default_value(10), "CA-CFAR nr guard cells")
        ("nb-window-cells", po::value<int>()->default_value(10), "CA-CFAR nr guard cells")
        ("store_graph", po::value<bool>()->default_value(false),"store_graph")
        ("save_radar_img", po::value<bool>()->default_value(false),"save_radar_img")
        ("bag_path", po::value<std::string>()->default_value("/home/daniel/rosbag/oxford-eval-sequences/2019-01-10-12-32-52-radar-oxford-10k/radar/2019-01-10-12-32-52-radar-oxford-10k.bag"), "bag file to open");

    po::variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help"))
      std::cout << desc << '\n';
    if (vm.count("res"))
      par.res = vm["res"].as<double>();
    if (vm.count("min_distance"))
      rad_par.min_distance = vm["min_distance"].as<double>();
    if (vm.count("max_distance"))
      rad_par.max_distance = vm["max_distance"].as<double>();
    if (vm.count("job_nr"))
      eval_par.job_nr = vm["job_nr"].as<int>();
    if (vm.count("cost_type"))
      par.cost_type = vm["cost_type"].as<std::string>();
    if (vm.count("loss_type"))
      par.loss_type_ = vm["loss_type"].as<std::string>();
    if (vm.count("loss_limit"))
      par.loss_limit_ = vm["loss_limit"].as<double>();
    if (vm.count("covar_scale"))
      par.covar_scale_ = vm["covar_scale"].as<double>();
    if (vm.count("regularization"))
      par.regularization_ = vm["regularization"].as<double>();
    if (vm.count("submap_scan_size"))
      par.submap_scan_size = vm["submap_scan_size"].as<int>();
    if (vm.count("k_strongest"))
      rad_par.k_strongest = vm["k_strongest"].as<int>();
    if (vm.count("registered_min_keyframe_dist"))
      par.min_keyframe_dist_= vm["registered_min_keyframe_dist"].as<double>();
    if (vm.count("est_directory"))
      eval_par.est_output_dir = vm["est_directory"].as<std::string>();
    if (vm.count("gt_directory"))
      eval_par.gt_output_dir = vm["gt_directory"].as<std::string>();
    if (vm.count("method_name"))
      eval_par.method = vm["method_name"].as<std::string>();
    if (vm.count("bag_path"))
      p.bag_file_path = vm["bag_path"].as<std::string>();
    if (vm.count("sequence"))
      eval_par.sequence = vm["sequence"].as<std::string>();
    if (vm.count("z-min"))
      rad_par.z_min = vm["z-min"].as<double>();
    if (vm.count("dataset"))
      rad_par.dataset = vm["dataset"].as<std::string>();
    if (vm.count("range-res"))
      rad_par.range_res = vm["range-res"].as<double>();
    if (vm.count("savepcd"))
      eval_par.save_pcd = true;
    if (vm.count("weight_option"))
      par.weight_opt = static_cast<weightoption>(vm["weight_option"].as<int>());
    if (vm.count("k_strongest"))
      rad_par.nb_guard_cells = vm["k_strongest"].as<int>();
    if (vm.count("regularization"))
      rad_par.false_alarm_rate = vm["regularization"].as<double>();
    if (vm.count("covar_scale"))
      rad_par.window_size = vm["covar_scale"].as<double>();


    p.save_radar_img = vm["save_radar_img"].as<bool>();;
    rad_par.filter_type_ = Str2filter(vm["filter-type"].as<std::string>());
    par.store_graph = vm["store_graph"].as<bool>();;
    par.weight_intensity_ = vm["weight_intensity"].as<bool>();;
    par.compensate = !vm["disable_compensate"].as<bool>();
    par.use_guess = true; //vm["soft_constraint"].as<bool>();
    par.soft_constraint = false; // soft constraint is rarely useful, this is changed for testing of initi // vm["soft_constraint"].as<bool>();
    par.radar_ccw = vm["radar_ccw"].as<bool>();

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cfear_radarodometry_node");
  OdometryKeyframeFuser::Parameters odom_pars;
  radarDriver::Parameters rad_pars;
  EvalTrajectory::Parameters eval_pars;


  eval_parameters eval_p;
  ReadOptions(argc, argv, odom_pars, rad_pars, eval_pars, eval_p);


  std::ofstream ofs_before(eval_pars.est_output_dir+std::string("../pars.txt")); // Write
  std::string par_str_before = rad_pars.ToString()+odom_pars.ToString()+eval_pars.ToString()+"nr_frames, "+std::to_string(0)+"\n"+CFEAR_Radarodometry::timing.GetStatistics();
  cout<<"Odometry parameters:\n" << par_str_before<<endl;
  ofs_before<<par_str_before<<endl;
  ofs_before.close();

  radarReader reader(odom_pars, rad_pars, eval_pars, eval_p);
  reader.Save();

  std::ofstream ofs(eval_pars.est_output_dir+std::string("../pars.txt")); // Write
  std::string par_str = rad_pars.ToString()+odom_pars.ToString()+eval_pars.ToString()+"\nnr_frames, "+std::to_string(reader.GetSize())+"\n"+CFEAR_Radarodometry::timing.GetStatistics();
  ofs<<par_str<<endl;
  ofs.close();


  return 0;
}



