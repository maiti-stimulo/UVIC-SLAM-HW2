#include±± <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <Eigen/Core>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

#include <uvic_msgs/SMToKFI.h>
#include <uvic_msgs/LSToPC.h>
#include <uvic_msgs/TKFS.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_broadcaster.h>

#include <iostream>

ros::Publisher scan_matching_kf_issuer_pub;
ros::ServiceClient scan_converter;
ros::ServiceClient kf_select;

pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_format_correction(sensor_msgs::PointCloud2 input) {
  pcl::PCLPointCloud2 pcl2_pointcloud;
  pcl_conversions::toPCL(input, pcl2_pointcloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl2_pointcloud, *pcl_pointcloud);

  return pcl_pointcloud;
}

geometry_msgs::PoseWithCovariance create_pose_covar_msg(double x, double y, double th, Eigen::MatrixXd m) {
  geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(th);
  geometry_msgs::Pose pose;

  pose.position.x  = x;
  pose.position.y  = y;
  pose.orientation = pose_quat;

  geometry_msgs::PoseWithCovariance pose_covar;

  pose_covar.pose = pose;

  for(int i = 0; i < m.rows(); i++) {
    for(int j = 0; j < m.cols(); j++) {
      pose_covar.covariance[( i * m.rows() ) + j] = m(i, j);
    }
  }

  return pose_covar;
}

uvic_msgs::SMToKFI create_SMToKFI_msg(ros::Time TS, bool KF_flag, bool LC_flag, sensor_msgs::LaserScan scan, double fitness, uvic_msgs::KFSToSM KF_ref, geometry_msgs::PoseWithCovariance D_l) {
  uvic_msgs::SMToKFI output;
  output.TS = TS;
  output.KF_flag = KF_flag;
  output.LC_flag = LC_flag;
  output.scan = scan;
  output.fitness = fitness;
  output.KF_ref = KF_ref;
  output.D_l = D_l;

  return output;
}

void laser_callback(const sensor_msgs::LaserScan& input) {
  uvic_msgs::LSToPC new_scan_srv;
  new_scan_srv.request.scan = input;
  bool new_scan_returned = scan_converter.call(new_scan_srv);

  if(new_scan_returned & !new_scan_srv.response.empty) {
    ROS_INFO("[scan_conversion][SUCCESS] New Scan Received - Conversion to sensor_msgs::PointCloud2.");
    uvic_msgs::TKFS kf_trigger_srv;
    bool kf_select_returned = kf_select.call(kf_trigger_srv);

    if(kf_select_returned) {
      ROS_INFO("[scan_conversion][SUCCESS] New KF Received.");
      uvic_msgs::LSToPC kf_scan_srv;
      kf_scan_srv.request.scan = kf_trigger_srv.response.KF_ref.scan;
      bool kf__scan_returned = scan_converter.call(kf_scan_srv);

      if(kf__scan_returned & !kf_scan_srv.response.empty) {
        ROS_INFO("[scan_conversion][SUCCESS] New KF Received - Conversion to sensor_msgs::PointCloud2.");

	//### Receiving of new laserscan converted into a pointcloud ###
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_pointcloud = pointcloud_format_correction(new_scan_srv.response.pointcloud);
	//### Receiving of keyframe scan converted into a pointcloud ###
        pcl::PointCloud<pcl::PointXYZ>::Ptr kf_pointcloud = pointcloud_format_correction(kf_scan_srv.response.pointcloud);
	//### Initialization of Generalized Interative Closest Point from PCL ###
	//### INSERT INITIALIZATION CALL of type <pcl::PointXYZ, pcl::PointXYZ>
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
	//### Setting the input for GICP as the new_pointcloud ###
        gicp.setInputSource(new_pointcloud);
	//### Setting the output for GICP as the kf_pointcloud ##
        gicp.setInputTarget(kf_pointcloud);
	//### A target for the converged pointcloud of new_pointcloud and kf_pointcloud ###
        //### INSERT POINTER INITIALIZATION of form pcl::PointCloud<pcl::PointXYZ>
        pcl::PointCloud<pcl::PointXYZ>::Ptr new_converged_PointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	//### Running alignment on GICP between new_pointcloud and kf_pointcloud ###
        gicp.align(*new_converged_PointCloud);

	//### Get the fitness score of the alignment from the GICP object
	//### The fitness of the alignment between new_pointcloud and kf_pointcloud ###
  //### converged_fitness = 0.0, means 100% match. Scale from 0.0 to 1.0.
	//### INSERT FUNCTION CALL HERE assigned to "double converged_fitness"
	      double converged_fitness = gicp.getFitnessScore();
	//### Get the boolean indicate whether the two scans (pointclouds) converged
	//### INSERT FUNCTION CALL HERE assigned to "bool converged"
        bool converged = gicp.hasConverged();
	//### Get the matrix transform between the two scans (pointclouds)
	//### INSERT FUNCTION CALL assigned to variable of type Eigen::Matrix4f
	      Eigen::Matrix4f Matrix_Result = gicp.getFinalTransformation();

        if(converged) {
          ROS_INFO("[scan_matching][SUCCESS] Current scan successfully converged with fake keyframe.");

	  //### The Delta transformation between the ref scan and the input scan
	  //### Translate the above "Eigen::Matrix4f transform" into Dx, Dy, Dth
	  //### Indicating the difference in x, y and rotation between scans
          double Dx = transform( 0 , 3);
          double Dy = transform( 1, 3);
          double Dth = atan2( transform( 1, 0), transform( 0, 0) ); // no entenc aquest valors de la transformada

	  //### The Delta transformation in 2D space
          //### EXTRACT THE 2D DELTA FROM THE 3D TRANSFORM ABOVE
          double k_disp_disp = 0.1;
          double k_rot_disp = 0.1;
          double k_rot_rot = 0.1;

	  //### the covariance of the delta
          //### COMPUTE THE COVARIANCE BASED ON THE FORMULAS IN THE README.md FILE
          //double Dl = ??;
          //double sigma_x_squared = ??;
          //double sigma_y_squared = ??;
          //double sigma_th_squared = ?;

          //Eigen::MatrixXd C_l(6, 6);
          //C_l( ?, ?) = sigma_x_squared;
          //C_l( ?, ?) = sigma_y_squared;
          //C_l( ?, ?) = sigma_th_squared;

          double converged_fitness_threshold = 0.15;
          if(converged_fitness > converged_fitness_threshold) {
            ROS_INFO("[scan_matching][SUCCESS] Converged fitness above threshold with %f.", converged_fitness);
            ROS_INFO("[scan_matching][SUCCESS] KF vote issued.");

	    //### Construction of the scan_matching to KF_issuer msg with result of GICP ###
            //### Location of SMToKFI message definition, roscd uvic_msgs/msg ###
	    //### Note KF_flag == true
	    uvic_msgs::SMToKFI output = create_SMToKFI_msg( input.header.stamp, true, false, input, converged_fitness, kf_trigger_srv.response.KF_ref, create_pose_covar_msg(Dx, Dy, Dth, C_l));

            scan_matching_kf_issuer_pub.publish(output);
          } else {
            ROS_ERROR("[scan_matching][FAILED] Converged fitness below threshold with %f.", converged_fitness);
            ROS_ERROR("[scan_matching][FAILED] KF vote not issued.");

	    //### Construction of the scan_matching to KF_issuer msg with result of GICP ###
            //### Location of SMToKFI message definition, roscd uvic_msgs/msg ###
	    //### Note KF_flag == false
	    uvic_msgs::SMToKFI output = create_SMToKFI_msg( input.header.stamp, false, false, input, converged_fitness, kf_trigger_srv.response.KF_ref, create_pose_covar_msg(Dx, Dy, Dth, C_l));

            scan_matching_kf_issuer_pub.publish(output);
          }
        } else {
          ROS_ERROR("[scan_matching][FAILED] Current scan did not converge with fake keyframe.");
        }
      } else {
        ROS_ERROR("[scan_conversion][FAILED] New KF Received - Conversion to sensor_msgs::PointCloud2.");
      }
    } else {
      ROS_ERROR("[scan_conversion][FAILED] New KF Not Received.");
    }
  } else {
    ROS_ERROR("[scan_conversion] New Scan Received - [FAILED] Conversion to sensor_msgs::PointCloud2.");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_matching");
  ros::NodeHandle n;

  scan_converter = n.serviceClient<uvic_msgs::LSToPC>("scan_to_pointcloud");
  kf_select = n.serviceClient<uvic_msgs::TKFS>("/graph/KF_select");

  ros::Subscriber laser_sub = n.subscribe("/base_scan", 50, laser_callback);
  scan_matching_kf_issuer_pub = n.advertise<uvic_msgs::SMToKFI>("/scanner/scan_matching_kf_issuer", 50);
  //In this application all user callbacks will be called from within the ros::spin() call.
  //ros::spin() will not return until the node has been shutdown, either through a call to ros::shutdown() or a Ctrl-C.
  ros::spin();

  return 0;
}
