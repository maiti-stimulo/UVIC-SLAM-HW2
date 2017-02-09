#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <sensor_msgs/PointCloud2.h>
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
 	//### INSERT INITIALIZATION CALL       

	//### Setting the input for GICP as the new_pointcloud ###
        //### INSERT FUNCTION CALL

	//### Setting the output for GICP as the kf_pointcloud ###
        //### INSERT FUNCTION CALL

	//### A target for the converged pointcloud of new_pointcloud and kf_pointcloud ###
	//### INSERT POINTER INITIALIZATION

	//### Running alignment on GICP between new_pointcloud and kf_pointcloud ###
	//### INSERT FUNCTION CALL

	//### The fitness of the alignment between new_pointcloud and kf_pointcloud ###
	//### converged_fitness = 0.0, means 100% match. Scale from 0.0 to 1.0.
        //### ASSIGN FITNESS VALUE OF GICP ALIGNMENT TO "converged_fitness" VARIABLE
	//### double converged_fitness = ??

	//### Boolean to indicate whether the two pointclouds have aligned at all. ###
	//### ASSIGN BOOLEAN VALUE OF GICP ALIGNMENT TO "converged" VARIABLE
	//### bool converged = ??

	//### Convergence check ###
        if(converged) {
          ROS_INFO("[scan_matching][SUCCESS] Current scan successfully converged with fake keyframe.");

	  //### Setting of the convergence threshold ###
	  //### ASSIGN FITNESS VALUE THRESHOLD OF GICP ALIGNMENT TO "converged_fitness_threshold" VARIABLE

          if(converged_fitness > converged_fitness_threshold) {
            ROS_INFO("[scan_matching][SUCCESS] Converged fitness above threshold with %f.", converged_fitness);
            ROS_INFO("[scan_matching][SUCCESS] KF vote issued.");

	    //### Construction of the scan_matching to KF_issuer msg with result of GICP ###
	    //### Location of SMToKFI message definition, roscd uvic_msgs/msg ###
            uvic_msgs::SMToKFI output;
            //### output.TS = ??
            //### output.KF_flag = ??
            //### output.LC_flag = ??
            //### output.scan = ??
            //### output.coverage = ?? 
            //### output.KF_ref = ??

	    //### Publishing of GICP result to KF_issuer msg ###
            scan_matching_kf_issuer_pub.publish(output);
          } else {
            ROS_ERROR("[scan_matching][FAILED] Converged fitness below threshold with %f.", converged_fitness);
            ROS_ERROR("[scan_matching][FAILED] KF vote not issued.");

	    //### Construction of the scan_matching to KF_issuer msg with result of GICP ###
	    uvic_msgs::SMToKFI output;
            //### output.TS = ??
            //### output.KF_flag = ??
            //### output.LC_flag = ??
            //### output.scan = ??
            //### output.coverage = ?? 
            //### output.KF_ref = ??

	    //### Publishing of GICP result to KF_issuer msg ###
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
  
  ros::spin();

  return 0;
}
