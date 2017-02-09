#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <uvic_msgs/KFGToKFS.h>
#include <uvic_msgs/KFIToGB.h>

ros::Time fake_kf_TS;
sensor_msgs::LaserScan fake_kf_scan;
nav_msgs::Odometry fake_kf_odom;

void odom_callback(const nav_msgs::Odometry& input) {
  fake_kf_odom = input;
}

void laser_callback(const sensor_msgs::LaserScan& input) {
  fake_kf_scan = input;
}

void kf_callback(const uvic_msgs::KFIToGB& input) {
  
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "/graph/kf_graph");
  ros::NodeHandle n;

  ros::Subscriber kf_fake_scan_sub = n.subscribe("/base_scan", 50, laser_callback);
  ros::Subscriber kf_fake_odom_sub = n.subscribe("/odom", 50, odom_callback);
  ros::Subscriber kf_fake_flag_sub = n.subscribe("/graph/kf_issuer_graph_build", 50, kf_callback);
  ros::Publisher kf_fake_last_pub = n.advertise<uvic_msgs::KFGToKFS>("/graph/kf_last", 50);

  uvic_msgs::KFGToKFS msg;

  while(ros::ok()) {
    msg.TS = ros::Time::now();
    msg.scan = fake_kf_scan;
    msg.state = fake_kf_odom;

    kf_fake_last_pub.publish(msg);
    
    ros::spinOnce();
  }
  
  return 0;
}
