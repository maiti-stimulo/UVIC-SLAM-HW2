#include <ros/ros.h>
#include <uvic_msgs/SMToKFI.h>
#include <uvic_msgs/KFIToGB.h>

ros::Publisher kf_issuer_graph_build_pub;

void callback(const uvic_msgs::SMToKFI& input) {
  if(input.KF_flag) {
    uvic_msgs::KFIToGB msg;
    msg.TS = input.TS;
    msg.D_l = input.D_l;
    msg.scan = input.scan;
  
    kf_issuer_graph_build_pub.publish(msg);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "/graph/kf_issuer");
  ros::NodeHandle n;
  
  ros::Subscriber kf_issuer_scan_match_sub = n.subscribe("/scanner/scan_matching_kf_issuer", 50, callback);
  kf_issuer_graph_build_pub = n.advertise<uvic_msgs::KFIToGB>("/graph/kf_issuer_graph_build", 50);
  
  ros::spin();
  return 0;
}
