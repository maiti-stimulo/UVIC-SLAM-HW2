#include <ros/ros.h>
#include <uvic_msgs/KFGToKFS.h>
#include <uvic_msgs/KFSToSM.h>
#include <uvic_msgs/TKFS.h>

ros::Time start_time;
uvic_msgs::KFSToSM kf_last;

void kf_last_callback(const uvic_msgs::KFGToKFS& input) {
  if(ros::Time::now().toSec() > start_time.toSec() + 5) {
    start_time = ros::Time::now();
    
    kf_last.TS = input.TS;
    kf_last.scan = input.scan;
    kf_last.state = input.state;
    
    ROS_INFO("[kf_select][SUCCESS] New fake keyframe generated.");
  }
}

bool select(uvic_msgs::TKFS::Request &req, uvic_msgs::TKFS::Response &res) {
  res.KF_ref.TS = kf_last.TS;
  res.KF_ref.scan = kf_last.scan;
  res.KF_ref.state = kf_last.state;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "/graph/kf_select");
  ros::NodeHandle n;

  ros::ServiceServer kf_select_service = n.advertiseService("/graph/KF_select", select);
  ros::Subscriber kf_fake_last_sub = n.subscribe("/graph/kf_last", 50, kf_last_callback);
  ROS_INFO("[kf_select][SUCCESS] Ready to provide past keyframes.");
  start_time = ros::Time::now();

  ros::spin();
  return 0;
}
