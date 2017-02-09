#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sstream>

double vx, vy, vth;
double Delta_x, Delta_y, Delta_th;
double x, y, th;
Eigen::Matrix3d C_D;

geometry_msgs::PoseStamped create_pose_stmpd_msg(double x, double y, double th) {
  geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(th);
  geometry_msgs::PoseStamped pose;

  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "odom";
  pose.pose.position.x  = x;
  pose.pose.position.y  = y;
  pose.pose.orientation = pose_quat;

  return pose;
}

geometry_msgs::PoseWithCovarianceStamped create_pose_covar_msg(double x, double y, double th, Eigen::Matrix3d m) {
  geometry_msgs::PoseWithCovarianceStamped pose_covar;
  geometry_msgs::PoseStamped pose = create_pose_stmpd_msg(x, y, th);

  pose_covar.header.stamp = pose.header.stamp;
  pose_covar.header.frame_id = pose.header.frame_id;
  pose_covar.pose.pose = pose.pose;
  
  for(int i = 0; i < m.rows(); i++) {
    for(int j = 0; j < m.cols(); j++) {
      pose_covar.pose.covariance[( i * m.rows() ) + j] = m(i, j);
    }
  }

  return pose_covar;
}

void velocity_callback(const geometry_msgs::Twist::ConstPtr& input) {
  vx  = input->linear.x;
  vy  = input->linear.y;
  vth = input->angular.z;
}

void keyframe_reset_callback(const std_msgs::Bool::ConstPtr& input) {
  if(input->data) {
    Delta_x  = 0.0;
    Delta_y  = 0.0;
    Delta_th = 0.0;
    C_D = Eigen::Matrix3d::Zero(3, 3);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry");
  ros::NodeHandle n;
  
  ros::Publisher delta_pub = n.advertise<geometry_msgs::PoseStamped>("odometry/delta_pose", 50);
  ros::Publisher global_pub = n.advertise<geometry_msgs::PoseStamped>("odometry/global_pose", 50);
  ros::Publisher Delta_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("odometry/Delta_pose", 50);
  ros::Subscriber vel_sub = n.subscribe("/cmd_vel", 50, velocity_callback);
  ros::Subscriber kf_sub = n.subscribe("odometry/keyframe_reset_signal", 50, keyframe_reset_callback);

  ros::Time current_time = ros::Time::now();
  ros::Time last_time = current_time;
  ros::Rate loop_rate(100); // 100Hz

  //### Initialize motion data ###
  vx = 0.0;
  vy = 0.0;
  vth = 0.0;
  //### Initial reset: pose ###
  x = 0.0;
  y = 0.0;
  th = 0.0;
  //### Initial reset: factor pose increment ###
  Delta_x = 0.0;
  Delta_y = 0.0;
  Delta_th = 0.0;

  //######################################
  //### SET THESE USER-DEFINED VALUES ####
  double sigma_vx = 0.1; // [m/s/sqrt(s)]
  double sigma_vy = 0.1; // [m/s/sqrt(s)]
  double sigma_vth = 0.1; // [m/s/sqrt(s)]
  //### SET THESE USER-DEFINED VALUES ####
  //######################################
  
  //### Set constant twist covariance ###
  Eigen::Matrix3d C_v = Eigen::Matrix3d::Zero(3, 3);
  C_v(0,0) = pow(sigma_vx, 2);
  C_v(1,1) = pow(sigma_vy, 2);
  C_v(2,2) = pow(sigma_vth, 2);

  //### Initial reset: factor pose increment's covariance ###
  C_D = Eigen::Matrix3d::Zero(3, 3);

  while(ros::ok()) {
    current_time  = ros::Time::now();
    double delta_t = (current_time - last_time).toSec();

    //### Time Integration of Velocity Data ###
    double delta_x = vx  * delta_t;
    double delta_y = vy  * delta_t;
    double delta_th = vth * delta_t;
     
    //### Jacobian stage ###
    Eigen::Matrix3d J_d_v_dt = Eigen::Matrix3d::Zero(3, 3);
    J_d_v_dt(0, 0) = 1;
    J_d_v_dt(1, 1) = 1;
    J_d_v_dt(2, 2) = 1;
            
    Eigen::Matrix3d C_d = J_d_v_dt * C_v * J_d_v_dt.transpose() * delta_t;
      
    //### Integrate Factor ###
    double Delta_th_cos = cos ( Delta_th );
    double Delta_th_sin = sin ( Delta_th );

    Delta_x += ( delta_x * Delta_th_cos ) - ( delta_y * Delta_th_sin );
    Delta_y += ( delta_x * Delta_th_sin ) + ( delta_y * Delta_th_cos );
    Delta_th += delta_th;
    Delta_th = std::fmod( Delta_th + M_PI, 2 * M_PI) - M_PI;
    
    //### Jacobian Stage ###
    Eigen::Matrix3d J_D_D = Eigen::Matrix3d::Zero(3, 3);
    J_D_D(0, 0) = 1;
    J_D_D(0, 2) = ( -1 * delta_x * Delta_th_sin ) - ( delta_y * Delta_th_cos );
    J_D_D(1, 1) = 1;
    J_D_D(1, 2) = ( delta_x * Delta_th_cos ) - ( delta_y * Delta_th_sin );
    J_D_D(2, 2) = 1;

    Eigen::Matrix3d J_D_d = Eigen::Matrix3d::Zero(3, 3);
    J_D_d(0, 0) = Delta_th_cos;
    J_D_d(0, 1) = -1 * Delta_th_sin;
    J_D_d(1, 0) = Delta_th_sin;
    J_D_d(1, 1) = Delta_th_cos;
    J_D_d(2, 2) = 1;
      
    //### Covariance stage ###
    C_D = J_D_D * C_D * J_D_D.transpose() + J_D_d * C_d * J_D_d.transpose();

    //### Integrate Global Pose ###
    x += delta_x * cos ( th ) - delta_y * sin ( th );
    y += delta_x * sin ( th ) + delta_y * cos ( th );
    th += delta_th;
    th = std::fmod( th + M_PI, 2 * M_PI) - M_PI;

    //### Publish d, D and global poses, and covariance C_D ###
    Delta_pub.publish(create_pose_covar_msg(Delta_x, Delta_y, Delta_th, C_D));
    delta_pub.publish(create_pose_stmpd_msg(delta_x, delta_y, delta_th));
    global_pub.publish(create_pose_stmpd_msg(x, y, th));

    last_time = current_time;
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
