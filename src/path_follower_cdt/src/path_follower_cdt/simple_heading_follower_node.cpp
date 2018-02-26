#include <ros/ros.h>
#include <ros/console.h>

#include <stdio.h>
#include <inttypes.h>
#include <iostream>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <joy_manager_msgs/AnyJoy.h>

#include <eigen_conversions/eigen_msg.h>

using namespace std;


void quat_to_euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}

class Pass{
  public:
    Pass(ros::NodeHandle node_);
    
    ~Pass(){
    }    
  private:
    ros::NodeHandle node_;

    double max_angular_velocity_;
    double angular_gain_p_;
    double des_yaw_;

    ros::Subscriber goalSub_;
    void goalHandler(const geometry_msgs::PoseStampedConstPtr& msg);

    ros::Subscriber poseSub_;
    void poseHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    ros::Publisher pathFollowerPub_;
    ros::Publisher velocityJoyPub_;

    void clipAngularVelocity(double &angular_velocity);

};



Pass::Pass(ros::NodeHandle node_){
  goalSub_     = node_.subscribe(std::string("/goal"), 100, &Pass::goalHandler, this);
  poseSub_     = node_.subscribe(std::string("/state_estimator/pose_in_odom"), 100, &Pass::poseHandler, this);
  pathFollowerPub_ = node_.advertise<geometry_msgs::Twist>("/path_follower_cmd", 10);
  velocityJoyPub_ = node_.advertise<joy_manager_msgs::AnyJoy>("/anyjoy/onboard", 10);

  angular_gain_p_ = 0.2; // // 0.6 used before cdt challenge
  max_angular_velocity_ = 0.2; 

  // the initial desired goal is zero
  des_yaw_ = 0;

}

void Pass::goalHandler(const geometry_msgs::PoseStampedConstPtr& msg){
  Eigen::Isometry3d msg_pose = Eigen::Isometry3d::Identity();
  tf::poseMsgToEigen(msg->pose, msg_pose);
  Eigen::Quaterniond q(msg_pose.rotation());

  double roll, pitch;
  quat_to_euler(q, roll, pitch, des_yaw_);
  ROS_INFO("new desired goal received: %f", des_yaw_);
}


// clip commanded angular velocity
void Pass::clipAngularVelocity(double &angular_velocity){
  if (angular_velocity > max_angular_velocity_)
    angular_velocity = max_angular_velocity_;
  else if (angular_velocity < -max_angular_velocity_)
    angular_velocity = -max_angular_velocity_;
}

// constrain angle to be -180:180 in radians
double constrainAngle(double x){
    x = fmod(x + M_PI,2.0*M_PI);
    if (x < 0)
        x += 2.0*M_PI;
    return x - M_PI;
}

void Pass::poseHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  //ROS_INFO_THROTTLE(2,"got ROS pose");

  // receive the current robot pose and compute the current yaw:
  Eigen::Isometry3d msg_pose = Eigen::Isometry3d::Identity();
  tf::poseMsgToEigen(msg->pose.pose, msg_pose);
  Eigen::Quaterniond q(msg_pose.rotation());
  double current_roll, current_pitch, current_yaw;
  quat_to_euler(q, current_roll, current_pitch, current_yaw);
  //ROS_INFO_THROTTLE(0.5,"euler angles %f , %f , %f   Me", roll, pitch, yaw);

  
  // compute the P control output:
  double headingErrorRaw = current_yaw - des_yaw_;
  double headingError = constrainAngle(headingErrorRaw);
  double angular_velocity = -headingError * angular_gain_p_;

  clipAngularVelocity(angular_velocity);  
  ROS_INFO_THROTTLE(0.5, "current_yaw: %f, raw error: %f, constrained error: %f, des ang vel: %f",current_yaw, headingErrorRaw, headingError, angular_velocity);


  // transmit - as a joy pad message:
  joy_manager_msgs::AnyJoy cmd_joy;
  cmd_joy.header.stamp = msg->header.stamp;
  // 0 : lateral in range -1:1
  // 1 : for/bk in range -1:1
  // 2 : unused part of right joiy
  // 3 : turning -1:1
  std::vector<float> axes(4);
  axes[0]=0.;
  axes[1]=0.;
  axes[2]=0.;
  axes[3]=angular_velocity;
  cmd_joy.joy.axes = axes;
  velocityJoyPub_.publish(cmd_joy);

// standard interface:
/*
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = angular_velocity;
  pathFollowerPub_.publish(cmd);
*/


}

int main( int argc, char** argv ){
  ros::init(argc, argv, "path_follower");
  ros::NodeHandle nh;
  Pass app(nh);
  cout << "Ready to follow path" << endl << "============================" << endl;
  ros::spin();
  return 0;
}