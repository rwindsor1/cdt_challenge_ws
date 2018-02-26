#include <ros/ros.h>
#include <ros/console.h>

#include <stdio.h>
#include <inttypes.h>
#include <iostream>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

#include <eigen_conversions/eigen_msg.h>

#include <path_follower_cdt/path_follower_cdt.hpp>

using namespace std;

struct CommandLineConfig
{
  std::string param_file;
};

class Pass{
  public:
    Pass(ros::NodeHandle node_, 
      const CommandLineConfig& cl_cfg_);
    
    ~Pass(){
    }    
  private:
    ros::NodeHandle node_;

    const CommandLineConfig cl_cfg_;
    PathFollower* pathFollower_;

    ros::Subscriber stopSub_, poseSub_, drivingRvizSub_, footstepSub_;
    void stopWalkingHandler(const std_msgs::StringConstPtr& msg);
    void poseHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void newDrivingGoalRvizHandler(const geometry_msgs::PoseStampedConstPtr& msg);
    void newFootstepPlanRequestHandler(const geometry_msgs::PoseStampedConstPtr& msg);

    ros::Publisher pathFollowerPub_,stopWalkingPub_;
    ros::Publisher visualizeCurrentGoalPub_, visualizeRemainingGoalsPub_;

};



Pass::Pass(ros::NodeHandle node_, const CommandLineConfig& cl_cfg_):
    cl_cfg_(cl_cfg_){

  stopSub_     = node_.subscribe(std::string("/stop_walking"), 100, &Pass::stopWalkingHandler, this);
  poseSub_     = node_.subscribe(std::string("/state_estimator/pose_in_odom"), 100, &Pass::poseHandler, this);

  drivingRvizSub_  = node_.subscribe(std::string("/goal"), 100, &Pass::newDrivingGoalRvizHandler, this);
  footstepSub_ = node_.subscribe(std::string("/footstep_plan_request"), 100, &Pass::newFootstepPlanRequestHandler, this);

  pathFollowerPub_ = node_.advertise<geometry_msgs::Twist>("/path_follower_cmd", 10);
  stopWalkingPub_ = node_.advertise<std_msgs::Int16>("/stop_walking_cmd",10);

  // diagnostics:
  visualizeCurrentGoalPub_ = node_.advertise<geometry_msgs::PoseStamped>("/path_follower_current_goal", 10);
  visualizeRemainingGoalsPub_ = node_.advertise<geometry_msgs::PoseArray>("/path_follower_remaining_goals", 10);


  pathFollower_ = new PathFollower();


}



void Pass::stopWalkingHandler(const std_msgs::StringConstPtr& msg){
  std::cout << "STOP_WALKING received. Following disabled\n";
  // TODO: implement this
  //pathFollower_->stopWalking();
}

void Pass::newDrivingGoalRvizHandler(const geometry_msgs::PoseStampedConstPtr& msg){
  ROS_INFO_STREAM("New Rviz goal received");
  Eigen::Isometry3d msg_pose = Eigen::Isometry3d::Identity();
  tf::poseMsgToEigen(msg->pose, msg_pose);
  pathFollower_->setGoalAndEnable( msg_pose);
}

void Pass::newFootstepPlanRequestHandler(const geometry_msgs::PoseStampedConstPtr& msg){
  std::cout << "New FOOTSTEP_PLAN_REQUEST goal received\n";
  ROS_INFO_STREAM("FOOTSTEP_PLAN_REQUEST goal received");
  Eigen::Isometry3d msg_pose = Eigen::Isometry3d::Identity();
  tf::poseMsgToEigen(msg->pose, msg_pose);
  pathFollower_->setGoalAndEnable( msg_pose);
}


void Pass::poseHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  ROS_INFO_THROTTLE(2,"got ROS pose");
  int64_t msg_utime = (int64_t)floor(msg->header.stamp.toNSec() / 1000); 
  Eigen::Isometry3d msg_pose = Eigen::Isometry3d::Identity();
  tf::poseMsgToEigen(msg->pose.pose, msg_pose);

  

  std::cout << "DEVELOP PATH FOLLOWER HERE\n";
  // send inputs to the provided class
  FOLLOWER_OUTPUT output_mode = pathFollower_->computeControlCommand( msg_pose, msg_utime );




  // get the output from the provided class and send to the path follower and viewer
  Eigen::Vector3d output_linear_velocity;
  Eigen::Vector3d output_angular_velocity;
  pathFollower_->getOutputVelocity(output_linear_velocity, output_angular_velocity);

  geometry_msgs::Twist cmd;
  cmd.linear.x = output_linear_velocity(0);
  cmd.linear.y = output_linear_velocity(1);
  cmd.linear.z = output_linear_velocity(2);
  cmd.angular.x = output_angular_velocity(0);
  cmd.angular.y = output_angular_velocity(1);
  cmd.angular.z = output_angular_velocity(2);
  pathFollowerPub_.publish(cmd);


  // Visualize the current goal
  geometry_msgs::PoseStamped m;
  m.header = msg->header;
  Eigen::Isometry3d currentGoal =  pathFollower_->getCurrentGoal();
  tf::poseEigenToMsg (currentGoal, m.pose);
  visualizeCurrentGoalPub_.publish(m);



  
}

int main( int argc, char** argv ){
  ros::init(argc, argv, "path_follower");
 
  CommandLineConfig cl_cfg;
  ros::NodeHandle nh;

  Pass app(nh, cl_cfg);
  cout << "Ready to follow path" << endl << "============================" << endl;
  ROS_INFO_STREAM("pathFollower ros ready");
  ROS_ERROR_STREAM("pathFollower ros ready");
  ros::spin();
  return 0;
}
