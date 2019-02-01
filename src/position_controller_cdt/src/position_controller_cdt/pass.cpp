#include "position_controller_cdt/pass.hpp"

using namespace std;

Pass::Pass(ros::NodeHandle node_) {
  stopSub_     = node_.subscribe(std::string("/stop_walking"), 100, &Pass::stopWalkingHandler, this);
  poseSub_     = node_.subscribe(std::string("/state_estimator/pose_in_odom"), 100, &Pass::poseHandler, this);

  drivingRvizSub_  = node_.subscribe(std::string("/goal"), 100, &Pass::newDrivingGoalRvizHandler, this);
  footstepSub_ = node_.subscribe(std::string("/footstep_plan_request"), 100, &Pass::newFootstepPlanRequestHandler, this);

  positionControllerPub_ = node_.advertise<geometry_msgs::Twist>("/position_controller/position_controller_cmd", 10);
  stopWalkingPub_ = node_.advertise<std_msgs::Int16>("/stop_walking_cmd",10);

  // diagnostics:
  visualizeCurrentGoalPub_ = node_.advertise<geometry_msgs::PoseStamped>("/position_controller_current_goal", 10);
  visualizeRemainingGoalsPub_ = node_.advertise<geometry_msgs::PoseArray>("/position_controller_remaining_goals", 10);

  positionController_ = new PositionController();

  // Celebration after reaching the ultimate goal
  actionSub_     = node_.subscribe(std::string("/action_cmd"), 100, &Pass::startActionHandler, this);
  controllerClient_ = node_.serviceClient<rocoma_msgs::SwitchController>("/anymal_highlevel_controller/switch_controller");
  modeClient_ = node_.serviceClient<anymal_msgs::SwitchController>("/trot_ros/go_to_mode");
  actionClient_ = node_.serviceClient<free_gait_msgs::SendAction>("/free_gait_action_loader/send_action");

  controllerSrv_.request.name = "trot_ros";
  int status;
  if (controllerClient_.call(controllerSrv_))
  {
    status = (int) controllerSrv_.response.status;
  }
  ROS_INFO("Status: %d", status);
  ros::Duration(1.0).sleep();

  modeSrv_.request.name = "walk";
  if (modeClient_.call(modeSrv_))
  {
    status = (int) modeSrv_.response.status;
  }
  ROS_INFO("Status: %d", status);
  ros::Duration(0.5).sleep();
}


void Pass::stopWalkingHandler(const std_msgs::StringConstPtr& msg){
  std::cout << "STOP_WALKING received. Following disabled\n";
  std_msgs::Int16 output_msg; // contents of message not important
  stopWalkingPub_.publish(output_msg);
}


void Pass::newDrivingGoalRvizHandler(const geometry_msgs::PoseStampedConstPtr& msg){
  ROS_INFO_STREAM("New Rviz goal received");
  Eigen::Isometry3d msg_pose = Eigen::Isometry3d::Identity();
  tf::poseMsgToEigen(msg->pose, msg_pose);
  positionController_->setGoalAndEnable( msg_pose);
}


void Pass::newFootstepPlanRequestHandler(const geometry_msgs::PoseStampedConstPtr& msg){
  std::cout << "New FOOTSTEP_PLAN_REQUEST goal received\n";
  ROS_INFO_STREAM("FOOTSTEP_PLAN_REQUEST goal received");
  Eigen::Isometry3d msg_pose = Eigen::Isometry3d::Identity();
  tf::poseMsgToEigen(msg->pose, msg_pose);
  positionController_->setGoalAndEnable( msg_pose);
}

// Celebration after reaching the ultimate goal
void Pass::startActionHandler(const std_msgs::Int16 actionMsg){
  ros::Duration(3.0).sleep(); // wait for the walking to finish

  controllerSrv_.request.name = "free_gait_impedance_ros";
  int status;
  if (controllerClient_.call(controllerSrv_))
  {
    status = (int) controllerSrv_.response.status;
  }
  ROS_INFO("Status: %d", status);
  ros::Duration(0.5).sleep();

  actionSrv_.request.goal.action_id = "square_up";
  actionClient_.call(actionSrv_);
  std::cout << "Goal reached. Time to partay!" << std::endl;
  ROS_INFO_STREAM("Goal reached, let me get ready to celebrate!");
  ros::Duration(2.0).sleep();

  actionSrv_.request.goal.action_id = "celebration";
  actionClient_.call(actionSrv_);
  ROS_INFO_STREAM("Woo! Time to party!");
  ros::Duration(5.0).sleep();
}


void Pass::poseHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  ROS_INFO_THROTTLE(2,"got ROS pose");
  int64_t msg_utime = (int64_t)floor(msg->header.stamp.toNSec() / 1000);
  Eigen::Isometry3d msg_pose = Eigen::Isometry3d::Identity();
  tf::poseMsgToEigen(msg->pose.pose, msg_pose);

  std::cout << "DEVELOP POSITION CONTROLLER HERE\n";
  // send inputs to the provided class
  FOLLOWER_OUTPUT output_mode = positionController_->computeControlCommand( msg_pose, msg_utime );

  // get the output from the provided class and send to the position controller and viewer
  Eigen::Vector3d output_linear_velocity;
  Eigen::Vector3d output_angular_velocity;
  positionController_->getOutputVelocity(output_linear_velocity, output_angular_velocity);

  geometry_msgs::Twist cmd;
  cmd.linear.x = output_linear_velocity(0);
  cmd.linear.y = output_linear_velocity(1);
  cmd.linear.z = output_linear_velocity(2);
  cmd.angular.x = output_angular_velocity(0);
  cmd.angular.y = output_angular_velocity(1);
  cmd.angular.z = output_angular_velocity(2);
  positionControllerPub_.publish(cmd);

  // Visualize the current goal
  geometry_msgs::PoseStamped m;
  m.header = msg->header;
  Eigen::Isometry3d currentGoal =  positionController_->getCurrentGoal();
  tf::poseEigenToMsg (currentGoal, m.pose);
  visualizeCurrentGoalPub_.publish(m);
}
