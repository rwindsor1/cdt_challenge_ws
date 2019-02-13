/*
 * NavigationDemo.cpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 *
 */

#include "grid_map_cdt/Challenge.hpp"
#include <tf_conversions/tf_eigen.h>
#include <grid_map_cv/grid_map_cv.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <cv.h>
#include <highgui.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen_conversions/eigen_msg.h>

using namespace grid_map;
using namespace std::chrono;


void quat_to_euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}

Eigen::Quaterniond euler_to_quat( double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Eigen::Quaterniond q;
    q.w() = cy * cp * cr + sy * sp * sr;
    q.x() = cy * cp * sr - sy * sp * cr;
    q.y() = sy * cp * sr + cy * sp * cr;
    q.z() = sy * cp * cr - cy * sp * sr;
    return q;
}

namespace grid_map_demos {

NavigationDemo::NavigationDemo(ros::NodeHandle& nodeHandle, bool& success)
    : nodeHandle_(nodeHandle),
      filterChain_("grid_map::GridMap"),
      demoMode_(false)
{
  if (!readParameters()) {
    success = false;
    return;
  }

  subscriber_ = nodeHandle_.subscribe(inputTopic_, 1, &NavigationDemo::callback, this);
  listener_ = new tf::TransformListener();

  outputGridmapPub_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/filtered_map", 1, true);
  footstepPlanRequestPub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/footstep_plan_request", 100);
  actionPub_ = nodeHandle_.advertise<std_msgs::Int16>("/action_cmd", 10);

  // Setup filter chain.
  if (!filterChain_.configure(filterChainParametersName_, nodeHandle)) {
    ROS_ERROR("Could not configure the filter chain!");
    success = false;
    return;
  }

  success = true;

  verbose_ = false;
  verboseTimer_ = true;
  plannerEnabled_ = true; // start enabled
}


NavigationDemo::~NavigationDemo()
{
}


bool NavigationDemo::readParameters()
{
  if (!nodeHandle_.getParam("input_topic", inputTopic_)) {
    ROS_ERROR("Could not read parameter `input_topic`.");
    return false;
  }
  nodeHandle_.param("filter_chain_parameter_name", filterChainParametersName_, std::string("grid_map_filters"));

  nodeHandle_.param("demo_mode", demoMode_, true);
  if (demoMode_)
    ROS_INFO("In demo mode [%d]. will use a hard coded gridmap bag and robot pose", int(demoMode_) );
  else
    ROS_INFO("In live mode [%d]. will listen for poses continuously", int(demoMode_) );

  return true;
}


void NavigationDemo::tic(){
  lastTime_ = high_resolution_clock::now();
}


std::chrono::duration<double> NavigationDemo::toc(){
  auto nowTime = high_resolution_clock::now();
  duration<double> elapsedTime = duration_cast<milliseconds>(nowTime - lastTime_);
  lastTime_ = nowTime;
  // std::cout << elapsedTime.count() << "ms elapsed" << std::endl;
  return elapsedTime;
}


void NavigationDemo::callback(const grid_map_msgs::GridMap& message)
{
  if (!plannerEnabled_){
    std::cout << "planner enabled. at the goal? grab a beer!\n";
    return;
  }

  // The all important position goal - get the robot there
  Position pos_goal(14.5,4.0);

  Eigen::Isometry3d pose_robot = Eigen::Isometry3d::Identity();
  if(demoMode_){ // demoMode

    Eigen::Vector3d robot_xyz = Eigen::Vector3d(0.0,0.0,0); //rpy
    Eigen::Vector3d robot_rpy = Eigen::Vector3d(0,0,0); //rpy

    pose_robot.setIdentity();
    pose_robot.translation() = robot_xyz;
    Eigen::Quaterniond motion_R = Eigen::AngleAxisd(robot_rpy(2), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(robot_rpy(1), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(robot_rpy(0), Eigen::Vector3d::UnitX()); // order is ypr

    pose_robot.rotate( motion_R );

  }else{ // online

    tf::StampedTransform transform;
    try {
        listener_->waitForTransform("/odom", "/base", ros::Time(0), ros::Duration(10.0) );
        listener_->lookupTransform("/odom", "/base", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    tf::transformTFToEigen (transform, pose_robot);
    if (verbose_) std::cout << pose_robot.translation().transpose() << " current pose_robot\n";
  }


  Eigen::Isometry3d pose_chosen_carrot = Eigen::Isometry3d::Identity();
  bool sendCommand = planCarrot(message, pose_robot, pos_goal, pose_chosen_carrot);

  if(sendCommand){
    // Send the carrot to the position controller
    geometry_msgs::PoseStamped m;
    m.header = message.info.header;
    tf::poseEigenToMsg (pose_chosen_carrot, m.pose);
    footstepPlanRequestPub_.publish(m); //traversability
  }

}

bool NavigationDemo::planCarrot(const grid_map_msgs::GridMap& message,
  Eigen::Isometry3d pose_robot, Position pos_goal,
  Eigen::Isometry3d& pose_chosen_carrot)
{
  std::cout << "start - carrot planner\n";
  tic();

  // Compute distance to the goal:
  Position pos_robot( pose_robot.translation().head(2) );
  double current_dist_to_goal = (pos_goal - pos_robot).norm();
  std::cout << "current distance to goal: " << current_dist_to_goal << std::endl;

  // If within 1.5m of goal - stop walkin g
  if (current_dist_to_goal < 1.5){
    // Determine a carrot pose: x and y from the above. z is the robot's height.
    // yaw in the direction of the carrot. roll,pitch zero
    Eigen::Vector4d carrot_relative_pose = pose_robot.matrix().inverse()*Eigen::Vector4d(pos_goal(0), pos_goal(1), 0, 1) ;
    double carrot_relative_theta = atan2(carrot_relative_pose(1),carrot_relative_pose(0));
    if (verbose_) std::cout << carrot_relative_pose.transpose() << " - relative carrot\n";
    if (verbose_) std::cout << carrot_relative_theta << " - relative carrot - theta\n";

    Eigen::Isometry3d pose_chosen_carrot_relative = Eigen::Isometry3d::Identity();
    pose_chosen_carrot_relative.translation() = Eigen::Vector3d( carrot_relative_pose(0),carrot_relative_pose(1),0);
    Eigen::Quaterniond motion_R = Eigen::AngleAxisd(carrot_relative_theta, Eigen::Vector3d::UnitZ()) // yaw
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) // pitch
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()); // roll

    pose_chosen_carrot_relative.rotate( motion_R );
    pose_chosen_carrot = pose_robot * pose_chosen_carrot_relative;
    std::cout << current_dist_to_goal << "m to goal. carrot is goal\n";
    // disable carrot planner
    plannerEnabled_ = false;

    // Send message to position_controller to start free gait action
    std_msgs::Int16 actionMsg;
    actionMsg.data = 1;
    ros::Duration(1.0).sleep();
    actionPub_.publish(actionMsg);

    return true;
  }



  // Convert message to map.
  GridMap inputMap;
  GridMapRosConverter::fromMessage(message, inputMap);
  // Apply filter chain.
  grid_map::GridMap outputMap;
  if (!filterChain_.update(inputMap, outputMap)) {
    ROS_ERROR("Could not update the grid map filter chain!");
    return false;
  }
  //if (verboseTimer_) std::cout << toc().count() << "ms: filter chain\n";


  ////// Put your code here ////////////////////////////////////
  // add output map to print the carrot to
  outputMap.add("carrots", Matrix::Zero(outputMap.getSize()(0), outputMap.getSize()(1)));
  double current_roll, current_pitch, current_yaw;
  Eigen::Quaterniond robot_q(pose_robot.rotation());
  quat_to_euler(robot_q, current_roll, current_pitch, current_yaw);

  const double pi= 3.14159265;
  double robot_heading_x = cos(current_yaw);
  double robot_heading_y = sin(current_yaw);

  std::cout << current_yaw << std::endl;
  std::cout << "goal is at: \n" <<pos_goal << std::endl;
  std::cout << "robot is at : \n" <<pos_robot <<std::endl;
  std::cout << "robot heading x: " << cos(current_yaw) << std::endl;
  std::cout << "robot heading y: " << sin(current_yaw) << std::endl;

  // calculate displacement of robot from goal
  double goal_x = (pos_goal[0] - pos_robot[0]);
  double goal_y = (pos_goal[1] - pos_robot[1]);


  // if robot more than a meter from the goal, set carrot to no more than 1 meter away
//  if(current_dist_to_goal > 5){
//    goal_x = goal_x*5/current_dist_to_goal;
 //   goal_y = goal_y*5/current_dist_to_goal;
 // }




  // vector pointing from the robot to the goal of magnitude 1

  //Position pt = Position(robot_heading_x.head(2));

  // vector pointing in direction of the robot
  //Eigen::Vector3d vector_robot()

  // check points along line and print out traversability

  // flag to check if current carrot is legit (traversabile)

  bool vector_ok  = false;
  double heading_goal_dot_product = robot_heading_x*goal_x + goal_y*robot_heading_y;
  bool positive_rotation;
  if(heading_goal_dot_product > 0){
    positive_rotation = true;
  }
  else{
    positive_rotation = false;
  }
  double carrot_pos_x = 1;
  double carrot_pos_y= 0;
  int rotation_counter = 1;
  double angle;
  double cone_angle = pi/18;
  double original_heading_x = robot_heading_x;
  double original_heading_y = robot_heading_y;
  while(!vector_ok){



    // for each orientation
    bool bad_terrain_flag = false;
    double test_x;
    double test_y;
    for(int j = 0; j < 3; j++){
	if(j==0){ test_x = robot_heading_x; test_y = robot_heading_y; }
	if(j==1){ test_x = robot_heading_x*cos(cone_angle)-robot_heading_y*sin(cone_angle);
		  test_y = robot_heading_x*sin(cone_angle)+robot_heading_y*cos(cone_angle);
		}
	if(j==2){ test_x = robot_heading_x*cos(-cone_angle)-robot_heading_y*sin(-cone_angle);
		  test_y = robot_heading_x*sin(-cone_angle)+robot_heading_y*cos(-cone_angle);
		}
	for(int i = 0 ; i < 140; i++ ) {
		// position to check traversability at
	      Position test_pos(pos_robot[0]+test_x*i*.01, pos_robot[1]+test_y*0.01*i);

	      // if position is inside the test map and traversability <
	      if ( outputMap.isInside(test_pos) ){
		Index pt_index;
		outputMap.getIndex( test_pos, pt_index );
		Position pt_cell;
		outputMap.getPosition(pt_index, pt_cell);
		if(outputMap.at("traversability", pt_index) < 1.0){
		  bad_terrain_flag = true;
		}
	      }
	      else{
		std::cout << "ERROR: test carrot is outside the map" << std::endl;
	      }
	    }
}

    if(bad_terrain_flag == false){
      vector_ok = true;
      carrot_pos_x = pos_robot[0]+robot_heading_x;
      carrot_pos_y = pos_robot[1]+robot_heading_y;
      // now exits while loop
    }
    else{
      //terrain is bad so rotate vector
      angle = rotation_counter*pi/18;
      if((rotation_counter % 2) == 0){
         angle = -angle;
	}
      robot_heading_x = robot_heading_x*cos(angle)-robot_heading_y*sin(angle);
      robot_heading_y = robot_heading_x*sin(angle)+robot_heading_y*cos(angle);
      std::cout<<"ROTATED!"<<std::endl;
      rotation_counter += 1;
      if(rotation_counter > 24){
        std::cout <<" WARNING: Rotated right round" << std::endl;

        carrot_pos_x = pos_robot[0]+original_heading_x;
        carrot_pos_y = pos_robot[1]+original_heading_y;
        vector_ok = true;
      }
    }
  }

  std::cout << "Carrot Location" << '\n';
  std::cout<<carrot_pos_x<<std::endl;
  std::cout<<carrot_pos_y<<std::endl;

  ////// Put your code here ////////////////////////////////////


  // Publish filtered output grid map.
  grid_map_msgs::GridMap outputMessage;
  GridMapRosConverter::toMessage(outputMap, outputMessage);
  outputGridmapPub_.publish(outputMessage);
  if (verbose_) std::cout << "finished processing\n";
  if (verboseTimer_) std::cout << toc().count() << "ms: publish output\n";

  std::cout << "finish - carrot planner\n\n";
  // a comment
  double target_yaw = atan2(carrot_pos_y-pos_robot[1],carrot_pos_x-pos_robot[0]);
  Eigen::Quaterniond carrot_q = euler_to_quat(target_yaw,0,0);
  std::cout << "Carrot pos x : " << carrot_pos_x << std::endl;
  std::cout << "Carrot pos y : " << carrot_pos_y << std::endl;
  pose_chosen_carrot.translation().x() = carrot_pos_x;
  pose_chosen_carrot.translation().y() = carrot_pos_y;
  pose_chosen_carrot.linear() = carrot_q.matrix();
  // REMOVE THIS -----------------------------------------

  return true;
}

} /* namespace */
