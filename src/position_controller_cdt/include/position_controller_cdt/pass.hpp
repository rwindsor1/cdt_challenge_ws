#pragma once

#include <ros/ros.h>
#include <ros/console.h>

#include <stdio.h>
#include <inttypes.h>
#include <iostream>

// Used by the celebratory action
#include <free_gait_msgs/SendAction.h>
#include <rocoma_msgs/SwitchController.h>
#include <anymal_msgs/SwitchController.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

#include <eigen_conversions/eigen_msg.h>

#include <position_controller_cdt/position_controller_cdt.hpp>


class Pass{
  public:
    Pass(ros::NodeHandle node_);

    ~Pass(){
    }
  private:
    ros::NodeHandle node_;

    PositionController* positionController_;

    void stopWalkingHandler(const std_msgs::StringConstPtr& msg);
    void poseHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void newDrivingGoalRvizHandler(const geometry_msgs::PoseStampedConstPtr& msg);
    void newFootstepPlanRequestHandler(const geometry_msgs::PoseStampedConstPtr& msg);

    ros::Subscriber stopSub_, poseSub_, drivingRvizSub_, footstepSub_;
    ros::Publisher positionControllerPub_,stopWalkingPub_;
    ros::Publisher visualizeCurrentGoalPub_, visualizeRemainingGoalsPub_;

    // Used by the celebratory action
    void startActionHandler(const std_msgs::Int16 actionMsg);
    ros::Subscriber actionSub_;
    ros::ServiceClient estopClient_, controllerClient_, actionClient_, modeClient_;
    free_gait_msgs::SendAction actionSrv_;
    rocoma_msgs::SwitchController controllerSrv_;
    anymal_msgs::SwitchController modeSrv_;
};
