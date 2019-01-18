#include <iostream>
#include <fstream>      // std::ifstream
#include <stdio.h>
#include <deque>

#include <Eigen/Dense>
#include <Eigen/StdVector>

enum FOLLOWER_OUTPUT {SEND_NOTHING=0, SEND_STOP_WALKING=1, SEND_COMMAND=2};

class PositionController{
  public:
    PositionController();
    
    ~PositionController(){
    }    

    void setGoalAndEnable(Eigen::Isometry3d new_goal){
      current_goal_ = new_goal;        
      // TODO: enable with logic - if you want!
    }

    FOLLOWER_OUTPUT computeControlCommand(Eigen::Isometry3d current_pose, int64_t utime );

    void getOutputVelocity(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity){
      output_linear_velocity = output_linear_velocity_;
      output_angular_velocity = output_angular_velocity_;
    }

    Eigen::Isometry3d getCurrentGoal(){
      return current_goal_;
    }

  private:

    // variables:
    Eigen::Isometry3d current_goal_;
    Eigen::Vector3d output_linear_velocity_;
    Eigen::Vector3d output_angular_velocity_;

};
