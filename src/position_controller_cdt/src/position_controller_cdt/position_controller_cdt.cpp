#include "position_controller_cdt/position_controller_cdt.hpp"

PositionController::PositionController(){
  std::cout << "Finished setting up PositionController\n";
}


void PositionController::quat_to_euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}


// constrain angle to be -180:180 in radians
double PositionController::constrainAngle(double x){
    x = fmod(x + M_PI,2.0*M_PI);
    if (x < 0)
        x += 2.0*M_PI;
    return x - M_PI;
}


FOLLOWER_OUTPUT PositionController::computeControlCommand(Eigen::Isometry3d current_pose, int64_t current_utime){
  double linear_forward_x = 0;
  double linear_forward_y = 0;
  double angular_velocity = 0;
  // Develop your controller here within the calls

  // EXAMPLE HEADING CONTROLLER CODE - ADD YOUR OWN POSITION + HEADING CONTROLLER HERE
  ///////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////
  std::cout << current_pose.rotation() << std::endl;
  Eigen::Quaterniond q(current_pose.rotation());
  double current_roll, current_pitch, current_yaw;
  quat_to_euler(q, current_roll, current_pitch, current_yaw);

  Eigen::Quaterniond q_goal(current_goal_.rotation());
  double goal_roll, goal_pitch, goal_yaw;
  quat_to_euler(q_goal, goal_roll, goal_pitch, goal_yaw);

  // compute the P control output:
  double headingErrorRaw = current_yaw - goal_yaw;
  double headingError = constrainAngle(headingErrorRaw);
  double angular_gain_p_ = 3; //1
  angular_velocity = -headingError * angular_gain_p_ ;

  // Linear:
  linear_forward_x = 100; //speed forward 10
 // linear_forward_y = 0.1;

  std::cout << "current_yaw: " << current_yaw << ", raw error: " << headingErrorRaw
            << ", constrained error: " << headingError << ", des ang vel: " << angular_velocity << std::endl;

  ///////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////

  // set outputs
  output_linear_velocity_ = Eigen::Vector3d(linear_forward_x, linear_forward_y, 0);
  output_linear_velocity_ = Eigen::AngleAxisd(goal_yaw,Eigen::Vector3d::UnitZ()) * output_linear_velocity_;
  output_angular_velocity_ = Eigen::Vector3d(0,0, angular_velocity) ;
  return SEND_COMMAND;
}
