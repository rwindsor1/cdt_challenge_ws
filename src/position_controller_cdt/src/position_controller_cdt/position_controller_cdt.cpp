#include <position_controller_cdt/position_controller_cdt.hpp>

PositionController::PositionController(){

  std::cout << "Finished setting up PositionController\n";

}



FOLLOWER_OUTPUT PositionController::computeControlCommand(Eigen::Isometry3d current_pose, int64_t current_utime){
  double linear_forward_x = 0;
  double linear_forward_y = 0;
  double angular_velocity = 0;
  // Develop your controller here within the calls

  // set outputs
  output_linear_velocity_ = Eigen::Vector3d(linear_forward_x, linear_forward_y, 0);
  output_angular_velocity_ = Eigen::Vector3d(0,0, angular_velocity) ;
  return SEND_NOTHING; 
}
