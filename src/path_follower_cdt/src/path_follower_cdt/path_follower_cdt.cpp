#include <path_follower_cdt/path_follower_cdt.hpp>

PathFollower::PathFollower(){

  std::cout << "Finished setting up PathFollower\n";

}



FOLLOWER_OUTPUT PathFollower::computeControlCommand(Eigen::Isometry3d current_pose, int64_t current_utime){
  double linear_forward_x = 0;
  double linear_forward_y = 0;
  double angular_velocity = 0;
  // Develop your controller here within the calls

  // set outputs
  output_linear_velocity_ = Eigen::Vector3d(linear_forward_x, linear_forward_y, 0);
  output_angular_velocity_ = Eigen::Vector3d(0,0, angular_velocity) ;
  return SEND_NOTHING; 
}