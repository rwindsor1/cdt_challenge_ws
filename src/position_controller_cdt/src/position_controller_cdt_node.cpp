#include <position_controller_cdt/pass.hpp>

using namespace std;

int main( int argc, char** argv ){
  ros::init(argc, argv, "position_controller");

  ros::NodeHandle nh;

  Pass app(nh);
  cout << "Ready to follow position goal" << endl << "============================" << endl;
  ROS_INFO_STREAM("positionController ros ready");
  ROS_ERROR_STREAM("positionController ros ready");
  ros::spin();
  return 0;
}
