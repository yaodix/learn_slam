

#include "ros/ros.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "params_get");


  ros::NodeHandle nh;

std::string res;
bool ret = nh.getParam("type",res);
// nh.param()
ROS_INFO(("get " + res).c_str());
  
  return 0;
}
