

#include "ros/ros.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "params_set");


  ros::NodeHandle nh;

// rosparam list
// rosparam get /type  -->xiaohuang
  nh.setParam("type", "xiaohuang");
  nh.setParam("caicai", "a");
  return 0;
}
