#include <sstream>

#include "ros/ros.h"
#include "plumbing_srv_client/add_func.h"


// bool型表示处理是否成功
bool doReq(plumbing_srv_client::add_func::Request& req,
plumbing_srv_client::add_func::Response& resp) {
  std::stringstream ss;
  ss << "receive " << req.num1 << " " << req.num2;
  ROS_INFO(ss.str().c_str());

  resp.sum = req.num1 + req.num2;
  return true;
}

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "plumbing_srv");

  ros::NodeHandle nh;

  // 有回调函数可以自行推导模板
  // 可以通过 rosservice call add_func(话题名) （tab补全提示参数）进行测试
  ros::ServiceServer srv = nh.advertiseService("add_func", doReq);  // add_func话题名
  ROS_INFO("start add func server");
  ros::spin();

  return 0;
}