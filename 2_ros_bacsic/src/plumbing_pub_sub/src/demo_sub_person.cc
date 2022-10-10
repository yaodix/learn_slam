// 实现订阅代码

#include "ros/ros.h"

#include "plumbing_pub_sub/Person.h"

// 回调函数，
void dogMsg(const plumbing_pub_sub::PersonPtr& msg) {
  //  通过msg获取并操作订阅到的数据
  std::stringstream ss;
  ss << "receive name " << msg->name << ", age " << msg->age << ", height " << msg->height;

  ROS_INFO(ss.str().c_str());
}

int main(int argc, char* argv[]) {
    // 1.初始化ros节点
  ros::init(argc, argv, "person_sub");  // 节点不能重名,否则会覆盖
  // 2.创建节点句柄
  ros::NodeHandle nh;
  // 3.创建订阅者对象。fang是话题名称，可以通过rostopic echo fang来常看
  ros::Subscriber pub = nh.subscribe("liaotian", 10, dogMsg);  // 订阅话题名称需一致
  // 4.处理订阅数据
  ros::spin();  // 有回调函数使用，设置循环调用回调函数
  return 0;
}

