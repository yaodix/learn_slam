#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "plumbing_pub_sub/Person.h"

int main(int argc, char* argv[]) {

  // 1.初始化ros节点
  ros::init(argc, argv, "pub_person");  // pub是节点名称
  // 2.创建节点句柄
  ros::NodeHandle nh;
  // 3.创建发布者对象。模板类型是话题类型(plumbing_pub_sub::Person)，fang是话题名称，可以通过rostopic echo fang来常看
  ros::Publisher pub = nh.advertise<plumbing_pub_sub::Person>("liaotian", 10);  
  // 4.编写发布消息，并发布消息

  ros::Rate rate(10);  // 设置发布频率

  ros::Duration(3).sleep();  // 防止前几条信息订阅失败
  plumbing_pub_sub::Person p;
  p.age = 22;
  p.height = 180;
  p.name = "zhangsan";
  while (ros::ok()) {  // ok表示节点不死
    p.age += 1;
    pub.publish(p);
    std::stringstream ss;
    ss << "output name " << p.name << ", age " << p.age << ", height " << p.height;
    ROS_INFO(ss.str().c_str());
    rate.sleep();
    ros::spinOnce();  // 官方建议，虽然没有回调函数
  }
  
  return 0;
}
