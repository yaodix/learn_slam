// 实现发布代码
#include "ros/ros.h"
#include "std_msgs/String.h"  // ros文本类型

int main(int argc, char* argv[]) {

  // 1.初始化ros节点
  ros::init(argc, argv, "pub");  // pub是节点名称
  // 2.创建节点句柄
  ros::NodeHandle nh;
  // 3.创建发布者对象。fang是话题名称，可以通过rostopic echo fang来常看
  ros::Publisher pub = nh.advertise<std_msgs::String>("fang", 10);  
  // 4.编写发布消息，并发布消息
  std_msgs::String msg;

  ros::Rate rate(10);  // 设置发布频率

  int cnt = 0;
  ros::Duration(3).sleep();  // 防止前几条信息订阅失败
  while (ros::ok()) {  // ok表示
    msg.data = "hello_" + std::to_string(cnt++);
    pub.publish(msg);
    ROS_INFO(("publish data is " + msg.data).c_str());
    rate.sleep();
    ros::spinOnce();  // 官方建议，虽然没有回调函数
  }
  
  return 0;
}