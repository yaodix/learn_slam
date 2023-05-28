// 实现订阅代码

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "message_filters/subscriber.h"

// 回调函数，
void doMsg(const std_msgs::String::ConstPtr& msg) {
  //  通过msg获取并操作订阅到的数据
  ROS_INFO(("receive " + msg->data).c_str());
}

class SubscribeAndPublish
{
public:
  SubscribeAndPublish() {
    //Topic you want to subscribe
    // sub_ = n_.subscribe("/fang", 1, &SubscribeAndPublish::callback, this);  // 订阅方法1
  }
  void doIt() {
    sub_ = n_.subscribe("/fang", 1, doMsg);  // 订阅方法2

  }
  void callback(const std_msgs::String::ConstPtr& msg) {
    //.... do something with the input and generate the output...
    ROS_INFO(("receive " + msg->data).c_str());
  }

private:
  ros::NodeHandle n_; 
  // ros::Publisher pub_;
  ros::Subscriber sub_;  // 如果sub_是类成员函数局部变量则无法订阅到消息

};//End of class SubscribeAndPublish

int main(int argc, char* argv[]) {
    // 1.初始化ros节点
  ros::init(argc, argv, "sub");  // 节点不能重名,否则会覆盖
  // 2.创建节点句柄
  SubscribeAndPublish  sa;
  sa.doIt();

  // 4.处理订阅数据
  ros::spin();  // 有回调函数使用，设置循环调用回调函数
  ROS_INFO("after spin");  // 后续语句在过程中不会执行
  return 0;
}

