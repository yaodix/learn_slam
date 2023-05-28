#include "ros/ros.h"
#include "plumbing_srv_client/add_func.h"


int main(int argc, char* argv[]) {

  if (argc != 3) {
    ROS_WARN("intput num should be two");
    return 1;
  }
  ros::init(argc, argv, "plumbing_client");

  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<plumbing_srv_client::add_func>("add_func");

  client.waitForExistence();  // 阻塞式函数，只有服务启动成功后才会继续执行
  // ros::service::waitForService("add_func");
  plumbing_srv_client::add_func ai;
  // 通过 rosrun plumbing_srv_client client_node 3(参数1) 45(参数2) 调用
  ai.request.num1 = atoi(argv[1]);
  ai.request.num2 = atoi(argv[2]);

  bool res = client.call(ai);  // 核心功能
  if(res) {
    ROS_INFO("call sucess sum %d", ai.response.sum);
  } else {
    ROS_WARN("call fail");
    return 1;
  }
  return 0;
}
