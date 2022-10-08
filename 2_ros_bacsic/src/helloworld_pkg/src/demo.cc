#include "ros/ros.h"

int main(int argc, char *argv[]) {

    // 解决中文乱码
    setlocale(LC_ALL, "");
    
    // 执行 ros 节点初始化
    ros::init(argc,argv,"hello");
    // 创建 ros 节点句柄(非必须)
    ros::NodeHandle n;
    // 控制台输出 hello world
    ROS_INFO("hello world!哈哈");
    

    return 0;
}
