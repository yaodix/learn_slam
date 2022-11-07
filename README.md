## learn slam

[code of autolabor ROS理论与实践](http://www.autolabor.com.cn/book/ROSTutorials/index.html)  
[homework of 深蓝学院 激光SLAM理论与实践](https://www.shenlanxueyuan.com/course/522)  


### 内容
- [x] 三种通讯方式 
- [x] tf2静态坐标演示
- [x] 机器人模型搭建
- [x] 机器人建图

如何运行机器人建图？  
* 安装 gmapping 包(用于构建地图):sudo apt install ros-<ROS版本>-gmapping

* 安装地图服务包(用于保存与读取地图):sudo apt install ros-<ROS版本>-map-server

* 安装 navigation 包(用于定位以及路径规划):sudo apt install ros-<ROS版本>-navigation  

>>运行


      cd 3_robot_simulation 
      catkin_make  
      source devel/setup.bash  
      roslaunch urdf02_gazebo car_with_sensor.launch  

>>新建控制台，运行

      source devel/setup.bash  
      roslaunch slam_and_navi slam_auto.launch 

>>使用2D 在扫描范围内点击机器人下一个目标点.  

![image](https://github.com/yaodix/learn_slam/blob/master/.file/2D%20nav%20goal.png)  

>>可以看到机器人开始建图
