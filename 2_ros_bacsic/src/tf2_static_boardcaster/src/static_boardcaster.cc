// 静态坐标系发布

#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"


int main(int argc, char* argv[]) {

  ros::init(argc, argv, "static_brocaster");
  tf2_ros::StaticTransformBroadcaster tf_caster;

  geometry_msgs::TransformStamped ts;

  ts.header.stamp = ros::Time::now();
  ts.header.frame_id = "base_linker";

  ts.child_frame_id = "laser";

  ts.transform.translation.x = 0.2;
  ts.transform.translation.y = 0.0;
  ts.transform.translation.z = 0.5;

  tf2::Quaternion qtn;
  qtn.setRPY(0, 0, 0);

  ts.transform.rotation.x = qtn.getX();
  ts.transform.rotation.y = qtn.getY();
  ts.transform.rotation.z = qtn.getY();
  ts.transform.rotation.w = qtn.getW();

  tf_caster.sendTransform(ts);
  ros::spin();
  return 0;
}
