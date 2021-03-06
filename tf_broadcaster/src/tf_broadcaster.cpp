#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>




void poseCallback(const geometry_msgs::PoseStamped& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "delta_base", "delta_ee_platform"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "delta_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/delta/set_cartesian", 10, &poseCallback);

  ros::spin();
  return 0;
};
