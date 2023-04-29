#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

  ros::Publisher pos=node.advertise<geometry_msgs::Pose>("pos_topic",0);
  geometry_msgs::Pose p;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("base_link", "tag_6",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    std::cout<<'['<<transformStamped.transform.translation.x<<','<<transformStamped.transform.translation.y<<','<<transformStamped.transform.translation.z<<']'<<std::endl;
    p.position.x=transformStamped.transform.translation.y;
    p.position.y=transformStamped.transform.translation.x;
    p.position.z=transformStamped.transform.translation.z;
    pos.publish(p);
    rate.sleep();
  }
  return 0;
};