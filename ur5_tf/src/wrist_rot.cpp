#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ur_msgs/SetIO.h>
#include <tf/tf.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle n;
  
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.

  ros::AsyncSpinner spinner(1);
  spinner.start();

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.

moveit::planning_interface::MoveGroupInterface move_group("manipulator");
moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
std::vector<double> joint_group_positions;
current_state->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), joint_group_positions);
for (int i = 0; i < 4; i++)
{
    joint_group_positions[5] += 1.57; // Rotate joint_1 by 45 degrees (0.785 radians)
    move_group.setJointValueTarget(joint_group_positions);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        move_group.move();
    }
    ros::Duration(2.0).sleep();
}

  ros::shutdown();
  return 0;
}
