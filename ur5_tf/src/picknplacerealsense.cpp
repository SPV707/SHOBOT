#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <ur_msgs/SetIO.h>

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
    static const std::string PLANNING_GROUP_ARM = "manipulator";

    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
            move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    // 1. Move to home position
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("basket"));
    
    bool success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    //2.
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("item_basket"));
    
    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

  ros::Duration(5.0).sleep();


  // // Object Position
  // tf2_ros::Buffer tfBuffer;
  // tf2_ros::TransformListener tfListener(tfBuffer);

  // ros::Rate rate(10.0);
  // int i=0;
  // float xp,yp,zp;
  // while (n.ok() && i<1){
  //   geometry_msgs::TransformStamped transformStamped;
  //   try{
  //     transformStamped = tfBuffer.lookupTransform("world", "dodo_detector_ros/chocolate_0",
  //                              ros::Time(0));                             
  //   }
  //   catch (tf2::TransformException &ex) {
  //     ROS_WARN("%s",ex.what());
  //     ros::Duration(1.0).sleep();
  //     continue;
  //   }
  //   std::cout<<"\nPositon of Item: \nTranslation["<<transformStamped.transform.translation.x<<","<<transformStamped.transform.translation.y<<","<<transformStamped.transform.translation.z<<"]"<<std::endl;
  //   std::cout<<"Rotation["<<transformStamped.transform.rotation.x<<","<<transformStamped.transform.rotation.y<<","<<transformStamped.transform.rotation.z<<","<<transformStamped.transform.rotation.w<<","<<std::endl;
  //   xp=transformStamped.transform.translation.x;
  //   yp=transformStamped.transform.translation.y;
  //   zp=transformStamped.transform.translation.z;
  //   i++;
  //   rate.sleep();
  //   }

  //   //3.
  //   move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("set"));
    
  //   success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //   ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success ? "" : "FAILED");

  //   move_group_interface_arm.move();
  
  //   // 2. Place the TCP (Tool Center Point, the tip of the robot) above the blue box
  //   geometry_msgs::PoseStamped current_pose;
  //   current_pose = move_group_interface_arm.getCurrentPose("tool0");

  //   geometry_msgs::Pose target_pose1;
  //   target_pose1.orientation = current_pose.pose.orientation;
  //   std::cout<<current_pose.pose.position<<std::endl;
  //   // std::cout<<xp<<","<<yp<<","<<zp<<std::endl;

  //   target_pose1.position.x = xp;
  //   target_pose1.position.y = yp + 0.05;
  //   target_pose1.position.z = zp + 0.32; 
  //   move_group_interface_arm.setPoseTarget(target_pose1);

  //   success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //   ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (pose goal) %s", success ? "" : "FAILED");

  //   move_group_interface_arm.move();

    
  //   //Gripper Suction
  //   ros::ServiceClient client = n.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");

  //   ur_msgs::SetIO srv;
  //   srv.request.fun = 1; // set the "fun" argument to 1
  //   srv.request.pin = 17; // set the "pin" argument to 2
  //   srv.request.state = 1; // set the "state" argument to 0.5

  //   if (client.call(srv))
  //   {
  //       ROS_INFO("SetIO success");
  //   }
  //   else
  //   {
  //       ROS_ERROR("Failed to call SetIO service");
  //       return 1;
  //   }

  //   ros::Duration(2.0).sleep();

    //5.
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("set"));
    
    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    // 6. 
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("basket"));
    
    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    // set->basket->bar1L->bar2L
    // set->bar1R->bar2R

    // 7.
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("bar1L"));
    
    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    ros::Duration(4.0).sleep();

    // 8.
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("bar2L"));
    
    success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 8 (pose goal) %s", success ? "" : "FAILED");

    move_group_interface_arm.move();

    // 9.
    moveit::core::RobotStatePtr current_state = move_group_interface_arm.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(move_group_interface_arm.getCurrentState()->getRobotModel()->getJointModelGroup(move_group_interface_arm.getName()), joint_group_positions);
    for (int i = 0; i < 4; i++)
    {
        joint_group_positions[5] += 1.57; // Rotate joint_1 by 45 degrees (0.785 radians)
        move_group_interface_arm.setJointValueTarget(joint_group_positions);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        success = (move_group_interface_arm.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "Visualizing plan rot%d (pose goal) %s", i+1, success ? "" : "FAILED");
        move_group_interface_arm.move();

        ros::Duration(2.0).sleep();
    }

  ros::shutdown();
  return 0;
}
