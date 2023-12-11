#include <ros/ros.h>
#include <iomanip>
#include <sstream>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ur_msgs/SetIO.h>



float pos[3] = {0};

// Callback function for the topic completion status
void PosCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  pos[0] = msg->position.x;
  pos[1] = msg->position.y;
  pos[2] = msg->orientation.z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur5_node");
  ros::NodeHandle n;

  // Create a subscriber to the topic that provides the completion status
  ros::Subscriber completionSub = n.subscribe("anscer_pos", 1, PosCallback);

  ros::Publisher completionPub = n.advertise<std_msgs::Int16>("completion_u", 1);

  while (pos[0] == 0)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP_ARM = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group_interface_arm(PLANNING_GROUP_ARM);

  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface_arm.getJointModelGroupNames().begin(),
          move_group_interface_arm.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  
  // 1. Move to home position
  move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("pick"));
  
  bool success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group_interface_arm.move();


  // 2. Place the TCP (Tool Center Point, the tip of the robot) above the blue box
  float waypoint[3] = {0.3158286318042849, -0.3378592424268706, 0.9998737971514666};

  geometry_msgs::PoseStamped current_pose;
  current_pose = move_group_interface_arm.getCurrentPose("tool0");

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = current_pose.pose.orientation;
  target_pose1.position = current_pose.pose.position;
  std::cout<<target_pose1.position.x<<","<<target_pose1.position.y<<","<<target_pose1.position.z<<std::endl;
  std::cout<<target_pose1.orientation.w<<","<<target_pose1.orientation.x<<","<<target_pose1.orientation.y<<","<<target_pose1.orientation.z<<std::endl;

  float pos_diff[3];
  for (int i = 0; i < sizeof(waypoint); i++){
      pos_diff[i] = waypoint[i] - pos[i];

      std::stringstream stream;
      stream << std::fixed << std::setprecision(2) << pos_diff[i];
      stream >> pos_diff[i];
      // std::cout<<"Pos"<<pos_diff[i];

  }

  //target_pose1.orientation.w = 0.228095;
  //target_pose1.orientation.x = -0.186355;
  //target_pose1.orientation.y = -0.58203;
  //target_pose1.orientation.z = -0.669387;
  std::cout<<"Updated pose:"<< -0.86 - pos_diff[0]<<','<< 0.55 - pos_diff[1] <<"\n";
  target_pose1.position.x = -0.86 - pos_diff[0];//-0.21-0.81;-0.679228,0.547254,1.20873
  target_pose1.position.y = 0.55 - pos_diff[1];//0.56;
  target_pose1.position.z = 1.19;//1.09+0.145;
  move_group_interface_arm.setPoseTarget(target_pose1);

  success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");

  move_group_interface_arm.move();

  // 3.
  
  float y1 = target_pose1.position.y;
  target_pose1.position.y = y1 + 0.055;
  move_group_interface_arm.setPoseTarget(target_pose1);

  success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success ? "" : "FAILED");

  move_group_interface_arm.move();

    //Gripper Suction
    ros::ServiceClient client = n.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");

    ur_msgs::SetIO srv;
    srv.request.fun = 1; // set the "fun" argument to 1
    srv.request.pin = 17; // set the "pin" argument to 2
    srv.request.state = 1; // set the "state" argument to 0.5

    if (client.call(srv))
    {
        ROS_INFO("SetIO success");
    }
    else
    {
        ROS_ERROR("Failed to call SetIO service");
        return 1;
    }


  ros::Duration(2.0).sleep();
  
  //4. 
  target_pose1.position.y = y1 - 0.2;
  move_group_interface_arm.setPoseTarget(target_pose1);

  success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (pose goal) %s", success ? "" : "FAILED");

  move_group_interface_arm.move();

  //5.
  move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    
  success = (move_group_interface_arm.plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal) %s", success ? "" : "FAILED");

  move_group_interface_arm.move();
  

      int pins[] = {17, 16}; // array of pin numbers to set

  for (int i = 0; i < sizeof(pins)/sizeof(pins[0]); i++)
  {
      srv.request.fun = 1; // set the "fun" argument to 1
      srv.request.pin = pins[i]; // set the "pin" argument to the current pin number in the array
      srv.request.state = 0; // set the "state" argument to 1

      if (client.call(srv))
      {
          ROS_INFO("SetIO success for pin %d", pins[i]);
      }
      else
      {
          ROS_ERROR("Failed to call SetIO service for pin %d", pins[i]);
          return 1;
      }
  }

  // Publish completion status of 100
  std_msgs::Int16 completionMsg;
  completionMsg.data = 100;
  completionPub.publish(completionMsg);

  ros::Duration(1.0).sleep();  // Wait for the message to be published


  ros::shutdown();
  return 0;
}
