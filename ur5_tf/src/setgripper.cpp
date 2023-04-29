#include <ros/ros.h>
#include <ur_msgs/SetIO.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_io_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");

    ur_msgs::SetIO srv;
    srv.request.fun = 1; // set the "fun" argument to 1
    srv.request.pin = 17; // set the "pin" argument to 2
    srv.request.state = 1; // set the "state" argument to 0.51

    if (client.call(srv))
    {
        ROS_INFO("SetIO success");
    }
    else
    {
        ROS_ERROR("Failed to call SetIO service");
        return 1;
    }

    return 0;
}
