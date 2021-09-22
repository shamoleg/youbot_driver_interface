#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_oodl_driver");
    while (ros::ok)
    {
        ROS_INFO("hi");
    }
    
    return 0;
}