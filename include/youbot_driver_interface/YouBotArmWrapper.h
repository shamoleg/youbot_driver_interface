#include <youbot_driver/youbot/YouBotManipulator.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "YouBotArmConfiguration.h"


namespace youBot{

class YouBotArmWrapper
{
public:
    YouBotArmWrapper(ros::NodeHandle n);

    void initializeArm();
    void readJointsSensor();



    ~YouBotArmWrapper();
private:
    YouBotArmWrapper();

    ros::NodeHandle node;
    

    ros::Publisher publisherJointState;

    YouBotArmConfiguration config;

    youbot::YouBotManipulator* youBotArm;

    youbot::GripperSensedBarPosition gripperBar1Position;
    youbot::GripperSensedBarPosition gripperBar2Position;

    std::vector<youbot::JointSensedAngle> jointAngle;
    std::vector<youbot::JointSensedVelocity> jointVelocity;
    std::vector<youbot::JointSensedTorque> jointTorque;

    sensor_msgs::JointState massageJointState;

};

    
}

