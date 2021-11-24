#include <youbot_driver/youbot/YouBotManipulator.hpp>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "brics_actuator/JointPositions.h"
#include "brics_actuator/JointVelocities.h"
#include "brics_actuator/JointTorques.h"

#include "youbot_driver/youbot/YouBotGripper.hpp"
#include "youbot_driver/youbot/YouBotJoint.hpp"
#include "YouBotArmConfiguration.h"

#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>


namespace youBot{

class YouBotArmWrapper
{
public:
    YouBotArmWrapper(ros::NodeHandle n);

    void initializeArm();
    void readJointsSensor();

    ~YouBotArmWrapper();
private:


    void callbackSetJointPosition(const brics_actuator::JointPositionsConstPtr& massegeJointPosition);
    void callbackSetJointVelocity(const brics_actuator::JointVelocitiesConstPtr& massegeJointVelocity);
    void callbackSetJointTorque(const brics_actuator::JointTorquesConstPtr& massegeJointTorque);
    void callbackSetGripperPosition(const brics_actuator::JointPositionsConstPtr& massegeGripperPosition);


    ros::NodeHandle node;

    ros::Subscriber subscriberJointPosition;
    ros::Subscriber subscriberJointVelocity;
    ros::Subscriber subscriberJointTorque;
    ros::Subscriber subscriberGripperPosition;

    ros::Publisher publisherJointState;

    tf2_ros::TransformBroadcaster tfBroadcaster;

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

