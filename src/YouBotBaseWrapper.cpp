#include "youbot_driver_interface/YouBotBaseWrapper.h"
#include <vector>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
 

namespace youBot
{

YouBotBaseWrapper::YouBotBaseWrapper(const ros::NodeHandle& n):
    node(n){

    config = YouBotConfiguration::GetInstance(node);

    if(config->baseControlType["baseVelocityControl"]){
        subBaseVelocity = node.subscribe("base/velocity", 1000, &YouBotBaseWrapper::callbackSetBaseVelocity, this);
    }
    if(config->baseControlType["basePositionControl"]){
        subBasePosition = node.subscribe("base/position", 1000, &YouBotBaseWrapper::callbackSetBasePosition, this);
    }
    if(config->baseControlType["baseJointVelocityControl"]){
        subJointVelocity = node.subscribe("base/jointVelocity", 1000, &YouBotBaseWrapper::callbackSetJointVelocity, this);
    }
    if(config->baseControlType["baseJointCurrentControl"]){
        subJointCurrent = node.subscribe("base/jointCurrent", 1000, &YouBotBaseWrapper::callbackSetJointCurrent, this);
    }
    if(config->baseControlType["baseJointToqueControl"]){
        subJointToque = node.subscribe("base/jointToque", 1000, &YouBotBaseWrapper::callbackSetJointToque, this);
    }

    pubOdometry = node.advertise<nav_msgs::Odometry>("odom", 1000);
    pubJointState = node.advertise<sensor_msgs::JointState>("base/jointState", 1000);

}

YouBotBaseWrapper::~YouBotBaseWrapper()
{
    delete youBotBase;
}


void YouBotBaseWrapper::initialize()
{
    try{
        youBotBase = new youbot::YouBotBase(config->baseName, config->configFilePath);
        youBotBase->doJointCommutation();
        ROS_INFO("Base is initialized.");
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_FATAL("%s", errorMessage.c_str());
        ROS_ERROR("Base \"%s\" could not be initialized.", config->baseName.c_str());
        return;
    }
}

void YouBotBaseWrapper::dataUpdateAndPublish()
{
    pubOdometry.publish(this->getOdometry());
    pubJointState.publish(this->getJointState());
}

nav_msgs::Odometry YouBotBaseWrapper::getOdometry() const
{
    quantity<si::length> longitudinalPosition;
    quantity<si::length> transversalPosition;
    quantity<plane_angle> orientation;

    quantity<si::velocity> longitudinalVelocity;
    quantity<si::velocity> transversalVelocity;
    quantity<si::angular_velocity> angularVelocity;

    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false);
        youBotBase->getBasePosition(longitudinalPosition, transversalPosition, orientation);
        youBotBase->getBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true);

    tf2::Quaternion odometryQuaternion;
    odometryQuaternion.setRPY(0, 0, orientation.value());
    odometryQuaternion.normalized();

    nav_msgs::Odometry msgOdometry;
//    msgOdometry.header.frame_id = config.ID_odometryFrame;
//    msgOdometry.child_frame_id = config.ID_odometryChildFrame;

    msgOdometry.header.stamp = ros::Time::now();
    msgOdometry.pose.pose.position.x = longitudinalPosition.value();
    msgOdometry.pose.pose.position.y = transversalPosition.value();
    msgOdometry.pose.pose.position.z = 0.0;
    msgOdometry.pose.pose.orientation = tf2::toMsg(odometryQuaternion);
    msgOdometry.twist.twist.linear.x = longitudinalVelocity.value();
    msgOdometry.twist.twist.linear.y = transversalVelocity.value();
    msgOdometry.twist.twist.angular.z = angularVelocity.value();
    return msgOdometry;
}

sensor_msgs::JointState YouBotBaseWrapper::getJointState() const
{
    sensor_msgs::JointState massageJointState;

    std::vector<youbot::JointSensedAngle> jointAngle;
    std::vector<youbot::JointSensedVelocity> jointVelocity;
    std::vector<youbot::JointSensedTorque> jointTorque;

    jointAngle.resize(config->numOfWheels);
    jointVelocity.resize(config->numOfWheels);
    jointTorque.resize(config->numOfWheels);

    try{
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
            massageJointState.header.stamp = ros::Time::now();
            youBotBase->getJointData(jointAngle);
            youBotBase->getJointData(jointTorque);
            youBotBase->getJointData(jointVelocity);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);

        for (int wheel = 0; wheel < config->numOfWheels; ++wheel){
            massageJointState.name.emplace_back("temp");
            massageJointState.position.emplace_back(jointAngle[wheel].angle.value());
            massageJointState.velocity.emplace_back(jointVelocity[wheel].angularVelocity.value());
            massageJointState.effort.emplace_back(jointTorque[wheel].torque.value());
        }
        massageJointState.position[0] = -massageJointState.position[0];
        massageJointState.position[2] = -massageJointState.position[2];
    } catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot get base joint State: %s", errorMessage.c_str());
    }
    return massageJointState;
}

void YouBotBaseWrapper::callbackSetBaseVelocity(const geometry_msgs::Twist& msgBaseVelocity) const
{
    const quantity<si::velocity> longitudinalVelocity = msgBaseVelocity.linear.x * meter_per_second;
    const quantity<si::velocity> transversalVelocity = msgBaseVelocity.linear.y * meter_per_second;
    const quantity<si::angular_velocity> angularVelocity = msgBaseVelocity.angular.z * radian_per_second;

    try{
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
            youBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot set base velocities: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetBasePosition(const geometry_msgs::Pose2D& msgBasePosition) const
{
    const quantity<si::length> longitudinalPosition = msgBasePosition.x * meter;
    const quantity<si::length> transversalPosition = msgBasePosition.y * meter;
    const quantity<plane_angle> orientation = msgBasePosition.theta * radian;

    try{
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
            youBotBase->setBasePosition(longitudinalPosition, transversalPosition, orientation);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot set base positions: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetJointVelocity(const std_msgs::Float32MultiArray::ConstPtr& msgJointVelocity) const
{
    try{
        std::vector<youbot::JointVelocitySetpoint> jointVelocitySetpoint;
        for(float data : msgJointVelocity->data)
        {
            jointVelocitySetpoint.emplace_back( data *  radian_per_second);
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
            youBotBase->setJointData(jointVelocitySetpoint);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot set base joints velocity: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetJointCurrent(const std_msgs::Float32MultiArray::ConstPtr& msgJointCurrent) const
{
    try{
        std::vector<youbot::JointCurrentSetpoint> JointCurrentSetpoint;
        for(float data : msgJointCurrent->data){
            JointCurrentSetpoint.emplace_back(data * ampere);
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
            youBotBase->setJointData(JointCurrentSetpoint);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot set base joints current: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetJointToque(const std_msgs::Float32MultiArray::ConstPtr& msgJointTorque) const
{
    try{
        std::vector<youbot::JointTorqueSetpoint> JointTorqueSetpoint;
        for(float iter : msgJointTorque->data){
            JointTorqueSetpoint.emplace_back(iter * newton_meter);
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
            youBotBase->setJointData(JointTorqueSetpoint);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    }
    catch (const std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot set base joints torque: %s", errorMessage.c_str());
    }
}

}