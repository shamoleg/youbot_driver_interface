#include "youbot_driver_interface/YouBotBaseWrapper.h"
#include <vector>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
 

namespace youBot
{

YouBotBaseWrapper::YouBotBaseWrapper(ros::NodeHandle n):
node(n), config(n){
    if(config.baseVelocityControl){
        subscriberBaseVelocity = node.subscribe("youBotBase/velocity", 1000, &YouBotBaseWrapper::callbackSetBaseVelocity, this);
    } if(config.basePositionControl){
        subscriberBasePosition = node.subscribe("youBotBase/position", 1000, &YouBotBaseWrapper::callbackSetBasePosition, this);
    } if(config.baseJointVelocityControl){
        subscriberJointVelocity = node.subscribe("youBotBase/jointVelocity", 1000, &YouBotBaseWrapper::callbackSetJointVelocity, this);
    } if(config.baseJointCurrentControl){
        subscriberJointCurrent = node.subscribe("youBotBase/jointCurrent", 1000, &YouBotBaseWrapper::callbackSetJointCurrent, this);
    } if(config.baseJointToqueControl){ 
        subscriberJointToque = node.subscribe("youBotBase/jointToque", 1000, &YouBotBaseWrapper::callbackSetJointToque, this);
    }

    publisherOdometry = node.advertise<nav_msgs::Odometry>("base/odom", 1000);
    publisherJointState = node.advertise<sensor_msgs::JointState>("base/jointState", 1000);

    odometryTransform.header.frame_id = config.ID_odometryFrame;
    odometryTransform.child_frame_id = config.ID_odometryChildFrame;

    odometryMessage.header.frame_id = config.ID_odometryFrame;
    odometryMessage.child_frame_id = config.ID_odometryChildFrame;

    jointAngle.resize(config.numberOfWheels);
    jointVelocity.resize(config.numberOfWheels);
    jointTorque.resize(config.numberOfWheels);

    massageJointState.name.resize(config.numberOfWheels);
    massageJointState.position.resize(config.numberOfWheels);
    massageJointState.velocity.resize(config.numberOfWheels);
    massageJointState.effort.resize(config.numberOfWheels);
}

YouBotBaseWrapper::~YouBotBaseWrapper()
{
    delete youBotBase;
    config.hasBase = false;
}


void YouBotBaseWrapper::initializeBase()
{
    try{
        youBotBase = new youbot::YouBotBase(config.baseName, config.configurationFilePath);
        youBotBase->doJointCommutation();
    }
    catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_FATAL("%s", errorMessage.c_str());
        ROS_ERROR("Base \"%s\" could not be initialized.", config.baseName.c_str());
        config.hasBase = false;
        return;
    }
    ROS_INFO("Base is initialized.");
    config.hasBase = true;
}

void YouBotBaseWrapper::dataUpdateAndPublish(){
    this->calculationOdometry();
    this->readJointsSensor();

    publisherOdometry.publish(odometryMessage);
    publisherJointState.publish(massageJointState);
}

void YouBotBaseWrapper::calculationOdometry(){
    try{
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

        odometryTransform.header.stamp = ros::Time::now();
        odometryTransform.transform.translation.x = longitudinalPosition.value();
        odometryTransform.transform.translation.y = transversalPosition.value();
        odometryTransform.transform.translation.z = 0.0;
        odometryTransform.transform.rotation = tf2::toMsg(odometryQuaternion);
        br.sendTransform(odometryTransform);

        odometryMessage.header.stamp = ros::Time::now();
        odometryMessage.pose.pose.position.x = longitudinalPosition.value();
        odometryMessage.pose.pose.position.y = transversalPosition.value();
        odometryMessage.pose.pose.position.z = 0.0;
        odometryMessage.pose.pose.orientation = tf2::toMsg(odometryQuaternion);
        odometryMessage.twist.twist.linear.x = longitudinalVelocity.value();
        odometryMessage.twist.twist.linear.y = transversalVelocity.value();
        odometryMessage.twist.twist.angular.z = angularVelocity.value();
    } catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_WARN("Cannot get base odometry: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::readJointsSensor(){
    try{
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
        massageJointState.header.stamp = ros::Time::now();
        youBotBase->getJointData(jointAngle); 
        youBotBase->getJointData(jointTorque);
        youBotBase->getJointData(jointVelocity);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);

        for (int wheel = 0; wheel < config.numberOfWheels; ++wheel){
            massageJointState.name[wheel] = config.ID_wheels[wheel];
            massageJointState.position[wheel] = jointAngle[wheel].angle.value();
            massageJointState.velocity[wheel] = jointVelocity[wheel].angularVelocity.value();
            massageJointState.effort[wheel] = jointTorque[wheel].torque.value();
        }
        massageJointState.position[0] = -massageJointState.position[0];
        massageJointState.position[2] = -massageJointState.position[2];
    } catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_WARN("Cannot get base odometry: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetBaseVelocity(const geometry_msgs::Twist& massageBaseVelocity){
    const quantity<si::velocity> longitudinalVelocity = massageBaseVelocity.linear.x * meter_per_second;
    const quantity<si::velocity> transversalVelocity = massageBaseVelocity.linear.y * meter_per_second;
    const quantity<si::angular_velocity> angularVelocity = massageBaseVelocity.angular.z * radian_per_second;

    try{
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
        youBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    }
    catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_WARN("Cannot set base velocities: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetBasePosition(const geometry_msgs::Pose2D& massageBasePosition){
    const quantity<si::length> longitudinalPosition = massageBasePosition.x * meter;
    const quantity<si::length> transversalPosition = massageBasePosition.y * meter;
    const quantity<plane_angle> orientation = massageBasePosition.theta * radian;

    try{
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
        youBotBase->setBasePosition(longitudinalPosition, transversalPosition, orientation);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    }
    catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_WARN("Cannot set base positions: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetJointVelocity(const std_msgs::Float32MultiArray::ConstPtr& massageJointVelocity){
    try{
        std::vector<youbot::JointVelocitySetpoint> jointVelocitySetpoint(config.numberOfWheels);

        int jointNumber = 0;
        for(std::vector<float>::const_iterator iter = massageJointVelocity->data.begin(); iter != massageJointVelocity->data.end(); ++iter){
            jointVelocitySetpoint[jointNumber++].angularVelocity = *iter *  radian_per_second;
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
        youBotBase->setJointData(jointVelocitySetpoint);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    }
    catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_WARN("Cannot set base joints velocity: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetJointCurrent(const std_msgs::Float32MultiArray::ConstPtr& massageJointCurrent){
    try{
        std::vector<youbot::JointCurrentSetpoint> JointCurrentSetpoint(config.numberOfWheels);

        int jointNumber = 0;
        for(std::vector<float>::const_iterator iter =  massageJointCurrent->data.begin(); iter !=  massageJointCurrent->data.end(); ++iter){
            JointCurrentSetpoint[jointNumber++].current = *iter * ampere;
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
        youBotBase->setJointData(JointCurrentSetpoint);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    }
    catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_WARN("Cannot set base joints current: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetJointToque(const std_msgs::Float32MultiArray::ConstPtr& massageJointTorque){
    try{
        std::vector<youbot::JointTorqueSetpoint> JointTorqueSetpoint(config.numberOfWheels);

        int jointNumber = 0;
        for(std::vector<float>::const_iterator iter =  massageJointTorque->data.begin(); iter !=  massageJointTorque->data.end(); ++iter){
            JointTorqueSetpoint[jointNumber++].torque = *iter * newton_meter;
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
        youBotBase->setJointData(JointTorqueSetpoint);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);
    }
    catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_WARN("Cannot set base joints torque: %s", errorMessage.c_str());
    }
}

}