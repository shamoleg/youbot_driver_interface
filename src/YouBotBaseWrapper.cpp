#include "youbot_driver_interface/YouBotBaseWrapper.h"
#include <vector>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
 

namespace youBot
{

YouBotBaseWrapper::YouBotBaseWrapper(ros::NodeHandle n):
node(n){
    this->initializeBase("youbot-base");

    subscriberBaseVelocity = node.subscribe("base/velocity", 1000, &YouBotBaseWrapper::callbackSetBaseVelocity, this);
    subscriberBasePosition = node.subscribe("base/position", 1000, &YouBotBaseWrapper::callbackSetBasePosition, this);
    subscriberJointVelocity = node.subscribe("base/joints/velocity", 1000, &YouBotBaseWrapper::callbackSetJointVelocity, this);
    subscriberJointCurrent = node.subscribe("base/joints/current", 1000, &YouBotBaseWrapper::callbackSetJointCurrent, this);
    subscriberJointToque = node.subscribe("base/joints/toque", 1000, &YouBotBaseWrapper::callbackSetJointToque, this);

    publisherOdometry = node.advertise<nav_msgs::Odometry>("base/odom", 1000);
    publisherJointsSensorData = node.advertise<youbot_msgs::ReadingsFromSensors>("base/joints/data", 1000);
}

YouBotBaseWrapper::~YouBotBaseWrapper()
{
    delete youBotBase;
    youBotConfiguration.hasBase = false;
}


void YouBotBaseWrapper::initializeBase(std::string baseName = "youbot-base")
{
    try{
        youBotBase = new youbot::YouBotBase(baseName, youBotConfiguration.configurationFilePath);
        youBotBase->doJointCommutation();
    }
    catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_FATAL("%s", errorMessage.c_str());
        ROS_ERROR("Base \"%s\" could not be initialized.", baseName.c_str());
        youBotConfiguration.hasBase = false;
        return;
    }

    ROS_INFO("Base is initialized.");
    youBotConfiguration.hasBase = true;
}

void YouBotBaseWrapper::dataUpdateAndPublish(){
    this->calculationOdometry();
    this->readJointsSensor();

    publisherOdometry.publish(odometryMessage);
    publisherJointsSensorData.publish(jointsSensorDataMessage);
}

void YouBotBaseWrapper::calculationOdometry(){
    try{
        youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false);
        quantity<si::length> longitudinalPosition;
        quantity<si::length> transversalPosition;
        quantity<plane_angle> orientation;
        youBotBase->getBasePosition(longitudinalPosition, transversalPosition, orientation);

        quantity<si::velocity> longitudinalVelocity;
        quantity<si::velocity> transversalVelocity;
        quantity<si::angular_velocity> angularVelocity;   
        youBotBase->getBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);

        tf2::Quaternion odometryQuaternion;
        odometryQuaternion.setRPY(0, 0, orientation.value());
        odometryQuaternion.normalized();

        currentTime = ros::Time::now();
        odometryTransform.header.stamp = currentTime;
        odometryTransform.header.frame_id = "youBotOdometryFrameID";
        odometryTransform.child_frame_id = "youBotOdometryChildFrameID";
        odometryTransform.transform.translation.x = longitudinalPosition.value();
        odometryTransform.transform.translation.y = transversalPosition.value();
        odometryTransform.transform.translation.z = 0.0;
        odometryTransform.transform.rotation = tf2::toMsg(odometryQuaternion);
        br.sendTransform(odometryTransform);

        currentTime = ros::Time::now();
        odometryMessage.header.stamp = currentTime;
        odometryMessage.header.frame_id = "youBotOdometryFrameID";
        odometryMessage.child_frame_id = "youBotOdometryChildFrameID";
        odometryMessage.pose.pose.position.x = longitudinalPosition.value();
        odometryMessage.pose.pose.position.y = transversalPosition.value();
        odometryMessage.pose.pose.position.z = 0.0;
        odometryMessage.pose.pose.orientation = tf2::toMsg(odometryQuaternion);
        odometryMessage.twist.twist.linear.x = longitudinalVelocity.value();
        odometryMessage.twist.twist.linear.y = transversalVelocity.value();
        odometryMessage.twist.twist.angular.z = angularVelocity.value();
        youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true);
    } catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_WARN("Cannot get base odometry: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::readJointsSensor(){
    try{
        int youBotNumberOfWheels = 4;
        std::vector<youbot::JointSensedAngle> jointAngle(youBotNumberOfWheels);
        std::vector<youbot::JointSensedTorque> jointTorque(youBotNumberOfWheels);
        std::vector<youbot::JointSensedCurrent> jointCurrent(youBotNumberOfWheels);
        std::vector<youbot::JointSensedVelocity> jointVelocity(youBotNumberOfWheels);

        youBotBase->getJointData(jointAngle); 
        youBotBase->getJointData(jointTorque);
        youBotBase->getJointData(jointCurrent);
        youBotBase->getJointData(jointVelocity);

        currentTime = ros::Time::now();
        jointsSensorDataMessage.header.stamp = currentTime;
        jointsSensorDataMessage.name.resize(youBotNumberOfWheels);
        jointsSensorDataMessage.torque.resize(youBotNumberOfWheels);
        jointsSensorDataMessage.current.resize(youBotNumberOfWheels);
        jointsSensorDataMessage.position.resize(youBotNumberOfWheels);
        jointsSensorDataMessage.velocity.resize(youBotNumberOfWheels);

        for (int i = 0; i < youBotNumberOfWheels; ++i)
        {
            jointsSensorDataMessage.name[i] = youBotConfiguration.baseConfiguration.wheelNames[i];
            jointsSensorDataMessage.torque[i] = jointTorque[i].torque.value();
            jointsSensorDataMessage.current[i] = jointCurrent[i].current.value();
            jointsSensorDataMessage.position[i] = jointAngle[i].angle.value();
            jointsSensorDataMessage.velocity[i] = jointVelocity[i].angularVelocity.value();
        }
        jointsSensorDataMessage.position[0] = -jointsSensorDataMessage.position[0];
        jointsSensorDataMessage.position[2] = -jointsSensorDataMessage.position[2];
    } catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_WARN("Cannot get base odometry: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetBaseVelocity(const geometry_msgs::Twist& msgBaseVelocity){
    quantity<si::velocity> longitudinalVelocity = msgBaseVelocity.linear.x * meter_per_second;
    quantity<si::velocity> transversalVelocity = msgBaseVelocity.linear.y * meter_per_second;
    quantity<si::angular_velocity> angularVelocity = msgBaseVelocity.angular.z * radian_per_second;

    try{
        youBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
    }
    catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_WARN("Cannot set base velocities: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetBasePosition(const geometry_msgs::Pose2D& msgBasePosition){
    quantity<si::length> longitudinalPosition = msgBasePosition.x * meter;
    quantity<si::length> transversalPosition = msgBasePosition.y * meter;
    quantity<plane_angle> orientation = msgBasePosition.theta * radian;

    try{
        youBotBase->setBasePosition(longitudinalPosition, transversalPosition, orientation);
    }
    catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_WARN("Cannot set base positions: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetJointVelocity(const std_msgs::Float32MultiArray::ConstPtr& msgJointVelocity){
    try{
        int jointNumber = 0;
        std::vector<youbot::JointVelocitySetpoint> jointVelocitySetpoint;
        jointVelocitySetpoint.resize(4);
        for(std::vector<float>::const_iterator iter = msgJointVelocity->data.begin(); iter != msgJointVelocity->data.end(); ++iter){
            jointVelocitySetpoint[jointNumber].angularVelocity = *iter *  radian_per_second;
            jointNumber++;
        }

        youBotBase->setJointData(jointVelocitySetpoint);
    }
    catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_WARN("Cannot set base joints velocity: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetJointCurrent(const std_msgs::Float32MultiArray::ConstPtr& msgJointCurrent){
    try{
        int jointNumber = 0;
        std::vector<youbot::JointCurrentSetpoint> JointCurrentSetpoint;
        JointCurrentSetpoint.resize(4);
        for(std::vector<float>::const_iterator iter =  msgJointCurrent->data.begin(); iter !=  msgJointCurrent->data.end(); ++iter){
            JointCurrentSetpoint[jointNumber].current = *iter * ampere;
            jointNumber++;
        }

        youBotBase->setJointData(JointCurrentSetpoint);
    }
    catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_WARN("Cannot set base joints current: %s", errorMessage.c_str());
    }
}

void YouBotBaseWrapper::callbackSetJointToque(const std_msgs::Float32MultiArray::ConstPtr& msgJointTorque){
    try{
        int jointNumber = 0;
        std::vector<youbot::JointTorqueSetpoint> JointTorqueSetpoint;
        JointTorqueSetpoint.resize(4);
        for(std::vector<float>::const_iterator iter =  msgJointTorque->data.begin(); iter !=  msgJointTorque->data.end(); ++iter){
            JointTorqueSetpoint[jointNumber].torque = *iter * newton_meter;
            jointNumber++;
        }

        youBotBase->setJointData(JointTorqueSetpoint);
    }
    catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_WARN("Cannot set base joints torque: %s", errorMessage.c_str());
    }
}

}