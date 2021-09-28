#include "youbot_driver_interface/YouBotBaseWrapper.h"
#include <vector>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
 

namespace youBot
{

YouBotBaseWrapper::YouBotBaseWrapper(ros::NodeHandle n):
node(n){

    subscriberBaseVelocity = node.subscribe("base/velocity", 1000, &YouBotBaseWrapper::callbackSetBaseVelocity, this);
    subscriberBasePosition = node.subscribe("base/position", 1000, &YouBotBaseWrapper::callbackSetBasePosition, this);
    subscriberJointVelocity = node.subscribe("base/joint/velocity", 1000, &YouBotBaseWrapper::callbackSetJointVelocity, this);
    subscriberJointCurrent = node.subscribe("base/joint/current", 1000, &YouBotBaseWrapper::callbackSetJointCurrent, this);
    subscriberJointToque = node.subscribe("base/joint/toque", 1000, &YouBotBaseWrapper::callbackSetJointToque, this);
    
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

void YouBotBaseWrapper::callbackSetBaseVelocity(const geometry_msgs::Twist& msgBaseVelocity){
    if (youBotConfiguration.hasBase){
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
    else{
        ROS_ERROR("No base initialized!");
    }
}

void YouBotBaseWrapper::callbackSetBasePosition(const geometry_msgs::Pose2D& msgBasePosition){
    if (youBotConfiguration.hasBase){
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
    else{
        ROS_ERROR("No base initialized!");
    }
}

void YouBotBaseWrapper::callbackSetJointVelocity(const std_msgs::Float32MultiArray::ConstPtr& msgJointVelocity){
    if (youBotConfiguration.hasBase){
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
    else{
        ROS_ERROR("No base initialized!");
    }
}

void YouBotBaseWrapper::callbackSetJointCurrent(const std_msgs::Float32MultiArray::ConstPtr& msgJointCurrent){
    if (youBotConfiguration.hasBase){

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
    else{
        ROS_ERROR("No base initialized!");
    }
}

void YouBotBaseWrapper::callbackSetJointToque(const std_msgs::Float32MultiArray::ConstPtr& msgJointTorque){
    if (youBotConfiguration.hasBase){
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
    else{
        ROS_ERROR("No base initialized!");
    }
}

void YouBotBaseWrapper::calculationOdometry(){
    try{
        currentTime = ros::Time::now();

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

        odometryTransform.header.stamp = currentTime;
        odometryTransform.header.frame_id = "youBotOdometryFrameID";
        odometryTransform.child_frame_id = "youBotOdometryChildFrameID";
        odometryTransform.transform.translation.x = longitudinalPosition.value();
        odometryTransform.transform.translation.y = transversalPosition.value();
        odometryTransform.transform.translation.z = 0.0;
        odometryTransform.transform.rotation = tf2::toMsg(odometryQuaternion);
        br.sendTransform(odometryTransform);

        odometryMessage.header.stamp = currentTime;
        odometryMessage.header.frame_id = "youBotOdometryFrameID";
        odometryMessage.pose.pose.position.x = longitudinalPosition.value();
        odometryMessage.pose.pose.position.y = transversalPosition.value();
        odometryMessage.pose.pose.position.z = 0.0;
        odometryMessage.pose.pose.orientation = tf2::toMsg(odometryQuaternion);
        odometryMessage.child_frame_id = "youBotOdometryChildFrameID";
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

    currentTime = ros::Time::now();

    youbot::JointSensedAngle jointAngle;
    youbot::JointSensedVelocity jointVelocity;
    youbot::JointSensedTorque jointTorque;
    youbot::JointSensedCurrent jointCurrent;

    int youBotNumberOfWheels = 4;
    baseJointStateMessage.header.stamp = currentTime;
    baseJointStateMessage.name.resize(youBotNumberOfWheels * 2); // *2 because of virtual wheel joints in the URDF description
    baseJointStateMessage.position.resize(youBotNumberOfWheels * 2);
    baseJointStateMessage.velocity.resize(youBotNumberOfWheels * 2);
    baseJointStateMessage.effort.resize(youBotNumberOfWheels * 2);
    baseJointStateMessage.current.resize(youBotNumberOfWheels * 2);

    // ROS_ASSERT((youBotConfiguration.baseConfiguration.wheelNames.size() == static_cast<unsigned int> (youBotNumberOfWheels)));
    for (int i = 0; i < youBotNumberOfWheels; ++i)
    {
        youBotBase->getBaseJoint(i + 1).getData(jointAngle); //youBot joints start with 1 not with 0 -> i + 1
        youBotBase->getBaseJoint(i + 1).getData(jointVelocity);
        youBotBase->getBaseJoint(i + 1).getData(jointTorque);
        youBotBase->getBaseJoint(i + 1).getData(jointCurrent);

        baseJointStateMessage.name[i] = youBotConfiguration.baseConfiguration.wheelNames[i];
        baseJointStateMessage.position[i] = jointAngle.angle.value();
        baseJointStateMessage.velocity[i] = jointVelocity.angularVelocity.value();
        baseJointStateMessage.effort[i] = jointTorque.torque.value();
        baseJointStateMessage.current[i] = jointCurrent.current.value();
    }

    /*
        * Here we add values for "virtual" rotation joints in URDF - robot_state_publisher can't
        * handle non-aggregated jointState messages well ...
        */
    // baseJointStateMessage.name[4] = "caster_joint_fl";
    // baseJointStateMessage.position[4] = 0.0;

    // baseJointStateMessage.name[5] = "caster_joint_fr";
    // baseJointStateMessage.position[5] = 0.0;

    // baseJointStateMessage.name[6] = "caster_joint_bl";
    // baseJointStateMessage.position[6] = 0.0;

    // baseJointStateMessage.name[7] = "caster_joint_br";
    // baseJointStateMessage.position[7] = 0.0;

    baseJointStateMessage.position[0] = -baseJointStateMessage.position[0];
    baseJointStateMessage.position[2] = -baseJointStateMessage.position[2];

}


//TODO: del this
int YouBotBaseWrapper::move() {
    std::vector<youbot::JointCurrentSetpoint> currentStopMovement;
    currentStopMovement.resize(4);    
    currentStopMovement[0] = 0.18 * ampere;
    currentStopMovement[1] = 0.18 * ampere;
    currentStopMovement[2] = 0.18 * ampere;
    currentStopMovement[3] = 0.18 * ampere;
    // this->setBaseJointData(currentStopMovement);

}

}