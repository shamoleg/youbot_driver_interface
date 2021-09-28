#include "youbot_driver_interface/YouBotBaseWrapper.h"
#include <vector>
#include <sstream>
 

namespace youBot
{

YouBotBaseWrapper::YouBotBaseWrapper(ros::NodeHandle n):
node(n){

    
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

void YouBotBaseWrapper::CallbackSetBaseVelocity(const geometry_msgs::Twist& msgBaseVelocity){
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

void YouBotBaseWrapper::CallbackSetBasePosition(const geometry_msgs::Pose2D& msgBasePosition){
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

void YouBotBaseWrapper::CallbackSetJointVelocity(const std_msgs::Float32MultiArray::ConstPtr& msgJointVelocity){
    if (youBotConfiguration.hasBase){
        try{
            int jointNumber = 0;
            std::vector<youbot::JointVelocitySetpoint> jointVelocitySetpoint;
            jointVelocitySetpoint.resize(4);
            for(std::vector<float>::const_iterator iter = msgJointVelocity->data.begin(); iter != msgJointVelocity->data.end(); ++iter){
                jointVelocitySetpoint[jointNumber].angularVelocity = *iter *  radian_per_second;
                jointNumber++;
            }

            this->setBaseJointData(jointVelocitySetpoint);
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

void YouBotBaseWrapper::CallbackSetJointCurrent(const std_msgs::Float32MultiArray::ConstPtr& msgJointCurrent){
    if (youBotConfiguration.hasBase){

        try{
            int jointNumber = 0;
            std::vector<youbot::JointCurrentSetpoint> JointCurrentSetpoint;
            JointCurrentSetpoint.resize(4);
            for(std::vector<float>::const_iterator iter =  msgJointCurrent->data.begin(); iter !=  msgJointCurrent->data.end(); ++iter){
                JointCurrentSetpoint[jointNumber].current = *iter * ampere;
                jointNumber++;
            }

            this->setBaseJointData(JointCurrentSetpoint);
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

void YouBotBaseWrapper::CallbackSetJointToque(const std_msgs::Float32MultiArray::ConstPtr& msgJointTorque){
    if (youBotConfiguration.hasBase){
        try{
            int jointNumber = 0;
            std::vector<youbot::JointTorqueSetpoint> JointTorqueSetpoint;
            JointTorqueSetpoint.resize(4);
            for(std::vector<float>::const_iterator iter =  msgJointTorque->data.begin(); iter !=  msgJointTorque->data.end(); ++iter){
                JointTorqueSetpoint[jointNumber].torque = *iter * newton_meter;
                jointNumber++;
            }

            this->setBaseJointData(JointTorqueSetpoint);
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

int YouBotBaseWrapper::setBaseJointData(auto data) {
	if (youBotConfiguration.hasBase) { 
		try {
            youBotBase->setJointData(data);
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			ROS_WARN("Cannot set BaseJointData: %s", errorMessage.c_str());
			return false;
		}
	} else {
		ROS_ERROR("No base initialized!");
		return false;
	}
	return true;
}


//TODO: del this
int YouBotBaseWrapper::move() {
    std::vector<youbot::JointCurrentSetpoint> currentStopMovement;
    currentStopMovement.resize(4);    
    currentStopMovement[0] = 0.18 * ampere;
    currentStopMovement[1] = 0.18 * ampere;
    currentStopMovement[2] = 0.18 * ampere;
    currentStopMovement[3] = 0.18 * ampere;
    this->setBaseJointData(currentStopMovement);

}

}