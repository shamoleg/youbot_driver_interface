#include "youbot_driver_interface/YouBotBaseWrapper.h"
#include <vector>
#include <sstream>


namespace youBot
{

YouBotBaseWrapper::YouBotBaseWrapper(ros::NodeHandle n)
:node(n){
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

void YouBotBaseWrapper::stop(){

}

void YouBotBaseWrapper::CallbackSetBaseVelocity(const geometry_msgs::Twist& youbotBaseVelocity){
    if (youBotConfiguration.hasBase){
        quantity<si::velocity> longitudinalVelocity = youbotBaseVelocity.linear.x * meter_per_second;
        quantity<si::velocity> transversalVelocity = youbotBaseVelocity.linear.y * meter_per_second;
        quantity<si::angular_velocity> angularVelocity = youbotBaseVelocity.angular.z * radian_per_second;

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

void YouBotBaseWrapper::CallbackSetBasePosition(const geometry_msgs::Pose2D& youbotBasePosition){
    if (youBotConfiguration.hasBase){
        quantity<si::length> longitudinalPosition = youbotBasePosition.x * meter;
        quantity<si::length> transversalPosition = youbotBasePosition.y * meter;
        quantity<plane_angle> orientation = youbotBasePosition.theta * radian;

        try{
            youBotBase->setBasePosition(longitudinalPosition, transversalPosition, orientation);
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

//TODO: del this
int YouBotBaseWrapper::move() {
    std::vector<youbot::JointCurrentSetpoint> currentStopMovement;
    currentStopMovement.resize(4);    
    currentStopMovement[0].current = 0.18 * ampere;
    currentStopMovement[1].current = 0.18 * ampere;
    currentStopMovement[2].current = 0.18 * ampere;
    currentStopMovement[3].current = 0.18 * ampere;
    this->setJointData(currentStopMovement);

}

int YouBotBaseWrapper::setJointData(auto data) {
	if (youBotConfiguration.hasBase) { 
		try {
            youBotBase->setJointData(data);

		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			ROS_WARN("Cannot switch off the base motors: %s", errorMessage.c_str());
			return false;
		}
	} else {
		ROS_ERROR("No base initialized!");
		return false;
	}
	return true;
}

}