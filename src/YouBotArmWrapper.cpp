#include "YouBotArmWrapper.h"

namespace youBot{

YouBotArmWrapper::YouBotArmWrapper(ros::NodeHandle n)
:node(n), config(n){

    publisherJointState = node.advertise<sensor_msgs::JointState>("arm/data", 1000);

}

YouBotArmWrapper::~YouBotArmWrapper(){
    delete youBotArm;
}

void YouBotArmWrapper::initializeArm(){
    try{
        youBotArm = new youbot::YouBotManipulator(config.armName, config.configurationFilePath);
        youBotArm->doJointCommutation();
        youBotArm->calibrateManipulator();
        youBotArm->calibrateGripper();
    }
    catch (std::exception& e){
        std::string errorMessage = e.what();
        ROS_FATAL("%s", errorMessage.c_str());
        ROS_ERROR("Arm \"%s\" could not be initialized.", config.armName.c_str());
    }
    ROS_INFO("Arm is initialized.");
}

void YouBotArmWrapper::readJointsSensor(){

    try {
        massageJointState.name.resize(7);
        massageJointState.position.resize(7);
        massageJointState.velocity.resize(7);
        massageJointState.effort.resize(7);

        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
        massageJointState.header.stamp = ros::Time::now();
        youBotArm->getJointData(jointAngle);
        youBotArm->getJointData(jointVelocity);
        youBotArm->getJointData(jointTorque);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);


            
        for (int i = 0; i < 5; ++i)
        {
            massageJointState.name[i] = config.ID_jointNames[i];
            massageJointState.position[i] = jointAngle[i].angle.value();
            massageJointState.velocity[i] = jointVelocity[i].angularVelocity.value();
            massageJointState.effort[i] = jointTorque[i].torque.value();
        }
        publisherJointState.publish(massageJointState);
    }
    
    catch (std::exception& e)
    {
        std::string errorMessage = e.what();
        ROS_WARN("Cannot read gripper values: %s", errorMessage.c_str());
    }


}

}