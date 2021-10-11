#include "YouBotArmWrapper.h"

namespace youBot{

YouBotArmWrapper::YouBotArmWrapper(ros::NodeHandle n)
:node(n), config(n){

    subscriberJointPosition = node.subscribe("arm/jointPosition", 1, &YouBotArmWrapper::callbackSetJointPosition, this);
    subscriberJointVelocity = node.subscribe("arm/jointVelocity", 1, &YouBotArmWrapper::callbackSetJointVelocity, this);
    subscriberJointTorque = node.subscribe("arm/jointTorque", 1, &YouBotArmWrapper::callbackSetJointTorque, this);
    subscriberGripperPosition = node.subscribe("arm/gripperPosition", 1, &YouBotArmWrapper::callbackSetGripperPosition, this);

    publisherJointState = node.advertise<sensor_msgs::JointState>("arm/data", 1000);

    massageJointState.name.resize(config.numberOfJoints + config.numberOfGripper);
    massageJointState.position.resize(config.numberOfJoints + config.numberOfGripper);
    massageJointState.velocity.resize(config.numberOfJoints + config.numberOfGripper);
    massageJointState.effort.resize(config.numberOfJoints + config.numberOfGripper);
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
    } catch (std::exception& e){
        const std::string errorMessage = e.what();
        ROS_FATAL("%s", errorMessage.c_str());
        ROS_ERROR("Arm \"%s\" could not be initialized.", config.armName.c_str());
    }
    ROS_INFO("Arm is initialized.");
}

void YouBotArmWrapper::readJointsSensor(){
    try {
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
        massageJointState.header.stamp = ros::Time::now();
        youBotArm->getJointData(jointAngle);
        youBotArm->getJointData(jointVelocity);
        youBotArm->getJointData(jointTorque);
        youBotArm->getArmGripper().getGripperBar1().getData(gripperBar1Position);
        youBotArm->getArmGripper().getGripperBar2().getData(gripperBar2Position);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);

        for (int i = 0; i < config.numberOfJoints; ++i)
        {
            massageJointState.name[i] = config.ID_jointNames[i];
            massageJointState.position[i] = jointAngle[i].angle.value();
            massageJointState.velocity[i] = jointVelocity[i].angularVelocity.value();
            massageJointState.effort[i] = jointTorque[i].torque.value();
        }

        massageJointState.name[config.numberOfJoints + 0] = config.ID_gripperFingerNames[0];
        massageJointState.position[config.numberOfJoints + 0] = gripperBar1Position.barPosition.value();

        massageJointState.name[config.numberOfJoints + 1] = config.ID_gripperFingerNames[1];
        massageJointState.position[config.numberOfJoints + 1] = gripperBar2Position.barPosition.value();

        publisherJointState.publish(massageJointState);

    } catch (std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot read gripper values: %s", errorMessage.c_str());
    }
}

void YouBotArmWrapper::callbackSetJointPosition(const brics_actuator::JointPositionsConstPtr& massegeJointPosition){
    try{
        youbot::JointAngleSetpoint jointPositionsSetpoint;

        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
        for(int jointNumber = 0; jointNumber < config.numberOfJoints; ++jointNumber){
            jointPositionsSetpoint = massegeJointPosition->positions[jointNumber].value * radians;
            youBotArm->getArmJoint(jointNumber + 1).setData(jointPositionsSetpoint);
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);

    } catch(std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot read gripper values: %s", errorMessage.c_str());
    }
}

void YouBotArmWrapper::callbackSetJointVelocity(const brics_actuator::JointVelocitiesConstPtr& massegeJointVelocity){
    try{
        youbot::JointVelocitySetpoint  jointVelocitySetpoint;

        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
        for(int jointNumber = 0; jointNumber < config.numberOfJoints; ++jointNumber){
            jointVelocitySetpoint =  massegeJointVelocity->velocities[jointNumber].value * radian_per_second;
            youBotArm->getArmJoint(jointNumber + 1).setData(jointVelocitySetpoint);
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);

    } catch(std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot read gripper values: %s", errorMessage.c_str());
    }
}

void YouBotArmWrapper::callbackSetJointTorque(const brics_actuator::JointTorquesConstPtr& massegeJointTorque){
    try{
        youbot::JointTorqueSetpoint  jointTorqueSetpoint;
        
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
        for(int jointNumber = 0; jointNumber < config.numberOfJoints; ++jointNumber){
            jointTorqueSetpoint = massegeJointTorque->torques[jointNumber].value * newton_meters;
            youBotArm->getArmJoint(jointNumber + 1).setData(jointTorqueSetpoint);
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);

    } catch(std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot read gripper values: %s", errorMessage.c_str());
    }
}

void YouBotArmWrapper::callbackSetGripperPosition(const brics_actuator::JointPositionsConstPtr& massegeGripperPosition){
    try{
        youbot::GripperBarPositionSetPoint rightGripperFingerPosition;
        youbot::GripperBarPositionSetPoint leftGripperFingerPosition;

        rightGripperFingerPosition.barPosition = massegeGripperPosition->positions[0].value * meter;
        leftGripperFingerPosition.barPosition = massegeGripperPosition->positions[1].value * meter;

        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
            youBotArm->getArmGripper().getGripperBar1().setData(rightGripperFingerPosition);
            youBotArm->getArmGripper().getGripperBar2().setData(leftGripperFingerPosition);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);

    } catch (std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot read gripper values: %s", errorMessage.c_str());
    }
}

}