#include "YouBotArmWrapper.h"
#include <robot_state_publisher/robot_state_publisher.h>

namespace youBot{

YouBotArmWrapper::YouBotArmWrapper(ros::NodeHandle n)
:node(n){

    config = YouBotConfiguration::GetInstance(node);

    if(config->armControlType["armJointPositionControl"]){
        subscriberJointPosition = node.subscribe("arm/joint_position", 1, &YouBotArmWrapper::callbackSetJointPosition, this);
    } if(config->armControlType["armJointVelocityControl"]){
        subscriberJointVelocity = node.subscribe("arm/joint_velocity", 1, &YouBotArmWrapper::callbackSetJointVelocity, this);
    } if(config->armControlType["armJointToqueControl"]){
        subscriberJointTorque = node.subscribe("arm/joint_torque", 1, &YouBotArmWrapper::callbackSetJointTorque, this);
    }
    subscriberGripperPosition = node.subscribe("arm/gripperPosition", 1, &YouBotArmWrapper::callbackSetGripperPosition, this);

    publisherJointState = node.advertise<sensor_msgs::JointState>("arm/joint_states", 1000);

}

YouBotArmWrapper::~YouBotArmWrapper(){
    delete youBotArm;
}

void YouBotArmWrapper::initialize(){
    try{
        youBotArm = new youbot::YouBotManipulator(config->armName, config->configFilePath);
        youBotArm->doJointCommutation();
        youBotArm->calibrateManipulator();
        youBotArm->calibrateGripper();
    } catch (std::exception& e){
        const std::string errorMessage = e.what();
        ROS_FATAL("%s", errorMessage.c_str());
        ROS_ERROR("Arm \"%s\" could not be initialized.", config->armName.c_str());
    }
    ROS_INFO("Arm is initialized.");
}

void YouBotArmWrapper::dataUpdateAndPublish(){
    try {
        
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
        youBotArm->getJointData(jointAngle);
        youBotArm->getJointData(jointVelocity);
        youBotArm->getJointData(jointTorque);

        if (this->gripperCycleCounter <= 0) {
            this->gripperCycleCounter = config->driverCycleFrequencyInHz/5;
            youBotArm->getArmGripper().getGripperBar1().getData(gripperBar1Position);
            youBotArm->getArmGripper().getGripperBar2().getData(gripperBar2Position);
        }
        this->gripperCycleCounter--;

        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);

        sensor_msgs::JointState msgJointState;
        msgJointState.header.stamp = ros::Time::now();
        msgJointState.name.resize(config->numOfJoints + config->numOfGripper);
        msgJointState.position.resize(config->numOfJoints + config->numOfGripper);
        msgJointState.velocity.resize(config->numOfJoints + config->numOfGripper);
        msgJointState.effort.resize(config->numOfJoints + config->numOfGripper);

        for (int i = 0; i < config->numOfJoints; ++i)
        {
            msgJointState.name[i] = config->name_jointsArm[i];
            msgJointState.position[i] = jointAngle[i].angle.value();
            msgJointState.velocity[i] = jointVelocity[i].angularVelocity.value();
            msgJointState.effort[i] = jointTorque[i].torque.value();
        }

        msgJointState.name[config->numOfJoints + 0] = config->name_gripperFingerNames[0];
        msgJointState.position[config->numOfJoints + 0] = gripperBar1Position.barPosition.value();

        msgJointState.name[config->numOfJoints + 1] = config->name_gripperFingerNames[1];
        msgJointState.position[config->numOfJoints + 1] = gripperBar2Position.barPosition.value();

        publisherJointState.publish(msgJointState);

    } catch (std::exception& e){
        const std::string errorMessage = e.what();
        ROS_WARN("Cannot read gripper values: %s", errorMessage.c_str());
    }
}

void YouBotArmWrapper::callbackSetJointPosition(const brics_actuator::JointPositionsConstPtr& massegeJointPosition){
    try{
        youbot::JointAngleSetpoint jointPositionsSetpoint;

        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
        for(int jointNumber = 0; jointNumber < config->numOfJoints; ++jointNumber){
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
        for(int jointNumber = 0; jointNumber < config->numOfJoints; ++jointNumber){
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
        for(int jointNumber = 0; jointNumber < config->numOfJoints; ++jointNumber){
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