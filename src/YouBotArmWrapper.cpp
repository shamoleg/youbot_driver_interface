#include "YouBotArmWrapper.h"
#include <robot_state_publisher/robot_state_publisher.h>

namespace youBot{

YouBotArmWrapper::YouBotArmWrapper(ros::NodeHandle n)
:node(n), config(n){

    if(1){
        subscriberJointPosition = node.subscribe("arm/jointPosition", 1, &YouBotArmWrapper::callbackSetJointPosition, this);
    } if(1){
        subscriberJointVelocity = node.subscribe("arm/jointVelocity", 1, &YouBotArmWrapper::callbackSetJointVelocity, this);
    } if(1){
        subscriberJointTorque = node.subscribe("arm/jointTorque", 1, &YouBotArmWrapper::callbackSetJointTorque, this);
    }
    
    subscriberGripperPosition = node.subscribe("arm/gripperPosition", 1, &YouBotArmWrapper::callbackSetGripperPosition, this);

    publisherJointState = node.advertise<sensor_msgs::JointState>("arm/jointState", 1000);

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
    std::vector<geometry_msgs::TransformStamped> transformStamped(config.numberOfJoints);

    try {
        
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false);
        youBotArm->getJointData(jointAngle);
        youBotArm->getJointData(jointVelocity);
        youBotArm->getJointData(jointTorque);
        // youBotArm->getArmGripper().getGripperBar1().getData(gripperBar1Position);
        // youBotArm->getArmGripper().getGripperBar2().getData(gripperBar2Position);
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true);

        massageJointState.header.stamp = ros::Time::now();

        std::vector<tf2::Quaternion> s(5);
        s[0].setRPY(0, 0, 170 * M_PI / 180  - jointAngle[0].angle.value());
        s[1].setRPY(0, -65 * M_PI / 180 + jointAngle[1].angle.value(), 0);
        s[2].setRPY(0, 146 * M_PI / 180 + jointAngle[2].angle.value(), 0);
        s[3].setRPY(0, -102.5 * M_PI / 180 + jointAngle[3].angle.value(), 0);
        s[4].setRPY(0, 0, 167.5 * M_PI / 180 - jointAngle[4].angle.value());

        transformStamped[0].transform.translation.x = 0.024;
        transformStamped[0].transform.translation.y = 0.0;
        transformStamped[0].transform.translation.z = 0.096;

        transformStamped[1].transform.translation.x = 0.033;
        transformStamped[1].transform.translation.y = 0.0;
        transformStamped[1].transform.translation.z = 0.019;

        transformStamped[2].transform.translation.x = 0.000;
        transformStamped[2].transform.translation.y = 0.0;
        transformStamped[2].transform.translation.z = 0.155;

        transformStamped[3].transform.translation.x = 0.0;
        transformStamped[3].transform.translation.y = 0.0;
        transformStamped[3].transform.translation.z = 0.135;

        transformStamped[4].transform.translation.x = -0.002;
        transformStamped[4].transform.translation.y = 0.0;
        transformStamped[4].transform.translation.z = 0.130;
        
        for (int i = 0; i < config.numberOfJoints; ++i)
        {
            massageJointState.name[i] = config.ID_jointNames[i];
            massageJointState.position[i] = jointAngle[i].angle.value();
            massageJointState.velocity[i] = jointVelocity[i].angularVelocity.value();
            massageJointState.effort[i] = jointTorque[i].torque.value();

            transformStamped[i].header.stamp = ros::Time::now();
            transformStamped[i].header.frame_id = config.ID_jointNames[i];
            transformStamped[i].child_frame_id = config.ID_jointNames[i+1];

            transformStamped[i].transform.rotation.x = s[i].x();
            transformStamped[i].transform.rotation.y = s[i].y();
            transformStamped[i].transform.rotation.z = s[i].z();
            transformStamped[i].transform.rotation.w = s[i].w();

            

        }

        massageJointState.name[config.numberOfJoints + 0] = config.ID_gripperFingerNames[0];
        massageJointState.position[config.numberOfJoints + 0] = gripperBar1Position.barPosition.value();

        massageJointState.name[config.numberOfJoints + 1] = config.ID_gripperFingerNames[1];
        massageJointState.position[config.numberOfJoints + 1] = gripperBar2Position.barPosition.value();

        

        tfBroadcaster.sendTransform(transformStamped);
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