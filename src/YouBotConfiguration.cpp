//
// Created by sham on 15.02.2022.
//
#include "YouBotConfiguration.h"
namespace youBot{

YouBotConfiguration *YouBotConfiguration::config = nullptr;

YouBotConfiguration *YouBotConfiguration::GetInstance(ros::NodeHandle node){
    if(config == nullptr){
        config = new YouBotConfiguration(node);
    }
    return config;
}

YouBotConfiguration::YouBotConfiguration(ros::NodeHandle n)
:node(n){

    this->node.param("youBotDriverCycleFrequencyInHz", this->driverCycleFrequencyInHz, 30);
    this->node.param<std::string>("youBotConfigurationFilePath", this->configFilePath, mkstr2(YOUBOT_CONFIGURATIONS_DIR));

    this->numOfWheels = 4;
    this->node.param<std::string>("youBotBaseName", this->baseName, "youbot-base");

    this->numOfJoints = 5;
    this->numOfGripper = 2;
    this->node.param<std::string>("youBotArmName",  this->armName,"youbot-manipulator");

    //set default values for control types and load values from param
    this->baseControlType = {{"baseVelocityControl",true},
                             {"basePositionControl",false},
                             {"baseJointVelocityControl", false},
                             {"baseJointCurrentControl",false},
                             {"baseJointToqueControl",false},
    };
    this->armControlType = {{"armJointPositionControl",true},
                            {"armJointVelocityControl",false},
                            {"armJointToqueControl",false},
    };

    this->node.param<std::map<std::string, bool>>("baseControlType", this->baseControlType, this->baseControlType);
    this->node.param<std::map<std::string, bool>>("armControlType", this->armControlType, this->armControlType);

}

}


