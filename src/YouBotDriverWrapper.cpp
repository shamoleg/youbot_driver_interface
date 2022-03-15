#include "YouBotDriverWrapper.h"

namespace youBot{
    
YouBotDriverWrapper::YouBotDriverWrapper(ros::NodeHandle n)
: base(n), arm(n){
    config = YouBotConfiguration::GetInstance(n);
    this->getEthercatInstance();
}

void YouBotDriverWrapper::getEthercatInstance()
{
    try {
        youbot::EthercatMaster::getInstance("youbot-ethercat.cfg", this->config->configFilePath);
        ROS_INFO("Ethercat initialize");
    } catch (const std::exception& e){
        ROS_ERROR("No EtherCAT connection:");
        ROS_FATAL("%s", e.what());
    }
}

void YouBotDriverWrapper::initialize(){
    base.initialize();
    arm.initialize();
}
void YouBotDriverWrapper::update(){
    base.dataUpdateAndPublish();
    arm.dataUpdateAndPublish();
};


}
