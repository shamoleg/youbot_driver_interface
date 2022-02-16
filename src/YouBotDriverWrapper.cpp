#include "YouBotDriverWrapper.h"

namespace youBot{
    
YouBotDriverWrapper::YouBotDriverWrapper(ros::NodeHandle n)
: base(n){
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

}
