#include "YouBotBaseConfiguration.h"

namespace youBot
{
YouBotBaseConfiguration::YouBotBaseConfiguration(ros::NodeHandle n):
node(n)
{
    /*
    *  numbering of youBot wheels:
    *    FRONT
    * 1 ---+--- 2
    *      |
    *      |
    *      |
    *      |
    * 3 ---+--- 4
    *    BACK
    */

    numberOfWheels = 4;
    ID_wheels = {"wheel_joint_fl", "wheel_joint_fr", "wheel_joint_bl", "wheel_joint_br"};

    node.param<std::string>("baseControlMethod", baseControlMethod, "all");

    node.param("youBotDriverCycleFrequencyInHz", youBotDriverCycleFrequencyInHz, 100.0);
    node.param<std::string>("youBotBaseName", baseName, "youbot-base");
    node.param<std::string>("youBotConfigurationFilePath", configurationFilePath, mkstr2(YOUBOT_CONFIGURATIONS_DIR));
    node.param<std::string>("ID_base", ID_base, "youbot-base");
    node.param<std::string>("ID_odometryFrame", ID_odometryFrame, "odom");
    node.param<std::string>("ID_odometryChildFrame", ID_odometryChildFrame, "base_footprint");
}

YouBotBaseConfiguration::~YouBotBaseConfiguration(){
}

}

 
