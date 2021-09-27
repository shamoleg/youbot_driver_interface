#include "youbot_driver_interface/YouBotConfiguration.h"

namespace youBot
{

    YouBotBaseConfiguration::YouBotBaseConfiguration()
    {
        /*
        *  numbering of youBot wheels:
        *
        *    FRONT
        *
        * 1 ---+--- 2
        *      |
        *      |
        *      |
        *      |
        * 3 ---+--- 4
        *
        *    BACK
        */
        wheelNames.clear();
        wheelNames.push_back("wheel_joint_fl"); //wheel #1
        wheelNames.push_back("wheel_joint_fr"); //wheel #2
        wheelNames.push_back("wheel_joint_bl"); //wheel #3
        wheelNames.push_back("wheel_joint_br"); //wheel #4
    }

    YouBotBaseConfiguration::~YouBotBaseConfiguration(){
    }

    YouBotConfiguration::YouBotConfiguration(){
    }

    YouBotConfiguration::~YouBotConfiguration(){
    }

}

 
