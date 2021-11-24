#include "YouBotBaseWrapper.h"
#include "YouBotArmWrapper.h"

#include "ros/ros.h"
namespace youBot{

class YouBotDriverWrapper{

public:
    YouBotDriverWrapper(ros::NodeHandle n);

    YouBotDriverWrapper();
    ros::NodeHandle node;
    youBot::YouBotBaseWrapper base;
};

}
