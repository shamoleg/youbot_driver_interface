#include "YouBotConfiguration.h"
#include "YouBotBaseWrapper.h"
#include "YouBotArmWrapper.h"


#include "ros/ros.h"
namespace youBot{

class YouBotDriverWrapper{

public:
    explicit YouBotDriverWrapper(ros::NodeHandle n);
    youBot::YouBotConfiguration* config;
    youBot::YouBotBaseWrapper base;
};

}
