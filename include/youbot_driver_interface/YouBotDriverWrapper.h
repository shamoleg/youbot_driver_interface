#include "YouBotBaseWrapper.h"
#include "ros/ros.h"
namespace youBot{

class YouBotDriverWrapper{

public:
    YouBotDriverWrapper(ros::NodeHandle n);
    ~YouBotDriverWrapper();

    YouBotDriverWrapper();
    ros::NodeHandle node;
    youBot::YouBotBaseWrapper base;
    // YouBotConfiguration youBotConfiguration;
};

}
