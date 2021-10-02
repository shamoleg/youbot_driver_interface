#include "ros/ros.h"

#define mkstr2(X) #X

namespace youBot
{

class YouBotBaseConfiguration{
public:
    YouBotBaseConfiguration(ros::NodeHandle n);
    virtual ~YouBotBaseConfiguration();
    
    bool hasBase;
    std::string baseName;

    std::string configurationFilePath;
    double youBotDriverCycleFrequencyInHz;

    int numberOfWheels;

    std::string ID_base;
    std::string ID_odometryFrame;
    std::string ID_odometryChildFrame;
    std::vector<std::string> ID_wheels;
    
private:
    ros::NodeHandle node;
};

}
