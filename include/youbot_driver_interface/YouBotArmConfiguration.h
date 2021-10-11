#include <string>


#include <ros/ros.h>

#define mkstr2(X) #X

namespace youBot{

class YouBotArmConfiguration
{
public:
    YouBotArmConfiguration(ros::NodeHandle n);
    ~YouBotArmConfiguration();


    std::string armName;
    std::string configurationFilePath;
    
    int numberOfJoints;
    int numberOfGripper;

    std::vector<std::string> ID_jointNames;
    std::vector<std::string> ID_gripperFingerNames;


private:
    ros::NodeHandle node;

};

}

