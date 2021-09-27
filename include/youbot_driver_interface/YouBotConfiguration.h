#include "ros/ros.h"
#include <youbot_driver/youbot/YouBotBase.hpp>

namespace youBot
{

    class YouBotBaseConfiguration{
    public:
        YouBotBaseConfiguration();
        virtual ~YouBotBaseConfiguration();

        std::string baseID;
        std::vector<std::string> wheelNames;


        ros::Subscriber baseCommandSubscriber;   

        ros::Publisher baseOdometryPublisher;
        ros::Publisher baseJointStatePublisher;

        ros::ServiceServer switchOffMotorsService;
        ros::ServiceServer switchONMotorsService;
    };


    class YouBotConfiguration{
    public:
        YouBotConfiguration();
        virtual ~YouBotConfiguration();
        
        bool hasBase;
        bool hasArms;

        std::string configurationFilePath;
        double youBotDriverCycleFrequencyInHz;

        YouBotBaseConfiguration baseConfiguration;

    };

}
