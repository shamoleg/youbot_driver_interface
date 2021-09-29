#include "ros/ros.h"
#include "youbot_driver_interface/YouBotDriverWrapper.h"

#define mkstr2(X) #X

void configurate(youBot::YouBotDriverWrapper &youBot, ros::NodeHandle &n);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_driver_interface");
    ros::NodeHandle n;
    youBot::YouBotDriverWrapper youBot(n);

    n.param("youBotDriverCycleFrequencyInHz", youBot.base.youBotConfiguration.youBotDriverCycleFrequencyInHz, 40.0);
    n.param<std::string>("youBotBaseName", youBot.base.youBotConfiguration.baseConfiguration.baseID, "youbot-base");
    n.param<std::string>("youBotConfigurationFilePath", youBot.base.youBotConfiguration.configurationFilePath, mkstr2(YOUBOT_CONFIGURATIONS_DIR));
    ros::Rate rate(youBot.base.youBotConfiguration.youBotDriverCycleFrequencyInHz);


	try {
		youbot::EthercatMaster::getInstance("youbot-ethercat.cfg", youBot.base.youBotConfiguration.configurationFilePath);
        ROS_INFO("Ethercat initialize");
	} catch (std::exception& e)	{
		ROS_ERROR("No EtherCAT connection:");
		ROS_FATAL("%s", e.what());
		return 0;
	}
    
    while(n.ok()){
        ros::spinOnce();
        rate.sleep();
    }
    // youBot.stop();
    return 0;
}