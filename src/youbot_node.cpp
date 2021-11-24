#include "ros/ros.h"
#include "youbot_driver_interface/YouBotDriverWrapper.h"

void configurate(youBot::YouBotDriverWrapper &youBot, ros::NodeHandle &n);

int main(int argc, char **argv){
    ros::init(argc, argv, "youbot_driver_interface");
    ros::NodeHandle n;
    youBot::YouBotDriverWrapper youBot(n);

    ros::Rate rate(youBot.base.config.youBotDriverCycleFrequencyInHz);    
    try {
		youbot::EthercatMaster::getInstance("youbot-ethercat.cfg", youBot.base.config.configurationFilePath);
        ROS_INFO("Ethercat initialize");
	} catch (std::exception& e)	{
		ROS_ERROR("No EtherCAT connection:");
		ROS_FATAL("%s", e.what());
	}
    youBot.base.initializeBase();

    while(n.ok()){
        ros::spinOnce();
        youBot.base.dataUpdateAndPublish();
        rate.sleep();
    }
    return 0;
}