#include "ros/ros.h"
#include "youbot_driver_interface/YouBotDriverWrapper.h"

void configurate(youBot::YouBotDriverWrapper &youBot, ros::NodeHandle &n);

int main(int argc, char **argv){
    ros::init(argc, argv, "youbot_driver_interface");
    ros::NodeHandle n;

    youBot::YouBotDriverWrapper youBot(n);

    ros::Rate rate(youBot.config->driverCycleFrequencyInHz);

    youBot.base.initialize();
    youBot.arm.initialize();
    while(n.ok()){
        ros::spinOnce();
        youBot.base.dataUpdateAndPublish();
        youBot.arm.dataUpdateAndPublish();
        rate.sleep();
    }
    return 0;
}