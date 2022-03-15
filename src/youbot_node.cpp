#include "ros/ros.h"
#include "youbot_driver_interface/YouBotDriverWrapper.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "youbot_driver");
    ros::NodeHandle n;

    youBot::YouBotDriverWrapper youBot(n);

    ros::Rate rate(youBot.config->driverCycleFrequencyInHz);

    youBot.initialize();
    while(n.ok()){
        ros::spinOnce();
        youBot.update();
        rate.sleep();
    }
    return 0;
}