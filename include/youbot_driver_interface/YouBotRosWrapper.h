/* BOOST includes */
#include <boost/units/io.hpp>

/* ROS includes */
#include "ros/ros.h"

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

/* youbot includes */
#include "youbot_driver/youbot/YouBotBase.hpp"

#define mkstr2(X) #X

namespace youBot {

class YouBotConfiguration {
public:
    static YouBotConfiguration *GetInstance(ros::NodeHandle n);

    int driverCycleFrequencyInHz;
    std::string configFilePath;

    int numOfWheels;
    std::string baseName;
    std::map<std::string, bool> baseControlType;

    int numOfArmJoints;
    int numOfGripper;
    std::string armName;
    std::map<std::string, bool> armControlType;

    std::vector<std::string> name_wheels;
    std::vector<std::string> name_jointsArm;
    std::vector<std::string> name_gripperFinger;

    std::string name_odomFrame;
    std::string name_odomChildFrame;

    YouBotConfiguration(YouBotConfiguration &other) = delete;
    void operator=(const YouBotConfiguration&) = delete;

protected:
    explicit YouBotConfiguration(ros::NodeHandle *n);
    static YouBotConfiguration* config;

private:
    ros::NodeHandle node;
};


struct BasePosition
{
    double X;
    double Y;
    double orientation;

    BasePosition(): X(0), Y(0), orientation(0) {};
};


struct BaseVelocity
{
    double X;
    double Y;
    double angularZ;

    BaseVelocity(): X(0), Y(0), angularZ(0) {};
};

struct SetPointJointState
{
    std::vector<double> Angle;
    std::vector<double> Velocity;
    std::vector<double> Torque;
};

class YouBotBaseWrapper
{
public:
    YouBotBaseWrapper(NodeHandle n);

private:
    ros::NodeHandle node;

    ros::Subscriber subSetpointPosition;
    void callbackSetpointPosition(const geometry_msgs::Pose2D& msg);
    BasePosition setpointPosition;

    ros::Subscriber subSetpointVelocity;
    void callbackSetpointVelocity(const geometry_msgs::Twist& msg);
    BaseVelocity setpointVelocity;

    ros::Subscriber subSetpointJoint;
    void callbackSetpointJointState(const sensor_msgs::JointState& msg);
    SetPointJointState setpointJointState;
};

}