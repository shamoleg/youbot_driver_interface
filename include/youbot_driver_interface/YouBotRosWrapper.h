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

    int numOfJoints;
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
    explicit YouBotConfiguration(ros::NodeHandle n);
    static YouBotConfiguration* config;

private:
    ros::NodeHandle node;
};

struct BasePosition
{
    double x;
    double y;
    double theta;

    BasePosition(): x(0), y(0), theta(0) {};
    BasePosition(double x, double y, double theta): x(x), y(y), theta(theta) {};
};


struct BaseVelocity
{
    double x;
    double y;
    double wz;

    BaseVelocity(): x(0), y(0), wz(0) {};
    BaseVelocity(double x, double y, double wz): x(x), y(y), wz(wz) {};
};

struct JointState
{
    std::vector<double> angle;
    std::vector<double> velocity;
    std::vector<double> torque;
};

class WrapperYouBotDriverBase
{
public:
    WrapperYouBotDriverBase(std::string baseName, std::string configFilePath);

    void setVelocity(BaseVelocity velocity);
    BaseVelocity getVelocity();

    void setPosition(BasePosition position);
    BasePosition getPosition();

    void setJointState(JointState jointState);
    JointState getJointState();

private:
    youbot::YouBotBase* youBotBase;
    
};

class YouBotBaseWrapper
{
public:
    YouBotBaseWrapper(ros::NodeHandle n);

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
    JointState setpointJointState;
};
}