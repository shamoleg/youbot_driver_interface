/* BOOST includes */
#include <boost/units/io.hpp>

/* ROS includes */
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>

#include <std_srvs/Empty.h>

/* youbot includes */
#include "YouBotConfiguration.h"

#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_msgs/ReadingsFromSensors.h"

namespace youBot
{

class YouBotBaseWrapper
{
public:

    youbot::YouBotBase* youBotBase;

    YouBotBaseWrapper(ros::NodeHandle n);
    ~YouBotBaseWrapper();    
    
    void initializeBase();
    void dataUpdateAndPublish();

    YouBotBaseConfiguration config;

private:
    YouBotBaseWrapper();

    void readJointsSensor();
    void calculationOdometry();

    void callbackSetBaseVelocity(const geometry_msgs::Twist& msgBaseVelocity);
    void callbackSetBasePosition(const geometry_msgs::Pose2D& msgBasePosition);
    void callbackSetJointVelocity(const std_msgs::Float32MultiArray::ConstPtr& msgJointVelocity);
    void callbackSetJointCurrent(const std_msgs::Float32MultiArray::ConstPtr& msgJointCurrent);
    void callbackSetJointToque(const std_msgs::Float32MultiArray::ConstPtr& msgJointTorque);
    
    ros::Subscriber subscriberBaseVelocity;
    ros::Subscriber subscriberBasePosition;
    ros::Subscriber subscriberJointVelocity;
    ros::Subscriber subscriberJointCurrent;
    ros::Subscriber subscriberJointToque;

    ros::Publisher publisherOdometry;
    ros::Publisher publisherJointsSensorData;
    
    nav_msgs::Odometry odometryMessage;
    youbot_msgs::ReadingsFromSensors jointsSensorDataMessage;

    ros::NodeHandle node;
    ros::Time currentTime;

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped odometryTransform;
};

}

