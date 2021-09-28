/* BOOST includes */
#include <boost/units/io.hpp>

/* ROS includes */
#include "youbot_msgs/ReadingsFromSensors.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Empty.h"

#include "tf/transform_broadcaster.h"

/* OODL includes */
#include "YouBotConfiguration.h"
#include <youbot_driver/youbot/YouBotBase.hpp>

namespace youBot
{

/**
 * @brief Wrapper class to map ROS messages to OODL method calls for the youBot platform.
 */
class YouBotBaseWrapper
{
public:

    YouBotBaseWrapper(ros::NodeHandle n);
    ~YouBotBaseWrapper();    
    
    youbot::YouBotBase* youBotBase;
    
    void initializeBase(std::string baseName);

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

    int move();
    void stop();

    /* Configuration: */
    YouBotConfiguration youBotConfiguration;
    ros::NodeHandle node;
    
    

    // std::vector<youbot::JointAngleSetpoint> JointData;
    // std::vector<youbot::JointVelocitySetpoint> JointData;
    // std::vector<youbot::JointCurrentSetpoint> JointData;
    // std::vector<youbot::JointTorqueSetpoint> JointData;

private:
    // ros::Subscriber sub3;
    // YouBotBaseWrapper(){};

    int setBaseJointData(auto data);

    
    ros::Time currentTime;

    nav_msgs::Odometry odometryMessage;

    geometry_msgs::TransformStamped odometryTransform;
    geometry_msgs::Quaternion odometryQuaternion;

    youbot_msgs::ReadingsFromSensors baseJointStateMessage;
};

}

