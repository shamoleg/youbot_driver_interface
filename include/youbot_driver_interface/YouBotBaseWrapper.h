/* BOOST includes */
#include <boost/units/io.hpp>

/* ROS includes */
#include "youbot_msgs/ReadingsFromSensors.h"
#include "sensor_msgs/JointState.h"
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

    void CallbackSetBaseVelocity(const geometry_msgs::Twist& youbotBaseVelocity);
    void CallbackSetBasePosition(const geometry_msgs::Pose2D& youbotBasePosition);
    void CallbackSetJointVelocity(const std_msgs::Int32MultiArray data);
    void CallbackSetJointCurrent(const std_msgs::Int32MultiArray data);
    void CallbackSetJointToque(const std_msgs::Int32MultiArray data);


    int move();
    void stop();

    /* Configuration: */
    YouBotConfiguration youBotConfiguration;
    
    



private:

    YouBotBaseWrapper(){};

    int setJointData(auto data);

    ros::NodeHandle node;
    ros::Time currentTime;

    nav_msgs::Odometry odometryMessage;

    geometry_msgs::TransformStamped odometryTransform;
    geometry_msgs::Quaternion odometryQuaternion;

    youbot_msgs::ReadingsFromSensors baseJointStateMessage;
};

}

