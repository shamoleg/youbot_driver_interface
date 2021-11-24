#include "YouBotArmConfiguration.h"

namespace youBot{

YouBotArmConfiguration::YouBotArmConfiguration(ros::NodeHandle n)
:node(n){
 
    node.param<bool>("armJointPositionControl", armJointPositionControl, false);
    node.param<bool>("armJointVelocityControl", armJointVelocityControl, true);
    node.param<bool>("armJointToqueControl", armJointToqueControl, false);
    
    node.param<std::string>("youBotConfigurationFilePath", configurationFilePath, mkstr2(YOUBOT_CONFIGURATIONS_DIR));
    node.param<std::string>("youBotArmName", armName, "youbot-manipulator");

    numberOfJoints = 5;
    numberOfGripper = 2;

    // ID_jointNames = {"arm_link_0", "arm_link_1", "arm_link_2", "arm_link_3", "arm_link_4", "arm_link_5", "arm_link_6"};

    ID_jointNames = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5","arm_joint_6"};
    ID_gripperFingerNames = {"gripper_finger_joint_l", "gripper_finger_joint_r"};
}


YouBotArmConfiguration::~YouBotArmConfiguration(){

}

}