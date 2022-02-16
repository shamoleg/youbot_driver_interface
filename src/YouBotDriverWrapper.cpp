#include "YouBotDriverWrapper.h"

namespace youBot{
    
YouBotDriverWrapper::YouBotDriverWrapper(ros::NodeHandle n)
: base(n){
    config = YouBotConfiguration::GetInstance(n);
}


}
