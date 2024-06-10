#ifndef ROSHELPERBASE_H
#define ROSHELPERBASE_H

#include <ros/ros.h>

class RosHelper
{
public:
    static RosHelper* instance();
    ros::NodeHandle& node_handle() { return node_handle_; }

protected:
    ros::NodeHandle node_handle_;
};

#endif // ROSHELPERBASE_H
