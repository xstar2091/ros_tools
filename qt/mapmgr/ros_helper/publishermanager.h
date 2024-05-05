#ifndef PUBLISHERMANAGER_H
#define PUBLISHERMANAGER_H

#include <ros/ros.h>
#include <robot_msg/AlterMap.h>
#include <robot_msg/PreloadCostmap.h>

#include "data/mapcollection.h"

class PublisherManager
{
public:
    static PublisherManager* instance();
    void init();
    void publishChangeMap(const MapGroup& group);
    void publishPreloadMap(const MapGroup& group);

private:
    ros::Publisher change_map_pub_;
    ros::Publisher preload_map_pub_;
};

#endif // PUBLISHERMANAGER_H
