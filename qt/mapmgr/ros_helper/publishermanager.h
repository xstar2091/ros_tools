#ifndef PUBLISHERMANAGER_H
#define PUBLISHERMANAGER_H

#include <ros/ros.h>
#include <robot_msg/AlterMap.h>
#include <robot_msg/MultimapMakePlan.h>
#include <robot_msg/MultimapSelectPath.h>
#include <robot_msg/PreloadCostmap.h>

class MapGroup;

class PublisherManager
{
public:
    static PublisherManager* instance();
    void init();
    void publishChangeMap(const MapGroup& group);
    void publishMultimapSelectPath(uint8_t path_index);
    void publishMultimapMakePlan(const robot_msg::MultimapMakePlan& msg);
    void publishPreloadMap(const MapGroup& group);

private:
    ros::Publisher change_map_pub_;
    ros::Publisher multimap_make_plan_pub_;
    ros::Publisher multimap_select_path_pub_;
    ros::Publisher preload_map_pub_;
};

#endif // PUBLISHERMANAGER_H
