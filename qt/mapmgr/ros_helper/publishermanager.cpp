#include "publishermanager.h"

#include "data/mapcollection.h"
#include "roshelper.h"

PublisherManager *PublisherManager::instance()
{
    static PublisherManager inst;
    return &inst;
}

void PublisherManager::init()
{
    ros::NodeHandle& nh = RosHelper::instance()->node_handle();
    change_map_pub_ = nh.advertise<robot_msg::AlterMap>("altermap_to_map", 1);
    multimap_make_plan_pub_ = nh.advertise<robot_msg::MultimapMakePlan>("/multimap_planner_2d_node/make_plan", 1);
    multimap_replan_pub_ = nh.advertise<robot_msg::MultimapReplan>("/multimap_planner_2d_node/replan", 1);
    multimap_select_path_pub_ = nh.advertise<robot_msg::MultimapSelectPath>("/multimap_planner_2d_node/select_path", 1);
    preload_map_pub_ = nh.advertise<robot_msg::PreloadCostmap>("preload_costmap", 1);
}

void PublisherManager::publishChangeMap(const MapGroup &group)
{
    robot_msg::AlterMap msg;
    msg.header.stamp = ros::Time::now();
    msg.collectPath = group.collection()->path();
    msg.collectionId = group.collection()->id();
    msg.groupPath = group.path();
    msg.groupId = group.id();
    msg.downloadType = 2;
    change_map_pub_.publish(msg);
}

void PublisherManager::publishMultimapSelectPath(uint8_t path_index)
{
    robot_msg::MultimapSelectPath msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.path_index = path_index;
    multimap_select_path_pub_.publish(msg);
}

void PublisherManager::publishMultimapMakePlan(const robot_msg::MultimapMakePlan &msg)
{
    multimap_make_plan_pub_.publish(msg);
}

void PublisherManager::publishMultimapReplan(const robot_msg::MultimapReplan &msg)
{
    multimap_replan_pub_.publish(msg);
}

void PublisherManager::publishPreloadMap(const MapGroup &group)
{
    robot_msg::PreloadCostmap msg;
    msg.header.stamp = ros::Time::now();
    msg.next_map_collection_path = group.collection()->path();
    msg.next_map_collection_id = group.collection()->id();
    msg.next_map_group_path = group.path();
    msg.next_map_group_id = group.id();
    msg.next_map_from_pose.header = msg.header;
    msg.next_map_from_pose.pose.orientation.w = 1;
    msg.next_map_to_pose.header = msg.header;
    msg.next_map_to_pose.pose.orientation.w = 1;
    preload_map_pub_.publish(msg);
}
