#include "publishermanager.h"

PublisherManager *PublisherManager::instance()
{
    static PublisherManager inst;
    return &inst;
}

void PublisherManager::init()
{
    ros::NodeHandle nh;
    change_map_pub_ = nh.advertise<robot_msg::AlterMap>("altermap_to_map", 1);
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
