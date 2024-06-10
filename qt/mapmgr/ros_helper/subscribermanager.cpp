#include "subscribermanager.h"

#include "roshelper.h"

SubscriberManager::SubscriberManager(QObject *parent)
    : QObject(parent)
{

}

SubscriberManager *SubscriberManager::instance()
{
    static SubscriberManager inst;
    return &inst;
}

void SubscriberManager::init()
{
    ros::NodeHandle& nh = RosHelper::instance()->node_handle();
    ec_sub_ = nh.subscribe("/exception", 1, &SubscriberManager::handleExceptionCode, this);
    multimap_status_sub_ = nh.subscribe("/multimap_planner_2d_node/status", 1, &SubscriberManager::handleMultimapStatus, this);
    multimap_topology_path_sub_ = nh.subscribe("/multimap_planner_2d_node/topology_path", 1, &SubscriberManager::handleMultimapTopologyPathList, this);

    thrd_ = std::thread([&]() {
        ros::spin();
    });
}

void SubscriberManager::handleExceptionCode(const robot_msg::ExceptionInfoConstPtr &msg)
{

}

void SubscriberManager::handleMultimapStatus(const robot_msg::MultimapStatusConstPtr &msg)
{

}

void SubscriberManager::handleMultimapTopologyPathList(robot_msg::MultimapTopologyPathListPtr msg)
{
    robot_msg::MultimapTopologyPathList* path_list = new robot_msg::MultimapTopologyPathList();
    *path_list = *msg;
    emit topologyPathReceivedEvent((void*)path_list);
}
