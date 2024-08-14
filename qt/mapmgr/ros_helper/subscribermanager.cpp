#include "subscribermanager.h"

#include "roshelper.h"

SubscriberManager::SubscriberManager(QObject *parent)
    : QObject(parent)
{

}

SubscriberManager::~SubscriberManager()
{
    if (thrd_.joinable())
    {
        ros::shutdown();
        thrd_.join();
    }
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
    robot_msg::ExceptionInfo* ei = new robot_msg::ExceptionInfo();
    *ei = *msg;
    emit exceptionCodeReceivedEvent(ei);
}

void SubscriberManager::handleMultimapStatus(const robot_msg::MultimapStatusConstPtr &msg)
{
    robot_msg::MultimapStatus* status = new robot_msg::MultimapStatus();
    *status = *msg;
    emit multimapStatusReceivedEvent(status);
}

void SubscriberManager::handleMultimapTopologyPathList(const robot_msg::MultimapTopologyPathListConstPtr& msg)
{
    robot_msg::MultimapTopologyPathList* path_list = new robot_msg::MultimapTopologyPathList();
    *path_list = *msg;
    emit multimapTopologyPathListReceivedEvent((void*)path_list);
}
