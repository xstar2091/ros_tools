#ifndef SUBSCRIBERMANAGER_H
#define SUBSCRIBERMANAGER_H

#include <thread>
#include <QObject>
#include <ros/ros.h>
#include <robot_msg/ExceptionInfo.h>
#include <robot_msg/MultimapStatus.h>
#include <robot_msg/MultimapTopologyPathList.h>

class SubscriberManager : public QObject
{
    Q_OBJECT

public:
    explicit SubscriberManager(QObject* parent = nullptr);
    static SubscriberManager* instance();
    void init();

private:
    void handleExceptionCode(const robot_msg::ExceptionInfoConstPtr& msg);
    void handleMultimapStatus(const robot_msg::MultimapStatusConstPtr& msg);
    void handleMultimapTopologyPathList(robot_msg::MultimapTopologyPathListPtr msg);

signals:
    void topologyPathReceivedEvent(void* msg);

private:
    std::thread thrd_;
    ros::Subscriber ec_sub_;
    ros::Subscriber multimap_status_sub_;
    ros::Subscriber multimap_topology_path_sub_;
};

#endif // SUBSCRIBERMANAGER_H
