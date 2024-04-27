#include "dialog.h"
#include <QApplication>
#include <ros/ros.h>

#include "ros_helper/publishermanager.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "map_manager_node");
    PublisherManager::instance()->init();

    QApplication a(argc, argv);
    Dialog w;
    w.show();

    return a.exec();
}
