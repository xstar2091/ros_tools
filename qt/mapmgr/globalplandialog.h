#ifndef GLOBALPLANDIALOG_H
#define GLOBALPLANDIALOG_H

#include <QDialog>
#include <ros/ros.h>
#include <robot_msg/ExceptionInfo.h>
#include <robot_msg/MultimapStatus.h>
#include <robot_msg/MultimapTopologyPathList.h>

namespace Ui {
class GlobalPlanDialog;
}

class MapGroup;

class GlobalPlanDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GlobalPlanDialog(QWidget *parent = 0);
    ~GlobalPlanDialog();

public:
    void setStartMap(const MapGroup& group);
    void setGoalMap(const MapGroup& group);

private:
    void initUi();
    void initRos();
    void clearAll();
    void clearTopologyTree();
    void handleExceptionInfoMessage(robot_msg::ExceptionInfoConstPtr msg);
    void handleMultimapStatusMessage(robot_msg::MultimapStatusConstPtr msg);
    void handleMultimapTopologyPathListMessage(robot_msg::MultimapTopologyPathListConstPtr msg);
    void onMakePlanButtonClicked();

private:
    Ui::GlobalPlanDialog *ui;
    ros::Subscriber status_sub_;
    ros::Subscriber topology_path_sub_;
    ros::Subscriber exception_code_sub_;
    ros::Publisher make_plan_pub_;
};

#endif // GLOBALPLANDIALOG_H
