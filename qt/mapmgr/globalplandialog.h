#ifndef GLOBALPLANDIALOG_H
#define GLOBALPLANDIALOG_H

#include <QDialog>
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
    void clearAll();
    void clearResultShow();
    void clearTopologyTree();
    void handleExceptionInfoMessage(robot_msg::ExceptionInfoConstPtr msg);
    void handleMultimapStatusMessage(void* ptr);
    void handleMultimapTopologyPathListMessage(void* ptr);
    void updateTopologyPathTree(const robot_msg::MultimapTopologyPath& topology_path);
    void updateTopologyNodeNameTree(const robot_msg::MultimapTopologyPath& topology_path);
    void updateMapChangeAreaTree(const robot_msg::MultimapTopologyPath& topology_path);
    void updateStartEndPointTree(const robot_msg::MultimapTopologyPath& topology_path);
    void onMakePlanButtonClicked();

private:
    Ui::GlobalPlanDialog *ui;
};

#endif // GLOBALPLANDIALOG_H
