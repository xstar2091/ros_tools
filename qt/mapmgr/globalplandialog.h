#ifndef GLOBALPLANDIALOG_H
#define GLOBALPLANDIALOG_H

#include <unordered_map>
#include <QDialog>
#include <robot_msg/ExceptionInfo.h>
#include <robot_msg/MultimapStatus.h>
#include <robot_msg/MultimapTopologyPathList.h>

namespace Ui {
class GlobalPlanDialog;
}

class MapGroup;
class QTreeWidgetItem;

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
    void handleExceptionInfoMessage(void* ptr);
    void handleMultimapStatusMessage(void* ptr);
    void handleMultimapTopologyPathListMessage(void* ptr);
    void updateTopologyPathTree(int startId, const robot_msg::MultimapTopologyPath& topology_path);
    void updateTopologyNodeNameTree(int startId, const robot_msg::MultimapTopologyPath& topology_path);
    void updateMapChangeAreaTree(int startId, const robot_msg::MultimapTopologyPath& topology_path);
    void updateStartEndPointTree(int startId, const robot_msg::MultimapTopologyPath& topology_path);
    void onMakePlanButtonClicked();
    void topologyPathTreeItemSelected(QTreeWidgetItem* item, int);
    void topologyNodeNameTreeItemSelected(QTreeWidgetItem* item, int);
    void mapChangeAreaTreeItemSelected(QTreeWidgetItem* item, int);
    void startEndPointTreeItemSelected(QTreeWidgetItem* item, int);
    void changeTreeSelectedItem(int itemId, std::unordered_map<int, QTreeWidgetItem*>& table);
    void updateSelectedPath(QTreeWidgetItem* item);
    void onSelectPathButtonClicked();

private:
    Ui::GlobalPlanDialog *ui;
    std::unordered_map<int, QTreeWidgetItem*> topologyPathTreeItemTable;
    std::unordered_map<int, QTreeWidgetItem*> topologyNodeNameTreeItemTable;
    std::unordered_map<int, QTreeWidgetItem*> mapChangeAreaTreeItemTable;
    std::unordered_map<int, QTreeWidgetItem*> startEndPointTreeItemTable;
};

#endif // GLOBALPLANDIALOG_H
