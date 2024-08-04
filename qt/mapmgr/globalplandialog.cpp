#include "globalplandialog.h"
#include "ui_globalplandialog.h"

#include <QMessageBox>
#include <QScrollBar>
#include <robot_msg/MultimapMakePlan.h>
#include "data/mapcollection.h"
#include "ros_helper/publishermanager.h"
#include "ros_helper/rosenumstring.h"
#include "ros_helper/subscribermanager.h"

GlobalPlanDialog::GlobalPlanDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GlobalPlanDialog)
{
    ui->setupUi(this);

    initUi();
}

GlobalPlanDialog::~GlobalPlanDialog()
{
    delete ui;
}

void GlobalPlanDialog::setStartMap(const MapGroup &group)
{
    ui->startMapCollectionLineEdit->setText(QString::fromStdString(group.collection()->id()));
    ui->startMapGroupLineEdit->setText(QString::fromStdString(group.id()));
    ui->startMapPointXLineEdit->setText("0.0");
    ui->startMapPointYLineEdit->setText("0.0");
}

void GlobalPlanDialog::setGoalMap(const MapGroup &group)
{
    ui->goalMapCollectionLineEdit->setText(QString::fromStdString(group.collection()->id()));
    ui->goalMapGroupLineEdit->setText(QString::fromStdString(group.id()));
    ui->goalMapPointXLineEdit->setText("0.0");
    ui->goalMapPointYLineEdit->setText("0.0");
}

void GlobalPlanDialog::initUi()
{
    QScrollBar* bar = new QScrollBar(this);
    ui->topologyPathTreeWidget->setVerticalScrollBar(bar);
    ui->topologyNodeNameTreeWidget->setVerticalScrollBar(bar);
    ui->mapChangeAreaTreeWidget->setVerticalScrollBar(bar);
    ui->startEndPointTreeWidget->setVerticalScrollBar(bar);

    ui->topologyPathTreeWidget->setHeaderHidden(true);
    ui->topologyNodeNameTreeWidget->setHeaderHidden(true);
    ui->mapChangeAreaTreeWidget->setHeaderHidden(true);
    ui->startEndPointTreeWidget->setHeaderHidden(true);

    connect(ui->makePlanButton, &QPushButton::clicked, this, &GlobalPlanDialog::onMakePlanButtonClicked);
    connect(ui->replanButton, &QPushButton::clicked, this, &GlobalPlanDialog::onReplanButtonClicked);
    connect(ui->selectPathButton, &QPushButton::clicked, this, &GlobalPlanDialog::onSelectPathButtonClicked);

    connect(ui->topologyPathTreeWidget, &QTreeWidget::itemClicked, this, &GlobalPlanDialog::topologyPathTreeItemSelected);
    connect(ui->topologyNodeNameTreeWidget, &QTreeWidget::itemClicked, this, &GlobalPlanDialog::topologyNodeNameTreeItemSelected);
    connect(ui->mapChangeAreaTreeWidget, &QTreeWidget::itemClicked, this, &GlobalPlanDialog::mapChangeAreaTreeItemSelected);
    connect(ui->startEndPointTreeWidget, &QTreeWidget::itemClicked, this, &GlobalPlanDialog::startEndPointTreeItemSelected);

    SubscriberManager* sub = SubscriberManager::instance();
    connect(sub, &SubscriberManager::exceptionCodeReceivedEvent, this, &GlobalPlanDialog::handleExceptionInfoMessage);
    connect(sub, &SubscriberManager::multimapStatusReceivedEvent, this, &GlobalPlanDialog::handleMultimapStatusMessage);
    connect(sub, &SubscriberManager::multimapTopologyPathListReceivedEvent, this, &GlobalPlanDialog::handleMultimapTopologyPathListMessage);
}

void GlobalPlanDialog::clearAll(int mask)
{
    if (mask & clear_mask_plan)
    {
        ui->startMapCollectionLineEdit->clear();
        ui->startMapGroupLineEdit->clear();
        ui->startMapPointXLineEdit->clear();
        ui->startMapPointYLineEdit->clear();

        ui->goalMapCollectionLineEdit->clear();
        ui->goalMapGroupLineEdit->clear();
        ui->goalMapPointXLineEdit->clear();
        ui->goalMapPointYLineEdit->clear();
    }

    if (mask & clear_mask_replan)
    {
        ui->replanMapCollectionLineEdit->clear();
        ui->replanMapGroupLineEdit->clear();
        ui->replanMapPointXLineEdit->clear();
        ui->replanMapPointYLineEdit->clear();
        ui->replanMapChangeAreaLineEdit->clear();
    }

    if (mask & clear_mask_select_path)
    {
        ui->selectPathLineEdit->clear();
    }

    if (mask & clear_mask_status_show)
    {
        ui->statusLineEdit->clear();
        ui->exceptionCodeLineEdit->clear();
    }

    if (mask & clear_mask_topology_tree)
    {
        ui->topologyPathTreeWidget->clear();
        ui->topologyNodeNameTreeWidget->clear();
        ui->mapChangeAreaTreeWidget->clear();
        ui->startEndPointTreeWidget->clear();
    }
}

void GlobalPlanDialog::handleExceptionInfoMessage(void* ptr)
{
    QString exception_code_str;
    robot_msg::ExceptionInfo* msg = (robot_msg::ExceptionInfo*)ptr;
    for (const std::string& ec : msg->exception_code)
    {
        exception_code_str.append(QString::fromStdString(ec)).append("; ");
    }
    delete msg;

    if (exception_code_str.isEmpty()) return;
    exception_code_str.remove(exception_code_str.size() - 2, 2);

    QString text = ui->exceptionCodeLineEdit->text();
    if (text.isEmpty())
    {
        text = exception_code_str;
    }
    else
    {
        text.append("; ").append(exception_code_str);
    }
    ui->exceptionCodeLineEdit->setText(text);
}

void GlobalPlanDialog::handleMultimapStatusMessage(void* ptr)
{
    robot_msg::MultimapStatus* msg = (robot_msg::MultimapStatus*)ptr;
    const QString& status = RosEnumString::instance()->multimap_status().toString(msg->status_code);
    delete msg;

    QString text = ui->statusLineEdit->text();
    if (text.isEmpty())
    {
        text = status;
    }
    else
    {
        text.append("; ").append(status);
    }
    ui->statusLineEdit->setText(text);
}

void GlobalPlanDialog::handleMultimapTopologyPathListMessage(void* ptr)
{
    topologyPathTreeItemTable.clear();
    topologyNodeNameTreeItemTable.clear();
    mapChangeAreaTreeItemTable.clear();
    startEndPointTreeItemTable.clear();
    robot_msg::MultimapTopologyPathList* msg = (robot_msg::MultimapTopologyPathList*)ptr;
    try
    {
        int startId = 1;
        for (auto& topology_path : msg->path_list)
        {
            updateTopologyPathTree(startId, topology_path);
            updateTopologyNodeNameTree(startId, topology_path);
            updateMapChangeAreaTree(startId, topology_path);
            updateStartEndPointTree(startId, topology_path);
            startId += (static_cast<int>(topology_path.path.size()) + 1);
        }
        ui->topologyPathTreeWidget->expandAll();
        ui->topologyNodeNameTreeWidget->expandAll();
        ui->mapChangeAreaTreeWidget->expandAll();
        ui->startEndPointTreeWidget->expandAll();
    }
    catch (const std::exception& err)
    {
        QMessageBox::critical(this, "错误", QString::fromStdString(std::string(err.what())));
    }
    delete msg;
}

namespace
{

enum
{
    tree_column_index_show_text = 0,
    tree_column_index_tree_item_id = 1,
    tree_column_index_path_index = 2,
    tree_column_index_from_map_switch_id = 3,
    tree_column_index_to_map_switch_id = 4,
};

}

void GlobalPlanDialog::updateTopologyPathTree(int startId, const robot_msg::MultimapTopologyPath &topology_path)
{
    QTreeWidgetItem* item = new QTreeWidgetItem(QStringList()<<QString("路径%1(%2)").arg(topology_path.path_index).arg(topology_path.path_cost)
                                                             <<QString("%1").arg(startId)
                                                             <<QString("%1").arg(topology_path.path_index)
                                                             <<QString()
                                                             <<QString());
    topologyPathTreeItemTable.insert(std::make_pair(startId++, item));
    ui->topologyPathTreeWidget->addTopLevelItem(item);
    QTreeWidgetItem* child = nullptr;
    for (auto& node : topology_path.path)
    {
        QStringList itemTextList;
        itemTextList << QString("%1").arg(QString::fromStdString(node.map_group_id))
                     << QString("%1").arg(startId)
                     << QString("%1").arg(topology_path.path_index)
                     << QString("%1").arg(node.from_switch_area.id)
                     << QString("%1").arg(node.to_switch_area.id);
        child = new QTreeWidgetItem(itemTextList);
        topologyPathTreeItemTable.insert(std::make_pair(startId++, child));
        item->addChild(child);
    }
}

void GlobalPlanDialog::updateTopologyNodeNameTree(int startId, const robot_msg::MultimapTopologyPath &topology_path)
{
    QTreeWidgetItem* item = new QTreeWidgetItem(QStringList()<<QString("路径%1(%2)").arg(topology_path.path_index).arg(topology_path.path_cost)
                                                             <<QString("%1").arg(startId)
                                                             <<QString("%1").arg(topology_path.path_index)
                                                             <<QString()
                                                             <<QString());
    topologyNodeNameTreeItemTable.insert(std::make_pair(startId++, item));
    ui->topologyNodeNameTreeWidget->addTopLevelItem(item);
    QTreeWidgetItem* child = nullptr;
    const MapGroup* group = nullptr;
    for (auto& node : topology_path.path)
    {
        group = MapCollection::instance()->findGroup(node.map_group_id);
        if (group == nullptr)
        {
            QString error = QString("无法找到返回的地图组信息\n地图集：%1\n地图组：%2")
                    .arg(QString::fromStdString(node.map_collection_id))
                    .arg(QString::fromStdString(node.map_group_id));
            throw std::runtime_error(error.toStdString());
        }
        QStringList itemTextList;
        itemTextList << QString("%1").arg(QString::fromStdString(group->name()))
                     << QString("%1").arg(startId)
                     << QString("%1").arg(topology_path.path_index)
                     << QString("%1").arg(node.from_switch_area.id)
                     << QString("%1").arg(node.to_switch_area.id);
        child = new QTreeWidgetItem(itemTextList);
        topologyNodeNameTreeItemTable.insert(std::make_pair(startId++, child));
        item->addChild(child);
    }
}

void GlobalPlanDialog::updateMapChangeAreaTree(int startId, const robot_msg::MultimapTopologyPath &topology_path)
{
    QTreeWidgetItem* item = new QTreeWidgetItem(QStringList()<<QString("路径%1(%2)").arg(topology_path.path_index).arg(topology_path.path_cost)
                                                             <<QString("%1").arg(startId)
                                                             <<QString("%1").arg(topology_path.path_index)
                                                             <<QString()
                                                             <<QString());
    mapChangeAreaTreeItemTable.insert(std::make_pair(startId++, item));
    ui->mapChangeAreaTreeWidget->addTopLevelItem(item);
    QTreeWidgetItem* child = nullptr;
    const MapSwitchType& map_switch_type = RosEnumString::instance()->map_switch_type();
    for (auto& node : topology_path.path)
    {
        QString text = QString("%1").arg(node.from_switch_area.id);
        if (node.from_pose_type != 0)
        {
            text.append(QString("(%1:%2)").arg(map_switch_type.toString(node.from_pose_type)).arg(int(node.from_pose_type)));
        }
        text.append(QString(" -> %1").arg(node.to_switch_area.id));
        if (node.to_pose_type != 0)
        {
            text.append(QString("(%1:%2)").arg(map_switch_type.toString(node.to_pose_type)).arg((int)node.to_pose_type));
        }
        QStringList itemTextList;
        itemTextList << text
                     << QString("%1").arg(startId)
                     << QString("%1").arg(topology_path.path_index)
                     << QString("%1").arg(node.from_switch_area.id)
                     << QString("%1").arg(node.to_switch_area.id);
        child = new QTreeWidgetItem(itemTextList);
        mapChangeAreaTreeItemTable.insert(std::make_pair(startId++, child));
        item->addChild(child);
    }
}

void GlobalPlanDialog::updateStartEndPointTree(int startId, const robot_msg::MultimapTopologyPath &topology_path)
{
    QTreeWidgetItem* item = new QTreeWidgetItem(QStringList()<<QString("路径%1(%2)").arg(topology_path.path_index).arg(topology_path.path_cost)
                                                             <<QString("%1").arg(startId)
                                                             <<QString("%1").arg(topology_path.path_index)
                                                             <<QString()
                                                             <<QString());
    startEndPointTreeItemTable.insert(std::make_pair(startId++, item));
    ui->startEndPointTreeWidget->addTopLevelItem(item);
    QTreeWidgetItem* child = nullptr;
    for (auto& node : topology_path.path)
    {
        QStringList itemTextList;
        itemTextList << QString("(%1, %2) -> (%3, %4)").arg(node.from_pose.position.x).arg(node.from_pose.position.y).arg(node.to_pose.position.x).arg(node.to_pose.position.y)
                     << QString("%1").arg(startId)
                     << QString("%1").arg(topology_path.path_index)
                     << QString("%1").arg(node.from_switch_area.id)
                     << QString("%1").arg(node.to_switch_area.id);
        child = new QTreeWidgetItem(itemTextList);
        startEndPointTreeItemTable.insert(std::make_pair(startId++, child));
        item->addChild(child);
    }
}

void GlobalPlanDialog::onMakePlanButtonClicked()
{
    clearAll(clear_mask_replan | clear_mask_select_path | clear_mask_status_show | clear_mask_topology_tree);
    robot_msg::MultimapMakePlan msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.from_map_collection_id = ui->startMapCollectionLineEdit->text().toStdString();
    msg.from_map_group_id = ui->startMapGroupLineEdit->text().toStdString();
    msg.from_pose.position.x = strtod(ui->startMapPointXLineEdit->text().toStdString().c_str(), nullptr);
    msg.from_pose.position.y = strtod(ui->startMapPointYLineEdit->text().toStdString().c_str(), nullptr);
    msg.from_pose.position.z = 0.0;
    msg.from_pose.orientation.w = 1.0;
    msg.to_map_collection_id = ui->goalMapCollectionLineEdit->text().toStdString();
    msg.to_map_group_id = ui->goalMapGroupLineEdit->text().toStdString();
    msg.to_pose.position.x = strtod(ui->goalMapPointXLineEdit->text().toStdString().c_str(), nullptr);
    msg.to_pose.position.y = strtod(ui->goalMapPointYLineEdit->text().toStdString().c_str(), nullptr);
    msg.to_pose.position.z = 0.0;
    msg.to_pose.orientation.w = 1.0;
    PublisherManager::instance()->publishMultimapMakePlan(msg);
}

void GlobalPlanDialog::topologyPathTreeItemSelected(QTreeWidgetItem *item, int)
{
    QString itemIdStr = item->text(tree_column_index_tree_item_id);
    if (itemIdStr.isEmpty())
    {
        return;
    }

    int itemId = itemIdStr.toInt();
    changeTreeSelectedItem(itemId, topologyNodeNameTreeItemTable);
    changeTreeSelectedItem(itemId, mapChangeAreaTreeItemTable);
    changeTreeSelectedItem(itemId, startEndPointTreeItemTable);
    updateReplanUi(item);
    updateSelectedPath(item);
}

void GlobalPlanDialog::topologyNodeNameTreeItemSelected(QTreeWidgetItem *item, int)
{
    QString itemIdStr = item->text(tree_column_index_tree_item_id);
    if (itemIdStr.isEmpty())
    {
        return;
    }

    int itemId = itemIdStr.toInt();
    changeTreeSelectedItem(itemId, topologyPathTreeItemTable);
    changeTreeSelectedItem(itemId, mapChangeAreaTreeItemTable);
    changeTreeSelectedItem(itemId, startEndPointTreeItemTable);
    updateReplanUi(item);
    updateSelectedPath(item);
}

void GlobalPlanDialog::mapChangeAreaTreeItemSelected(QTreeWidgetItem *item, int)
{
    QString itemIdStr = item->text(tree_column_index_tree_item_id);
    if (itemIdStr.isEmpty())
    {
        return;
    }

    int itemId = itemIdStr.toInt();
    changeTreeSelectedItem(itemId, topologyPathTreeItemTable);
    changeTreeSelectedItem(itemId, topologyNodeNameTreeItemTable);
    changeTreeSelectedItem(itemId, startEndPointTreeItemTable);
    updateReplanUi(item);
    updateSelectedPath(item);
}

void GlobalPlanDialog::startEndPointTreeItemSelected(QTreeWidgetItem *item, int)
{
    QString itemIdStr = item->text(tree_column_index_tree_item_id);
    if (itemIdStr.isEmpty())
    {
        return;
    }

    int itemId = itemIdStr.toInt();
    changeTreeSelectedItem(itemId, topologyPathTreeItemTable);
    changeTreeSelectedItem(itemId, topologyNodeNameTreeItemTable);
    changeTreeSelectedItem(itemId, mapChangeAreaTreeItemTable);
    updateReplanUi(item);
    updateSelectedPath(item);
}

void GlobalPlanDialog::changeTreeSelectedItem(int itemId, std::unordered_map<int, QTreeWidgetItem *> &table)
{
    QTreeWidgetItem* item = nullptr;
    auto it = table.find(itemId);
    if (it != table.end())
    {
        item = it->second;
    }

    for (auto& pair : table)
    {
        pair.second->setSelected(false);
    }
    if (item)
    {
        item->setSelected(true);
    }
}

void GlobalPlanDialog::updateSelectedPath(QTreeWidgetItem *item)
{
    QString selectedPathIndexStr = item->text(tree_column_index_path_index);
    if (selectedPathIndexStr.isEmpty()) return;
    ui->selectPathLineEdit->setText(selectedPathIndexStr);
}

void GlobalPlanDialog::onSelectPathButtonClicked()
{
    QString selectedPathIndexStr = ui->selectPathLineEdit->text();
    if (selectedPathIndexStr.isEmpty())
    {
        QMessageBox::critical(this, "错误", "请先选择一条路径");
        return;
    }
    bool ok = false;
    unsigned int selectedPathIndex = selectedPathIndexStr.toUInt(&ok);
    if (!ok)
    {
        QMessageBox::critical(this, "错误", "路径索引必须是一个数字");
        return;
    }
    clearAll(clear_mask_replan | clear_mask_status_show);
    PublisherManager::instance()->publishMultimapSelectPath(static_cast<uint8_t>(selectedPathIndex));
}

void GlobalPlanDialog::updateReplanUi(QTreeWidgetItem* item)
{
    clearAll(clear_mask_replan);
    if (item->parent() == nullptr) return;
    QString itemIdStr = item->text(tree_column_index_tree_item_id);
    if (itemIdStr.isEmpty())
    {
        return;
    }
    bool ok = false;
    int itemId = itemIdStr.toInt(&ok);
    if (!ok) return;
    auto it = topologyPathTreeItemTable.find(itemId);
    if (it == topologyPathTreeItemTable.end()) return;
    const MapGroup* group = MapCollection::instance()->findGroup(it->second->text(tree_column_index_show_text).toStdString());
    if (group == nullptr) return;

    ui->replanMapCollectionLineEdit->setText(QString::fromStdString(group->collection()->id()));
    ui->replanMapGroupLineEdit->setText(QString::fromStdString(group->id()));
    ui->replanMapPointXLineEdit->setText("0.0");
    ui->replanMapPointYLineEdit->setText("0.0");
    ui->replanMapChangeAreaLineEdit->setText(item->text(tree_column_index_to_map_switch_id));
}

void GlobalPlanDialog::onReplanButtonClicked()
{
    if (ui->replanMapCollectionLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, "错误", "地图集不可为空");
        return;
    }
    if (ui->replanMapGroupLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, "错误", "地图组不可为空");
        return;
    }
    if (ui->replanMapPointXLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, "错误", "坐标X不可为空");
        return;
    }
    if (ui->replanMapPointYLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, "错误", "坐标Y不可为空");
        return;
    }
    if (ui->replanMapChangeAreaLineEdit->text().isEmpty())
    {
        QMessageBox::critical(this, "错误", "切换区不可为空");
        return;
    }
    bool ok = false;
    double pointX = ui->replanMapPointXLineEdit->text().toDouble(&ok);
    if (!ok)
    {
        QMessageBox::critical(this, "错误", "坐标X必须是一个数字");
        return;
    }
    ok = false;
    double pointY = ui->replanMapPointYLineEdit->text().toDouble(&ok);
    if (!ok)
    {
        QMessageBox::critical(this, "错误", "坐标Y必须是一个数字");
        return;
    }
    ok = false;
    int64_t switchAreaId = ui->replanMapChangeAreaLineEdit->text().toLongLong(&ok);
    if (!ok)
    {
        QMessageBox::critical(this, "错误", "切换区ID必须是一个数字");
        return;
    }
    clearAll(clear_mask_select_path | clear_mask_status_show | clear_mask_topology_tree);
    robot_msg::MultimapReplan msg;
    msg.header.frame_id = std::string("map");
    msg.header.stamp = ros::Time::now();
    msg.from_map_collection_id = ui->replanMapCollectionLineEdit->text().toStdString();
    msg.from_map_group_id = ui->replanMapGroupLineEdit->text().toStdString();
    msg.from_pose.position.x = pointX;
    msg.from_pose.position.y = pointY;
    msg.from_pose.position.z = 0.0;
    msg.from_pose.orientation.w = 1.0;
    msg.unreachable_switch_areas.push_back(switchAreaId);
    PublisherManager::instance()->publishMultimapReplan(msg);
}
