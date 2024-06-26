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

    SubscriberManager* sub = SubscriberManager::instance();
    connect(sub, &SubscriberManager::multimapStatusReceiveEvent, this, &GlobalPlanDialog::handleMultimapStatusMessage);
    connect(sub, &SubscriberManager::multimapTopologyPathListReceivedEvent, this, &GlobalPlanDialog::handleMultimapTopologyPathListMessage);
}

void GlobalPlanDialog::clearAll()
{
    ui->startMapCollectionLineEdit->clear();
    ui->startMapGroupLineEdit->clear();
    ui->startMapPointXLineEdit->clear();
    ui->startMapPointYLineEdit->clear();

    ui->goalMapCollectionLineEdit->clear();
    ui->goalMapGroupLineEdit->clear();
    ui->goalMapPointXLineEdit->clear();
    ui->goalMapPointYLineEdit->clear();

    ui->replanMapCollectionLineEdit->clear();
    ui->replanMapGroupLineEdit->clear();
    ui->replanMapPointXLineEdit->clear();
    ui->replanMapPointYLineEdit->clear();
    ui->replanMapChangeAreaLineEdit->clear();

    ui->selectPathLineEdit->clear();

    ui->statusLineEdit->clear();
    ui->exceptionCodeLineEdit->clear();

    ui->topologyPathTreeWidget->clear();
    ui->topologyNodeNameTreeWidget->clear();
    ui->mapChangeAreaTreeWidget->clear();
    ui->startEndPointTreeWidget->clear();
}

void GlobalPlanDialog::clearResultShow()
{
    ui->selectPathLineEdit->clear();

    ui->statusLineEdit->clear();
    ui->exceptionCodeLineEdit->clear();

    ui->topologyPathTreeWidget->clear();
    ui->topologyNodeNameTreeWidget->clear();
    ui->mapChangeAreaTreeWidget->clear();
    ui->startEndPointTreeWidget->clear();
}

void GlobalPlanDialog::clearTopologyTree()
{
    ui->topologyPathTreeWidget->clear();
    ui->topologyNodeNameTreeWidget->clear();
    ui->mapChangeAreaTreeWidget->clear();
    ui->startEndPointTreeWidget->clear();
}

void GlobalPlanDialog::handleExceptionInfoMessage(robot_msg::ExceptionInfoConstPtr msg)
{

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
    robot_msg::MultimapTopologyPathList* msg = (robot_msg::MultimapTopologyPathList*)ptr;
    try
    {
        for (auto& topology_path : msg->path_list)
        {
            updateTopologyPathTree(topology_path);
            updateTopologyNodeNameTree(topology_path);
            updateMapChangeAreaTree(topology_path);
            updateStartEndPointTree(topology_path);
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

void GlobalPlanDialog::updateTopologyPathTree(const robot_msg::MultimapTopologyPath &topology_path)
{
    QTreeWidgetItem* item = new QTreeWidgetItem(QStringList()<<QString("路径%1(%2)").arg(topology_path.path_index).arg(topology_path.path_cost));
    ui->topologyPathTreeWidget->addTopLevelItem(item);
    QTreeWidgetItem* child = nullptr;
    for (auto& node : topology_path.path)
    {
        QString text = QString("%1").arg(QString::fromStdString(node.map_group_id));
        child = new QTreeWidgetItem(QStringList()<<text);
        item->addChild(child);
    }
}

void GlobalPlanDialog::updateTopologyNodeNameTree(const robot_msg::MultimapTopologyPath &topology_path)
{
    QTreeWidgetItem* item = new QTreeWidgetItem(QStringList()<<QString("路径%1(%2)").arg(topology_path.path_index).arg(topology_path.path_cost));
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
        QString text = QString("%1").arg(QString::fromStdString(group->name()));
        child = new QTreeWidgetItem(QStringList()<<text);
        item->addChild(child);
    }
}

void GlobalPlanDialog::updateMapChangeAreaTree(const robot_msg::MultimapTopologyPath &topology_path)
{
    QTreeWidgetItem* item = new QTreeWidgetItem(QStringList()<<QString("路径%1(%2)").arg(topology_path.path_index).arg(topology_path.path_cost));
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
        child = new QTreeWidgetItem(QStringList()<<text);
        item->addChild(child);
    }
}

void GlobalPlanDialog::updateStartEndPointTree(const robot_msg::MultimapTopologyPath &topology_path)
{
    QTreeWidgetItem* item = new QTreeWidgetItem(QStringList()<<QString("路径%1(%2)").arg(topology_path.path_index).arg(topology_path.path_cost));
    ui->startEndPointTreeWidget->addTopLevelItem(item);
    QTreeWidgetItem* child = nullptr;
    for (auto& node : topology_path.path)
    {
        QString text = QString("(%1, %2) -> (%3, %4)").arg(node.from_pose.position.x).arg(node.from_pose.position.y).arg(node.to_pose.position.x).arg(node.to_pose.position.y);
        child = new QTreeWidgetItem(QStringList()<<text);
        item->addChild(child);
    }
}

void GlobalPlanDialog::onMakePlanButtonClicked()
{
    clearResultShow();
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
