#include "globalplandialog.h"
#include "ui_globalplandialog.h"

#include <QScrollBar>
#include <robot_msg/MultimapMakePlan.h>
#include "data/mapcollection.h"

GlobalPlanDialog::GlobalPlanDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GlobalPlanDialog)
{
    ui->setupUi(this);

    initUi();
    initRos();
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
    ui->mapChangeAreaTreeWidget->setVerticalScrollBar(bar);
    ui->startEndPointTreeWidget->setVerticalScrollBar(bar);

    ui->topologyPathTreeWidget->setHeaderHidden(true);
    ui->mapChangeAreaTreeWidget->setHeaderHidden(true);
    ui->startEndPointTreeWidget->setHeaderHidden(true);

    connect(ui->makePlanButton, &QPushButton::clicked, this, &GlobalPlanDialog::onMakePlanButtonClicked);
}

void GlobalPlanDialog::initRos()
{
    ros::NodeHandle nh;
    make_plan_pub_ = nh.advertise<robot_msg::MultimapMakePlan>("/multimap_planner_2d_node/make_plan", 1);
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
    ui->mapChangeAreaTreeWidget->clear();
    ui->startEndPointTreeWidget->clear();
}

void GlobalPlanDialog::clearTopologyTree()
{
    ui->topologyPathTreeWidget->clear();
    ui->mapChangeAreaTreeWidget->clear();
    ui->startEndPointTreeWidget->clear();
}

void GlobalPlanDialog::handleExceptionInfoMessage(robot_msg::ExceptionInfoConstPtr msg)
{

}

void GlobalPlanDialog::handleMultimapStatusMessage(robot_msg::MultimapStatusConstPtr msg)
{

}

void GlobalPlanDialog::handleMultimapTopologyPathListMessage(robot_msg::MultimapTopologyPathListConstPtr msg)
{

}

void GlobalPlanDialog::onMakePlanButtonClicked()
{
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
    make_plan_pub_.publish(msg);
}
