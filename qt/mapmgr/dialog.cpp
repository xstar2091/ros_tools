#include "dialog.h"
#include "ui_dialog.h"

#include <QHeaderView>
#include <QMessageBox>
#include <QStringList>

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
    initUi();
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::initUi()
{
    QStringList headerList;
    headerList << "地图编号" << "地图名称" << "使用中" << "预加载";
    ui->mapGroupTable->setColumnCount(headerList.size());
    ui->mapGroupTable->setHorizontalHeaderLabels(headerList);
    // 列自适应宽度
    ui->mapGroupTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    // 只能选中一行
    ui->mapGroupTable->setSelectionBehavior(QTableWidget::SelectRows);
    ui->mapGroupTable->setSelectionMode(QTableWidget::SingleSelection);

    connect(ui->loadButton, &QPushButton::clicked, this, &Dialog::onLoadButtonClicked);
    connect(ui->openButton, &QPushButton::clicked, this, &Dialog::onOpenButtonClicked);

    // test data
    ui->mapGroupTable->setRowCount(2);
    ui->mapGroupTable->setItem(0, 0, new QTableWidgetItem("id11111"));
    ui->mapGroupTable->setItem(0, 1, new QTableWidgetItem("test map 11111"));
    ui->mapGroupTable->setItem(1, 0, new QTableWidgetItem("id22222"));
    ui->mapGroupTable->setItem(1, 1, new QTableWidgetItem("test map 22222"));
}

void Dialog::onLoadButtonClicked()
{
    QModelIndexList list = ui->mapGroupTable->selectionModel()->selectedRows();
    QString msg = QString("selected row index: %1").arg(list.at(0).row());
    QMessageBox mb;
    mb.information(this, "test", msg);
}

void Dialog::onOpenButtonClicked()
{

}
