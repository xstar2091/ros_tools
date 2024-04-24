#include "dialog.h"
#include "ui_dialog.h"

#include <memory>

#include <QFileDialog>
#include <QHeaderView>
#include <QMessageBox>
#include <QStringList>

#include "data/mapcollection.h"

namespace
{

enum
{
    column_index_id = 0,
    column_index_name,
    column_index_load,
    column_index_preload,
};

QStringList mapGroupTableHeaderLabels{"地图编号", "地图名称", "使用中", "预加载"};

}

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
    ui->mapGroupTable->setColumnCount(mapGroupTableHeaderLabels.size());
    ui->mapGroupTable->setHorizontalHeaderLabels(mapGroupTableHeaderLabels);
    // 列自适应宽度
    ui->mapGroupTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    // 只能选中一行
    ui->mapGroupTable->setSelectionBehavior(QTableWidget::SelectRows);
    ui->mapGroupTable->setSelectionMode(QTableWidget::SingleSelection);

    connect(ui->loadButton, &QPushButton::clicked, this, &Dialog::onLoadButtonClicked);
    connect(ui->openButton, &QPushButton::clicked, this, &Dialog::onOpenButtonClicked);

    // test data
    ui->mapGroupTable->setRowCount(1);
    ui->mapGroupTable->setItem(0, 0, new QTableWidgetItem("id11111"));
    ui->mapGroupTable->setItem(0, 1, new QTableWidgetItem("test map 11111"));
//    ui->mapGroupTable->setItem(1, 0, new QTableWidgetItem("id22222"));
//    ui->mapGroupTable->setItem(1, 1, new QTableWidgetItem("test map 22222"));
//    ui->mapGroupTable->setItem(2, 0, new QTableWidgetItem("id33333"));
//    ui->mapGroupTable->setItem(2, 1, new QTableWidgetItem("test map 33333"));
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
    std::unique_ptr<QFileDialog> dlg(new QFileDialog(nullptr));
    dlg->setWindowTitle("打开数据集文件");
    dlg->setNameFilter("*.xml");
    dlg->setViewMode(QFileDialog::Detail);

    QString file_path;
    if (dlg->exec() == QDialog::Accepted)
    {
        file_path = dlg->selectedFiles()[0];
    }
    if (file_path.isEmpty())
    {
        return;
    }

    try
    {
        MapCollection::instance()->init(file_path.toStdString());
        resetMapGroupTable();
    }
    catch (const std::exception& err)
    {
        QString error_msg = QString("发生错误:\n%1").arg(QString::fromStdString(err.what()));
    }
}

void Dialog::resetMapGroupTable()
{
    auto& group_list = MapCollection::instance()->group_list();
    ui->mapGroupTable->clearContents();
    ui->mapGroupTable->setRowCount(static_cast<int>(group_list.size()));

    for (int i = 0; i < static_cast<int>(group_list.size()); i++)
    {
        const MapGroup& group = group_list[i];
        ui->mapGroupTable->setItem(i, column_index_id, new QTableWidgetItem(QString::fromStdString(group.id())));
        ui->mapGroupTable->setItem(i, column_index_name, new QTableWidgetItem(QString::fromStdString(group.name())));
    }
}
