#include "dialog.h"
#include "ui_dialog.h"

#include <memory>

#include <QFileDialog>
#include <QHeaderView>
#include <QMessageBox>
#include <QStringList>

#include "data/mapcollection.h"
#include "ros_helper/publishermanager.h"

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

    current_map_index_ = -1;
    preload_map_index_ = -1;
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
    connect(ui->preloadButton, &QPushButton::clicked, this, &Dialog::onPreloadButtonClicked);
    connect(ui->openButton, &QPushButton::clicked, this, &Dialog::onOpenButtonClicked);
}

void Dialog::onLoadButtonClicked()
{
    int selected_index = getSelectedIndex();
    if (selected_index < 0)
    {
        QMessageBox::critical(this, "错误", "请先选择一个地图组");
        return;
    }
    auto& group_list = MapCollection::instance()->group_list();
    if (selected_index >= static_cast<int>(group_list.size()))
    {
        QString msg = QString("索引错乱了\n选中项的索引似乎是: %1\n但地图组总数是: %2\n索引的取值范围应该是: [0, %2)")
                .arg(selected_index).arg(group_list.size());
        QMessageBox::critical(this, "错误", msg);
    }

    if (current_map_index_ >= 0)
    {
        ui->mapGroupTable->setItem(current_map_index_, column_index_load, new QTableWidgetItem(""));
    }
    ui->mapGroupTable->setItem(selected_index, column_index_load, new QTableWidgetItem("加载中"));
    auto& group = group_list[selected_index];
    PublisherManager::instance()->publishChangeMap(group);
    current_map_index_ = selected_index;
    ui->mapGroupTable->setItem(current_map_index_, column_index_load, new QTableWidgetItem("完成"));
}

void Dialog::onPreloadButtonClicked()
{
    int selected_index = getSelectedIndex();
    if (selected_index < 0)
    {
        QMessageBox::critical(this, "错误", "请先选择一个地图组");
        return;
    }
    auto& group_list = MapCollection::instance()->group_list();
    if (selected_index >= static_cast<int>(group_list.size()))
    {
        QString msg = QString("索引错乱了\n选中项的索引似乎是: %1\n但地图组总数是: %2\n索引的取值范围应该是: [0, %2)")
                .arg(selected_index).arg(group_list.size());
        QMessageBox::critical(this, "错误", msg);
    }

    if (preload_map_index_ >= 0)
    {
        ui->mapGroupTable->setItem(preload_map_index_, column_index_preload, new QTableWidgetItem(""));
    }
    ui->mapGroupTable->setItem(selected_index, column_index_preload, new QTableWidgetItem("预加载中"));
    auto& group = group_list[selected_index];
    PublisherManager::instance()->publishPreloadMap(group);
    preload_map_index_ = selected_index;
    ui->mapGroupTable->setItem(preload_map_index_, column_index_preload, new QTableWidgetItem("完成"));
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

int Dialog::getSelectedIndex()
{
    QModelIndexList selected_index_list = ui->mapGroupTable->selectionModel()->selectedRows();
    if (selected_index_list.count() == 0)
    {
        return -1;
    }
    return selected_index_list.at(0).row();
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
