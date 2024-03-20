#include "dialog.h"
#include <QApplication>
#include "data/fileinfo.h"//lxdebug
#include <QMessageBox>//lxdebug
#include <QString>//lxdebug
void lxdebug()
{
    FileInfo fi;
    fi.reset("/home/lx/catkin_ws/src/my_tools/qt/mapmgr/main.cpp");
    QString msg = QString("full_path: %1\nfile_name: %2\nextention: %3\nparent: %4")
            .arg(QString::fromStdString(fi.full_path()))
            .arg(QString::fromStdString(fi.file_name()))
            .arg(QString::fromStdString(fi.extention()))
            .arg(QString::fromStdString(fi.parent()));
    QMessageBox mb;
    mb.information(nullptr, "test", msg);
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Dialog w;
    lxdebug();
    w.show();

    return a.exec();
}
