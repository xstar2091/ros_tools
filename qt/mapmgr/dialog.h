#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>

#include "globalplandialog.h"

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();

private:
    void initUi();
    void onLoadButtonClicked();
    void onPreloadButtonClicked();
    void onGlobalPlanButtonClicked();
    void onStartPointButtonClicked();
    void onGoalPointButtongClicked();
    void onOpenButtonClicked();

    QString getCellText(int row, int column);
    int getSelectedIndex();
    bool checkSelectedIndex(int selected_index);
    bool checkGlobalPlanDialog();
    void resetMapGroupTable();

private:
    Ui::Dialog *ui;
    int current_map_index_;
    int preload_map_index_;
    int start_map_index_;
    int goal_map_index_;
    GlobalPlanDialog* global_plan_dlg;
};

#endif // DIALOG_H
