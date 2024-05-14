#ifndef GLOBALPLANDIALOG_H
#define GLOBALPLANDIALOG_H

#include <QDialog>

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
    Ui::GlobalPlanDialog *ui;
};

#endif // GLOBALPLANDIALOG_H
