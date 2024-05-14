#include "globalplandialog.h"
#include "ui_globalplandialog.h"

#include "data/mapcollection.h"

GlobalPlanDialog::GlobalPlanDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GlobalPlanDialog)
{
    ui->setupUi(this);
}

GlobalPlanDialog::~GlobalPlanDialog()
{
    delete ui;
}

void GlobalPlanDialog::setStartMap(const MapGroup &group)
{

}

void GlobalPlanDialog::setGoalMap(const MapGroup &group)
{

}
