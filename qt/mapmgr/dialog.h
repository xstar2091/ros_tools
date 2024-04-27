#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>

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
    void onOpenButtonClicked();

    int getSelectedIndex();
    void resetMapGroupTable();

private:
    Ui::Dialog *ui;
};

#endif // DIALOG_H
