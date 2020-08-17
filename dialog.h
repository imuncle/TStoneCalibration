#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(QWidget *parent = nullptr);
    ~Dialog();

private slots:
    void enter();
    void cancel();
    void choose_left_dir();
    void choose_right_dir();

signals:
    void SendSignal(QString str);

private:
    Ui::Dialog *ui;
};

#endif // DIALOG_H
