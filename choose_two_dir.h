#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>

namespace Ui {
class choose_two_dir;
}

class choose_two_dir : public QDialog
{
    Q_OBJECT

public:
    explicit choose_two_dir(QWidget *parent = nullptr);
    ~choose_two_dir();

private slots:
    void enter();
    void cancel();
    void choose_left_dir();
    void choose_right_dir();

signals:
    void SendSignal(QString str);

private:
    Ui::choose_two_dir *ui;
};

#endif // DIALOG_H
