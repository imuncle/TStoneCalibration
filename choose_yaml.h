#ifndef CHOOSE_YAML_H
#define CHOOSE_YAML_H

#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>

namespace Ui {
class choose_yaml;
}

class choose_yaml : public QDialog
{
    Q_OBJECT

public:
    explicit choose_yaml(QWidget *parent = nullptr);
    ~choose_yaml();

private slots:
    void enter();
    void cancel();
    void choose_yaml_file();

signals:
    void SendSignal(QString str);

private:
    Ui::choose_yaml *ui;
};

#endif // CHOOSE_YAML_H
