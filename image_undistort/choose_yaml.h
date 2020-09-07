#ifndef CHOOSE_YAML_H
#define CHOOSE_YAML_H

#include <QDialog>

namespace Ui {
class choose_yaml;
}

class choose_yaml : public QDialog
{
    Q_OBJECT

public:
    explicit choose_yaml(QWidget *parent = nullptr);
    ~choose_yaml();

private:
    Ui::choose_yaml *ui;
};

#endif // CHOOSE_YAML_H
