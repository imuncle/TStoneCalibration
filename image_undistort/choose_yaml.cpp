#include "choose_yaml.h"
#include "ui_choose_yaml.h"

choose_yaml::choose_yaml(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::choose_yaml)
{
    ui->setupUi(this);
}

choose_yaml::~choose_yaml()
{
    delete ui;
}
