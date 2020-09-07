#include "choose_yaml.h"
#include "ui_choose_yaml.h"

choose_yaml::choose_yaml(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::choose_yaml)
{
    ui->setupUi(this);
    connect(ui->yes, SIGNAL(clicked()), this, SLOT(enter()));
    connect(ui->no, SIGNAL(clicked()), this, SLOT(cancel()));
    connect(ui->browse, SIGNAL(clicked()), this,SLOT(choose_yaml_file()));
}

choose_yaml::~choose_yaml()
{
    delete ui;
}

void choose_yaml::enter()
{
    if(ui->yaml_src->text().isEmpty())
    {
        QMessageBox::critical(NULL, "错误", "请将路径填写完毕", QMessageBox::Yes, QMessageBox::Yes);
        return;
    }
    this->close();
    emit SendSignal(ui->yaml_src->text());
}

void choose_yaml::cancel()
{
    this->close();
}

void choose_yaml::choose_yaml_file()
{
    QString srcPath = QFileDialog::getOpenFileName(nullptr, tr("Choose yaml file"), tr("./"), tr("yaml文件(*.yaml);;所有文件（*.*)"));
    if(srcPath.isEmpty())
    {
        return;
    }
    ui->yaml_src->setText(srcPath);
}
