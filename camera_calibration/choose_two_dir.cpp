#include "choose_two_dir.h"
#include "ui_choose_two_dir.h"

choose_two_dir::choose_two_dir(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::choose_two_dir)
{
    ui->setupUi(this);
    connect(ui->yes, SIGNAL(clicked()), this, SLOT(enter()));
    connect(ui->no, SIGNAL(clicked()), this, SLOT(cancel()));
    connect(ui->browse_left, SIGNAL(clicked()), this,SLOT(choose_left_dir()));
    connect(ui->browse_right, SIGNAL(clicked()), this,SLOT(choose_right_dir()));
}

choose_two_dir::~choose_two_dir()
{
    delete ui;
}

void choose_two_dir::enter()
{
    if(ui->left_src->text().isEmpty() || ui->right_src->text().isEmpty())
    {
        QMessageBox::critical(NULL, "错误", "请将路径填写完毕", QMessageBox::Yes, QMessageBox::Yes);
        return;
    }
    if(ui->corner_size->value() <= 0)
    {
        QMessageBox::critical(NULL, "错误", "角点间距应大于零", QMessageBox::Yes, QMessageBox::Yes);
        return;
    }
    QString str = ui->left_src->text()+","+ui->right_src->text()+","+QString::number(ui->corner_size->value(),10,5);
    this->close();
    emit SendSignal(str);
}

void choose_two_dir::cancel()
{
    this->close();
}

void choose_two_dir::choose_left_dir()
{
    QString srcDirPath = QFileDialog::getExistingDirectory(nullptr, "Choose Directory", "./");
    if(srcDirPath.isEmpty())
    {
        return;
    }
    ui->left_src->setText(srcDirPath);
}

void choose_two_dir::choose_right_dir()
{
    QString srcDirPath = QFileDialog::getExistingDirectory(nullptr, "Choose Directory", "./");
    if(srcDirPath.isEmpty())
    {
        return;
    }
    ui->right_src->setText(srcDirPath);
}
