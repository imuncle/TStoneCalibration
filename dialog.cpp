#include "dialog.h"
#include "ui_dialog.h"

Dialog::Dialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog)
{
    ui->setupUi(this);
    connect(ui->yes, SIGNAL(clicked()), this, SLOT(enter()));
    connect(ui->no, SIGNAL(clicked()), this, SLOT(cancel()));
    connect(ui->browse_left, SIGNAL(clicked()), this,SLOT(choose_left_dir()));
    connect(ui->browse_right, SIGNAL(clicked()), this,SLOT(choose_right_dir()));
}

Dialog::~Dialog()
{
    delete ui;
}

void Dialog::enter()
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
    emit SendSignal(str);
    this->close();
}

void Dialog::cancel()
{
    this->close();
}

void Dialog::choose_left_dir()
{
    QString srcDirPath = QFileDialog::getExistingDirectory(nullptr, "Choose Directory", "./");
    if(srcDirPath.isEmpty())
    {
        return;
    }
    ui->left_src->setText(srcDirPath);
}

void Dialog::choose_right_dir()
{
    QString srcDirPath = QFileDialog::getExistingDirectory(nullptr, "Choose Directory", "./");
    if(srcDirPath.isEmpty())
    {
        return;
    }
    ui->right_src->setText(srcDirPath);
}
