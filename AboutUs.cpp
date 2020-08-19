#include "AboutUs.h"
#include "ui_AboutUs.h"

AboutUs::AboutUs(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AboutUs)
{
    ui->setupUi(this);
    ui->label_2->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::LinksAccessibleByMouse);
    ui->label_3->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::LinksAccessibleByMouse);
    ui->label_2->setOpenExternalLinks(true);
    ui->label_3->setOpenExternalLinks(true);
}

AboutUs::~AboutUs()
{
    delete ui;
}
