#ifndef ABOUT_H
#define ABOUT_H

#include <QDialog>

namespace Ui {
class AboutUs;
}

class AboutUs : public QDialog
{
    Q_OBJECT

public:
    explicit AboutUs(QWidget *parent = nullptr);
    ~AboutUs();

private:
    Ui::AboutUs *ui;
};

#endif // ABOUT_H
