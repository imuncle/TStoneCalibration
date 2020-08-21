#ifndef DOUBLE_CAPTURE_LINUX_H
#define DOUBLE_CAPTURE_LINUX_H

#include <QMainWindow>

#ifdef Q_OS_LINUX
#include "v4l2.hpp"

namespace Ui {
class double_capture;
}

class double_capture_linux : public QMainWindow
{
    Q_OBJECT

public:
    explicit double_capture_linux(QWidget *parent = nullptr);
    ~double_capture_linux();

private slots:
    void capture();

private:
    Ui::double_capture *ui;
};

#endif

#endif // DOUBLE_CAPTURE_LINUX_H
