#ifndef IMAGEUNDISTORT_H
#define IMAGEUNDISTORT_H

#include <QMainWindow>

namespace Ui {
class ImageUndistort;
}

class ImageUndistort : public QMainWindow
{
    Q_OBJECT

public:
    explicit ImageUndistort(QWidget *parent = nullptr);
    ~ImageUndistort();

private:
    Ui::ImageUndistort *ui;
};

#endif // IMAGEUNDISTORT_H
