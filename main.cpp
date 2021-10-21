#include "CameraCalibration.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    CameraCalibration w;
    QFont font = w.font();
    font.setPixelSize(12);
    w.setFont(font);
    w.setWindowTitle("TStoneCalibration");
    w.show();
    return a.exec();
}
