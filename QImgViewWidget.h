#ifndef QIMGVIEW_WIDGET_H
#define QIMGVIEW_WIDGET_H

#include <QWidget>
#include <QImage>

// widget for displaying 2d image
class QImgViewWidget :
    public QWidget
{
public:
    QImgViewWidget(QWidget * parent = 0);
    ~QImgViewWidget() = default;

    /**\brief load image date from file path */
    void SetImage(const QImage& img);

    void ResetTransform();
private:
    /* image data for displaying */
    QImage  img_display_;
    /* original pixmap data*/
    QPixmap pix_ori_;
    /* pixmap data for displaying */
	QPixmap pix_display_;

	/* zoom scale controlled by mouse wheel */
    float   zoom_scale_;
	/* move step controlled by mouse moving*/
    QPoint  move_step_;
    bool    move_start_;
    bool    is_moving_;
    QPoint  mouse_point_;

protected: 
    void paintEvent(QPaintEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
};

#endif