#include "QImgViewWidget.h"
#include <QPainter>
#include <QWheelEvent>

QImgViewWidget::QImgViewWidget(QWidget * parent):QWidget(parent)
{
    zoom_scale_ = 1.0f;
    move_start_ = false;
    is_moving_ = false;
}

void QImgViewWidget::SetImage(const QImage& img)
{
	// reset the transformation
    ResetTransform();
    QSize view_size = size();
	// loading by QImage
	img_display_ = img;
    pix_ori_ = QPixmap::fromImage(img_display_);
    if(pix_ori_.isNull()) return;
	pix_display_ = pix_ori_.scaled(zoom_scale_ * size(), Qt::KeepAspectRatio);
}

void QImgViewWidget::ResetTransform()
{
    // reset the zoom scale and move step
    zoom_scale_ = 1.0f;     
    move_step_ = QPoint(0, 0);
}

void QImgViewWidget::paintEvent(QPaintEvent* event)
{
	// rander the loading image
	// TODO(Ethan): It's slow and memory confusing when zooming in
    if(pix_display_.isNull()) return;
	QPainter painter(this);
	painter.drawPixmap(move_step_.x() +(width() - pix_display_.width()) / 2, move_step_ .y()+ (height() - pix_display_.height()) / 2, pix_display_);
}

void QImgViewWidget::wheelEvent(QWheelEvent* event)
{
	// zoom in and out when scrolling mouse
    if (event->delta() > 0) {
        zoom_scale_ *= 1.1;
    }
    else {
        zoom_scale_ *= 0.9;
    }
    // TODO(Ethan): It's slow and memory confusing when zooming in
    if(!pix_ori_.isNull())
	    pix_display_ = pix_ori_.scaled(zoom_scale_ * size(), Qt::KeepAspectRatio);
    update();
}

void QImgViewWidget::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        if (!move_start_) {
            move_start_ = true;
            is_moving_ = false;
            mouse_point_ = event->globalPos();
        }
    }
}

void QImgViewWidget::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
    	  if(move_start_) {
              move_start_ = false;
              is_moving_ = false;
    	  }
    }
}

void QImgViewWidget::mouseMoveEvent(QMouseEvent* event)
{
	// move image when moving mouse
    if (move_start_) {
        const QPoint mos_pt = event->globalPos();
        move_step_ += mos_pt - mouse_point_;
        is_moving_ = true;
        mouse_point_ = mos_pt;
        repaint();
    }
}

void QImgViewWidget::resizeEvent(QResizeEvent* event)
{
    if(!pix_ori_.isNull())
        pix_display_ = pix_ori_.scaled(zoom_scale_ * size(), Qt::KeepAspectRatio);
    update();
}
