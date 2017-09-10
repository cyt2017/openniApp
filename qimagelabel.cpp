#include "qimagelabel.h"


QImageLabel::QImageLabel(QWidget *parent) : QLabel(parent)
{
    this->setMouseTracking(true);
}

void QImageLabel::mouseMoveEvent(QMouseEvent *ev)
{
    point = ev->pos();
}

void QImageLabel::mousePressEvent(QMouseEvent *ev)
{
    point = ev->pos();
}

void QImageLabel::mouseReleaseEvent(QMouseEvent *ev)
{
    point = ev->pos();
}
