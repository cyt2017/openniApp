#ifndef QIMAGELABEL_H
#define QIMAGELABEL_H
#include <QWidget>
#include <QLabel>
#include <QMouseEvent>

class QImageLabel : public QLabel
{
    Q_OBJECT
public:
    explicit QImageLabel(QWidget *parent = 0);

    void mouseMoveEvent(QMouseEvent *ev);
    void mousePressEvent(QMouseEvent *ev);
    void mouseReleaseEvent(QMouseEvent *ev);
    QPoint point;
signals:

public slots:
};

#endif // QIMAGELABEL_H
