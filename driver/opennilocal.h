#ifndef OPENNILOCAL_H
#define OPENNILOCAL_H

#include <QObject>
#include <openni2/OpenNI.h>
#include <opencv2/opencv.hpp>
#include <QTimer>

using namespace std;
using namespace openni;
using namespace cv;

class openniLocal : public QObject
{
    Q_OBJECT
public:
    explicit openniLocal(QObject *parent = 0);
    ~openniLocal();

    void init_local();
    Device device;
    VideoStream depthStream,rgbStream;
    VideoFrameRef depthFrame,rgbFrame;
    Mat rgbOrg,depthOrg;
    QTimer *timer;


signals:
    void pushImage(Mat,Mat);
public slots:
    void onTimeOutLocal();
};

#endif // OPENNILOCAL_H
