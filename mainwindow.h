#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "driver/openniinterface.h"
#include <QImage>
#include "qimagelabel.h"
#include "pcl/qpclviewer.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    //openni
    openniInterFace camera;
    void displayOpenniImage(Mat inSrc);
    Mat depthGray2Color(Mat inDepth);

protected slots:
    void onOpenBtn();
    void imageReached(Mat depth,Mat rgb);

private:
    Ui::MainWindow *ui;
    Mat depthOrg,rgbOrg;
    QImageLabel *depthLabel,*rgbLabel;
    QPclViewer pclViewer;

};

#endif // MAINWINDOW_H
