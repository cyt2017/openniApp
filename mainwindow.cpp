#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->tabWidget->setCurrentIndex(1);
    depthLabel = new QImageLabel(ui->tabWidget->currentWidget());
    depthLabel->setGeometry(10,10,640,480);
    ui->tabWidget->setCurrentIndex(0);
    rgbLabel = new QImageLabel(ui->tabWidget->currentWidget());
    rgbLabel->setGeometry(10,10,640,480);

    connect(ui->openBtn,SIGNAL(clicked(bool)),this,SLOT(onOpenBtn()));

}

MainWindow::~MainWindow()
{
    depthLabel->deleteLater();
    rgbLabel->deleteLater();
    delete ui;
}

void MainWindow::displayOpenniImage(Mat inSrc)
{
    Mat src = inSrc.clone();
    QImage img;
    cv::resize(src,src,Size(640,480));
    if(src.channels()==3)
    {
        cvtColor(src,src,CV_BGR2RGB);
        img = QImage(src.data,src.cols,src.rows,src.cols*3,QImage::Format_RGB888);
        rgbLabel->setPixmap(QPixmap::fromImage(img));
//        int r = src.at<Vec3b>(labelRgb->point.ry(),labelRgb->point.rx())[0];
//        int g = src.at<Vec3b>(labelRgb->point.ry(),labelRgb->point.rx())[1];
//        int b = src.at<Vec3b>(labelRgb->point.ry(),labelRgb->point.rx())[2];
//        QString str = QString::asprintf("[%03d,%03d]~R:%03d G:%03d B:%03d",labelRgb->point.rx(),labelRgb->point.ry(),\
//                                        r,g,b);//QString::number(labelRgb->point.rx())+;
//        ui->label_rgb->setText(str);
    }
    else if(src.channels()==1)
    {
        Mat colorDepth = depthGray2Color(src);
        cvtColor(colorDepth,colorDepth,CV_BGR2RGB);
        img = QImage(colorDepth.data,colorDepth.cols,colorDepth.rows,colorDepth.cols*3,QImage::Format_RGB888);
//        int d = src.at<ushort>(labelDepth->point.ry(),labelDepth->point.rx());
//        QString str = QString::asprintf("[%03d,%03d]~depth:%d",labelDepth->point.rx(),labelDepth->point.ry(),\
//                                        d);
        depthLabel->setPixmap(QPixmap::fromImage(img));
//        ui->label_depth->setText(str);
    }
}

Mat MainWindow::depthGray2Color(Mat inDepth)
{
    Mat depth = inDepth.clone();
    double minVal,maxVal;
    cv::minMaxLoc(depth,&minVal,&maxVal);
    Mat dst = Mat(depth.size(),CV_8UC1);
    depth.convertTo(dst,CV_8UC1,255/maxVal);
    Mat color = Mat(depth.size(),CV_8UC3);
    int tmp;
    for(int row=0;row<dst.rows;row++)
        for(int col=0;col<dst.cols;col++)
        {
            tmp = dst.at<uchar>(row,col);
            if(tmp==0)
            {
                color.at<Vec3b>(row,col)[0] = tmp;
                color.at<Vec3b>(row,col)[1] = tmp;
                color.at<Vec3b>(row,col)[2] = tmp;
            }
            else
            {
                color.at<Vec3b>(row,col)[0] = abs(255-tmp);
                color.at<Vec3b>(row,col)[1] = abs(127-tmp);
                color.at<Vec3b>(row,col)[2] = tmp;
            }
        }
    return color;
}

void MainWindow::onOpenBtn()
{
    if(ui->radioButton_local->isChecked() /*&& !ui->radioButton_net->isChecked()*/)
    {
        camera.openniStart(true);

    }
    else if(ui->radioButton_net->isChecked())
    {
        camera.openniStart(false);
    }

   connect(&camera,SIGNAL(emitImage(Mat,Mat)),this,SLOT(imageReached(Mat,Mat)));
   connect(&camera,SIGNAL(emitImage(Mat,Mat)),&pclViewer,SLOT(recvImages(Mat,Mat)));
}

void MainWindow::imageReached(Mat depth, Mat rgb)
{
    rgbOrg = rgb.clone();
    depthOrg = depth.clone();
    displayOpenniImage(rgbOrg);
    displayOpenniImage(depthOrg);

}


