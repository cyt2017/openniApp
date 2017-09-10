#include "opennilocal.h"

openniLocal::openniLocal(QObject *parent) : QObject(parent)
{

}

openniLocal::~openniLocal()
{
    depthStream.stop();
    rgbStream.stop();
    depthStream.destroy();
    rgbStream.destroy();
    device.close();
    openni::OpenNI::shutdown();
}

void openniLocal::init_local()
{
    Status rc = STATUS_OK;
    rc = OpenNI::initialize();
    if(rc!=STATUS_OK)
    {
        printf("OpenNI::initialize() fail..\n");
        return ;
    }
    rc = device.open(openni::ANY_DEVICE);
    if(rc!=STATUS_OK)
    {
        printf("device.open() fail..\n");
        return ;
    }
    if(device.getSensorInfo(openni::SENSOR_COLOR)!=NULL)
    {
        rc = rgbStream.create(device,SENSOR_COLOR);
        if(rc!=STATUS_OK)
        {
            printf("rgbStream.create() fail..\n");
            return ;
        }
        VideoMode model;
        model.setFps(30);
        model.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
        model.setResolution(640,480);
        rc = rgbStream.setVideoMode(model);
        if(rc!=STATUS_OK)
        {
            printf("rgbStream.setVideoMode() fail..\n");
            return ;
        }
    }
    if(device.getSensorInfo(openni::SENSOR_DEPTH)!=NULL)
    {
        rc = depthStream.create(device,SENSOR_DEPTH);
        if(rc!=STATUS_OK)
        {
            printf("depthStream.create() fail..\n");
            return ;
        }
        VideoMode model;
        model.setFps(30);
        model.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
        model.setResolution(640,480);
        rc = depthStream.setVideoMode(model);
        if(rc!=STATUS_OK)
        {
            printf("depthStream.setVideoMode() fail..\n");
            return ;
        }
    }

    rc = rgbStream.start();
    if(rc!=STATUS_OK)
    {
       printf("rgbStream.start() fail..\n");
       return ;
    }
    rc = depthStream.start();
    if(rc!=STATUS_OK)
    {
        printf("depthStream.start() fail..\n");
        return ;
    }

    timer = new QTimer();

    connect(timer,SIGNAL(timeout()),this,SLOT(onTimeOutLocal()));
    timer->setInterval(20);
    timer->start();
}

void openniLocal::onTimeOutLocal()
{
    Status rc = STATUS_OK;

    rc = rgbStream.readFrame(&rgbFrame);
    if(rc!=STATUS_OK)
    {
        printf("rgbStream.readFrame() fail..\n");
        return ;
    }

    rc = depthStream.readFrame(&depthFrame);
    if(rc!=STATUS_OK)
    {
        printf("depthStream.readFrame() fail..\n");
        return ;
    }

    Mat rgb = Mat(rgbFrame.getHeight(),rgbFrame.getWidth(),CV_8UC3,(void*)rgbFrame.getData());
    if(rgb.empty())
    {
        printf("rgb.empty() fail..\n");
        return ;
    }
    rgbOrg = rgb.clone();

    Mat depth = Mat(depthFrame.getHeight(),depthFrame.getWidth(),CV_16UC1,(void*)depthFrame.getData());
    if(depth.empty())
    {
        printf("depth.empty() fail..\n");
        return ;
    }
    depthOrg = depth.clone();
    emit pushImage(depthOrg,rgbOrg);
}
