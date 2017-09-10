#ifndef QPCLVIEWER_H
#define QPCLVIEWER_H

#include <QObject>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>

#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

typedef struct cam_intrinsics
{
    int           width;     /**< Width of the image in pixels */
    int           height;    /**< Height of the image in pixels */
    float         cx;       /**< Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge */
    float         cy;       /**< Vertical coordinate of the principal point of the image, as a pixel offset from the top edge */
    float         fx;        /**< Focal length of the image plane, as a multiple of pixel width */
    float         fy;        /**< Focal length of the image plane, as a multiple of pixel height */
    float         coeffs[5]; /**< Distortion coefficients */
} cam_intrinsics_St;

typedef struct cam_extrinsics
{
    float rotation[9];    /**< Column-major 3x3 rotation matrix */
    float translation[3]; /**< Three-element translation vector, in meters */
} cam_extrinsics_st;


class QPclViewer : public QObject
{
    Q_OBJECT
public:
    explicit QPclViewer(QObject *parent = 0);
    ~QPclViewer();

    //pcl
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::visualization::CloudViewer *viewer;

    cam_intrinsics_St rgbIntrin,depthIntrin;
    cam_extrinsics_st depth2rgbExtrin;

    void depth2Cloud(Mat inDepth);
    void init_PclViewer();

    Mat rgbOrg,depthOrg;

signals:

public slots:
    void recvImages(Mat depth,Mat rgb);
};

#endif // QPCLVIEWER_H
