#include "qpclviewer.h"


/*
 * 把图像坐标转换到世界坐标，输入pixel是图像坐标，depth是该坐标的深度值，intrin是相机内参和畸变系数，point是输出的世界坐标
*/
void transformPix2World(float pixel[2],float depth,cam_intrinsics *intrin,float *point)
{
    bool needDisort=true;
    float x = (pixel[0] - intrin->cx) / intrin->fx;
    float y = (pixel[1] - intrin->cy) / intrin->fy;
    if(needDisort)
    {
        float r2  = x*x + y*y;
        float f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
        float ux = x*f + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
        float uy = y*f + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
        x = ux;
        y = uy;
    }
    point[0] = depth * x;
    point[1] = depth * y;
    point[2] = depth;
}



//根据摄像头之间的外参数进行坐标转换
void  transform_point_to_point(float to_point[3], const cam_extrinsics_st * extrin, const float from_point[3])
{
    to_point[0] = extrin->rotation[0] * from_point[0] + extrin->rotation[3] * from_point[1] + extrin->rotation[6] * from_point[2] + extrin->translation[0];
    to_point[1] = extrin->rotation[1] * from_point[0] + extrin->rotation[4] * from_point[1] + extrin->rotation[7] * from_point[2] + extrin->translation[1];
    to_point[2] = extrin->rotation[2] * from_point[0] + extrin->rotation[5] * from_point[1] + extrin->rotation[8] * from_point[2] + extrin->translation[2];
}


void project_point_to_pixel(float pixel[2], const cam_intrinsics_St * intrin, const float point[3])
{
    bool needDisort=true;
    float x = point[0] / point[2], y = point[1] / point[2];
    if(needDisort)
    {
        float r2  = x*x + y*y;
        float f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
        x *= f;
        y *= f;
        float dx = x + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
        float dy = y + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
        x = dx;
        y = dy;
    }
    pixel[0] = x * intrin->fx + intrin->cx;
    pixel[1] = y * intrin->fy + intrin->cy;
}


QPclViewer::QPclViewer(QObject *parent) : QObject(parent)
{
    init_PclViewer();
}

QPclViewer::~QPclViewer()
{
    delete viewer;
}

void QPclViewer::depth2Cloud(Mat inDepth)
{
    Mat depth = inDepth.clone();
    if(rgbOrg.empty())
        return;
    Mat rgb=rgbOrg.clone() ;
    flip(rgbOrg,rgb,1);
    flip(depth,depth,1);

    cloud->resize(depth.rows*depth.cols);

    int index=0;
    for(int row = 0; row < depth.rows; row++ )
        for(int col = 0; col < depth.cols; col++ )
        {
            float z = depth.at<unsigned short>(row,col);
            float pixelDepth[2];
            float pixelRgb[2];
            float pointDepth[3]={0};
            float pointRgb[3]={0};

            pixelDepth[0]=col;
            pixelDepth[1]=row;
            transformPix2World(pixelDepth ,z,&depthIntrin,pointDepth);
            transform_point_to_point(pointRgb, &depth2rgbExtrin, pointDepth);
            project_point_to_pixel(pixelRgb, &rgbIntrin, pointRgb);

//            float x = z*(col-3.1295888425350620e+02)/5.7997950835080553e+02;
//            float y = z*(row-2.4885853713651699e+02)/5.7819998625099618e+02;

            cloud->at(index).x = pointDepth[1];
            cloud->at(index).y = pointDepth[0];
            cloud->at(index).z = pointDepth[2];

            if(pixelRgb[1]>=0 && pixelRgb[1]<480 && pixelRgb[0]>=0 && pixelRgb[0]<640 )
            {
                cloud->at(index).b= rgb.at<Vec3b>(pixelRgb[1],pixelRgb[0])[0];
                cloud->at(index).g= rgb.at<Vec3b>(pixelRgb[1],pixelRgb[0])[1];
                cloud->at(index).r= rgb.at<Vec3b>(pixelRgb[1],pixelRgb[0])[2];
            }
            else
            {
                cloud->at(index).b= 0;
                cloud->at(index).g= 0;
                cloud->at(index).r= 0;
            }
            index++;
        }

    viewer->showCloud(cloud);
}

void QPclViewer::init_PclViewer()
{
    viewer = new pcl::visualization::CloudViewer("Cloud Viewer");
    if(cloud == nullptr)
    {
            printf("new pcl::PointCloud<pcl::PointXYZRGB>");
            cloud  = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    rgbIntrin.width = 640;
    rgbIntrin.height = 480;
    rgbIntrin.cx = 3.1516460817203853e+02;
    rgbIntrin.cy = 2.3349435020557826e+02;
    rgbIntrin.fx = 5.4142915229715300e+02;
    rgbIntrin.fy = 5.3782839334806715e+02;
    float rgbDistCoeffs[5]=
       { 6.1954367146895985e-02, -2.3738915663735546e-01,
         1.2988409838247921e-03, 4.0444763860041782e-04,
         1.9511163744201454e-01};
    memcpy(rgbIntrin.coeffs,rgbDistCoeffs,sizeof(rgbDistCoeffs));

    depthIntrin.width = 640;
    depthIntrin.height = 480;
    depthIntrin.cx = 3.1979722164705595e+02;
    depthIntrin.cy = 2.4274295377861483e+02;
    depthIntrin.fx = 5.7442499331057195e+02;
    depthIntrin.fy = 5.7065993321562019e+02;

    float depthDistCoeffs[5]=
       {-3.7810163177446933e-02, 6.7699959624872172e-02,
        1.7436018551194777e-04, 3.6298478991780446e-04,
        -1.0890570136580913e-01};
    memcpy(depthIntrin.coeffs,depthDistCoeffs,sizeof(depthDistCoeffs));



    float R[9]={9.9995490675064336e-01, -9.3459495507398436e-03,
                1.6845451335979775e-03, 9.3474095179228462e-03,
                9.9995594141935551e-01, -8.6090384746995402e-04,
                -1.6764249510038581e-03, 8.7661115973327057e-04,
                9.9999821057452809e-01};
    float T[3]={-2.4717915252985989e+01, -6.6435997315783049e-02,
                3.1540349316341016e+00};
    memcpy(depth2rgbExtrin.rotation,R,sizeof(R));
    memcpy(depth2rgbExtrin.translation,T,sizeof(T));
}

void QPclViewer::recvImages(Mat depth, Mat rgb)
{
    rgbOrg = rgb.clone();
    depthOrg = depth.clone();
    depth2Cloud(depthOrg);
}
