#ifndef OPENNINET_H
#define OPENNINET_H

#include <QObject>
#include <QCoreApplication>
#include <QEventLoop>
#include <QTimer>
#include <QTcpServer>
#include<QNetworkInterface>
#include<QTcpSocket>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#include "util/opennilzo.h"
#include "util/rngLib.h"

#define SERVER_ADDR "192.168.1.106"
#define SERVER_PORT 8899
#define Client_PORT 9080


#define MSGTAIL 0x55AA55AA
#define MSGHEAD 0x88886666


typedef enum
{
    ID_DAT = (uint)0x01,//图像数据
    ID_ROBOT_Run = (uint)0x02,
    ID_ROBOT_Stop = (uint)0x03,
    ID_ROBOT_Right = (uint)0x04,
    ID_ROBOT_Left = (uint)0x05,
}MESSAGE_ID;


#pragma pack(1)

typedef struct
{
    uint head;
    uint id;
    uint length;

}MESSAGE;

#pragma pack()

class openniNet : public QObject
{
    Q_OBJECT
public:
    explicit openniNet(QObject *parent = 0);
    ~openniNet();

    void init_net();
    bool connectToServer();
    void procMsg();
    void procImg(uchar *imgDat, uint imgLen);

    QTcpSocket *tcpClientSocket;
    //openniLzo
    openniLzo lzo;
    unsigned char *imgBuf;

    //rnglib
    RING_ID recvRng;

    QTimer *timer;

    Mat orgRgb,orgDepth;

    //fps
    int count;
    int64 tCount;
    float fps;

signals:
    void pushImage(Mat,Mat);

public slots:
    void readTcpSlot();
    void clientSocketDisconnectedSlot();
};

#endif // OPENNINET_H
