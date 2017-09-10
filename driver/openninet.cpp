#include "openninet.h"

openniNet::openniNet(QObject *parent) : QObject(parent)
{
    tcpClientSocket =NULL;
    recvRng=NULL;
    timer=NULL;
}

openniNet::~openniNet()
{
    if(tcpClientSocket!=NULL)
    {
        tcpClientSocket->deleteLater();
        tcpClientSocket=NULL;
    }
    if(recvRng!=NULL)rngDelete(recvRng);
    if(timer!=NULL){
        timer->stop();
        timer->deleteLater();
    }
}

void openniNet::init_net()
{
    tcpClientSocket = new QTcpSocket(this);
    tcpClientSocket->setReadBufferSize(1024*1024);
    tcpClientSocket->setSocketOption(QAbstractSocket::ReceiveBufferSizeSocketOption,1024*1024);
    imgBuf = (unsigned char *)malloc(2000*1024);
    recvRng = rngCreate(2000*1024);
    if(recvRng==NULL)
    {
        printf("rng err!!!!\n");
    }
    connectToServer();
}

bool openniNet::connectToServer()
{
    tcpClientSocket->connectToHost(QHostAddress(SERVER_ADDR), SERVER_PORT);
    int cnt=10;
    while(1)
    {
        //重要=========
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
        if(tcpClientSocket->waitForConnected(300))
        {
            timer = new QTimer(this);
            timer->setInterval(30);
            connect(timer, SIGNAL(timeout()), this, SLOT(readTcpSlot())/*,Qt::AutoConnection*/);
            timer->start(30);
            //connect(tcpClientSocket, SIGNAL(readyRead()), this, SLOT(readTcpSlot()),Qt::AutoConnection);
            connect(tcpClientSocket, SIGNAL(disconnected()), this, SLOT(clientSocketDisconnectedSlot()));
            tcpClientSocket->setReadBufferSize(1024*1024);
            tcpClientSocket->setSocketOption(QAbstractSocket::ReceiveBufferSizeSocketOption,1024*1024);
            break;
        }
        else
        {
            if(cnt==1)
                qDebug() <<tcpClientSocket->errorString();
        }
        cnt--;
        if(cnt==0)
            return false;
    }
    qDebug()<<"socket is ok!";

    return true;
}

void openniNet::procMsg()
{
    MESSAGE *h = (MESSAGE*)imgBuf;
    switch(h->id)
    {
        case ID_DAT:
        {
            procImg(imgBuf+12,h->length-16);
            break;
        }
        case ID_ROBOT_Run:
        {
            break;
        }
        default:break;
    }
}

void openniNet::procImg(uchar *imgDat, uint imgLen)
{
    uint len=0;
   // int64 t1 = cv::getTickCount();
    lzo.fillData(imgDat,imgLen);
    memset(imgBuf,0,2000*1024);
    lzo.decompressData(imgBuf,len);
    Mat depth = Mat(480,640,CV_16UC1,imgBuf);
    Mat1b tmp(1,len-480*640*2,(uchar *)(imgBuf+480*640*2));
//    QFile file("1.jpg");
//    file.open(QIODevice::ReadWrite);
//    file.write((const char *)(imgBuf+480*640*2),len-480*640*2);
//    file.flush();
//    file.close();

    Mat rgb = cv::imdecode(tmp,CV_LOAD_IMAGE_COLOR);
//    Mat rgb = Mat(480,640,CV_8UC3,imgBuf+480*640*2);

    orgDepth=depth.clone();
    orgRgb = rgb.clone();
    emit pushImage(orgDepth,orgRgb);

//    depth2Cloud(orgDepth);
//    displayImage(orgDepth);
//    displayImage(orgRgb);
//    imshow("depth11111",orgDepth);
    //    imshow("rgb1111",orgRgb);
}
#define SIZE_OF_HEAD  sizeof(MESSAGE)
bool checkMsgId(uint id)
{
    uint id_Arry[]={
        ID_DAT,
        ID_ROBOT_Run,
        };

    for(int i=0;i<sizeof(id_Arry);i++)
    {
        if(id ==  id_Arry[i])
            return true;
    }
    return false;
}
void openniNet::readTcpSlot()
{
    long long recvLen;
    char buf[1024*1024]={0};
    char tmp[SIZE_OF_HEAD] = {0};

    disconnect(timer, SIGNAL(timeout()), this, SLOT(readTcpSlot())/*,Qt::AutoConnection*/);
    MESSAGE *h = (MESSAGE*)tmp;
    int64 t1 = cv::getTickCount();
    while(1)
    {
//        int bytes = tcpClientSocket->bytesAvailable();
//        if(bytes<1024*800)
//            break;
        recvLen = tcpClientSocket->read(buf, 1024*1024);
        if(recvLen<=0)
            break;
        rngBufPut(recvRng,buf,recvLen);
    }

    while(1)
    {
        if(rngNBytes(recvRng)<= SIZE_OF_HEAD)
            break;

        rngBufGetNoDel(recvRng,tmp,SIZE_OF_HEAD);

        uint head = h->head;
        if (head != MSGHEAD || !checkMsgId(h->id))
        {
            rngBufGet(recvRng,tmp,1);
            printf(" %02X ",(unsigned char)tmp[0]);
        }
        else
        {
            if(rngNBytes(recvRng)>=h->length)
            {

                int need = h->length;
                int n = rngBufGet(recvRng,(char*)imgBuf,need);
                if(n!=need)
                {
                    printf("err: rngBufGet,n!=need");
                }
//                handlCompleteCmd();

                uint tail = *(uint *)&imgBuf[h->length-4];
                if(tail == MSGTAIL)
                {

                    printf("  tail ok\n ");

                    if(count==0)
                        tCount = getTickCount();
                    count++;

                    procMsg();
                    if(count==30)
                    {
                        fps=30.0/((cv::getTickCount()-tCount)/getTickFrequency());

                        count=0;
                    }
                    qDebug()<<"fps:"<<fps;
                    qDebug()<<"time:"<<(cv::getTickCount()-t1)/getTickFrequency();
                }
                else
                {
                    printf("id % 02X tail bad %08X\n ",h->id,tail);
                }
            }
            else
            {
                break;
            }
        }

    }

    connect(timer, SIGNAL(timeout()), this, SLOT(readTcpSlot())/*,Qt::AutoConnection*/);
        fflush(stdout);

}

void openniNet::clientSocketDisconnectedSlot()
{
    if(tcpClientSocket->isValid() == true)
    {
        tcpClientSocket->close();
//        m_cmdTcpSocket->deleteLater();  //留着,为了后面不需要重新NEW
        //connectToServer();
    }
}
