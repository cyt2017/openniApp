#include "openniinterface.h"

openniInterFace::openniInterFace(QObject *parent) : QObject(parent)
{

}

bool openniInterFace::openniStart(bool blocal)
{
    if(blocal)
    {
        local.init_local();
        connect(&local,SIGNAL(pushImage(Mat,Mat)),this,SLOT(postImageMessage(Mat,Mat)));
    }
    else
    {
        net.init_net();
        connect(&net,SIGNAL(pushImage(Mat,Mat)),this,SLOT(postImageMessage(Mat,Mat)));
    }
}

void openniInterFace::postImageMessage(Mat depth, Mat rgb)
{
    emit emitImage( depth,   rgb);
}
