#ifndef OPENNIINTERFACE_H
#define OPENNIINTERFACE_H

#include <QObject>
#include "driver/openninet.h"
#include "driver/opennilocal.h"

class openniInterFace : public  QObject
{
    Q_OBJECT
public:
    explicit openniInterFace(QObject *parent = 0);

    openniLocal local;
    openniNet net;
    bool openniStart(bool local);
signals:
    void emitImage(Mat depth,Mat rgb);
public slots:
    void postImageMessage(Mat depth,Mat rgb);
};

#endif // OPENNIINTERFACE_H
