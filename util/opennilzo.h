#ifndef OPENNILZO_H
#define OPENNILZO_H

#include <QObject>
#include <lzo/lzoconf.h>
#include <lzo/lzo1x.h>

#ifndef IN_LEN
#define IN_LEN      (2000*1024L)
#endif
#define OUT_LEN     (IN_LEN + IN_LEN / 16 + 64 + 3)

class openniLzo : public QObject
{
    Q_OBJECT
public:
    explicit openniLzo(QObject *parent = 0);
    ~openniLzo();

    lzo_bytep in;
    lzo_bytep out;
    lzo_voidp wrkmem;
    lzo_uint in_len;
    lzo_uint out_len;


    void fillData(lzo_bytep bytesIn,lzo_uint len);
    bool compressData(unsigned char * bytesOut, unsigned int &bytes_len);
    bool decompressData(unsigned char * bytesOut,unsigned int &bytes_len);

signals:

public slots:
};

#endif // OPENNILZO_H
