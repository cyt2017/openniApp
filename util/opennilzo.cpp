#include "opennilzo.h"


openniLzo::openniLzo(QObject *parent) : QObject(parent)
{
    if (lzo_init() != LZO_E_OK)
    {
        printf("internal error - lzo_init() failed !!!\n");
        printf("(this usually indicates a compiler bug - try recompiling\nwithout optimizations, and enable '-DLZO_DEBUG' for diagnostics)\n");
       fflush(stdout);
        return ;
    }
    in = (lzo_bytep) malloc(IN_LEN);
    out = (lzo_bytep) malloc(OUT_LEN);
    wrkmem = (lzo_voidp) malloc(LZO1X_1_MEM_COMPRESS);
    if (in == NULL || out == NULL || wrkmem == NULL)
    {
       printf("out of memory\n");
       fflush(stdout);
       return ;
    }
}
openniLzo::~openniLzo()
{
    if(in!=NULL)
    {
        free(in);
        in = NULL;
    }
    if(out!=NULL)
    {
        free(out);
        out = NULL;
    }
    if(wrkmem!=NULL)
    {
        free(wrkmem);
        wrkmem = NULL;
    }
}

void openniLzo::fillData(unsigned char *bytesIn, lzo_uint len)
{
    memset(in,0,IN_LEN);
    memcpy(in,bytesIn,len);
    in_len=len;
}

bool openniLzo::compressData(unsigned char * bytesOut,unsigned int &bytes_len)
{
    int r;
    memset(out,0,OUT_LEN);
    r = lzo1x_1_compress(in, in_len, out, &out_len, wrkmem);
    if (r == LZO_E_OK)
//       printf("compressed %lu bytes into %lu bytes\n",
//              (unsigned long) in_len, (unsigned long) out_len)
               ;
    else
    {
       /* this should NEVER happen */
       printf("internal error - compression failed: %d\n", r);
       return 0;
    }
    /* check for an incompressible block */
    if (out_len >= in_len)
    {
       printf("This block contains incompressible data.\n");
       return 0;
    }

    if(bytesOut!=NULL)
    {
        memcpy(bytesOut,out,out_len);
        bytes_len = out_len;
        return true;
    }
    return 0;
}

bool openniLzo::decompressData(unsigned char * bytesOut,unsigned int &bytes_len)
{
    int r;
    r = lzo1x_decompress(in, in_len, out, &out_len, NULL);
    if (r == LZO_E_OK /*&& out_len==1536000*/)//1536000=640*480*3*2
        printf("decompressed %lu bytes back into %lu bytes\n",
               (unsigned long) in_len, (unsigned long) out_len);
    else
    {
        /* this should NEVER happen */
        printf("internal error - decompression failed: %d\n", r);
        return 0;
    }
    if(bytesOut!=NULL)
    {
        memcpy(bytesOut,out,out_len);
        bytes_len = out_len;
        return true;
    }
    return 0;
}
