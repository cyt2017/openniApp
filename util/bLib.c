

#include "bLib.h"
#include<qglobal.h>
#define  ALIGNMENT              4     /* for word alignment */

void bcopy
    (
    const char *source,       	/* pointer to source buffer      */
    char *destination,  	/* pointer to destination buffer */
    int nbytes          	/* number of bytes to copy       */
    )
    {
    int i = 0;
    if((source == NULL)||(destination==NULL)/*||\
        ((uint)source < 0x80000000)||((uint)destination < 0x80000000)||\
        ((uint)source > 0x9fffffff)||((uint)destination > 0x9fffffff)*/)
	{
        printf("bcopy err source%08X,destination %08X\r\n",(uint)source,(uint)destination);
		return;
	}
	for(i = 0; i < nbytes/4; i ++)
	{
		destination[(4*i)] = source[(4*i)];
		destination[(4*i+1)] = source[(4*i+1)];
		destination[(4*i+2)] = source[(4*i+2)];
		destination[(4*i+3)] = source[(4*i+3)];
	}
	i = i*4;
	while (i < nbytes){
		destination[i] = source[i];
		i ++;
	}
    }

void bwrite(uint addr, const uchar * pData, int len)
{
    int i = 0;
    volatile uchar * pAddr = (uchar *)(addr);
	for(i = 0; i < len/4; i ++)
	{
		pAddr[(4*i)] = pData[(4*i)];
		pAddr[(4*i+1)] = pData[(4*i+1)];
		pAddr[(4*i+2)] = pData[(4*i+2)];
		pAddr[(4*i+3)] = pData[(4*i+3)];
	}
	i = i*4;
	while (i < len){
		pAddr[i] = pData[i];
		i ++;
	}
}

/*
数据总线宽度都是8bit，而APB数据总线的宽度32bit，memory的读写操作中，
仅使用APB总线的低8bit数据。因此memory与APB总线的地址对应关系是i*4
*/
void bwriteLong(uint addr, const uchar * pData, int len)
{
    int i = 0;
    volatile uint * pAddr = (uint *)(addr);
	
	for(i = 0; i < len/4; i ++)
	{
		pAddr[(4*i)] = pData[(4*i)];
		pAddr[(4*i+1)] = pData[(4*i+1)];
		pAddr[(4*i+2)] = pData[(4*i+2)];
		pAddr[(4*i+3)] = pData[(4*i+3)];
	}
	i = i*4;
	while (i < len){
		pAddr[i] = pData[i];
		i ++;
	}
}

void bread(uint addr, uchar * pData, int len)
{
    int i = 0;
    volatile uchar * pAddr = (uchar *)(addr);
	
	for(i = 0; i < len/4; i ++)
	{
		pData[(4*i)] = pAddr[(4*i)];
		pData[(4*i+1)] = pAddr[(4*i+1)];
		pData[(4*i+2)] = pAddr[(4*i+2)];
		pData[(4*i+3)] = pAddr[(4*i+3)];
	}
	i = i*4;
	while (i < len){
		pData[i] = pAddr[i];
		i ++;
	}
}

void breadLong(uint addr, uchar * pData, int len)
{
    int i = 0;
    volatile uint * pAddr = (uint *)(addr);

	for(i = 0; i < len/4; i ++)
	{
        pData[(4*i)] = (uchar)pAddr[(4*i)];
        pData[(4*i+1)] = (uchar)pAddr[(4*i+1)];
        pData[(4*i+2)] = (uchar)pAddr[(4*i+2)];
        pData[(4*i+3)] = (uchar)pAddr[(4*i+3)];
	}
	i = i*4;
	while (i < len){
		pData[i] = pAddr[i];
		i ++;
	}
}

#ifdef ICE_AUTO_TEST_ENABLE

#include "IceAutoTestC.h"

#endif
