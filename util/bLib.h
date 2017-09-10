/**
@file		bLib.h Bin Zheng 
@author		zb@nnc.com
@date		2011/03/12 10:03
@version	v1.0 Copyright(c) 2010-2011 - All Rights Reserved
@warning 	

*************************************************************************/

#ifndef bLib_h__
#define bLib_h__
#include<qglobal.h>
#ifdef __cplusplus
extern "C"{
#endif



//void bcopy
//	(
//	const char *source,       	/* pointer to source buffer      */
//	char *destination,  	/* pointer to destination buffer */
//	int nbytes          	/* number of bytes to copy       */
//);

void bwrite(uint addr, const uchar * pData, int len);

void bwriteLong(uint addr, const uchar * pData, int len);

void bread(uint addr, uchar * pData, int len);

void breadLong(uint addr, uchar * pData, int len);

#ifdef __cplusplus
}
#endif

#endif /* bLib_h__ */

