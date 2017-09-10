


/* rngLib.h - ring buffer subroutine library header */


#ifndef RNGBUFFER_H_20050118_XW
#define RNGBUFFER_H_20050118_XW



#ifdef __cplusplus
extern "C" {
#endif


/* HIDDEN */

/* typedefs */

typedef struct		/* RING - ring buffer */
    {
    int pToBuf;		/* offset from start of buffer where to write next */
    int pFromBuf;	/* offset from start of buffer where to read next */
    int bufSize;	/* size of ring in bytes */
    char *buf;		/* pointer to start of buffer */
    } RING;

/* END_HIDDEN */

typedef RING *RING_ID;


/*
 * The following macros are designed to do various operations on
 * the RING object.  By using them, users can avoid having to know
 * the structure of RING.  However they are intended to be very
 * efficient and this is why they are macros in the first place.
 * In general the parameters to them should be register variables
 * for maximum efficiency.
 */

/*******************************************************************************
*
* RNG_ELEM_GET - get one character from a ring buffer
*
* This macro gets a single character from the specified ring buffer.
* Must supply temporary variable (register int) 'fromP'.
*
* RETURNS: 1 if there was a char in the buffer to return, 0 otherwise
*
* NOMANUAL
*/

#define RNG_ELEM_GET(ringId, pCh, fromP)		\
    (						\
    fromP = (ringId)->pFromBuf,			\
    ((ringId)->pToBuf == fromP) ?		\
	0 					\
    :						\
	(					\
	*pCh = (ringId)->buf[fromP],		\
	(ringId)->pFromBuf = ((++fromP == (ringId)->bufSize) ? 0 : fromP), \
	1					\
	)					\
    )

/*******************************************************************************
*
* RNG_ELEM_PUT - put one character into a ring buffer
*
* This macro puts a single character into the specified ring buffer.
* Must supply temporary variable (register int) 'toP'.
*
* RETURNS: 1 if there was room in the buffer for the char, 0 otherwise
*
* NOMANUAL
*/

#define RNG_ELEM_PUT(ringId, ch, toP)		\
    (						\
    toP = (ringId)->pToBuf,			\
    (toP == (ringId)->pFromBuf - 1) ?		\
	0 					\
    :						\
	(					\
    	(toP == (ringId)->bufSize - 1) ?	\
	    (					\
	    ((ringId)->pFromBuf == 0) ?		\
		0				\
	    :					\
		(				\
		(ringId)->buf[toP] = ch,	\
		(ringId)->pToBuf = 0,		\
		1				\
		)				\
	    )					\
	:					\
	    (					\
	    (ringId)->buf[toP] = ch,		\
	    (ringId)->pToBuf++,			\
	    1					\
	    )					\
	)					\
    )

/* function declarations */

/**详细说明见C文件
*/

/**缓冲区是否满
*/
 int 	rngIsFull (RING_ID ringId);

/**创建一个环形缓冲区
*/
 RING_ID 	rngCreate (int nbytes);

/**取出环形缓冲区的数据
@return The number of bytes actually received from the ring buffer;
*/
 int 	rngBufGet (RING_ID rngId, char *buffer, int maxbytes);

/**向环形缓冲区添加数据
@return The number of bytes actually put into the ring buffer;
*/
 int 	rngBufPut (RING_ID rngId, char *buffer, int nbytes);

/**复制出环形缓冲区的数据，但不删除
@return The number of bytes actually received from the ring buffer;
*/
 int	rngBufGetNoDel (RING_ID rngId, char *buffer, int maxbytes);


/**缓冲区空闲的字节数
*/
 int 	rngFreeBytes (RING_ID ringId);

/**缓冲区已使用的字节数
*/
 int 	rngNBytes (RING_ID ringId);

/**释放缓冲区
*/
 void 	rngDelete (RING_ID ringId);

/**清空缓冲区
*/
 void 	rngFlush (RING_ID ringId);


/**write指针向后移n个字节
*/
 void 	rngMoveAhead (RING_ID ringId, int n);

/**把字符写入缓冲区，但不移动指针
*/
 void 	rngPutAhead (RING_ID ringId, char byte, int offset);

/**read指针向后移n个字节
*/
 void 	rngMoveRead (RING_ID ringId, int n);
 int rngTestBufPut(RING_ID rngId, char *buffer, int nbytes );
int rngTestBufGet(RING_ID rngId, char *buffer, int maxbytes );


#ifdef __cplusplus
}
#endif

#endif /* RNGBUFFER_H_20050118_XW */
