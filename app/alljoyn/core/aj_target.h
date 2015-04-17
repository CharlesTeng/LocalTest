#ifndef _AJ_TARGET_H
#define _AJ_TARGET_H
#include <string.h>
#include "c_types.h"
#include "c_stdio.h"
#include "ssl/ssl_crypto.h"

#define AJ_EXPORT 

#define HOST_IS_LITTLE_ENDIAN	1
#define HOST_IS_BIG_ENDIAN		0

#define AJ_NVRAM_SIZE 			0

typedef unsigned long int64_t;
typedef unsigned int size_t;

#define AJ_Printf c_printf
#define AJ_ASSERT(x)	((void)0)

#define WORD_ALIGN(x)	x

//#define min(x,y) (x>y)?y:x
//#define max(x,y) (x<y)?y:x
/*function prototype*/
int AJ_Main(void);
void ms_tick_timer_start(void);
#endif