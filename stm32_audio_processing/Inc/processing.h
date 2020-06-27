#ifndef _PROCESSING_H_
#define _PROCESSING_H_

#include "main.h"
#include "cmsis_os.h"


struct proccesing
{
	
	osThreadId 		proccesingTaskHandle;
	void (*ptrTask)(void const * );

	struct 
	{
		uint64_t packet_cnt;
	}statistic;
};

struct proccesing* proccesing_init(void * p);

#endif