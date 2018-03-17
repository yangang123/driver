#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "work_queue.h"
#include "time.h"

int work_queue( struct work_s *work, worker_t worker, uint32_t delay)          
{
  work->worker = worker;           /* Work callback */
  work->delay  = delay;            /* Delay until work performed */
  work->qtime  = timestamp_get();
   
  return 0;
}

void work_queue_check(struct work_s  *work_m)
{   		
	if((timestamp_get() - work_m->qtime) >= work_m->delay) {
      	work_m->worker();
		work_m->qtime = timestamp_get();
	}
}


