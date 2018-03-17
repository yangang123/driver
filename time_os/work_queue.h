#ifndef _WOKR_QUEUE_H
#define _WOKR_QUEUE_H

#include <stdint.h>

typedef void (*worker_t)(void);

struct work_s {
	worker_t  worker;      /* Work callback */
	void *arg;             /* Callback argument */
	uint32_t  qtime;       /* Time work queued */
	uint32_t  delay;       /* Delay until work performed */
};

int work_queue( struct work_s *work, worker_t worker, uint32_t delay);

#endif /* _WOKR_QUEUE_H */