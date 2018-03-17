#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "time.h"

uint32_t LocalTime = 0;

uint32_t timestamp_get()
{
	return LocalTime;
}

void timestamp_tick()
{
	 LocalTime++;
}


