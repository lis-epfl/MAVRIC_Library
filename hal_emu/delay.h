#ifndef DELAY_H 
#define DELAY_H


#include "time_keeper.h"


static inline void delay_ms(int32_t t) {
		uint32_t now=time_keeper_get_micros();
		
		while (time_keeper_get_micros()<now+1000*t) usleep(100);
};




#endif