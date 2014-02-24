#ifndef DELAY_H 
#define DELAY_H


#include "time_keeper.h"
static inline void delay_ms(int t) {
		uint32_t now=get_micros();
		while (get_micros()<now+1000*t);
};




#endif