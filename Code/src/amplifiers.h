/*
 * amplifiers.h
 *
 * Created: 07.11.2011 11:30:41
 *  Author: David Morisod
 */


#ifndef AMPLIFIERS_H_
#define AMPLIFIERS_H_

//#include "uc3c1512c.h"
#include "gpio.h"

#define SDN			AVR32_PIN_PA14
//#define GAIN_0		17  // AVR32_PIN_PA17		// doesn't work
//#define GAIN_1		18  // AVR32_PIN_PA18		// doesn't work
#define GAIN_2		AVR32_PIN_PA19
#define AMP_CS1		AVR32_PIN_PA22
#define AMP_CS2		AVR32_PIN_PA23
#define AMP_CS3		AVR32_PIN_PA24
#define AMP_CS4		AVR32_PIN_PA25


void init_amplifiers();

void AMP_set_gain(int amp_number);	// amp_number is from 1 to 4


#endif /* AMPLIFIERS_H_ */