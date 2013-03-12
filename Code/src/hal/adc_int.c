#include "adc_int.h"
#include "gpio.h"
#include "tc.h"
#include "intc.h"
#include "led.h"


// Initializes ADC (configures Pins, starts Clock, sets defaults)
void Init_ADCI();

// enables continuous sampling  -- not implemented yet
void ADCI_Start_Sampling();

// starts sampling, captures one buffer length and then stops
void ADCI_Start_Oneshot();

// stops sampling immediately
void ADCI_Stop_Sampling();

// Returns true if one-shot sampling has finished
Bool ADCI_Sampling_Complete();
