#include "adc_int.h"
#include "gpio.h"
#include "tc.h"
#include "intc.h"
#include "led.h"

#include "adcifa.h"

#define ADC_INT_SEOS0 1
#define ADC_INT_SEOS1 16
#define ADC_INT_SEOC0 2
#define ADC_INT_SEOC1 32

	// GPIO pin/adc-function map.
	static const gpio_map_t ADCIFA_GPIO_MAP = {
	{AVR32_ADCREF0_PIN,AVR32_ADCREF0_FUNCTION},
	{AVR32_ADCREFP_PIN,AVR32_ADCREFP_FUNCTION},
	{AVR32_ADCREFN_PIN,AVR32_ADCREFN_FUNCTION},
		
	{RADAR_0_I_PIN, RADAR_0_I_FUNCTION},
	{RADAR_0_Q_PIN, RADAR_0_Q_FUNCTION},
	{RADAR_1_I_PIN, RADAR_1_I_FUNCTION},
	{RADAR_1_Q_PIN, RADAR_1_Q_FUNCTION}

		
	};

	volatile avr32_adcifa_t *adcifa = &AVR32_ADCIFA; // ADCIFA IP registers address

	static volatile int32_t sample_counter, oversampling_counter;
	//32bits version
	//int32_t adci_buffer[ADCI_INPUT_CHANNELS][ADCI_BUFFER_SIZE];
	//16bits version
	int16_t adci_buffer[ADCI_INPUT_CHANNELS][ADCI_BUFFER_SIZE];
	

			// ADC Configuration
	volatile adcifa_opt_t adc_config_options = {
				.frequency                = ADC_FREQUENCY,  // ADC frequency (Hz)
				.reference_source         = ADCIFA_REF1V, // Reference Source
				.sample_and_hold_disable  = false,    // Disable Sample and Hold Time
				.single_sequencer_mode    = true,    // Single Sequencer Mode
				.free_running_mode_enable = false,    // Free Running Mode
				.sleep_mode_enable        = false     // Sleep Mode
			};
			
			// Sequencer Configuration
			adcifa_sequencer_opt_t adcifa_sequence_opt = {
				.convnb               = 4, // Number of sequence
				.resolution           = ADCIFA_SRES_12B,         // Resolution selection
				.trigger_selection    = ADCIFA_TRGSEL_ITIMER,      // Trigger selection
				.start_of_conversion  = ADCIFA_SOCB_ALLSEQ,      // Conversion Management
				.sh_mode              = ADCIFA_SH_MODE_OVERSAMP, // Oversampling Management
				.half_word_adjustment = ADCIFA_HWLA_NOADJ,       // Half word Adjustment
				.software_acknowledge = ADCIFA_SA_NO_EOS_SOFTACK // Software Acknowledge
			};
			
			// Conversions in the Sequencer Configuration
			adcifa_sequencer_conversion_opt_t
			adcifa_sequencer0_conversion_opt[2] = {
				{
					.channel_p = RADAR_0_I_INP,   // Positive Channel
					.channel_n = RADAR_0_I_INN,   // Negative Channel
					.gain      = ADCIFA_SHG_4                     // Gain of the conversion
				},
				{
					.channel_p = RADAR_0_Q_INP,             // Positive Channel
					.channel_n = RADAR_0_Q_INN,             // Negative Channel
					.gain      = ADCIFA_SHG_4               // Gain of the conversion
				},

				{
					.channel_p = RADAR_1_I_INP,   // Positive Channel
					.channel_n = RADAR_1_I_INN,   // Negative Channel
					.gain      = ADCIFA_SHG_4                     // Gain of the conversion
				},
				{
					.channel_p = RADAR_1_Q_INP,             // Positive Channel
					.channel_n = RADAR_1_Q_INN,             // Negative Channel
					.gain      = ADCIFA_SHG_4                     // Gain of the conversion
				}

			};
			
			adcifa_sequencer_conversion_opt_t
			adcifa_sequencer1_conversion_opt[2] = {
				{
					.channel_p = RADAR_0_Q_INP,             // Positive Channel
					.channel_n = RADAR_0_Q_INN,             // Negative Channel
					.gain      = ADCIFA_SHG_1               // Gain of the conversion
				},

				{
					.channel_p = RADAR_1_Q_INP,             // Positive Channel
					.channel_n = RADAR_1_Q_INN,             // Negative Channel
					.gain      = ADCIFA_SHG_1                     // Gain of the conversion
				}
			};
			
			
__attribute__((__interrupt__))
static void processData() {
	int ch;
	int16_t value;
	if (sample_counter>=ADCI_BUFFER_SIZE)  {
		adcifa_disable_interrupt(adcifa, ADC_INT_SEOS0);
		//adcifa_disable_interrupt(adcifa, ADC_INT_SEOS1);
		adcifa_stop_itimer(adcifa);
		return;
	}
	if (((adcifa->sr&ADC_INT_SEOS0) ==0) 
	//|| ((adcifa->sr&ADC_INT_SEOS1) ==0) 
	) {return;};
		
	for (ch=0; ch<4; ch++) {
		value=adcifa->resx[ch];
		if (oversampling_counter==0) {
			//adci_buffer[ch][sample_counter] = (int32_t)value ;
			adci_buffer[ch][sample_counter]=value;
		}else {			
			//adci_buffer[ch][sample_counter] += (int32_t)value ;
			adci_buffer[ch][sample_counter]+=value;
		}			
		
	}
	
	//if (function_generator!=NULL) {
	//	DAC_set_value((*function_generator)(sampleCounter));
	//}
	oversampling_counter++;
	if (oversampling_counter>= OVERSAMPLING) {
		oversampling_counter=0;
		sample_counter++;
	}		
	// acknowledge processing finished
	//adcifa->scr=ADC_INT_SEOS0 | ADC_INT_SEOS1;
}


// Initializes ADC (configures Pins, starts Clock, sets defaults)
void Init_ADCI(){

		// Assign and enable GPIO pins to the ADC function.
		gpio_enable_module(ADCIFA_GPIO_MAP, sizeof(ADCIFA_GPIO_MAP) / sizeof(ADCIFA_GPIO_MAP[0]));

		// Get ADCIFA Factory Configuration
		adcifa_get_calibration_data(adcifa, &adc_config_options);
		if ((uint16_t)adc_config_options.offset_calibration_value == 0xFFFF){
			// Set default calibration if Engineering samples and part is not programmed
			adc_config_options.offset_calibration_value = 0x3B;
			adc_config_options.gain_calibration_value = 0x4210;
			adc_config_options.sh0_calibration_value = 0x210;
			adc_config_options.sh1_calibration_value = 0x210;
		}
		adc_config_options.offset_calibration_value = 0x3B; // offset correction

		// Configure ADCIFA core
		adcifa_configure(adcifa, &adc_config_options, FOSC0);

		// Configure ADCIFA sequencer 0
		adcifa_configure_sequencer(adcifa, 0, &adcifa_sequence_opt, adcifa_sequencer0_conversion_opt);
		// Configure ADCIFA sequencer 1
		//adcifa_configure_sequencer(adcifa, 1, &adcifa_sequence_opt, adcifa_sequencer1_conversion_opt);
		
		adcifa_disable_interrupt(adcifa, 0xffffffff);
		INTC_register_interrupt( (__int_handler) &processData, AVR32_ADCIFA_SEQUENCER0_IRQ, AVR32_INTC_INT1);
		//INTC_register_interrupt( (__int_handler) &processData, AVR32_ADCIFA_SEQUENCER1_IRQ, AVR32_INTC_INT1);

//	int period_us=1000000/samplingrate;
}

// starts sampling, captures one buffer length and then stops
void ADCI_Start_Oneshot(int samplingrate){
	
	int period_us=ADC_FREQUENCY/(samplingrate*OVERSAMPLING);	
	oversampling_counter=0;
	sample_counter=0;
	
	adcifa_enable_interrupt(adcifa, ADC_INT_SEOS0);
	//adcifa_enable_interrupt(adcifa, ADC_INT_SEOS1);
	adcifa_start_itimer(adcifa, period_us);
}

// stops sampling immediately
void ADCI_Stop_Sampling(){
	adcifa_stop_itimer(adcifa);
	
}

// Returns true if one-shot sampling has finished
Bool ADCI_Sampling_Complete(){
	return (sample_counter>=ADCI_BUFFER_SIZE);
}




//void set_DAC_generator_function(generatorfunction new_function_generator ) {
//	function_generator=new_function_generator;
	
//}

float ADCI_get_sample(int channel, int sample) {
	return adci_buffer[channel][sample];
}

int ADCI_get_sampling_status() {
	return sample_counter;
}
