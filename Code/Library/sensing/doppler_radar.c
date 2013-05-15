/*
 * doppler_radar.c
 *
 * Created: 19/04/2013 16:41:52
 *  Author: sfx
 */ 
#include "doppler_radar.h"
#include "print_util.h"

#include "math.h"

#include "time_keeper.h"


dsp16_complex_t vect_outputI[ ADCI_BUFFER_SIZE];	// Variable pour le DSP
dsp16_complex_t vect_outputQ[ ADCI_BUFFER_SIZE];
dsp16_t vect_inputI[ ADCI_BUFFER_SIZE];
dsp16_t vect_inputQ[ ADCI_BUFFER_SIZE];
dsp16_t vect_realI[ ADCI_BUFFER_SIZE];
dsp16_t vect_imagI[ ADCI_BUFFER_SIZE];
dsp16_t vect_realQ[ ADCI_BUFFER_SIZE];
dsp16_t vect_imagQ[ ADCI_BUFFER_SIZE];
	
dsp16_t vect_realpow[ADCI_BUFFER_SIZE];
dsp16_t vect_imagpow[ADCI_BUFFER_SIZE];

radar_target main_target;


	int32_t alpha=12;		//constant (*100) for the lowpass filtering of amplitude
	int32_t input_max=0;
	int32_t input_min=0;
	int32_t rms=0;
	int32_t amplitude=0;
	int32_t amplitude2=0;
	int32_t amp_old=0;
	int32_t mean_amp=0;
	int32_t frequency=0;
	int32_t speed=0;
	int32_t speed_old=0;
	int32_t speed2=0;
	int32_t index2=0;
	int32_t amp_in=0;
	int32_t taille=0;
	int32_t direction=0;
	int32_t f_v_factor=Sampling_frequency/ADCI_BUFFER_SIZE;		//Conversion factor to compute speed

	uint32_t time1, time2,time_result; //time1-time2=time_result for measure


void calculate_radar() {
	int i=0;
	int j=0;
	int counter=0;
	int reading_status=3;
	int *c=0;
	int read_value=0;
	int32_t index=0;



	////////////FFT 16 bits version
	for(j=0;j< ADCI_BUFFER_SIZE;j++)
	{
		vect_inputI[j]=ADCI_get_sample(0,j);
	}

	dsp16_trans_realcomplexfft(vect_outputI,vect_inputI,10);  //WARNING !! If you change the buffer size you need to change the base 2 power size
																//ex: Buffer=1024-->n=10 / Buffer=512-->n=9
	for(i=0;i<ADCI_BUFFER_SIZE;i++)
	{
		if(vect_inputI[i]>input_max)
			input_max=vect_inputI[i];
		if(vect_inputI[i]<input_min)
			input_min=vect_inputI[i];
					
		rms=+(vect_inputI[i]*vect_inputI[i]);
	}
	rms/=ADCI_BUFFER_SIZE;
																	
	for(j=0;j< ADCI_BUFFER_SIZE;j++)
	{
		vect_inputI[j]=ADCI_get_sample(1,j);
	}
	dsp16_trans_realcomplexfft(vect_outputQ,vect_inputI,10);
		
			
	//Calcul du module
			
	for(i=0;i< ADCI_BUFFER_SIZE;i++)
	{
		vect_realI[i]=vect_outputI[i].real;
		vect_imagI[i]=vect_outputI[i].imag;
		vect_realQ[i]=vect_outputQ[i].real;
		vect_imagQ[i]=vect_outputQ[i].imag;
	}
			
						
	for (i=0;i< ADCI_BUFFER_SIZE;i++)
	{
				
		vect_realpow[i]=vect_realI[i]*vect_realI[i];
		vect_imagpow[i]=vect_imagI[i]*vect_imagI[i];
	}
			
		
	//16bits version
			
	dsp16_vect_add(vect_inputI,vect_realpow,vect_imagpow,ADCI_BUFFER_SIZE);
	for (i=0;i< ADCI_BUFFER_SIZE;i++)
	{
		//16bits version
		vect_inputQ[i]=dsp16_op_sqrt(vect_inputI[i]);	 //Here we re-use vect_inputQ as the module
		//putnum(STDOUT,vect_inputQ[i],10);		//print the raw FFT buffer
		//putstring(STDOUT,";");
//
	}
	//putstring(STDOUT,"\n");
	//putstring(STDOUT,"\n");
			
			

			
	//Find maximum of FFT and corresponding frequency
	//time1=get_micros();
	for(i=1;i<50;i++) //ignore the element 0 (low frequency noise)
	{
				
		if(vect_inputQ[i]>amplitude)
		{
					
			amplitude=vect_inputQ[i];
			index=i;			//find index of max
		}

	}
			
	//Let's find the second maximum peak
	for(i=1;i<50;i++)
	{
		if(vect_inputQ[i]>amplitude2 && i!=index)
		{
			amplitude2=vect_inputQ[i];
			index2=i;
					
		}
				
	}
			
	//Don't need to test the following with index2 because index corresponds to the absolute maximum anyway
			
			
			
	if(index>1)
	{
		mean_amp=(vect_inputQ[index]+vect_inputQ[index+1]+vect_inputQ[index-1])/3; //Average on the three peaks in Fourier domain
	}
			
	else
	{
		mean_amp=(vect_inputQ[index]+vect_inputQ[index+1])/2; //Same average
				
	}
			
	if(mean_amp<THRESHOLD)
	{
		mean_amp=0;
		index=0;
		index2=0;
		amp_old=0;
		speed=0;
	}
	else
	{
			//lowpass exponential filtering applied (alpha =0.75 so we multiply by 100, do the operations then divide by 100)
		if(index>1)
		{
			mean_amp=(alpha*((vect_inputQ[index]+vect_inputQ[index+1]+vect_inputQ[index-1])/3)+(filter_conversion-alpha)*amp_old)/filter_conversion;
			amp_old=mean_amp;	//update the value of the previous amplitude for next iteration
		}
			
		else
		{
			mean_amp=(alpha*((vect_inputQ[index]+vect_inputQ[index+1])/2)+(filter_conversion-alpha)*amp_old)/filter_conversion;
			amp_old=mean_amp;
		}
		//Factor 1000 for speed to avoid decimal value
		//The true speed is (speed)/100 (m/s) or interpret it as cm/s
				
			
		frequency=f_v_factor*10*(index);
		speed=1000*(3*powf(10,8)*frequency/(2*2415*pow(10,9)));	//compute speed (*0.01 because of Fsample and *10 because of freq)
			
		frequency=f_v_factor*10*(index2);
		speed2=1000*(3*powf(10,8)*frequency/(2*2415*pow(10,9)));	//compute speed (*0.01 because of Fsample and *10 because of freq)
			
			
			
			
														
				
			
		//Let's find the speed that is closer to the previous one (to avoid high frequency changes for the speed)
					
		if(abs(speed-speed_old)<=abs(speed2-speed_old))
		{
			speed_old=speed;			//update the old speed
					
	
		}
		else if(abs(speed-speed_old)>abs(speed2-speed_old)) // in this case the speed is the one corresponding to the second peak
		{
			speed_old=speed2;		//update the old speed
			speed=speed2;	
			index=index2;
					
			if(index>1)
			{
						
				mean_amp=(alpha*((vect_inputQ[index2]+vect_inputQ[index2+1]+vect_inputQ[index2-1])/3)+(filter_conversion-alpha)*amp_old)/filter_conversion;
				amp_old=mean_amp;
			}					
			else
			{
				//mean_amp=(vect_inputQ[index2]+vect_inputQ[index2+1])/2;	
				mean_amp=(alpha*((vect_inputQ[index2]+vect_inputQ[index2+1])/2)+(filter_conversion-alpha)*amp_old)/filter_conversion;
				amp_old=mean_amp;
					
			}					
		}
				
	}
			

	//Find the direction of motion by doing vector product between I and Q channel
	if(vect_realI[index]*vect_imagQ[index]-vect_imagI[index]*vect_realQ[index]<0)
	{
		direction=-1;
	}
	else if(speed==0)
	{
		direction=0;
	}
	else
		direction=1;
	time2=get_micros();
	time_result=time2-time1;
	//
	time1=get_micros();
	
	main_target.velocity=direction*speed/100.0;
	//main_target.velocity=speed/100.0;
	main_target.amplitude=amplitude/32000.0;
	/*
	dbg_print_num(direction*speed,10);
			
	dbg_print( "\t");
	dbg_print_num( direction*speed2,10);
	dbg_print( "\t");
		
			
			
	dbg_print_num(amplitude,10);
	dbg_print( "\t");
	dbg_print_num(mean_amp,10);
	dbg_print("\t");
			
			
	dbg_print_num( time_result,10);
	dbg_print("\n");
			
			
		*/	
			
	amplitude=0;
	amplitude2=0;
	index=0;
	index2=0;
	input_min=0;
	input_max=0;
	rms=0;
	mean_amp=0;
	read_value=0;
	
}

radar_target* get_tracked_target() {
	return &main_target;
}

int16_t* get_raw_values() {
	return &vect_inputI;
}
