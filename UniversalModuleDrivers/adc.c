/*
 * adc.c
 *
 * Created: 18.02.2017 10:41:35
 *  Author: olesot, free running by Tanguy Simon
 */ 

//Written for UM

#include "adc.h"
#include <avr/io.h>

void adc_init(void){
	
	/* Voltage ref AVcc with external capacitor on AREF pin */
	ADMUX |= (1<<REFS0);
	
	/* Select prescaler to 64 --> conversion f= 125kHz */
	ADCSRA |= (1<<ADPS2)|(1<<ADPS2);
	
	/* Enable the ADC */
	ADCSRA |= (1<<ADEN);
}

void adc_Free_running_init(void){ //WORKS - check precision
	
	/* Voltage ref AVcc with external capacitor on AREF pin */
	ADMUX |= (1<<REFS0);
	
	/* Select prescaler to 64 --> conversion f= 125kHz */
	ADCSRA |= (1<<ADPS2);
	
	/* Auto trigger of ADC */
	ADCSRA |= (1<<ADATE);
	
	/* Free running mode */
	ADCSRB &= ~((1<<ADTS2)|(1<<ADTS1)|(1<<ADTS0)); // (initially at 0 so not useful)
	
	/* Enable the ADC */
	ADCSRA |= (1<<ADEN);
	
	/* Start the conversion */
	ADCSRA |= (1<<ADSC);
}

uint16_t adc_read(adc_channel_t channel){
	
	//Setting channel and type of reading, see enum in adc.h 
	ADMUX &= 0b11100000;
	ADMUX |= (int8_t)channel;	
		
	/* Start the conversion */
	ADCSRA |= (1<<ADSC);
	
	/* Wait for the conversion to complete */
	while(ADCSRA & (1<<ADSC));
	
	return ADC;
}

void adc_Free_running_read(adc_channel_t channel_A, uint16_t *reg_ADC_A, adc_channel_t channel_B, uint16_t *reg_ADC_B){ //to test - check precision & working
	
	//Setting channel and type of reading, see enum in adc.h
	ADMUX &= 0b11100000;
	ADMUX |= (int8_t)channel_A;
	
	/* Wait for the conversion to complete */
	while(ADCSRA & (1<<ADSC));
	
	*reg_ADC_A = ADC;
	
	//Setting channel and type of reading, see enum in adc.h
	ADMUX &= 0b11100000;
	ADMUX |= (int8_t)channel_B;
	
	/* Wait for the conversion to complete */
	while(ADCSRA & (1<<ADSC));
	
	*reg_ADC_B = ADC;
}