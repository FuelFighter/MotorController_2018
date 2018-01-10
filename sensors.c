/*
 * sensors.c
 *
 * Created: 10/01/2018 17:28:30
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 */ 

#include "sensors.h"
#include <avr/io.h>



void handle_current_sensor(float *f32_current, uint16_t u16_ADC_reg)
{
	
	volatile float f_new_current = ((((float)u16_ADC_reg*5/4096) - TRANSDUCER_OFFSET)/TRANSDUCER_SENSIBILITY)/3 ;// /3 because current passes 3x in transducer for more precision.
	f_new_current = (f_new_current+0.11)*1.1 ;// correction of offset and ramp error (conversion + hardware) measured with ampmeter of the power supply : bad
	//*f32_prev_current = (*f32_prev_current)*(1-LOWPASS_CONSTANT) + LOWPASS_CONSTANT*f_new_current ;// low pass filter ---------------------TODO test
	*f32_current = f_new_current;
}

void handle_temp_sensor(uint8_t *u8_temp, uint16_t u16_ADC_reg)
{
	volatile float f_sens_volt = ((float)u16_ADC_reg*5/4096)
	// give temp by comparing with voltage thresholds (see calc on drive)
}