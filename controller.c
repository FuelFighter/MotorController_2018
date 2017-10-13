/*
 * controller.c
 *
 * Created: 18.03.2017 13:30:14
 *  Author: jorgejac
 */ 

#include <avr/io.h>
#include "UniversalModuleDrivers/usbdb.h"
#include "UniversalModuleDrivers/adc.h"
#include "motor_controller_selection.h"
#include "pid.h"

#define RPMTO8BIT 0.051

#define TC 94			//Torque constant
#define SG (0.666)		//Speed/torque gradient
#define SC 102			//Speed constant
#define IMAX 20
#define VCC 50
#define V2PWM 0xFF/VCC

#define V_BATT 48.0
#define R 0.9
#define L (76.0e-6)*2.0

const float Kp=3.0*R/(2.0*V_BATT);
const float Ti= 2.0*L*V_BATT/(3.0*R*R);

void controller(float f32_current_cmd, float f32_prev_current){
	static float f32_CurrentDelta ;
	static float f32_Integrator = 0.0 ;
	static float f32_DutyCycleCmd=0.0 ;
	
	f32_CurrentDelta=f32_current_cmd-f32_prev_current	;

	f32_Integrator+=f32_CurrentDelta*200e-6 ;
	f32_DutyCycleCmd=Kp*f32_CurrentDelta+f32_Integrator/Ti ;
	f32_DutyCycleCmd=(f32_DutyCycleCmd+50) ;
	
	//bounding of duty cycle for well function of bootstrap capacitors

	if (f32_DutyCycleCmd > 95)
	{
		f32_DutyCycleCmd = 95;
	}
	
	if (f32_DutyCycleCmd < 5)
	{
		f32_DutyCycleCmd = 5;
	}
	
	OCR3A = (int)((f32_DutyCycleCmd/100)*ICR3) ; //PWM_PE3 (non inverted)
	OCR3B = OCR3A ; //PWM_PE4 (inverted)
}


void current_saturation(uint16_t *rpm, uint16_t *pwm){
	int pwmMax = (*rpm + TC*SG*IMAX)/SC;
	printf("RPM: %u \t",*rpm);
	printf("PWMMAX: %u \t",pwmMax);
	if(*pwm > pwmMax){
		*pwm = pwmMax;
	}
}

void set_pwm(float duty_cycle){ // duty_cycle c [0,1]------------------------- DOESNT WORK
	//set PWM duty cycle
	OCR3A = (int)(duty_cycle*ICR3) ; //PWM_PE3 (non inverted)
	OCR3B = OCR3A ; //PWM_PE4 (inverted)
}