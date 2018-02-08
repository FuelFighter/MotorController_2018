/*
 * controller.c
 *
 * Created: 18.03.2017 13:30:14
 *  Author: Tanguy Simon for DNV GL Fuel fighter
 */ 

#include <avr/io.h>
#include "UniversalModuleDrivers/usbdb.h"
#include "UniversalModuleDrivers/adc.h"
#include "motor_controller_selection.h"
#include "pid.h"
#include "controller.h"

#define RPMTO8BIT 0.051

#define TC 94			//Torque constant
#define SG (0.666)		//Speed/torque gradient
#define SC 102			//Speed constant
#define IMAX 20
#define VCC 50
#define V2PWM 0xFF/VCC
//250W motor
/*
#define V_BATT 20.0
#define R 0.365
#define L 0.000423
*/

//200W motor
#define V_BATT 20.0
#define R 0.608
#define L 0.000161

const float Kp=L*1 ;
const float Ki=R*10 ;
const float TimeStep = 0.01 ; //10ms (see timer 0 in main.c)


static float f32_Integrator = 0.0 ;
static float f32_DutyCycleCmd = 50.0 ;

static bool b_saturation = false;

void controller(float f32_current_cmd, float f32_prev_current, uint8_t b_reset_I){
	
	if (b_reset_I)
	{
		f32_Integrator = 0.0;
	}
	
	float f32_CurrentDelta=f32_current_cmd-f32_prev_current	;
	
	if (!b_saturation) // prevents over integration of an error that cannot be dealt with (because the duty cycle reaches a limit) intgral windup protection
	{
		f32_Integrator+=f32_CurrentDelta*TimeStep ;
	}

	f32_DutyCycleCmd=Kp*f32_CurrentDelta+f32_Integrator*Ki ;
	f32_DutyCycleCmd=(f32_DutyCycleCmd+50) ;
	
	//bounding of duty cycle for well function of bootstrap capacitors
	if (f32_DutyCycleCmd > 95)
	{
		f32_DutyCycleCmd = 95;
		b_saturation = true ;
	}
	
	if (f32_DutyCycleCmd < 5)
	{
		f32_DutyCycleCmd = 5;
		b_saturation = true ;
	}
	
	OCR3A = (int)((f32_DutyCycleCmd/100)*ICR3) ; //PWM_PE3 (non inverted)
	OCR3B = OCR3A ; //PWM_PE4 (inverted)
}

void drivers_init()
{
	DDRB |= (1 << PB4) ;
}
void drivers(uint8_t b_state)
{
	if (b_state)
	{
		PORTB |= (1 << PB4) ;
	}else{
		PORTB &= ~(1 << PB4) ;
	}
}