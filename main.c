/*
 * PWMtest1803.c
 *
 * Created: 18.03.2017 19:25:19
 * Author : Ultrawack
 */ 

#define F_CPU 8000000UL
#define TRANSDUCER_SENSIBILITY 0.0416
#define TRANSDUCER_OFFSET 2.26

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "pid.h"
#include "UniversalModuleDrivers/timer.h"
#include "controller.h"
#include "UniversalModuleDrivers/rgbled.h"
#include "UniversalModuleDrivers/usbdb.h"
#include "UniversalModuleDrivers/pwm.h"
#include "UniversalModuleDrivers/can.h"
#include "UniversalModuleDrivers/adc.h"
#include "UniversalModuleDrivers/uart.h"
#include "motor_controller_selection.h"

// Change Motor in motor_controller_selection.h

uint8_t mode = NORMAL_MODE;
MotorControllerState_t state = IDLE;

// Types
CanMessage_t rxFrame;
CanMessage_t txFrame;
Pid_t Speed;
Pid_t Current;

// Physical values
//static uint32_t mamp = 0;

// Setpoints and commands
//static uint16_t setPoint_rpm = 2000;
//static uint16_t setPoint_pwm = 0;
//static uint16_t setPoint_mamp = 0;
//static uint16_t duty_setpoint = 0;

// Control values
static float f32_prev_current = 0;
static uint8_t send_can = 0;
static uint8_t read_current = 0;
static uint16_t u16_ADC2_reg = 0;
static uint16_t u16_ADC3_reg = 0;
static uint8_t u8_ADC_mux = 0;
volatile uint16_t time_elapsed1 = 0;
volatile float pot_val = 0;





void timer1_init_ts(){
	TCCR1B |= (1<<CS10)|(1<<CS11); // timer 1 prescaler set CLK/64
	TCCR1B |= (1<<WGM12); //CTC
	TCNT1 = 0; //reset timer value
	TIMSK1 |= (1<<OCIE1A); //enable interrupt
	OCR1A = 125 - 1; //compare value //every 1ms
}

void timer0_init_ts(){ 
	TCCR0A |= (1<<CS10)|(1<<CS11); // timer 0 prescaler set CLK/1024
	TCCR0A |= (1<<WGM01); //CTC
	TCNT0 = 0; //reset timer value
	TIMSK0 |= (1<<OCIE0A); //enable interrupt
	OCR0A = 79; //compare value
} // => reload time timer 0 = 100ms

typedef struct{
	uint8_t BMS_status;
	uint8_t throttle_cmd;
	uint8_t restart_overload;
	uint16_t rpm;	
	uint8_t braking;
	uint32_t mamp;
	MotorControllerState_t motor_status; // [||||||statebit2|statebit1]
	uint8_t deadman;
}ModuleValues_t;


ModuleValues_t ComValues = {
	.BMS_status = 0x0,
	.throttle_cmd = 0,
	.restart_overload = 0,
	.rpm = 0,
	.braking = 0,
	.mamp = 0,
	.motor_status = IDLE,
	.deadman = 0
};



void toggle_DCDC(uint8_t OnOff){
	if (OnOff){
		PORTB &= ~(1 << PB3);
	}else{
		PORTB |= (1 << PB3);
	}
}

void handle_can(ModuleValues_t *vals, CanMessage_t *rx){
	if (can_read_message_if_new(rx)){
		switch (rx->id){
			case BRAKE_CAN_ID:
				vals->braking = rx->data.u8[0];
				break;
			case BMS_STATUS_CAN_ID:
				vals->BMS_status = rx->data.u8[0];
				break;
			case STEERING_WHEEL_CAN_ID:
				vals->throttle_cmd = rx->data.u8[3];
				vals->restart_overload = rx->data.u8[1] & HORN;
				vals->deadman = rx->data.u8[2];
				break;
			case ENCODER_CAN_ID:
				vals->rpm = rx->data.u16[ENCODER_CHANNEL];
				break;
		}
	}
}

void handle_motor_status_can_msg(uint8_t *send, ModuleValues_t *vals){
	if(*send){
		txFrame.data.u8[0] = vals->motor_status;
		txFrame.data.u8[1] = vals->throttle_cmd;
		txFrame.data.u16[1] = vals->mamp;
		txFrame.data.u16[2] = OCR3B;
		txFrame.data.u16[3] = vals->rpm;
		
		can_send_message(&txFrame);
		*send = 0;
	}
}
void handle_current_sensor(float *f32_prev_current, uint16_t u16_ADC_reg){ //----------------------------------------------------------------------------TODO test if ok with uint32, otherwise use float
	
	volatile float f_new_current = ((((float)u16_ADC_reg*3.3/1024) - TRANSDUCER_OFFSET)/TRANSDUCER_SENSIBILITY)/3 ;// /3 because current passes 3x in transducer for more precision.
	f_new_current = (f_new_current+0.11)*1.1 ;// correction of offset and ramp error (conversion + hardware) measured with ampmeter of the power supply : bad
	//*f32_prev_current = (*f32_prev_current)*(1-LOWPASS_CONSTANT) + LOWPASS_CONSTANT*f_new_current ;// low pass filter ---------------------TODO test
	*f32_prev_current = f_new_current;
}

int main(void)	
{
	cli();
	pid_init(&Current, 0.1, 0.05, 0, 0);
	usbdbg_init();
	//uart_init();
	//USART0_Init ((unsigned int)(9600));
	pwm_init();
	//pwm_set_top_t3(0x319);
	can_init(0,0);
	timer1_init_ts();
	timer0_init_ts();
	//ADC
	adc_Free_running_init();
	ADMUX &= 0b11100000;
	ADMUX |= CH_ADC2;
	
	rgbled_init();
	txFrame.id = MOTOR_CAN_ID;
	txFrame.length = 8;
	sei();
	
	// Output pin to turn off DCDC
	DDRB |= (1 << PB3);
	toggle_DCDC(OFF);
	
	rgbled_turn_on(LED_BLUE);
	
    while (1){
		
		handle_motor_status_can_msg(&send_can, &ComValues);
		handle_can(&ComValues, &rxFrame);
		
	
		//simple mode with pwm controlled by potentiometer /
	
		pot_val = (float)u16_ADC2_reg/1024 ;
		char buff1 [8];
		itoa(100*pot_val,buff1,10);
		printf("%s,\n",buff1);
		/*
		char buff2 [8];
		itoa(f32_prev_current,buff2,10);
		printf("%s,\n",buff2);
*/
		
		
	}
}

ISR(TIMER1_COMPA_vect){// every 1ms
	send_can = 1;
	read_current = 1;
	handle_current_sensor(&f32_prev_current, u16_ADC3_reg);
	
}

ISR(TIMER0_COMP_vect){ // every 100ms
	controller(pot_val*20-10, f32_prev_current); // 
}


ISR(ADC_vect)
{
	if (u8_ADC_mux == 3)
	{
		u16_ADC3_reg = (ADCL+(ADCH<<8)); // reading conversion result
		u8_ADC_mux = 0;
	}
	if (u8_ADC_mux == 2)
	{
		Set_ADC_Channel(CH_ADC3);
		u8_ADC_mux++ ;
	}
	if (u8_ADC_mux == 1)
	{
		u16_ADC2_reg = (ADCL+(ADCH<<8)); // reading conversion result
		u8_ADC_mux++ ;
	}
	if (u8_ADC_mux == 0)
	{
		Set_ADC_Channel(CH_ADC2);
		u8_ADC_mux++ ;
	}
}
