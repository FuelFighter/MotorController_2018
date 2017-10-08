/*
 * PWMtest1803.c
 *
 * Created: 18.03.2017 19:25:19
 * Author : Ultrawack
 */ 

#define F_CPU 8000000UL
#define TRANSDUCER_SENSIBILITY 0.0416
#define TRANSDUCER_OFFSET 2.5

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "pid.h"
#include "controller.h"
#include "UniversalModuleDrivers/rgbled.h"
#include "UniversalModuleDrivers/usbdb.h"
#include "UniversalModuleDrivers/pwm.h"
#include "UniversalModuleDrivers/can.h"
#include "UniversalModuleDrivers/adc.h"
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

volatile float pot_voltage = 0;





void timer_init_ts(){
	TCCR1B |= (1<<CS10)|(1<<CS11);
	TCCR1B |= (1<<WGM12); //CTC
	TCNT1 = 0;
	TIMSK1 |= (1<<OCIE1A);
	OCR1A = 12500 - 1;
}


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

void handle_current_sensor(float *f32_prev_current){ //----------------------------------------------------------------------------TODO test if ok with uint32, otherwise use float
	
	volatile float f_new_current = ((((float)u16_ADC2_reg*5/1024) - TRANSDUCER_OFFSET)/TRANSDUCER_SENSIBILITY)/3 ;// /3 because current passes 3x in transducer for more precision.
	*f32_prev_current = (*f32_prev_current)*(1-LOWPASS_CONSTANT) + LOWPASS_CONSTANT*f_new_current ;// low pass filter ---------------------TODO test
}

int main(void)	
{
	cli();
	pid_init(&Current, 0.1, 0.05, 0, 0);
	usbdbg_init();
	pwm_init();
	pwm_set_top_t3(0x319);
	can_init(0,0);
	timer_init_ts();
	//adc_init();
	adc_Free_running_init();
	rgbled_init();
	txFrame.id = MOTOR_CAN_ID;
	txFrame.length = 8;
	sei();
	
	// Output pin to turn off DCDC
	DDRB |= (1 << PB3);
	toggle_DCDC(OFF);
	
	rgbled_turn_on(LED_BLUE);
	
    while (1){
		adc_Free_running_read(CH_ADC2, &u16_ADC2_reg, CH_ADC3, &u16_ADC2_reg) ;
		handle_motor_status_can_msg(&send_can, &ComValues);
		handle_can(&ComValues, &rxFrame);
		handle_current_sensor(&f32_prev_current);
		
		//simple mode with pwm controlled by potentiometer /
		
		pot_voltage = (float)u16_ADC3_reg/1024 ;
		
		//bounding of duty cycle for well function of bootstrap capacitors
		if (pot_voltage > 0.95)
		{
			pot_voltage = 0.95;
		}
		
		if (pot_voltage < 0.05)
		{
			pot_voltage = 0.05;
		}
		
		//set_pwm(pot_voltage);
		OCR3A = (int)(pot_voltage*ICR3) ; //PWM_PE3 (non inverted)
		OCR3B = OCR3A ; //PWM_PE4 (inverted)
	}
}

ISR(TIMER1_COMPA_vect){
	send_can = 1;
	read_current = 1;
}