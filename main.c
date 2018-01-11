/*
 * PWMtest1803.c
 *
 * Created: 10.01.2018
 * Author : Tanguy Simon for DNV GL Fuel fighter
 * Corresponding Hardware : Motor Drive V2.0
 */ 

#define WATCHDOG_RELOAD_VALUE 5

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "speed.h"
#include "sensors.h"
#include "pid.h"
#include "controller.h"
#include "UniversalModuleDrivers/spi.h"
#include "UniversalModuleDrivers/timer.h"
#include "UniversalModuleDrivers/rgbled.h"
#include "UniversalModuleDrivers/usbdb.h"
#include "UniversalModuleDrivers/pwm.h"
#include "UniversalModuleDrivers/can.h"
#include "UniversalModuleDrivers/adc.h"
#include "UniversalModuleDrivers/uart.h"
#include "motor_controller_selection.h"

// Types
CanMessage_t rxFrame;
CanMessage_t txFrame;
Pid_t Speed;
Pid_t Current;

//security
static uint8_t u8_watchdog = WATCHDOG_RELOAD_VALUE ;

// Control values to be moved in ComValues struct
static float f32_motor_current = 0;
static float f32_batt_current = 0;
static float f32_batt_volt = 0;
static uint8_t u8_motor_temp = 0;
static uint8_t u8_car_speed = 0;

//ADC buffers
static uint16_t u16_ADC1_reg = 0;
static uint16_t u16_ADC2_reg = 0;
static uint16_t u16_ADC3_reg = 0;
static uint16_t u16_ADC5_reg = 0;

static uint8_t send_can = 0;

//for SPI
static uint8_t u8_SPI_count = 0; 
static uint8_t u8_ADC_mux = 0; 
static uint8_t u8_rxBuffer = 0;

//for speed
static uint16_t u16_speed_count = 0;

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
	uint8_t u8_throttle_cmd;
	uint8_t restart_overload;
	uint16_t rpm;	
	uint8_t braking;
	uint32_t mamp;
	MotorControllerState_t motor_status; // [||||||statebit2|statebit1]
	uint8_t deadman;
}ModuleValues_t;


ModuleValues_t ComValues = {
	.u8_throttle_cmd = 0, //in amps
	.restart_overload = 0,
	.rpm = 0,
	.braking = 0,
	.mamp = 0,
	.motor_status = IDLE,
};


void handle_can(ModuleValues_t *vals, CanMessage_t *rx){
	if (can_read_message_if_new(rx)){
		switch (rx->id){
			case BRAKE_CAN_ID:
				vals->braking = rx->data.u8[0];
				u8_watchdog = WATCHDOG_RELOAD_VALUE ;
				break;
			case FORWARD_CAN_ID:
				vals->u8_throttle_cmd = rx->data.u8[3];
				u8_watchdog = WATCHDOG_RELOAD_VALUE ;
				break;
				/*
			case ENCODER_CAN_ID:
				vals->rpm = rx->data.u16[ENCODER_CHANNEL];
				break;
				*/
		}
	}
}

void handle_motor_status_can_msg(uint8_t *send, ModuleValues_t *vals){
	if(*send){
		txFrame.data.u8[0] = vals->motor_status;
		txFrame.data.u8[1] = vals->u8_throttle_cmd;
		txFrame.data.u16[1] = vals->mamp;
		txFrame.data.u16[2] = OCR3B ;   //not useful
		txFrame.data.u16[3] = vals->rpm;
		
		can_send_message(&txFrame);
		*send = 0;
	}
}

int main(void)	
{
	cli();
	pid_init(&Current, 0.1, 0.05, 0, 0);
	pwm_init();
	can_init(0,0);
	timer1_init_ts();
	timer0_init_ts();
	speed_init();
	
	spi_init(DIV_2); // init of SPI for external ADC device

	
	rgbled_init();
	txFrame.id = MOTOR_CAN_ID;
	txFrame.length = 8;
	
	sei();
	
	rgbled_turn_on(LED_BLUE);
	
    while (1){
		
		handle_motor_status_can_msg(&send_can, &ComValues);
		handle_can(&ComValues, &rxFrame);
	}
}


ISR(TIMER0_COMP_vect){ // every 100ms
	if (u8_watchdog == 0)
	{
		ComValues.u8_throttle_cmd = 0 ;
		/*TODO
		* send CAN to demand motor disengage
		* drivers disable
		*/
	} else {
		u8_watchdog -- ;	
	}
	send_can = 1;
	controller(ComValues.u8_throttle_cmd, f32_motor_current);
	handle_speed_sensor(&u8_car_speed, u16_speed_count, 100);
}


/////////////////////////////////////COMMUNICATION WITH EXTERNAL ADC////////////////////////////////
/*External ADC HW setup (on Motor Drive V2.0):
*	CH1 : Motor current
*	CH2 : Battery current
*	CH3 : Battery voltage
*	CH5 : Motor temperature
*/

ISR(TIMER1_COMPA_vect){// every 1ms

	if (u8_SPI_count == 1)
	{
		//motor current
		u8_ADC_mux = 1; 
		spi_trancieve(&u8_ADC_mux, &u8_rxBuffer, 16, 0);
		u16_ADC1_reg = u8_rxBuffer;
		u8_SPI_count ++ ;

	}

	if (u8_SPI_count == 2)
	{
		//batt current
		u8_ADC_mux = 2;
		spi_trancieve(&u8_ADC_mux, &u8_rxBuffer, 16, 0);
		u16_ADC2_reg = u8_rxBuffer;
		u8_SPI_count ++ ;

	}

	if (u8_SPI_count == 3)
	{
		//batt volt
		u8_ADC_mux = 3;
		spi_trancieve(&u8_ADC_mux, &u8_rxBuffer, 16, 0);
		u16_ADC3_reg = u8_rxBuffer;
		u8_SPI_count ++ ;

	}

	if (u8_SPI_count == 4)
	{
		//motor temp
		u8_ADC_mux = 5;
		spi_trancieve(&u8_ADC_mux, &u8_rxBuffer, 16, 0);
		u16_ADC5_reg = u8_rxBuffer;
		u8_SPI_count = 1 ;

	}
	
	////////////////////INTERPRETATION OF RECEIVED ADC VALUES//////////////
	handle_current_sensor(&f32_motor_current, u16_ADC1_reg);
	handle_current_sensor(&f32_batt_current, u16_ADC2_reg);
	f32_batt_volt = (float)u16_ADC3_reg/82; //*5/4096 (12bit ADC with Vref = 5V) *0.1 (divider bridge 50V -> 5V)
	handle_temp_sensor(&u8_motor_temp, u16_ADC5_reg);
}

ISR(INT5_vect)
{
	u16_speed_count ++ ;
}
