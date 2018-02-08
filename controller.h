/*
 * controller.h
 *
 * Created: 18.03.2017 16:06:03
 *  Author: Tanguy Simon for DNV GL Fuel fighter
 */ 


#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <avr/io.h>
#include "pid.h"
void controller(float f32_current_cmd, float f32_prev_current, uint8_t b_reset_I);
void drivers(uint8_t b_state);
void drivers_init();

#endif /* CONTROLLER_H_ */