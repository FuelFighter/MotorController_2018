/*
 * controller.h
 *
 * Created: 18.03.2017 16:06:03
 *  Author: jorgejac
 */ 


#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <avr/io.h>
#include "pid.h"
void controller(float f32_current_cmd, float f32_prev_current);
int32_t controller_current(Pid_t *PID, uint16_t amp, uint16_t amp_sp);
int32_t controller_trq(Pid_t *PID, uint16_t amp, uint16_t amp_sp);
void current_sample(uint32_t *current_cumulative);
void set_pwm(float duty_cycle);
#endif /* CONTROLLER_H_ */