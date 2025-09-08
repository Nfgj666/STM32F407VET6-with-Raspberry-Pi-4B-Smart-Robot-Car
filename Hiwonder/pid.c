/**
 * @file pid.c
 * @author
 * @brief PID实现
 * @version 0.1
 * @date 2023-07-12
 *
 * @copyright Copyright (c) 2023
 *
 */


#include "pid.h"

void pid_controller_update(PID_ControllerTypeDef *self, float actual, float time_delta) {
	float err = self->set_point - actual;	//计算差值
	float proportion = err - self->previous_0_err;	//计算比例
	
	float integral = err * time_delta;	//计算积分
	float derivative = (err - 2 * self->previous_1_err + self->previous_0_err) / time_delta; //计算微分
	
	self->output = (self->kp * err) + (self->ki * integral) + (self->kd * derivative); //计算校准后的输出
	self->previous_1_err = self->previous_0_err;	//更新上上次的差值
	self->previous_0_err = err;	//更新上次的差值
}


void pid_controller_init(PID_ControllerTypeDef *self, float kp, float ki, float kd) {
	self->set_point = 0;
	self->kp = kp;
	self->ki = ki;
	self->kd = kd;
	self->previous_0_err = 0;
	self->previous_1_err = 0;
	self->output = 0;
}

