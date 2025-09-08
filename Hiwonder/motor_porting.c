/**
 * @file motor_porting.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 编码器电机测速示例，马达硬件接口适配
 * @version 0.1
 * @date 2023-07-12
 *
 * @copyright Copyright (c) 2023
 *
 */


#include "tim.h"
#include "encoder_motor.h"
#include "differential_chassis.h"

// 定义差速底盘对象
DifferentialChassisTypeDef chassis;

/* 全局变量 */
EncoderMotorObjectTypeDef motor1; 
EncoderMotorObjectTypeDef motor2;
EncoderMotorObjectTypeDef motor3;
EncoderMotorObjectTypeDef motor4;/* 电机对象实例 */


/**
 * @brief 设置电机1速度  -1000 ~ 1000
 * @param self 编码器电机对象
 * @param pulse 新的PWM值, -1000~1000, 对应正反转0~100%转速
 * @retval None.
*/
void motor1_set_pulse(EncoderMotorObjectTypeDef *self, int pulse)
{
    if(pulse > 0) {  /* 正转 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse);
    } else if(pulse < 0) {  /* 反转 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -pulse);
    } else {  /* 停止 */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

/**
 * @brief 设置电机2速度
 */
void motor2_set_pulse(EncoderMotorObjectTypeDef *self, int pulse)
{
    if(pulse > 0) {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);  // PE5
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, pulse);  // PE6
    } else if(pulse < 0) {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, -pulse);
    } else {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
    }
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
}

/**
 * @brief 设置电机3速度
 */
void motor3_set_pulse(EncoderMotorObjectTypeDef *self, int pulse)
{
    if(pulse > 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);      // PE13
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);  // PE14 (adjust if CH1 is PE14)
    } else if(pulse < 0) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -pulse);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

/**
 * @brief 设置电机4速度
 */
void motor4_set_pulse(EncoderMotorObjectTypeDef *self, int pulse)
{
    if(pulse > 0) {
        __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);  // PB8
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pulse);  // PB9
    } else if(pulse < 0) {
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, -pulse);
    } else {
        __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
    }
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
}


void diff_drive_control(void *self, float rps_l, float rps_r)
{
	encoder_motor_set_speed(&motor1, rps_l);
	encoder_motor_set_speed(&motor2, rps_l);
	encoder_motor_set_speed(&motor3, rps_r);
	encoder_motor_set_speed(&motor4, rps_r);
}


/**
 * @brief 电机初始化
 * @detials 初始化电机相关内存及定时器
 * @retval None.
 *
 */
void motor_init(void)
{
	encoder_motor_object_init(&motor1);
	encoder_motor_object_init(&motor2);
	encoder_motor_object_init(&motor3);
	encoder_motor_object_init(&motor4);	/* 初始化结构体 */
	
	motor1.ticks_overflow = 60000;
	motor1.set_pulse = motor1_set_pulse;

	/*********************************************  
	* 这里需要换成对应的电机型号参数的宏定义，      
	* 型号参数在 encoder_motor.h 文件中             
	**********************************************/ 
	motor1.rps_limit = MOTOR_JGB520_RPS_LIMIT;
    motor1.ticks_per_circle = MOTOR_JGB520_TICKS_PER_CIRCLE; /* 设置电机每转产生的脉冲总数， 用于计算转速转换为转每秒*/
	motor1.pid_controller.kp = MOTOR_JGB520_PID_KP;
	motor1.pid_controller.ki = MOTOR_JGB520_PID_KI;
	motor1.pid_controller.kd = MOTOR_JGB520_PID_KD;
	
	// 复制到其他电机
	motor2 = motor1;  // 结构体赋值
	motor2.set_pulse = motor2_set_pulse;
	motor3 = motor1;
	motor3.set_pulse = motor3_set_pulse;
	motor4 = motor1;
	motor4.set_pulse = motor4_set_pulse;

	// 初始化底盘
	chassis.wheelbase = 145.0f;  // 设置车轮基距
	chassis.shaft_length = 129.0f;  // 设置车轴长度
	chassis.wheel_diameter = 45.0f;  // 设置车轮直径
	chassis.correction_factor = 1.0f;  // 设置修正因子
	chassis.set_motors = diff_drive_control;  // 设置控制电机转速的函数

    /* 启动PWM定时器 */
    __HAL_TIM_SET_COUNTER(&htim1, 0); /* 清除定时器计数 */
    __HAL_TIM_ENABLE(&htim1);         /* 启动PWM信号生成定时器 */
    __HAL_TIM_MOE_ENABLE(&htim1);     /* 开启PWM信号输出 */

    __HAL_TIM_SET_COUNTER(&htim9, 0);
	__HAL_TIM_ENABLE(&htim9);
	__HAL_TIM_MOE_ENABLE(&htim9);

	__HAL_TIM_SET_COUNTER(&htim10, 0);
	__HAL_TIM_ENABLE(&htim10);
	__HAL_TIM_MOE_ENABLE(&htim10);

	__HAL_TIM_SET_COUNTER(&htim11, 0);
	__HAL_TIM_ENABLE(&htim11);
	__HAL_TIM_MOE_ENABLE(&htim11);

    /* 编码器 */
	// Motor1 (TIM2)
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	__HAL_TIM_SET_AUTORELOAD(&htim2, motor1.ticks_overflow);
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE(&htim2);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	// Motor2 (TIM3)
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	__HAL_TIM_SET_AUTORELOAD(&htim3, motor2.ticks_overflow);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE(&htim3);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	// Motor3 (TIM4)
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
	__HAL_TIM_SET_AUTORELOAD(&htim4, motor3.ticks_overflow);
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE(&htim4);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	// Motor4 (TIM5)
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
	__HAL_TIM_SET_AUTORELOAD(&htim5, motor4.ticks_overflow);
	__HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE(&htim5);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

    //__HAL_TIM_SET_COUNTER(&htim5, 0);               /* 清除定时器计数 */
    //__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);      /* 清除更新中断标志 */
    //__HAL_TIM_SET_AUTORELOAD(&htim5, motor1.ticks_overflow); /* 设置计数溢出值， 可以通过cubemx设置 */
    //__HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);     /* 开启更新中断， 用于计数溢出计算 */
    //__HAL_TIM_ENABLE(&htim5);                       /* 启动定时器 */
    //HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); /* 启动定时器编码器模式 */

    /* 定时器 */
    /* 需要每10ms计算一次编码器数值来获取速度 */
    __HAL_TIM_SET_PRESCALER(&htim6, 83);        /* TIM6连接到APB1, 时钟为84MHz, 84分频==1MHz */
    __HAL_TIM_SET_AUTORELOAD(&htim6, 10000);    /* 1MHz == 1us/cycle, 10000us == 10ms, 每 10ms产生一次更新中断 */
    __HAL_TIM_SET_COUNTER(&htim6, 0);           /* 清除计数 */
    __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);  /* 清除更新中断 */
    __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE); /* 开启更新中断 */
    __HAL_TIM_ENABLE(&htim6);                   /* 启动定时器 */
}

/**
 * @brief TIM2 IRQ (motor1 overflow)
 */
void TIM2_IRQHandler(void)
{
    if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
            --motor1.overflow_num;
        } else {
            ++motor1.overflow_num;
        }
    }
}

/**
 * @brief TIM3 IRQ (motor2)
 */
void TIM3_IRQHandler(void)
{
    if(__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
            --motor2.overflow_num;
        } else {
            ++motor2.overflow_num;
        }
    }
}

/**
 * @brief TIM4 IRQ (motor3)
 */
void TIM4_IRQHandler(void)
{
    if(__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) {
            --motor3.overflow_num;
        } else {
            ++motor3.overflow_num;
        }
    }
}

/**
  * @brief 定时器5中断处理函数
  * @retval None.
  */
void TIM5_IRQHandler(void)
{
    if(__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET) { /* 定时器更新中断 */
        __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE); /* 清除更新中断触发标志 */
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5)) {   /* 检测到编码器为减计数 */
            --motor4.overflow_num; /* 将溢出计数减一 */
        } else {
            ++motor4.overflow_num; /* 将溢出计数加一 */
        }
    }
}


/**
  * @brief 定时器6的中断处理函数
  * @retval None.
  */
void TIM6_DAC_IRQHandler(void)
{
    if(__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET) { /* 定时器更新中断 */
        __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);         /* 清除更新中断标志 */
		//encoder_update(&motor1, 0.01, __HAL_TIM_GET_COUNTER(&htim5)); /* 更新计算电机速度 */
		//encoder_motor_control(&motor1, 0.01);  /* 更新 PID 速度控制 */
        // 更新每个电机
		encoder_update(&motor1, 0.01, __HAL_TIM_GET_COUNTER(&htim2));
		encoder_motor_control(&motor1, 0.01);

		encoder_update(&motor2, 0.01, __HAL_TIM_GET_COUNTER(&htim3));
		encoder_motor_control(&motor2, 0.01);

		encoder_update(&motor3, 0.01, __HAL_TIM_GET_COUNTER(&htim4));
		encoder_motor_control(&motor3, 0.01);

		encoder_update(&motor4, 0.01, __HAL_TIM_GET_COUNTER(&htim5));
		encoder_motor_control(&motor4, 0.01);
    }
}



