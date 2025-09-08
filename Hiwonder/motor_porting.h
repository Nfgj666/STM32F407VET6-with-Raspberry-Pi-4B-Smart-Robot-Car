#ifndef __MOTOR_PORTING_H
#define __MOTOR_PORTING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"  // STM32 HAL header (modify according to your microcontroller)
#include "encoder_motor.h"
#include "differential_chassis.h"

// 电机对象实例
extern EncoderMotorObjectTypeDef motor1;
extern EncoderMotorObjectTypeDef motor2;
extern EncoderMotorObjectTypeDef motor3;
extern EncoderMotorObjectTypeDef motor4;  // 电机对象

// 差速底盘对象
extern DifferentialChassisTypeDef chassis;

/**
 * @brief 设置电机1速度 -1000 ~ 1000
 * @param self 编码器电机对象
 * @param pulse 新的PWM值, -1000~1000, 对应正反转0~100%转速
 * @retval None.
 */
void motor1_set_pulse(EncoderMotorObjectTypeDef *self, int pulse);

/**
 * @brief 设置电机2速度
 */
void motor2_set_pulse(EncoderMotorObjectTypeDef *self, int pulse);

/**
 * @brief 设置电机3速度
 */
void motor3_set_pulse(EncoderMotorObjectTypeDef *self, int pulse);

/**
 * @brief 设置电机4速度
 */
void motor4_set_pulse(EncoderMotorObjectTypeDef *self, int pulse);

/**
 * @brief 差速底盘控制
 * @param self 差速底盘对象
 * @param rps_l 左轮的转速
 * @param rps_r 右轮的转速
 * @retval None.
 */
void diff_drive_control(void *self, float rps_l, float rps_r);

/**
 * @brief 电机初始化
 * @retval None.
 */
void motor_init(void);

/**
 * @brief TIM2 IRQ (motor1 overflow)
 */
void TIM2_IRQHandler(void);

/**
 * @brief TIM3 IRQ (motor2)
 */
void TIM3_IRQHandler(void);

/**
 * @brief TIM4 IRQ (motor3)
 */
void TIM4_IRQHandler(void);

/**
 * @brief TIM5 IRQ (motor4)
 */
void TIM5_IRQHandler(void);

/**
 * @brief 定时器6中断处理
 */
void TIM6_DAC_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_PORTING_H */
