/**
 * @file encoder_motor.h
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 编码器电机控制头文件, !!! 此文件已被修改用于编码电机测速示例 !!!
 * @version 0.1
 * @date 2023-05-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __ENCODER_MOTOR_H_
#define __ENCODER_MOTOR_H_

#include <stdint.h>
#include "pid.h"


/* 电机轴每转产生11个脉冲,  计数器在AB相上升下降沿均计数即计数值为脉冲数的4倍, 减速比为90.0。
 * 即电机输出轴每转一圈计数器计数值改变 11.0 * 4.0 * 90.0 = 3960
 */
#define MOTOR_JGB520_TICKS_PER_CIRCLE 3960.0f
#define MOTOR_JGB520_PID_KP  63.0f
#define MOTOR_JGB520_PID_KI  2.6f
#define MOTOR_JGB520_PID_KD  2.4f
#define MOTOR_JGB520_RPS_LIMIT 1.5f

/* 电机轴每转产生11个脉冲,  计数器在AB相上升下降沿均计数即计数值为脉冲数的4倍, 减速比为45.0:1。
 * 即电机输出轴每转一圈计数器计数值改变 11.0 * 4.0 * 45.0 = 1980
 */
#define MOTOR_JGB37_TICKS_PER_CIRCLE 1980.0f
#define MOTOR_JGB37_PID_KP  40.0f
#define MOTOR_JGB37_PID_KI  2.0f
#define MOTOR_JGB37_PID_KD  2.0f
#define MOTOR_JGB37_RPS_LIMIT 3.0f

/* 电机轴每转产生13个脉冲,  计数器在AB相上升下降沿均计数即计数值为脉冲数的4倍, 减速比为20.0:1。
 * 即电机输出轴每转一圈计数器计数值改变 13.0 * 4.0 * 20.0 = 1040
 */
#define MOTOR_JGA27_TICKS_PER_CIRCLE 1040.0f
#define MOTOR_JGA27_PID_KP  -36.0f
#define MOTOR_JGA27_PID_KI  -1.0f
#define MOTOR_JGA27_PID_KD  -1.0f
#define MOTOR_JGA27_RPS_LIMIT 6.0f 

/* 电机轴每转产生11个脉冲,  计数器在AB相上升下降沿均计数即计数值为脉冲数的4倍, 减速比为131.0:1。
 * 即电机输出轴每转一圈计数器计数值改变 11.0 * 4.0 * 131.0 = 5764
 */
#define MOTOR_JGB528_TICKS_PER_CIRCLE 5764.0f
#define MOTOR_JGB528_PID_KP  300.0f
#define MOTOR_JGB528_PID_KI  2.0f
#define MOTOR_JGB528_PID_KD  12.0f
#define MOTOR_JGB528_RPS_LIMIT 1.1f 

/* 电机轴每转产生11个脉冲,  计数器在AB相上升下降沿均计数即计数值为脉冲数的4倍, 减速比为90.0。
 * 即电机输出轴每转一圈计数器计数值改变 11.0 * 4.0 * 90.0 = 3960
 */
#define MOTOR_DEFAULT_TICKS_PER_CIRCLE 3960.0f
#define MOTOR_DEFAULT_PID_KP  63.0f
#define MOTOR_DEFAULT_PID_KI  2.6f
#define MOTOR_DEFAULT_PID_KD  2.4f
#define MOTOR_DEFAULT_RPS_LIMIT 1.35f

typedef struct EncoderMotorObject EncoderMotorObjectTypeDef;

/**
 * @brief 编码器电机对象结构体
*/
struct EncoderMotorObject{
    int64_t counter;        /**< @brief 总计数值, 64bit 认为不会溢出 */
    int64_t overflow_num;   /**< @brief 溢出计数 */
    int32_t ticks_overflow; /**< @brief 计数溢出值 */
    float tps;              /**< @brief ticks per second 计数器频率 */
    float rps;              /**< @brief revolutions per second 输出轴转速 转每秒 */
    int current_pulse;      /**< @brief 当前输出的PWM值, 有符号对应正反转 */
    PID_ControllerTypeDef pid_controller; /**< @brief PID 控制器 */

    /** porting 可移植硬件接口 **/
    int32_t ticks_per_circle; /**< @brief 电机输出轴旋转一圈产生的计数个数, 根据电机实际情况填写 */
	float rps_limit;  /**< @brief 电机地转速极限，一般会取比100% PWM占空比时地转速稍小的值以确保PID控制器对速度的控制 */
    /**
      * @brief 设置电机速度  -1000 ~ 1000
      * @param self 编码器电机对象
      * @param pulse 新的PWM值, -1000~1000, 对应正反转0~100%转速
      * @retval None.
      */
    void (*set_pulse)(EncoderMotorObjectTypeDef *self, int pulse);
};


/**
 * @breif 编码器电机对象初始化
 * @param self 编码器电机对象指针
 * @retval None.
*/
void encoder_motor_object_init(EncoderMotorObjectTypeDef *self);


/**
 * @brief 编码器电机速度测量更新
 * @detials
 * @param self 编码器电机对象
 * @param period 本次调用与上次调用的时间间隔，单位为秒
 * @param counter 编码器当前计数值
 * @retval None.
*/
void encoder_update(EncoderMotorObjectTypeDef *self, float period, int64_t new_counter);


/**
 * @brief 编码器电机速度控制任务
 * @detials 编码器电机速度PID控制任务,需要定时指定以完成PID控制更新
 * @param self 编码器电机对象
 * @param period 当前更新距离上次更新的时间间隔(更新周期), 单位 sec
 * @retval None.
*/
void encoder_motor_control(EncoderMotorObjectTypeDef *self, float period);


/**
 * @brief 编码器电机设置PID控制目标速度
 * @param rps 目标速度， 单位转每秒
 * @retval None.
*/
void encoder_motor_set_speed(EncoderMotorObjectTypeDef *self, float rps);

#endif

