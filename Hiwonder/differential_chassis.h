#ifndef __DIFFERENTIAL_CHASSIS_H
#define __DIFFERENTIAL_CHASSIS_H

// 包含底盘类型的头文件
#include "chassis.h"

// 结构体 DifferentialChassisTypeDef 定义了差速底盘的参数和控制函数
// 它继承了 ChassisTypeDef 基类，并增加了与差速底盘相关的控制参数和函数指针。
typedef struct {
    ChassisTypeDef base;            // 继承自 ChassisTypeDef 基类，包含基本的底盘控制接口（如 stop、set_velocity 等）

    float wheelbase;                // 轮距，表示左右车轮之间的距离（单位：mm）
    float shaft_length;             // 轴长，表示车轮中心到转向中心的距离（单位：mm）
    float wheel_diameter;           // 车轮直径（单位：mm）
    float correction_factor;        // 修正因子，用于补偿误差或调整实际控制与理论控制之间的差距

    // 控制电机转速的函数指针，通过该指针调用实际的电机控制函数
    void (*set_motors)(void *self, float rps_l, float rps_r);  // set_motors 控制左右电机的转速（rps_l 和 rps_r）
} DifferentialChassisTypeDef;

// 初始化差速底盘对象，并为其设置相应的函数指针
void diff_chassis_object_init(DifferentialChassisTypeDef *self);

// 通过设置线性速度和角速度来控制底盘的运动
// vx 是线性速度，angular_rate 是角速度（单位：rad/s）
static void set_velocity(void *self, float vx, float vy, float angular_rate);

// 根据线性速度和半径来设置底盘的运动
// linear 是车速，r 是转弯半径，insitu 表示是否原地转弯
static void set_velocity_radius(void* self, float linear, float r, bool insitu);

// 停止底盘的运动
static void stop(void *self);

// 控制底盘的运动，根据线性速度 vx 和角速度 angular_rate 来计算左右电机的转速
// 该函数会调用 set_motors 函数来设置电机的速度
void diff_chassis_move(DifferentialChassisTypeDef *self, float vx, float angule_rate);

// 将线性速度转换为电机的转速（rps）
// speed 是线性速度，单位是 mm/s，返回值是电机的转速，单位是 rps（转每秒）
static inline float linear_speed_to_rps(DifferentialChassisTypeDef *self, float speed);

#endif
