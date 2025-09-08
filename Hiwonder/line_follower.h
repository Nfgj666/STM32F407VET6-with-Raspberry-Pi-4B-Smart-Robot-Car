#ifndef __LINE_FOLLOWER_H
#define __LINE_FOLLOWER_H

#include "stm32f4xx_hal.h"
#include "gpio.h"
#include <stdbool.h>

// 巡线控制函数的原型
float line_folower(float kp, float ki, float kd);

// 可选：定义传感器的GPIO引脚
#define LF_S1_Pin GPIO_PIN_0  // 传感器1引脚（根据实际情况修改）
#define LF_S2_Pin GPIO_PIN_1  // 传感器2引脚（根据实际情况修改）
#define LF_S3_Pin GPIO_PIN_2  // 传感器3引脚（根据实际情况修改）
#define LF_S4_Pin GPIO_PIN_3  // 传感器4引脚（根据实际情况修改）

#define LF_S1_GPIO_Port GPIOB  // 传感器1端口（根据实际情况修改）
#define LF_S2_GPIO_Port GPIOB  // 传感器2端口（根据实际情况修改）
#define LF_S3_GPIO_Port GPIOB  // 传感器3端口（根据实际情况修改）
#define LF_S4_GPIO_Port GPIOB  // 传感器4端口（根据实际情况修改）

// 定义积分最大值和最小值（如果需要控制积分溢出或下溢）
#define INTEGRAL_MAX 80
#define INTEGRAL_MIN -80

#endif /* __LINE_FOLLOWER_H */
