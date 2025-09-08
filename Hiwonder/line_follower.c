#include "stm32f4xx_hal.h"
#include "gpio.h"
#include <stdbool.h>


/**
  * @brief 四路巡线读取并完成pid允许
  * @reval 底盘的角速度
  *
  */

float line_folower(float kp, float ki, float kd)
{
    static float error = 0, last_error = 0, integral = 0;
    bool s1 = HAL_GPIO_ReadPin(LF_S1_GPIO_Port, LF_S1_Pin);
    bool s2 = HAL_GPIO_ReadPin(LF_S2_GPIO_Port, LF_S2_Pin);
    bool s3 = HAL_GPIO_ReadPin(LF_S3_GPIO_Port, LF_S3_Pin);
    bool s4 = HAL_GPIO_ReadPin(LF_S4_GPIO_Port, LF_S4_Pin);
	
	//printf("%d, %d, %d, %d\r\n", s1, s2, s3, s4);
	
    if( !s1 && s2 && !s3 && !s4) { /* 0 1 0 0 */
        error = -1;
    } else if(s1 && s2 && !s3 && !s4) { /* 1 1 0 0 */
        error = -2;
    } else if(s1 && !s2 && !s3 && !s4) { /* 1 0 0 0 */
        error = -6;
    } else if(!s1 && !s2 && s3 && !s4) { /* 0 0 1 0 */
        error = 1;
    } else if(!s1 && !s2 && s3 && s4) { /* 0 0 1 1 */
        error = 2;
    } else if(!s1 && !s2 && !s3 && s4) { /* 0 0 0 1 */
        error = 6;
    } else {
        error = 0;
    }
	integral += error;
	integral = integral > 80 ? 80 : (integral < -80 ? -80 : integral);
	float output = kp * error + ki * integral + kd * (error - last_error);
	printf("Error: %f, Output: %f\n", error, output);
    return output;
}
