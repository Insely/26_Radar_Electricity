#include "Servos.h"
#include "tim.h"

void PWM_control_init(void)
{
	HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_Base_Start(&htim2);	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

}
//设定舵机角度
void set_servo_angle(uint8_t channel, float angle) //统一给180舵机
{
	uint16_t CCR = (2000.0 / 180.0) * angle + 500;
	switch (channel)
	{
	case PWM_PIN_1:
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, CCR);
		break;
	case PWM_PIN_2:
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, CCR);//被串口占用了
		break;
	case PWM_PIN_3:
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, CCR);
		break;
	case PWM_PIN_4:
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, CCR);
		break;
	}
}