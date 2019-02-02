#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

#include "stm32f1xx_hal.h"

#define B_2_Pin GPIO_PIN_14
#define B_2_GPIO_Port GPIOB
#define B_1_Pin GPIO_PIN_15
#define B_1_GPIO_Port GPIOB
#define B_0_Pin GPIO_PIN_8
#define B_0_GPIO_Port GPIOA
#define A_2_Pin GPIO_PIN_3
#define A_2_GPIO_Port GPIOB
#define A_1_Pin GPIO_PIN_4
#define A_1_GPIO_Port GPIOB
#define A_0_Pin GPIO_PIN_5
#define A_0_GPIO_Port GPIOB



#define B0_HIGH					HAL_GPIO_WritePin(B_0_GPIO_Port, B_0_Pin,GPIO_PIN_SET)
#define B0_LOW					HAL_GPIO_WritePin(B_0_GPIO_Port, B_0_Pin,GPIO_PIN_RESET)

#define B1_HIGH					HAL_GPIO_WritePin(B_1_GPIO_Port, B_1_Pin,GPIO_PIN_SET)
#define B1_LOW					HAL_GPIO_WritePin(B_1_GPIO_Port, B_1_Pin,GPIO_PIN_RESET)

#define B2_HIGH					HAL_GPIO_WritePin(B_2_GPIO_Port, B_2_Pin,GPIO_PIN_SET)
#define B2_LOW					HAL_GPIO_WritePin(B_2_GPIO_Port, B_2_Pin,GPIO_PIN_RESET)

#define A0_INPUT				HAL_GPIO_ReadPin(A_0_GPIO_Port, A_0_Pin)
#define A1_INPUT				HAL_GPIO_ReadPin(A_1_GPIO_Port, A_1_Pin)
#define A2_INPUT				HAL_GPIO_ReadPin(A_2_GPIO_Port, A_2_Pin)

uint8_t Read_keyboard(void);

#endif //_KEYBOARD_H_

