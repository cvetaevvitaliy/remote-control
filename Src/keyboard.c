#include "keyboard.h"

#include "stm32f1xx_hal.h"

uint8_t KEY_SENSE=0;

uint8_t Key_Count=0;

uint8_t Key_input[3][3]={0};
uint8_t Key_input_filter[3][3]={0};
uint8_t Key_input_count[3][3]={0};



//-----------------------------------------------------
void Change_Pins(void)
{
	Key_Count = (Key_Count+1)%3;
	
	if (Key_Count == 0)
	{
		B0_HIGH;
		B1_LOW;
		B2_LOW;

	} else if (Key_Count == 1) {
		B1_HIGH;
		B0_LOW;
		B2_LOW;

	} else if (Key_Count == 2) {
		B2_HIGH;
		B1_LOW;
		B0_LOW;

	} 

	
	//HAL_GPIO_WritePin(GPIOB,B0_Pin, GPIO_PIN_SET);
}

//------------------------------------------------
void Key_read(void)
{
	Key_input[Key_Count][0] = A0_INPUT;
	Key_input[Key_Count][1] = A1_INPUT;
	Key_input[Key_Count][2] = A2_INPUT;

}

//------------------------------------------------
//------------------------------------------------
void key_one_filter(uint8_t x, uint8_t y)
{
	if (Key_input[x][y] == 1)
	{
		if (Key_input_count[x][y] < KEY_SENSE)
		{
			Key_input_count[x][y]++;
		} else {
			Key_input_filter[x][y] = 1;
		}
	} else {
		if (Key_input_count[x][y] > 0)
		{
			Key_input_count[x][y]--;
		} else {
			Key_input_filter[x][y] = 0;
		}
	}
}

//------------------------------------------------
void All_buttons_filter(void)
{
	uint8_t i,j;
	
	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			key_one_filter(i,j);
		}
	}
}

//------------------------------------------------
uint8_t read_button(void)
{
	uint8_t i,j, k=0;
	
	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			k++;
			if (Key_input_filter[i][j] == 1)
			{
				return k;
			}
		}
	}
	return 0;
}

//------------------------------------------------
uint8_t Read_keyboard()
{
	uint8_t answer=0;
	
	Key_read();
	if (Key_Count == 2)
	{
		All_buttons_filter();
	}
	Change_Pins();
	answer = read_button();
	return answer;
}

