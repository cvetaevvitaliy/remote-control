
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <RFM69.h>
#include "ssd1306.h" 
#include "fonts.h"
#include "fonts3.h"
#include <stdio.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "keyboard.h"



#define ENCRYPTKEY	"sampleEncryptKey"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

char command_p[10]={"CM:"};
char id_p[10]={"id:"};
char status_p[10]={"st:"};
char remote_command_p[10]={"RC:"};
char data_power_p[11]={"V:"};
char data_1_p[10]={"Temp:"};
char data_2_p[10]={"Pres:"};
char data_3_p[10]={"Humi:"};
char data_4_p[10]={"RSSI:"};


typedef struct{
	uint8_t command;
	uint8_t id;
	uint8_t status;
	uint16_t remote_command;
	int16_t data_power;
	int16_t data_1;
	int16_t data_2;
	int16_t data_3;
	int16_t data_4;
}Data;

Data theData;
Data DataRecive;

enum screen
{
	screen_1,
	screen_2,
	screen_3,
};

uint8_t active_screen=0;


uint8_t key_num=0;

int16_t A_X=0;
int16_t A_Y=0;
int16_t A_Z=0;
int16_t G_X=0;
int16_t G_Y=0;
int16_t G_Z=0;


const uint8_t NODEID = 99;

uint32_t adc_buffer;
uint32_t time=0;

uint8_t time_vbat=10;

char A_x[15];
char A_y[15];
char A_z[15];

//extern  uint8_t data[61];
uint8_t* pdata;
int8_t datarecive[61];
uint8_t len=0;

float v_bat = 0.0;
float v_bat_temp = 0.0;

volatile int32_t timeResivMs=0;
char printRSSI_T[20]={0};
char printCounter[20]={0};
char printTemp[20]={0};
char printDelay[20]={0};
char v_bat_print[8]={0};

char rssi2_print[20]={0};


volatile uint16_t rssi_vaue=0;

bool clear_lcd=true;


uint16_t tick=0;

volatile int16_t rssi2;
volatile int16_t rssi_pos;



volatile int16_t rssi_data[200]={61};



uint16_t rssi_counter=0;


typedef struct {
  uint16_t           a_x;
  uint16_t			 a_y;
  uint16_t        	 a_z;
} Transmit;


Transmit Value;



bool timer_ON=true;
bool timer_ON_state=false;




//extern  uint8_t datalen;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

long map (long x , long in_min , long in_max , long out_min , long out_max) {
	if ( x < in_min )
		return out_min;

	if ( x > in_max )
		return out_max;

	return ( x - in_min ) * ( out_max - out_min ) / ( in_max - in_min ) + out_min;
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//		if (hadc->Instance == ADC1)
//			for (int i=0; i<2; i++)
//		{
//					value[i]=adc_buffer[i];
//		}

//}


//void RF_DONE(){
//	//RFM69_receiveBegin();
////  if(RFM69_receiveDone()){
////				rssi2=RFM69_readRSSI2();
////        pdata = RFM69_receive(&len);
////          for (int i = 0; i < len; i++)
////            datarecive[i]=((char)pdata[i]);
////     theData = *(Payload*)datarecive;
////     tick=0;
////
////		//RFM69_sleep();
////    }
//				rssi2=RFM69_readRSSI2();
//        pdata = RFM69_receive(&len);
//          for (int i = 0; i < len; i++)
//            datarecive[i]=((char)pdata[i]);
//     DataReceive = *(Frame*)datarecive;
//     tick=0;
//		//	RFM69_setMode(RF69_MODE_STANDBY);
//	//	RFM69_sleep();

//}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	PWR->CSR |= PWR_CSR_EWUP;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin (GPIOB , GPIO_PIN_1 , GPIO_PIN_SET);
	I2Cdev_init (&hi2c2);
	HAL_ADC_Start_DMA (&hadc1 , &adc_buffer , 1);
	HAL_Delay (10);
	MPU6050_initialize ();
//	MPU6050_setIntDMPEnabled(true);
//	MPU6050_setAccelerometerPowerOnDelay(3);
//	MPU6050_setIntMotionEnabled(1);
//	MPU6050_setMotionDetectionThreshold(2);
//	MPU6050_setMotionDetectionDuration(5);
//	MPU6050_setZeroMotionDetectionThreshold(54);
//	MPU6050_setZeroMotionDetectionDuration(52);
	MPU6050_setStandbyXGyroEnabled (false);
	MPU6050_setStandbyYGyroEnabled (false);
	MPU6050_setStandbyZGyroEnabled (false);
	MPU6050_setStandbyXAccelEnabled (false);
	MPU6050_setStandbyYAccelEnabled (false);
	MPU6050_setStandbyZAccelEnabled (false);

	ssd1306_Init ();
	// ws28xx_init();
	HAL_Delay (50);
	RFM69_initialize (RF69_433MHZ , NODEID , 100);
	RFM69_listen (false);
	RFM69_initialize_listen ();
	RFM69_encrypt (ENCRYPTKEY);
	RFM69_promiscuous (true);
	RFM69_setPowerLevel (31);
	// RFM69_sleep();
	RFM69_receiveBegin ();
	//RF_DONE();
	HAL_Delay (200);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while ( 1 ) {

/*

		if ( timer_ON == true ) {
			time ++;
			timer_ON_state = false;
		}

		time_vbat ++;

		key_num = Read_keyboard ();

		MPU6050_getMotion6 (&A_X , &A_Y , &A_Z , &G_X , &G_Y , &G_Z);


		if ( RFM69_receiveDone ()) {
			rssi2 = RFM69_readRSSI2 ();
			rssi_pos = rssi2;
			//rssi_pos=~rssi_pos+1;
			pdata = RFM69_receive (&len);
			for ( int i = 0 ; i < len ; i ++ )
				datarecive[ i ] = (( char )pdata[ i ] );
			DataRecive = *( Data * )datarecive;
			sprintf (command_p , "CM:%d" , DataRecive.command);
			sprintf (id_p , "id:%d" , DataRecive.id);
			sprintf (status_p , "st:%d" , DataRecive.status);
			sprintf (remote_command_p , "RC:%d" , DataRecive.remote_command);
			sprintf (data_power_p , "V:%.2fV" , ( DataRecive.data_power / 100.0 ));
			sprintf (data_1_p , "Temp:%d" , DataRecive.data_1);
			sprintf (data_2_p , "Pres:%d" , DataRecive.data_2);
			sprintf (data_3_p , "Humi:%d" , DataRecive.data_3);
			sprintf (data_4_p , "RSSI:%d" , DataRecive.data_4);

			sprintf (rssi2_print , "R:%d" , rssi2);
			//RFM69_sleep();
			rssi_vaue = map (rssi_pos , - 99 , - 28 , 3 , 61);
			rssi_data[ rssi_counter ] = rssi_vaue;
			rssi_counter += 5;


		}

		if ( clear_lcd == true ) {
			SSD1306_DrawFilledRectangle (0 , 0 , 128 , 64 , Black);
			clear_lcd = false;
		}


		if ( active_screen == screen_1 ) {
			clear_lcd = true;
			ssd1306_SetCursor (0 , 0);
			ssd1306_WriteString2 (command_p , Font_7x9 , White);
			ssd1306_SetCursor (0 , 16);
			ssd1306_WriteString2 (id_p , Font_7x9 , White);
			ssd1306_SetCursor (0 , 27);
			ssd1306_WriteString2 (status_p , Font_7x9 , White);
			ssd1306_SetCursor (0 , 38);
			ssd1306_WriteString2 (remote_command_p , Font_7x9 , White);
			ssd1306_SetCursor (0 , 50);
			ssd1306_WriteString2 (data_power_p , Font_7x9 , White);
			ssd1306_SetCursor (60 , 16);
			ssd1306_WriteString2 (data_1_p , Font_7x9 , White);
			ssd1306_SetCursor (60 , 27);
			ssd1306_WriteString2 (data_2_p , Font_7x9 , White);
			ssd1306_SetCursor (60 , 38);
			ssd1306_WriteString2 (data_3_p , Font_7x9 , White);
			ssd1306_SetCursor (60 , 50);
			ssd1306_WriteString2 (data_4_p , Font_7x9 , White);

			ssd1306_SetCursor (40 , 0);
			ssd1306_WriteString2 (rssi2_print , Font_7x9 , White);
		}

		switch ( key_num ) {
			case 4:
				active_screen = screen_2;
				clear_lcd = true;
				break;
			case 1:
				active_screen = screen_1;
				clear_lcd = true;
				break;

		}


		if ( active_screen == screen_2 ) {

			SSD1306_DrawRectangle (1 , 1 , 127 , 63 , White);
			for ( uint8_t i = 0 ; i < 122 ; i += 5 ) {
				SSD1306_DrawLine (125 - i , 61 - rssi_data[ i ] , 125 - i , 61 , White);
				SSD1306_DrawLine (125 - i - 1 , 61 - rssi_data[ i ] , 125 - i - 1 , 61 , White);
				SSD1306_DrawLine (125 - i - 2 , 61 - rssi_data[ i ] , 125 - i - 2 , 61 , White);

			}

			//SSD1306_DrawLine(125-rssi_counter,61-rssi_vaue,125-rssi_counter,61,White);
			ssd1306_SetCursor (5 , 0);
			ssd1306_WriteString (rssi2_print , Font_7x10 , White);

			if ( rssi_counter >= 125 ) {
				rssi_counter = 0;
				SSD1306_DrawFilledRectangle (0 , 0 , 128 , 64 , Black);
				for ( uint8_t i = 0 ; i < sizeof (rssi_data) ; i ++ )
					rssi_data[ i ] = 0;

			}






			//time=0;
		}

		//sprintf(rssi2_print,"RSSI R:%d",rssi2);



//		ssd1306_SetCursor(0,0);
//		ssd1306_WriteString(rssi2_print,Font_7x10,White);
//	//	ssd1306_WriteString(printTemp,Font_7x10,White);
//		ssd1306_SetCursor(0,16);
//		ssd1306_WriteString(printRSSI_T,Font_7x10,White);
//		ssd1306_SetCursor(0,30);
//		ssd1306_WriteString(printCounter,Font_7x10,White);
//		ssd1306_SetCursor(0,45);
//		ssd1306_WriteString(printTemp,Font_7x10,White);


//		ssd1306_SetCursor(60,16);
//		ssd1306_WriteString(A_x,Font_7x10,White);
//		ssd1306_SetCursor(60,30);
//		ssd1306_WriteString(A_y,Font_7x10,White);
//		ssd1306_SetCursor(60,0);
//		ssd1306_WriteString(printRSSI_T,Font_7x10,White);

		ssd1306_SetCursor (90 , 0);
		ssd1306_WriteString (v_bat_print , Font_7x10 , White);

		if ( HAL_GPIO_ReadPin (GPIOB , GPIO_PIN_2)) {
			time = 0;
			ssd1306_SetCursor (100 , 45);
			ssd1306_WriteString ("S" , Font_7x10 , White);
			theData.command = 0;
			theData.id = 2;
			theData.data_power = v_bat * 100;
			theData.remote_command = map (A_X , - 12000 , 12000 , 3000 , 0);
			RFM69_send (100 , ( const void * )( &theData ) , sizeof (theData) , true);
		}

		//--------------------------------------------------------------------------------

		if ( HAL_GPIO_ReadPin (GPIOA , GPIO_PIN_2)) {

			SSD1306_DrawFilledRectangle (0 , 0 , 128 , 64 , Black);
			ssd1306_SetCursor (0 , 0);
			if ( timer_ON == false ) {
				ssd1306_WriteString ("timer On" , Font_7x10 , White);
				timer_ON = true;
			} else {
				ssd1306_WriteString ("timer Off" , Font_7x10 , White);
				timer_ON = false;
			}
			HAL_Delay (80);
			time = 0;
			ssd1306_UpdateScreen ();
			if ( active_screen == screen_2 )
				SSD1306_DrawFilledRectangle (0 , 0 , 128 , 64 , Black);


		} else
			timer_ON_state = false;


//---------------------------------------------------------------------------------

		if ( HAL_GPIO_ReadPin (GPIOB , GPIO_PIN_12)) {
			time = 0;
			ssd1306_SetCursor (100 , 45);
			ssd1306_WriteString ("+" , Font_7x10 , White);
			theData.command = 0;
			theData.id = 2;
			theData.remote_command = 3000;
			theData.data_power = v_bat * 100.0;
			theData.status = 0;
			//	RFM69_listen(false);
			RFM69_send (100 , ( const void * )( &theData ) , sizeof (theData) , true);
			//	RFM69_listen(true);
		}

		if ( HAL_GPIO_ReadPin (GPIOB , GPIO_PIN_13)) {
			time = 0;
			ssd1306_SetCursor (100 , 45);
			ssd1306_WriteString ("-" , Font_7x10 , White);
			theData.command = 0;
			theData.id = 2;
			theData.remote_command = 0;
			theData.data_power = v_bat * 100.0;
			theData.status = 1;
			//	RFM69_listen(false);
			RFM69_send (100 , ( const void * )( &theData ) , sizeof (theData) , true);
			//	RFM69_listen(true);
		}

		if ( HAL_GPIO_ReadPin (GPIOA , GPIO_PIN_0)) {
			time = 0;
			ssd1306_SetCursor (100 , 45);
			//ssd1306_WriteString("K",Font_7x10,White);

		}

		if ( key_num > 0 ) {
			char key_numP[10] = { 0 };
			sprintf (key_numP , "%u" , key_num);
			ssd1306_SetCursor (100 , 45);
			ssd1306_WriteString (key_numP , Font_7x10 , White);
		}


		ssd1306_UpdateScreen ();

		if ( time_vbat > 10 ) {
			float v_bat_temp;
			v_bat_temp = ( float )(( adc_buffer * 3.33 ) / 4095.0 );
			v_bat = ( float )( v_bat_temp / ( 100000.0 / ( 100000.0 + 33000.0 )));
			sprintf (v_bat_print , "%.2fV" , v_bat);
			time_vbat = 0;
		}


		if ( time > 200 ) {
			MPU6050_setIntDMPEnabled (true);
			MPU6050_setAccelerometerPowerOnDelay (3);
			MPU6050_setIntMotionEnabled (1);
			MPU6050_setMotionDetectionThreshold (2);
			MPU6050_setMotionDetectionDuration (5);
			MPU6050_setZeroMotionDetectionThreshold (54);
			MPU6050_setZeroMotionDetectionDuration (52);
			MPU6050_setStandbyXGyroEnabled (true);
			MPU6050_setStandbyYGyroEnabled (true);
			MPU6050_setStandbyZGyroEnabled (true);
//			MPU6050_setStandbyXAccelEnabled(true);
//			MPU6050_setStandbyYAccelEnabled(true);
//			MPU6050_setStandbyZAccelEnabled(true);
			SSD1306_OFF ();
			HAL_Delay (50);

			//RFM69_sleep();
			//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
			PWR->CSR |= PWR_CSR_EWUP;
			PWR->CR |= PWR_CR_CWUF;
			PWR->CR = PWR_CR_PDDS | PWR_CR_CWUF;
			//HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

			HAL_PWR_EnterSTANDBYMode ();

			// PWR->CR &= ~PWR_CR_PDDS;
			// PWR->CR |= PWR_CR_LPDS;

		}
*/

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Buzz_Pin|SPI_NSS_Pin|B0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|B2_Pin|B1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Pairing_key_Pin */
  GPIO_InitStruct.Pin = Pairing_key_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Pairing_key_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzz_Pin SPI_NSS_Pin B0_Pin */
  GPIO_InitStruct.Pin = Buzz_Pin|SPI_NSS_Pin|B0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 B2_Pin B1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|B2_Pin|B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SenseKey_Pin KeyMinus_Pin KeyPlus_Pin A2_Pin 
                           A1_Pin A0_Pin */
  GPIO_InitStruct.Pin = SenseKey_Pin|KeyMinus_Pin|KeyPlus_Pin|A2_Pin 
                          |A1_Pin|A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

		if ( timer_ON == true ) {
			time ++;
			timer_ON_state = false;
		}

		time_vbat ++;

		key_num = Read_keyboard ();

		MPU6050_getMotion6 (&A_X , &A_Y , &A_Z , &G_X , &G_Y , &G_Z);


		if ( RFM69_receiveDone ()) {
			rssi2 = RFM69_readRSSI2 ();
			rssi_pos = rssi2;
			//rssi_pos=~rssi_pos+1;
			pdata = RFM69_receive (&len);
			for ( int i = 0 ; i < len ; i ++ )
				datarecive[ i ] = (( char )pdata[ i ] );
			DataRecive = *( Data * )datarecive;
			sprintf (command_p , "CM:%d" , DataRecive.command);
			sprintf (id_p , "id:%d" , DataRecive.id);
			sprintf (status_p , "st:%d" , DataRecive.status);
			sprintf (remote_command_p , "RC:%d" , DataRecive.remote_command);
			sprintf (data_power_p , "V:%.2fV" , ( DataRecive.data_power / 100.0 ));
			sprintf (data_1_p , "Temp:%d" , DataRecive.data_1);
			sprintf (data_2_p , "Pres:%d" , DataRecive.data_2);
			sprintf (data_3_p , "Humi:%d" , DataRecive.data_3);
			sprintf (data_4_p , "RSSI:%d" , DataRecive.data_4);

			sprintf (rssi2_print , "R:%d" , rssi2);
			//RFM69_sleep();
			rssi_vaue = map (rssi_pos , - 99 , - 28 , 3 , 61);
			rssi_data[ rssi_counter ] = rssi_vaue;
			rssi_counter += 5;


		}

		if ( clear_lcd == true ) {
			SSD1306_DrawFilledRectangle (0 , 0 , 128 , 64 , Black);
			clear_lcd = false;
		}


		if ( active_screen == screen_1 ) {
			clear_lcd = true;
			ssd1306_SetCursor (0 , 0);
			ssd1306_WriteString2 (command_p , Font_7x9 , White);
			ssd1306_SetCursor (0 , 16);
			ssd1306_WriteString2 (id_p , Font_7x9 , White);
			ssd1306_SetCursor (0 , 27);
			ssd1306_WriteString2 (status_p , Font_7x9 , White);
			ssd1306_SetCursor (0 , 38);
			ssd1306_WriteString2 (remote_command_p , Font_7x9 , White);
			ssd1306_SetCursor (0 , 50);
			ssd1306_WriteString2 (data_power_p , Font_7x9 , White);
			ssd1306_SetCursor (60 , 16);
			ssd1306_WriteString2 (data_1_p , Font_7x9 , White);
			ssd1306_SetCursor (60 , 27);
			ssd1306_WriteString2 (data_2_p , Font_7x9 , White);
			ssd1306_SetCursor (60 , 38);
			ssd1306_WriteString2 (data_3_p , Font_7x9 , White);
			ssd1306_SetCursor (60 , 50);
			ssd1306_WriteString2 (data_4_p , Font_7x9 , White);

			ssd1306_SetCursor (40 , 0);
			ssd1306_WriteString2 (rssi2_print , Font_7x9 , White);
		}

		switch ( key_num ) {
			case 4:
				active_screen = screen_2;
				clear_lcd = true;
				break;
			case 1:
				active_screen = screen_1;
				clear_lcd = true;
				break;

		}


		if ( active_screen == screen_2 ) {

			SSD1306_DrawRectangle (1 , 1 , 127 , 63 , White);
			for ( uint8_t i = 0 ; i < 122 ; i += 5 ) {
				SSD1306_DrawLine (125 - i , 61 - rssi_data[ i ] , 125 - i , 61 , White);
				SSD1306_DrawLine (125 - i - 1 , 61 - rssi_data[ i ] , 125 - i - 1 , 61 , White);
				SSD1306_DrawLine (125 - i - 2 , 61 - rssi_data[ i ] , 125 - i - 2 , 61 , White);

			}

			//SSD1306_DrawLine(125-rssi_counter,61-rssi_vaue,125-rssi_counter,61,White);
			ssd1306_SetCursor (5 , 0);
			ssd1306_WriteString (rssi2_print , Font_7x10 , White);

			if ( rssi_counter >= 125 ) {
				rssi_counter = 0;
				SSD1306_DrawFilledRectangle (0 , 0 , 128 , 64 , Black);
				for ( uint8_t i = 0 ; i < sizeof (rssi_data) ; i ++ )
					rssi_data[ i ] = 0;

			}






			//time=0;
		}

		//sprintf(rssi2_print,"RSSI R:%d",rssi2);



//		ssd1306_SetCursor(0,0);
//		ssd1306_WriteString(rssi2_print,Font_7x10,White);
//	//	ssd1306_WriteString(printTemp,Font_7x10,White);
//		ssd1306_SetCursor(0,16);
//		ssd1306_WriteString(printRSSI_T,Font_7x10,White);
//		ssd1306_SetCursor(0,30);
//		ssd1306_WriteString(printCounter,Font_7x10,White);
//		ssd1306_SetCursor(0,45);
//		ssd1306_WriteString(printTemp,Font_7x10,White);


//		ssd1306_SetCursor(60,16);
//		ssd1306_WriteString(A_x,Font_7x10,White);
//		ssd1306_SetCursor(60,30);
//		ssd1306_WriteString(A_y,Font_7x10,White);
//		ssd1306_SetCursor(60,0);
//		ssd1306_WriteString(printRSSI_T,Font_7x10,White);

		ssd1306_SetCursor (90 , 0);
		ssd1306_WriteString (v_bat_print , Font_7x10 , White);

		if ( HAL_GPIO_ReadPin (GPIOB , GPIO_PIN_2)) {
			time = 0;
			ssd1306_SetCursor (100 , 45);
			ssd1306_WriteString ("S" , Font_7x10 , White);
			theData.command = 0;
			theData.id = 2;
			theData.data_power = v_bat * 100;
			theData.remote_command = map (A_X , - 12000 , 12000 , 3000 , 0);
			RFM69_send (100 , ( const void * )( &theData ) , sizeof (theData) , true);
		}

		//--------------------------------------------------------------------------------

		if ( HAL_GPIO_ReadPin (GPIOA , GPIO_PIN_2)) {

			SSD1306_DrawFilledRectangle (0 , 0 , 128 , 64 , Black);
			ssd1306_SetCursor (0 , 0);
			if ( timer_ON == false ) {
				ssd1306_WriteString ("timer On" , Font_7x10 , White);
				timer_ON = true;
			} else {
				ssd1306_WriteString ("timer Off" , Font_7x10 , White);
				timer_ON = false;
			}
			HAL_Delay (80);
			time = 0;
			ssd1306_UpdateScreen ();
			if ( active_screen == screen_2 )
				SSD1306_DrawFilledRectangle (0 , 0 , 128 , 64 , Black);


		} else
			timer_ON_state = false;


//---------------------------------------------------------------------------------

		if ( HAL_GPIO_ReadPin (GPIOB , GPIO_PIN_12)) {
			time = 0;
			ssd1306_SetCursor (100 , 45);
			ssd1306_WriteString ("+" , Font_7x10 , White);
			theData.command = 0;
			theData.id = 2;
			theData.remote_command = 3000;
			theData.data_power = v_bat * 100.0;
			theData.status = 0;
			//	RFM69_listen(false);
			RFM69_send (100 , ( const void * )( &theData ) , sizeof (theData) , true);
			//	RFM69_listen(true);
		}

		if ( HAL_GPIO_ReadPin (GPIOB , GPIO_PIN_13)) {
			time = 0;
			ssd1306_SetCursor (100 , 45);
			ssd1306_WriteString ("-" , Font_7x10 , White);
			theData.command = 0;
			theData.id = 2;
			theData.remote_command = 0;
			theData.data_power = v_bat * 100.0;
			theData.status = 1;
			//	RFM69_listen(false);
			RFM69_send (100 , ( const void * )( &theData ) , sizeof (theData) , true);
			//	RFM69_listen(true);
		}

		if ( HAL_GPIO_ReadPin (GPIOA , GPIO_PIN_0)) {
			time = 0;
			ssd1306_SetCursor (100 , 45);
			//ssd1306_WriteString("K",Font_7x10,White);

		}

		if ( key_num > 0 ) {
			char key_numP[10] = { 0 };
			sprintf (key_numP , "%u" , key_num);
			ssd1306_SetCursor (100 , 45);
			ssd1306_WriteString (key_numP , Font_7x10 , White);
		}


		if ( time_vbat > 10 ) {
			v_bat_temp = (( adc_buffer * 3.33 ) / 4095.0) ;
			v_bat = ( v_bat_temp / ( 100000.0 / ( 100000.0 + 33000.0 )));
			sprintf (v_bat_print , "%.2fV", v_bat);
			time_vbat = 0;
		}

		ssd1306_UpdateScreen ();

		if ( time > 200 ) {
			MPU6050_setIntDMPEnabled (true);
			MPU6050_setAccelerometerPowerOnDelay (3);
			MPU6050_setIntMotionEnabled (1);
			MPU6050_setMotionDetectionThreshold (2);
			MPU6050_setMotionDetectionDuration (5);
			MPU6050_setZeroMotionDetectionThreshold (54);
			MPU6050_setZeroMotionDetectionDuration (52);
			MPU6050_setStandbyXGyroEnabled (true);
			MPU6050_setStandbyYGyroEnabled (true);
			MPU6050_setStandbyZGyroEnabled (true);
//			MPU6050_setStandbyXAccelEnabled(true);
//			MPU6050_setStandbyYAccelEnabled(true);
//			MPU6050_setStandbyZAccelEnabled(true);
			SSD1306_OFF ();
			HAL_Delay (50);

			//RFM69_sleep();
			//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
			PWR->CSR |= PWR_CSR_EWUP;
			PWR->CR |= PWR_CR_CWUF;
			PWR->CR = PWR_CR_PDDS | PWR_CR_CWUF;
			//HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

			HAL_PWR_EnterSTANDBYMode ();

			// PWR->CR &= ~PWR_CR_PDDS;
			// PWR->CR |= PWR_CR_LPDS;

		}


    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
