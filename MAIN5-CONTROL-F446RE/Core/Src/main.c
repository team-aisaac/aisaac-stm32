/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include "sd_hal_mpu6050.h"
#include "command.h"
#include "MadgwickAHRS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*CAN*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_TxHeaderTypeDef TxHeader1;
CAN_RxHeaderTypeDef RxHeader1;
CAN_FilterTypeDef sFilterConfig1;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
CAN_FilterTypeDef sFilterConfig;
/*I2C*/
I2C_HandleTypeDef hi2c3;
/*GYRO*/
SD_MPU6050 mpu1;
SD_MPU6050_Result result;
Madgwick MadgwickFilter;
/*SPI*/
Si8902 si8902;
SPI_HandleTypeDef hspi1;
/*ENCODER*/
ENCODER Encoder;

//#define CALI_MODE
#define FIR 9999
#define Kf 0.9
#define A_X 0.024
#define A_Y 0.002
#define A_Z 0.13
#define G_X -4.029
#define G_Y 0.626
#define G_Z -0.386
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t TxData[8]={0,0,0,0,0,0,0,0};
uint8_t RxData[8]={0,0,0,0,0,0,0,0};
uint32_t TxMailbox=0;

int yaw=0;

float g_x[2];
float g_y[2];
float g_z[2];
float a_x[2];
float a_y[2];
float a_z[2];

float a_x_correct=0;
float a_y_correct=0;
float a_z_correct=0;
float g_x_correct=0;
float g_y_correct=0;
float g_z_correct=0;

float a_x_buf;
float a_y_buf;
float a_z_buf;
float g_x_buf;
float g_y_buf;
float g_z_buf;


int md_rpm=0;
int md_dir=0;
int rpm=0;

int count_current=0;
float v_ave=0;
float a_ave=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Gyro_Setup(){


	int correct_count=0;

	while(correct_count<FIR){
		if(SD_MPU6050_ReadAll_Mult(&hi2c3,&mpu1)==SD_MPU6050_Result_Ok){
			a_x_buf=mpu1.Accelerometer_X_Mult;
			a_y_buf=mpu1.Accelerometer_Y_Mult;
			a_z_buf=mpu1.Accelerometer_Z_Mult;
			g_x_buf=mpu1.Gyroscope_X_Mult;
			g_y_buf=mpu1.Gyroscope_Y_Mult;
			g_z_buf=mpu1.Gyroscope_Z_Mult;

		}
		a_x_correct+=a_x_buf;
		a_y_correct+=a_y_buf;
		a_z_correct+=a_z_buf;
		g_x_correct+=g_x_buf;
		g_y_correct+=g_y_buf;
		g_z_correct+=g_z_buf;

		LED_TOGGLE(GPIOB,GPIO_PIN_12);
		LED_TOGGLE(GPIOB,GPIO_PIN_13);
		correct_count++;
	}

	a_x_correct/=FIR;
	a_y_correct/=FIR;
	a_z_correct=(a_z_correct/FIR) -1.0;

	g_x_correct/=FIR;
	g_y_correct/=FIR;
	g_z_correct/=FIR;


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == TIM3)
	{
		//LED_TOGGLE(GPIOB,GPIO_PIN_12);
		EncoderGetRPM(&Encoder);
		/*ENCODER*/
		rpm=(Encoder.EncoderRPM+1500)*10;
	}

	if(htim->Instance == TIM4)
	{
		/*GYRO*/
		if(SD_MPU6050_ReadAll_Mult(&hi2c3,&mpu1)==SD_MPU6050_Result_Ok){
			/*
			if(mpu1.Gyroscope_X_Mult < g_x_correct[1] && mpu1.Gyroscope_X_Mult > g_x_correct[0] )
				g_x[0]=0;
			else{

				g_x[1] = mpu1.Gyroscope_X_Mult;
				//g_x[1] = HPF * g_x[1] + ( 1.0 - HPF ) * (g_x[0]-g_x_correct[2]);
				g_x[0] = g_x[1];
*/

			g_x[1] = mpu1.Gyroscope_X_Mult-g_x_correct;
			g_x[1] = Kf * g_x[1] + ( 1.0 - Kf ) * g_x[0];
			g_x[0] = g_x[1];

			g_y[1] = mpu1.Gyroscope_Y_Mult-g_y_correct;
			g_y[1] = Kf * g_y[1] + ( 1.0 - Kf ) * g_y[0];
			g_y[0] = g_y[1];

			g_z[1] = mpu1.Gyroscope_Z_Mult-g_z_correct;
			g_z[1] = Kf * g_z[1] + ( 1.0 - Kf ) * g_z[0];
			g_z[0] = g_z[1];

			a_x[1]=mpu1.Accelerometer_X_Mult-a_x_correct;
			a_x[1] = Kf * a_x[1] + ( 1.0 - Kf ) * a_x[0];
			a_x[0]=a_x[1];

			a_y[1]=mpu1.Accelerometer_Y_Mult-a_y_correct;
			a_y[1] = Kf * a_y[1] + ( 1.0 - Kf ) * a_y[0];
			a_y[0]=a_y[1];

			a_z[1]=mpu1.Accelerometer_Z_Mult-a_z_correct;
			a_z[1] = Kf * a_z[1] + ( 1.0 - Kf ) * a_z[0];
			a_z[0]=a_z[1];

			MadgwickAHRSupdateIMU(&MadgwickFilter,g_x[1],g_y[1],g_z[1],a_x[1],a_y[1],a_z[1]);
			getAngle(&MadgwickFilter);

		}else{
			LED_TOGGLE(GPIOB,GPIO_PIN_12);
		}

		//rpm=Encoder.EncoderRPM*10;
		/*CAN*/
		HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&RxHeader,RxData);
		yaw=(int)((MadgwickFilter.yaw+180)*100);
		int gyro_z=(int)(g_z[1]+100)*100;
		TxData[0]=rpm&255;
		TxData[1]=rpm>>8;
		TxData[2]=yaw &255;
		TxData[3]=yaw >>8;
		TxData[4]=gyro_z&255;
		TxData[5]=gyro_z>>8;
		TxData[6]=(TxData[0]+TxData[1]+TxData[2]+TxData[3]+TxData[4]+TxData[5])&0xFF;
		HAL_CAN_AddTxMessage(&hcan2,&TxHeader,TxData,&TxMailbox);
		/*CAN*/
		//HAL_UART_Transmit(&huart2,TxData,4, 100);
		MD_Start_SMB(&htim1, TIM_CHANNEL_2,md_rpm);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  EncoderGetCount(&Encoder,GPIO_Pin,GPIOA,GPIO_PIN_0,GPIOA,GPIO_PIN_1);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /*GYRO*/
  GYRO_Start(&hi2c3,&mpu1,result);

  /*CAN*/
  //CAN_Start(&hcan1, &TxHeader1,&sFilterConfig1,0x10);
  CAN_Start(&hcan2, &TxHeader,&sFilterConfig,0x11);


  while(RxData[0]!='c'){
  	HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&RxHeader,RxData);
  	//LED_RED(GPIOB,GPIO_PIN_12);
  	LED_RED(GPIOB,GPIO_PIN_13);
  }

  /*GYRO*/
#ifdef CALI_MODE
  HAL_Delay(2000);
  Gyro_Setup();
#else
  	a_x_correct=A_X;
  	a_y_correct=A_Y;
  	a_z_correct=A_Z;

  	g_x_correct=G_X;
  	g_y_correct=G_Y;
  	g_z_correct=G_Z;
#endif
  /*ENCODER*/
  EncoderStart(&Encoder, 48, 20.4, 0,0.1);
  /*TIMER interrupt*/
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);


  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



	  /*
	  GetCurrent(&si8902,&hspi1, GPIOA, GPIO_PIN_4);
	  v_ave+=si8902.CurrentValue;
	  count_current++;
	  if(count_current==100){
		  a_ave=v_ave/count_current;
		  count_current=0;
		  v_ave=0;
	  }
	   */
	  if(RxHeader.StdId==0x10){
		  md_dir=RxData[0];

		  if(md_dir==0)
			  md_rpm=(RxData[1]&255)|(RxData[2]<<8)&65280;
		  else if(md_dir==1)
			  md_rpm=-((RxData[1]&255)|(RxData[2]<<8)&65280);

		  if(RxData[0]=='x'){
			  //yaw_buf=0;
			  MadgwickFilter.q_z=0;
			  LED_RED(GPIOB,GPIO_PIN_13);
			  HAL_Delay(100);
		  }

		  LED_TOGGLE(GPIOB,GPIO_PIN_13);
	  }



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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
