/*
 * command.h
 *
 *  Created on: 2019/12/31
 *  Last update : 2020/02/04
 *      Author: keita
 */

#ifndef COMMAND_H_
#define COMMAND_H_

#include "main.h"
#ifdef HAL_I2C_MODULE_ENABLED
/* CODE BEGIN GYRO */
#include "sd_hal_mpu6050.h"
/* CODE END GYRO */
#endif
#include <stdbool.h>

#define M_PI 3.141592654

/* CODE BEGIN GPIO */
#ifdef HAL_GPIO_MODULE_ENABLED
/* CODE BEGIN LED */
void LED_RED(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
void LED_WHITE(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
void LED_TOGGLE(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
/* CODE END LED*/

/* CODE BEGIN ENCODER*/
typedef struct  {
	int32_t EncoderResolution;
	float GearRatio;
	float Diametar;
	float Samplingrate;

	int32_t EncoderCount;
	int32_t EncoderCount_old;
	float EncoderVelocity;
	float EncoderAnglerVelocity;
	float EncoderRPM;
	float EncoderDistance;

	float CalVelocity_rec;

} ENCODER;

void EncoderStart(ENCODER* DataStruct,float,float,float,float);
void EncoderGetCount(ENCODER* DataStruct,uint16_t GPIO_Pin,GPIO_TypeDef* GPIOx_A,uint16_t GPIO_Pin_A,GPIO_TypeDef* GPIOx_B,uint16_t GPIO_Pin_B);
void EncoderGetAnglerVelocity(ENCODER* DataStruct);
void EncoderGetRPM(ENCODER* DataStruct);
/* CODE END ENCODER*/
#endif
/* CODE END GPIO */

/* CODE BEGIN I2C */
#ifdef HAL_I2C_MODULE_ENABLED
/* CODE BEGIN GYRO */
void GYRO_Start(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct,SD_MPU6050_Result result);
/* CODE END GYRO */
#endif
/* CODE END I2C */

/* CODE BEGIN CAN */
#ifdef HAL_CAN_MODULE_ENABLED
void CAN_Start(CAN_HandleTypeDef* canHandle,CAN_TxHeaderTypeDef* TxHeader,CAN_FilterTypeDef* sFilterConfig,uint8_t stdid);
#endif
/* CODE END CAN */

/* CODE BEGIN TIM */
#ifdef HAL_TIM_MODULE_ENABLED
/* CODE BEGIN MD */
void MD_Start_SMB(TIM_HandleTypeDef *htim, uint32_t Channel,int32_t value);
void MD_Start_LAP(TIM_HandleTypeDef *htim, uint32_t Channel,uint32_t value);
void MD_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
/* CODE END MD */
#endif
/* CODE END TIM */

/* CODE BEGIN SPI */
#ifdef HAL_SPI_MODULE_ENABLED
typedef struct  {

	float VoltageValue;
	float CurrentValue;

} Si8902;

void GetCurrent(Si8902 *DataStruct,SPI_HandleTypeDef *hspi,GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
#endif
/* CODE END SPI */

#endif /* COMMAND_H_ */
