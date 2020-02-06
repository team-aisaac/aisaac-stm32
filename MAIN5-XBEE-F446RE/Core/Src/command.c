/*
 * command.c
 *
 *  Created on: 2019/12/31
 *  Last update : 2020/02/04
 *      Author: keita
 */

#include "command.h"

/* CODE BEGIN GPIO */
#ifdef HAL_GPIO_MODULE_ENABLED
/* CODE BEGIN LED */
void LED_RED(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin){
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}
void LED_WHITE(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin){
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}
void LED_TOGGLE(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin){
	HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
}
/* CODE END LED */

/* CODE BEGIN ENCODER */

void EncoderStart(ENCODER* DataStruct,float Resolution,float GearRatio,float Diameter,float dt){

	DataStruct->EncoderResolution=Resolution;
	DataStruct->GearRatio=GearRatio;
	DataStruct->Diametar=Diameter;
	DataStruct->Samplingrate=1/dt;
	DataStruct->CalVelocity_rec=1/(Resolution*GearRatio*dt);
	DataStruct->EncoderCount=0;
	DataStruct->EncoderCount_old=0;

}
void EncoderGetCount(ENCODER* DataStruct,uint16_t GPIO_Pin,GPIO_TypeDef* GPIOx_A,uint16_t GPIO_Pin_A,GPIO_TypeDef* GPIOx_B,uint16_t GPIO_Pin_B){
	bool en_stateA=0,en_stateB=0;

	if(GPIO_Pin==GPIO_Pin_A) {
		en_stateA=HAL_GPIO_ReadPin(GPIOx_A, GPIO_Pin_A);
		en_stateB=HAL_GPIO_ReadPin(GPIOx_B, GPIO_Pin_B);
		if(en_stateA==GPIO_PIN_SET && en_stateB==GPIO_PIN_RESET) DataStruct->EncoderCount++;
		else if(en_stateA==GPIO_PIN_RESET && en_stateB==GPIO_PIN_SET) DataStruct->EncoderCount++;
		else if(en_stateA==GPIO_PIN_SET && en_stateB==GPIO_PIN_SET) DataStruct->EncoderCount--;
		else if(en_stateA==GPIO_PIN_RESET && en_stateB==GPIO_PIN_RESET) DataStruct->EncoderCount--;
	}else if(GPIO_Pin==GPIO_Pin_B) {
		en_stateA=HAL_GPIO_ReadPin(GPIOx_A, GPIO_Pin_A);
		en_stateB=HAL_GPIO_ReadPin(GPIOx_B, GPIO_Pin_B);
		if(en_stateB==GPIO_PIN_SET && en_stateA==GPIO_PIN_SET) DataStruct->EncoderCount++;
		else if(en_stateB==GPIO_PIN_RESET && en_stateA==GPIO_PIN_RESET) DataStruct->EncoderCount++;
		else if(en_stateB==GPIO_PIN_SET && en_stateA==GPIO_PIN_RESET) DataStruct->EncoderCount--;
		else if(en_stateB==GPIO_PIN_RESET && en_stateA==GPIO_PIN_SET) DataStruct->EncoderCount--;
	}
}

void EncoderGetAnglerVelocity(ENCODER* DataStruct){
	//static int32_t EncoderCount_old=0;
	//DataStruct->EncoderAnglerVelocity=(DataStruct->EncoderCount-DataStruct->EncoderCount_old)/(DataStruct->EncoderResolution*DataStruct->GearRatio)*2*M_PI/dt;
	DataStruct->EncoderAnglerVelocity=2*M_PI*(DataStruct->EncoderCount-DataStruct->EncoderCount_old)*DataStruct->CalVelocity_rec;
	DataStruct->EncoderCount_old=DataStruct->EncoderCount;
}

void EncoderGetRPM(ENCODER* DataStruct){
	//static int32_t EncoderCount_old=0;
	//DataStruct->EncoderRPM=(DataStruct->EncoderCount-DataStruct->EncoderCount_old)/(dt*DataStruct->EncoderResolution*DataStruct->GearRatio)*60;
	DataStruct->EncoderRPM=(DataStruct->EncoderCount-DataStruct->EncoderCount_old)*DataStruct->CalVelocity_rec*60;
	DataStruct->EncoderCount_old=DataStruct->EncoderCount;
}
/* CODE BEGIN ENCODER */
#endif
/* CODE END GPIO */

/* CODE BEGIN I2C */
#ifdef HAL_I2C_MODULE_ENABLED
/* CODE BEGIN GYRO */
void GYRO_Start(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct,SD_MPU6050_Result result){

	result = SD_MPU6050_Init(I2Cx,DataStruct,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_250s );
	while(result != SD_MPU6050_Result_Ok){
	  result = SD_MPU6050_Init(I2Cx,DataStruct,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_250s );
	  LED_TOGGLE(GPIOB,GPIO_PIN_12);
	  HAL_Delay(500);
	}
	LED_RED(GPIOB,GPIO_PIN_12);
}
/* CODE END GYRO */
#endif
/* CODE END I2C */

/* CODE BEGIN CAN */
#ifdef HAL_CAN_MODULE_ENABLED
void CAN_Start(CAN_HandleTypeDef* canHandle,CAN_TxHeaderTypeDef* TxHeader,CAN_FilterTypeDef* sFilterConfig,uint8_t stdid){
	TxHeader->DLC=8;
	TxHeader->StdId=stdid;
	TxHeader->ExtId=0x01;
	TxHeader->IDE=CAN_ID_STD;
	TxHeader->RTR=CAN_RTR_DATA;
	TxHeader->TransmitGlobalTime=DISABLE;

	sFilterConfig->FilterBank = 0;
	sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig->FilterIdHigh = 0x0000;
	sFilterConfig->FilterIdLow = 0x0000;
	sFilterConfig->FilterMaskIdHigh = 0x0000;
	sFilterConfig->FilterMaskIdLow = 0x0000;
	sFilterConfig->FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig->FilterActivation = ENABLE;
	sFilterConfig->SlaveStartFilterBank = 0;

	if (HAL_CAN_ConfigFilter(canHandle,sFilterConfig)!=HAL_OK) {
	  Error_Handler();
	    LED_TOGGLE(GPIOB,GPIO_PIN_13);
	}

	if(HAL_CAN_Start(canHandle)!=HAL_OK) {
	  Error_Handler();
	    LED_TOGGLE(GPIOB,GPIO_PIN_13);
	}

	if(HAL_CAN_ActivateNotification(canHandle, CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK) {
	  Error_Handler();
	    LED_TOGGLE(GPIOB,GPIO_PIN_13);
	}
}
#endif
/* CODE END CAN */

/* CODE BEGIN TIM */
#ifdef HAL_TIM_MODULE_ENABLED
/* CODE BEGIN MD */
void MD_Start_SMB(TIM_HandleTypeDef *htim, uint32_t Channel,int32_t value)
{
	uint8_t dir;
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   if(value>=0){
	   sConfigOC.Pulse = value;
	   dir=1;
   }else{
	   sConfigOC.Pulse = -value;
	   dir=0;
   }
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, Channel);
   if(dir==1){
	   HAL_TIM_PWM_Start(htim, Channel);
	   HAL_TIMEx_PWMN_Stop(htim, Channel);
   }else if(dir==0){
	   HAL_TIMEx_PWMN_Start(htim, Channel);
	   HAL_TIM_PWM_Stop(htim, Channel);
   }
}
void MD_Start_LAP(TIM_HandleTypeDef *htim, uint32_t Channel,uint32_t value)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, Channel);

   HAL_TIM_PWM_Start(htim, Channel);
   HAL_TIMEx_PWMN_Start(htim, Channel);

}
void MD_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
   TIM_OC_InitTypeDef sConfigOC;
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = 0;
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
   HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, Channel);

   HAL_TIMEx_PWMN_Stop(htim, Channel);
   HAL_TIM_PWM_Stop(htim, Channel);

}
/* CODE BEGIN MD */
#endif
/* CODE END TIM */

/* CODE BEGIN SPI */
#ifdef HAL_SPI_MODULE_ENABLED
void GetCurrent(Si8902 *DataStruct,SPI_HandleTypeDef *hspi,GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)
{
	const float VoltageResolution=3.3/1024;
	const float Threshold=3.297;
	uint8_t TxBuffer[2] ={203,0xFF};
	uint8_t RxBuffer[3]={0,0,0};
	uint32_t Current_buf=0;


	HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi,&TxBuffer[0],1,2000);
	HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi,&TxBuffer[1],&RxBuffer[0],1,2000);
	HAL_SPI_TransmitReceive(hspi,&TxBuffer[1],&RxBuffer[1],1,2000);
	HAL_SPI_TransmitReceive(hspi,&TxBuffer[1],&RxBuffer[2],1,2000);
	HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);

	Current_buf = (( RxBuffer[1] & 0x0F) << 6) | ((RxBuffer[2] & 0x7E) >> 1);
	DataStruct->VoltageValue=Current_buf*VoltageResolution;
	DataStruct->CurrentValue=(Threshold-DataStruct->VoltageValue)*5;//2m Ohm /0.2 -> *5

}
#endif
/* CODE END SPI */

