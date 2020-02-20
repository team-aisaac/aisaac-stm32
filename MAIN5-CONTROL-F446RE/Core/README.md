# LED
GPIO_TypeDef* GPIOx->ピン名 ex)GPIOA

uint16_t GPIO_Pin->ピン番号 ex)GPIO_PIN_0

```c
void LED_RED(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
void LED_WHITE(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
void LED_TOGGLE(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
```

# ENCODER
ENCODER* DataStruct -> 構造体

float Resolution -> エンコーダ分解能

float GearRatio -> ギア比

float Diameter -> タイヤ直径

float dt -> 割り込み時間

uint16_t GPIO_Pin -> 割り込みが発生したピン

GPIO_TypeDef* GPIOx_A -> A相ピン名

uint16_t GPIO_Pin_A -> A相ピン番号

GPIO_TypeDef* GPIOx_B -> B相ピン名

uint16_t GPIO_Pin_B -> B相ピン番号

## 構造体
```c
int32_t EncoderResolution; //エンコーダ分解能
float GearRatio; //ギア比
float Diametar; //タイヤ直径
float Samplingrate;
int32_t EncoderCount; //カウント
int32_t EncoderCount_old; //カウント(速度計算用)
float EncoderVelocity;//速度
float EncoderAnglerVelocity;//角速度
float EncoderRPM;//回転数
float EncoderDistance;//移動距離
float CalVelocity_rec;
```

## 関数
```c
void EncoderStart(ENCODER* DataStruct,float Resolution,float GearRatio,float Diameter,float dt);
void EncoderGetCount(ENCODER* DataStruct,uint16_t GPIO_Pin,GPIO_TypeDef* GPIOx_A,uint16_t GPIO_Pin_A,GPIO_TypeDef* GPIOx_B,uint16_t GPIO_Pin_B);
void EncoderGetAnglerVelocity(ENCODER* DataStruct);
void EncoderGetRPM(ENCODER* DataStruct);
```

# GYRO
## 関数
```c
void GYRO_Start(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct,SD_MPU6050_Result result);
```

# CAN
## 関数
```c
void CAN_Start(CAN_HandleTypeDef* canHandle,CAN_TxHeaderTypeDef* TxHeader,CAN_FilterTypeDef* sFilterConfig,uint8_t stdid);
```

# SPI
## 構造体
```c
float VoltageValue;//電圧(取得値)
float CurrentValue;//電流(計算値)
```

## 関数
```c
void GetCurrent(Si8902 *DataStruct,SPI_HandleTypeDef *hspi,GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);
```
