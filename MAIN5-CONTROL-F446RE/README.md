# 設定

# pinout
![be80ccac.png](https://user-images.githubusercontent.com/32444109/74931823-c7bf1a00-5423-11ea-8ae2-348cf3c26598.png)



## SYSTEM CORE
### GPIO
**PA0 & PA1 -> ENCODER(gpio or tim2 encoder mode)**

- [ ] External Interrupt Mode with Riging/Falling edge trigger detection
- [ ] Pull-up

**PC7 & PC8 -> ENCODER**
- [ ] External Interrupt Mode with Riging/Falling edge trigger detection
- [ ] Pull-up 
### NVIC
- [ ] EXTI line 0 interrupts -> Enable
- [ ] EXTI line 1 interrupts -> Enable
- [ ] EXTI line[9:5] interrupts -> Enable
### RCC
- [ ] High speed clock(HSE) -> Crystal/Ceramic Resonator
### SYS
- [ ] Debug -> Serial Wire

## TIMER
### TIM1
**MD**
#### Mode
- [ ] Clock Source -> Internal clock
- [ ] Channel2 -> PWM Generation CH2 CH2N
#### Parameter Settings
##### Counter Period
- [ ] Prescaler = 8
- [ ] Period = 999

### TIM4
**割り込み 0.01s**
#### Mode
- [ ] Internal clock -> Enable 
#### Parameter Settings
##### Counter Period
- [ ] Prescaler = 89
- [ ] Period = 9999
#### NVIC Settings
- [ ] TIM4 global interrupt -> Enable 

## Connectivity
### CAN1
**connect to BLDC MD**
#### Mode
- [ ] Master Mode -> Enable
#### Parameter Settings
##### Bit Timings Parameters
- [ ] Prescaler(for Time Quantum) -> 5
- [ ] Time Quanta in Bit Segment 1 -> 7 Times
- [ ] Time Quanta in Bit Segment 2 -> 1 Time
- [ ] ReSynchronization Jump Windth -> 1 Time

### CAN2
**connect to Wireless controller**
#### Mode
- [ ] Slave Mode -> Enable
#### Parameter Settings
##### Bit Timings Parameters
- [ ] Prescaler(for Time Quantum) -> 5
- [ ] Time Quanta in Bit Segment 1 -> 7 Times
- [ ] Time Quanta in Bit Segment 2 -> 1 Time
- [ ] ReSynchronization Jump Windth -> 1 Time

### I2C3
**GYRO**
#### Mode
- [ ] I2C -> I2C

### SPI1
**connect to Si8902**
### Mode
- [ ] Full-Duplex Master
- [ ] Hardware NSS Signal -> Disable
### Parameter Setting
#### Clock Parameters
- [ ] Prascaler -> 128
- [ ] Clock Polarity -> High
- [ ] Clock Phase -> 2 Edge

## USART2
**connect to PC(Debug)**
### Mode
- [ ] Asynchronous
- [ ] Hardware Flow Control(RS232) -> Disable
### Parameter Settings
- [ ] Baud Rate -> 9600 Bits/s

## USB_OTG_HS
**connect to PC with USB Type-C**
### Mode
- [ ] Internal Fs Phy -> Device_Only

## Middleware
### USE_DEVISE
- [ ] Class For HS IP -> Communication Device Class (Virtual Port Com) 

## clock configuration
![5b331f61.png](https://user-images.githubusercontent.com/32444109/74931974-21274900-5424-11ea-8e98-768ef812cf35.png)