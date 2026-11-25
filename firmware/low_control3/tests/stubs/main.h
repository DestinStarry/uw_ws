#pragma once
#include <cstdint>
#include <cstddef>
enum HAL_StatusTypeDef { HAL_OK=0,HAL_ERROR=1,HAL_BUSY=2,HAL_TIMEOUT=3 };
struct TIM_TypeDef { uint32_t CCR1=0,CCR2=0,CCR3=0,CCR4=0; };
struct TIM_HandleTypeDef { TIM_TypeDef *Instance; };
struct UART_HandleTypeDef {};
struct SPI_HandleTypeDef {};
struct ADC_HandleTypeDef { uint32_t channel=0; };
struct ADC_ChannelConfTypeDef { uint32_t Channel=0,Rank=0,SamplingTime=0; };
struct GPIO_InitTypeDef { uint32_t Pin=0,Mode=0,Pull=0,Speed=0; };
extern TIM_TypeDef fake_tim1,fake_tim8;
#define TIM1 (&fake_tim1)
#define TIM8 (&fake_tim8)
#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 1
#define TIM_CHANNEL_3 2
#define TIM_CHANNEL_4 3
#define GPIO_PIN_12 12
#define GPIO_PIN_0 0
#define GPIOB 2
#define GPIOA 1
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_MODE_INPUT 0
#define GPIO_PULLUP 1
#define GPIO_SPEED_FREQ_MEDIUM 1
#define GPIO_SPEED_FREQ_LOW 0
#define ADC_CHANNEL_VREFINT 17
#define ADC_CHANNEL_8 8
#define ADC_SAMPLETIME_480CYCLES 480
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __NOP() ((void)0)
void __HAL_TIM_SetCompare(TIM_HandleTypeDef *,uint32_t,uint32_t);
uint32_t HAL_GetTick();
void HAL_Delay(uint32_t);
uint32_t __get_PRIMASK();
void __disable_irq();
void __set_PRIMASK(uint32_t);
void __DMB();
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *,uint32_t);
void HAL_GPIO_WritePin(int,uint32_t,int);
void HAL_GPIO_Init(int,GPIO_InitTypeDef *);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *,uint8_t *,uint8_t *,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *,ADC_ChannelConfTypeDef *);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *,uint32_t);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
void Error_Handler();
