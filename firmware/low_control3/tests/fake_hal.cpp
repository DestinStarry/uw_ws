#include "fake_hal.h"
#include "Buzzer.h"
#include <stdexcept>
#include <algorithm>
TIM_TypeDef fake_tim1,fake_tim8;
TIM_HandleTypeDef htim1{TIM1},htim8{TIM8};
UART_HandleTypeDef huart6;
SPI_HandleTypeDef hspi2;
ADC_HandleTypeDef hadc1,hadc3;
volatile bool system_init_flag=true;
Buzzer g_buzzer;
void Buzzer::Init(){}void Buzzer::Handle(){}void Buzzer::Receive(uint8_t *,uint16_t){}
void Buzzer::RequestBeep(uint8_t){}
namespace fake {
uint32_t tick=0,irqmask=0,spi_calls=0,tx_calls=0,tx_aborts=0;
bool spi_fail=false,adc_fail=false,tx_fail=false;
int spi_stuck=-1,corrupt_dll=-1,fifo_disabled=-1;
uint32_t spi_fail_until=0,clock_writes=0;
uint8_t divisors[4][2]={};
uint32_t ref_raw=1500,battery_raw=1875;
std::string tx;
std::vector<uint8_t> fifo[4];
uint8_t regs[128]={},address=0,fifo_channel=0;
bool first=true,write=false;
void Reset(){
 tick=irqmask=spi_calls=tx_calls=tx_aborts=0;spi_fail=adc_fail=tx_fail=false;
 spi_stuck=corrupt_dll=fifo_disabled=-1;spi_fail_until=clock_writes=0;
 for(auto &d:divisors)d[0]=d[1]=0;
 tx.clear();for(auto &q:fifo)q.clear();std::fill(regs,regs+128,0);
 first=true;ref_raw=1500;battery_raw=1875;
}
}
uint32_t HAL_GetTick(){return fake::tick;}
void HAL_Delay(uint32_t ms){fake::tick+=ms;}
uint32_t __get_PRIMASK(){return fake::irqmask;}
void __disable_irq(){fake::irqmask=1;}void __set_PRIMASK(uint32_t p){fake::irqmask=p;}void __DMB(){}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *tim,uint32_t ch){
 const uint32_t values[]={tim->Instance->CCR1,tim->Instance->CCR2,tim->Instance->CCR3,tim->Instance->CCR4};
 if(values[ch]!=1500)throw std::runtime_error("PWM enabled before neutral");
 return HAL_OK;
}
void __HAL_TIM_SetCompare(TIM_HandleTypeDef *t,uint32_t ch,uint32_t value){
 switch(ch){case 0:t->Instance->CCR1=value;break;case 1:t->Instance->CCR2=value;break;
 case 2:t->Instance->CCR3=value;break;case 3:t->Instance->CCR4=value;break;}
}
void HAL_GPIO_Init(int,GPIO_InitTypeDef *){}
void HAL_GPIO_WritePin(int,uint32_t pin,int value){if(pin==12 && value==GPIO_PIN_RESET)fake::first=true;}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *,uint8_t *tx,uint8_t *rx,uint16_t,uint32_t timeout){
 ++fake::spi_calls;
 if(timeout>2)throw std::runtime_error("unbounded SPI timeout");
 if(fake::spi_fail || fake::tick<fake::spi_fail_until){fake::tick+=timeout;return HAL_TIMEOUT;}
 *rx=0;
 if(fake::first){fake::address=*tx&0x7f;fake::write=(*tx&0x80)!=0;fake::first=false;return HAL_OK;}
 const unsigned uart=(fake::address>>4)&3, offset=fake::address&15;
 const bool banked=fake::address<0x40 && offset<2 && (fake::regs[uart*16+3]&0x80);
 if(fake::write){
  if(banked)fake::divisors[uart][offset]=*tx;
  else fake::regs[fake::address]=*tx;
  if(fake::address==0x42)fake::fifo_channel=*tx&3;
  if(fake::address==0x48)++fake::clock_writes;
 }
 else if(fake::spi_stuck>=0)*rx=uint8_t(fake::spi_stuck);
 else if(banked){*rx=fake::divisors[uart][offset];if(offset==0 && int(uart)==fake::corrupt_dll)*rx^=1;}
 else if(fake::address<0x40 && offset==2)*rx=((fake::regs[fake::address]&1) && int(uart)!=fake::fifo_disabled)?0xC2:0x01;
 else if(fake::address==0x43)*rx=uint8_t(fake::fifo[fake::fifo_channel].size());
 else if(fake::address==0x44)*rx=uint8_t(fake::fifo[fake::fifo_channel].size()>>8);
 else if((fake::address&15)==0 && !(fake::regs[(fake::address&0x30)+3]&0x80)){
  auto &q=fake::fifo[(fake::address>>4)&3];if(!q.empty()){*rx=q.front();q.erase(q.begin());}
 }else *rx=fake::regs[fake::address];
 return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *a,ADC_ChannelConfTypeDef *c){
 if(c->SamplingTime!=480)throw std::runtime_error("ADC sample too short");
 a->channel=c->Channel;return fake::adc_fail?HAL_ERROR:HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *){return HAL_OK;}
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *,uint32_t timeout){
 if(timeout>2)throw std::runtime_error("unbounded ADC timeout");
 return HAL_OK;
}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *a){return a==&hadc1?fake::ref_raw:fake::battery_raw;}
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *){return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *,uint8_t *data,uint16_t len){
 ++fake::tx_calls;if(fake::tx_fail)return HAL_BUSY;
 fake::tx.assign(reinterpret_cast<char *>(data),len);return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *){++fake::tx_aborts;return HAL_OK;}
void Error_Handler(){throw std::runtime_error("firmware Error_Handler");}
