#include "fake_hal.h"
#include "CommUART1.h"
#include "MotionController.h"
#include "Motor.h"
#include "Servo.h"
#include "VoltageMonitor.h"
#include "TelemetryService.h"
#include "CH9434.h"
#include "RpmDecoder.h"
#include "Safety.h"
#include <iostream>
#include <stdexcept>
#include <cmath>
static void require(bool v,const char *why){if(!v)throw std::runtime_error(why);}
static const char *zero="CTRL:0,0,0,0,0,0,-0.5\n";
static const char *move="CTRL:0.25,0,0,0,1,-1,0.5\n";
static void rx(const char *s){while(*s)g_comm.RxByteISR(uint8_t(*s++));}
static void step(uint32_t ms){fake::tick+=ms;Safety_Tick(fake::tick);}
static void neutral(){require(TIM1->CCR1==1500&&TIM1->CCR2==1500&&TIM1->CCR3==1500&&TIM1->CCR4==1500,"motors neutral");}
static void handshake(){rx(zero);g_comm.Handle();rx(zero);g_comm.Handle();require(g_comm.Active(),"neutral handshake");}
static void rpm_frame(uint8_t idx,uint16_t erpm){
 uint8_t p[10]={25,1,2,3,4,5,6,uint8_t(erpm>>8),uint8_t(erpm),0};
 p[9]=low::RpmDecoder::Crc(p,9);fake::fifo[idx].insert(fake::fifo[idx].end(),p,p+10);
}
int main(){try{
 fake::Reset();g_motor.Init();g_servo.Init();Safety_Init();g_comm.Init();neutral();
 rx(move);g_comm.Handle();neutral();require(g_comm.NeedsNeutral(),"boot refuses nonzero thrust");
 handshake();rx(move);g_comm.Handle();
 require(TIM1->CCR1==1400,"direction inversion retained");
 require(TIM8->CCR1==2000&&TIM8->CCR2==1000&&TIM8->CCR3==2000,"servo/gripper mapping retained");
 step(201);neutral();require(Safety_PendingFault()&SAFETY_TIMEOUT,"IRQ deadman even without main loop");
 g_comm.Handle();require(g_comm.NeedsNeutral(),"latched timeout");rx(move);g_comm.Handle();neutral();
 handshake();rx(move);rx("OFF\n");rx(move);g_comm.Handle();neutral();require(!g_comm.Active(),"OFF wins buffered CTRL");
 handshake();step(150);rx("BUZZ\n");g_comm.Handle();step(51);neutral();g_comm.Handle();
 handshake();rx("CTRL:nan,0,0,0,0,0,0\n");g_comm.Handle();neutral();
 handshake();rx("1 1900\n");g_comm.Handle();neutral();require(!g_comm.Active(),"raw PWM bypass disabled");
 handshake();rx(move);g_comm.Handle();g_comm.RxErrorISR();neutral();g_comm.Handle();
 handshake();rx(move);step(101);g_comm.Handle();neutral();require(!g_comm.Active(),"aged queued CTRL cannot rearm");
 handshake();rx(move);g_comm.Handle();
 for(unsigned i=0;i<140;++i) { g_comm.RxByteISR('x'); }
 neutral();g_comm.Handle();
 require(!g_comm.Active(),"overflow fault stops");
 g_voltage.Init();g_voltage.Handle();require(g_voltage.Valid(fake::tick),"ADC valid after successful sample");
 require(std::fabs(g_voltage.voltage-15.13636f)<.01f,"voltage scaling");
 fake::adc_fail=true;step(100);g_voltage.Handle();require(!g_voltage.Valid(fake::tick),"ADC failure invalidates");
 fake::adc_fail=false;step(100);g_voltage.Handle();require(g_voltage.Valid(fake::tick),"ADC recovers");
 fake::ref_raw=0;step(100);g_voltage.Handle();require(!g_voltage.Valid(fake::tick),"zero reference invalidates");
 fake::ref_raw=1500;step(100);g_voltage.Handle();
 CH9434::ch9434.Init();require(CH9434::ch9434.Ready(),"CH9434 SPI write/read self-check");
 for(unsigned ch=0;ch<4;++ch){rpm_frame(ch,uint16_t(700+ch*70));rpm_frame(ch,uint16_t(700+ch*70));}
 for(unsigned i=0;i<4;++i)CH9434::ch9434.Handle();
 for(unsigned ch=0;ch<4;++ch){require(CH9434::ch9434.RpmValid(ch,fake::tick),"all four RPM channels");require(CH9434::ch9434.GetRPM(ch)==10000+1000*int(ch),"RPM scaling");}
 g_telemetry.TxCompleteISR();step(100);g_telemetry.Handle();
 require(fake::tx.find("CH0:10000 ")!=std::string::npos,"valid RPM reaches wire");
 g_telemetry.TxCompleteISR();
 fake::spi_fail=true;uint32_t started=fake::tick,calls=fake::spi_calls;
 CH9434::ch9434.Handle();require(fake::tick-started<=1&&fake::spi_calls-calls<=1,"fail fast SPI");
 require(!CH9434::ch9434.RpmValid(0,fake::tick),"SPI fault invalidates stale RPM");
 fake::spi_fail=false;step(1000);CH9434::ch9434.Handle();require(CH9434::ch9434.Ready(),"SPI device reinitializes");
 g_telemetry.TxCompleteISR();step(100);fake::tx_fail=true;g_telemetry.Handle();uint32_t failed=fake::tx_calls;
 fake::tx_fail=false;g_telemetry.Handle();require(fake::tx_calls==failed+1,"failed transmit does not stick busy");
 require(fake::tx.find("CH0:-- ")!=std::string::npos,"invalid RPM reaches wire as --");
 step(101);g_telemetry.Handle();require(fake::tx_aborts==1,"lost TX callback recovers");
 g_telemetry.TxCompleteISR();
 TelemetryService compact;
 fake::tick=100;compact.Handle();
 require(fake::tx.rfind("VOLT:",0)==0&&fake::tx.find("DIAG")==std::string::npos,"normal frames have no blank or diagnostics");
 compact.TxCompleteISR();fake::tick=1000;compact.Handle();
 require(fake::tx.find("\r\nDIAG ")!=std::string::npos,"diagnostics once due");
 compact.TxCompleteISR();fake::tick=1100;compact.Handle();
 require(fake::tx.find("DIAG")==std::string::npos,"diagnostics not repeated at 10Hz");
 fake::tick=1201;compact.Handle();
 require(fake::tx.rfind("\r\nVOLT:",0)==0,"TX abort delimits partial frame");
 compact.TxCompleteISR();fake::tick=1301;compact.Handle();
 require(fake::tx.rfind("VOLT:",0)==0,"resync prefix occurs only once");
 compact.TxCompleteISR();
 std::cout<<"firmware fault injection passed\n";
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
