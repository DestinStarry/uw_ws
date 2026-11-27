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
#include <cstdio>
static void require(bool v,const char *why){if(!v)throw std::runtime_error(why);}
static const char *zero="CTRL:0,0,0,0,0,0,-0.5\n";
static const char *move="CTRL:0.25,0,0,0,1,-1,0.5\n";
static void rx(const char *s){while(*s)g_comm.RxByteISR(uint8_t(*s++));}
static void step(uint32_t ms){fake::tick+=ms;Safety_Tick(fake::tick);}
static void neutral(){require(TIM1->CCR1==1500&&TIM1->CCR2==1500&&TIM1->CCR3==1500&&TIM1->CCR4==1500,"motors neutral");}
static void control_ready(){rx(zero);g_comm.Handle();rx(zero);g_comm.Handle();require(g_comm.Active(),"valid CTRL accepted");}
static void rpm_frame(uint8_t idx,uint16_t erpm){
 uint8_t p[10]={25,1,2,3,4,5,6,uint8_t(erpm>>8),uint8_t(erpm),0};
 p[9]=low::RpmDecoder::Crc(p,9);fake::fifo[idx].insert(fake::fifo[idx].end(),p,p+10);
}
int main(){try{
 fake::Reset();g_motor.Init();g_servo.Init();Safety_Init();g_comm.Init();neutral();
 rx(move);g_comm.Handle();require(g_comm.Active() && TIM1->CCR1==1400,"first valid CTRL executes directly");
 control_ready();rx(move);g_comm.Handle();
 require(TIM1->CCR1==1400,"direction inversion retained");
 require(TIM8->CCR1==uint32_t(1500+int32_t(90.f*(500.f/90.f)))&&TIM8->CCR2==uint32_t(1500+int32_t(-90.f*(500.f/90.f)))&&TIM8->CCR3==1750,"steering mapping retained; channel7 is camera");
 step(201);neutral();require(Safety_PendingFault()&SAFETY_TIMEOUT,"IRQ deadman even without main loop");
 g_comm.Handle();require(!g_comm.Active(),"timeout stops output");rx(move);g_comm.Handle();rx(move);g_comm.Handle();require(g_comm.Active(),"valid CTRL recovers without handshake");
 control_ready();rx(move);rx("OFF\n");rx(move);g_comm.Handle();neutral();require(!g_comm.Active(),"OFF wins buffered CTRL");
 control_ready();step(150);rx("BUZZ\n");g_comm.Handle();step(51);neutral();g_comm.Handle();
 control_ready();rx("CTRL:nan,0,0,0,0,0,0\n");g_comm.Handle();neutral();
 control_ready();rx("1 1900\n");g_comm.Handle();require(TIM1->CCR1==1900,"raw PWM bypasses inversion like low2");step(201);g_comm.Handle();require(!g_comm.Active(),"raw PWM does not feed deadman");
 control_ready();rx(move);g_comm.Handle();g_comm.RxErrorISR();g_comm.Handle();
 require(TIM1->CCR1==1400,"isolated UART error holds last valid output within timeout");
 step(201);neutral();g_comm.Handle();
 control_ready();rx(move);step(101);g_comm.Handle();neutral();
 step(100);g_comm.Handle();require(!g_comm.Active(),"aged CTRL does not feed deadman");
 control_ready();rx(move);g_comm.Handle();
 for(unsigned i=0;i<140;++i) { g_comm.RxByteISR('x'); }
 g_comm.Handle();require(TIM1->CCR1==1400,"bad long line cannot change last valid output");
 step(201);neutral();g_comm.Handle();require(!g_comm.Active(),"bad lines do not feed deadman");
 require(g_comm.RxTooLong()>0,"long line classified");
 control_ready();rx(move);g_comm.Handle();g_comm.RxErrorISR(8);g_comm.Handle();
 require(g_comm.RxUartFlags()==8,"UART ORE captured");
 rx(move);g_comm.Handle();rx(move);g_comm.Handle();require(g_comm.Active(),"fresh CTRL resumes after UART recovery");
 control_ready();rx(move);g_comm.Handle();
 for(unsigned i=0;i<1100;++i)g_comm.RxByteISR('x');
 g_comm.Handle();require(g_comm.RxByteOverflow()>0,"byte queue overflow classified");
 step(201);neutral();g_comm.Handle();require(!g_comm.Active(),"overflow cannot extend deadman");
 g_voltage.Init();g_voltage.Handle();require(g_voltage.Valid(fake::tick),"ADC valid after successful sample");
 require(std::fabs(g_voltage.voltage-15.13636f)<.01f,"voltage scaling");
 fake::adc_fail=true;step(100);g_voltage.Handle();require(!g_voltage.Valid(fake::tick),"ADC failure invalidates");
 fake::adc_fail=false;step(100);g_voltage.Handle();require(g_voltage.Valid(fake::tick),"ADC recovers");
 fake::ref_raw=0;step(100);g_voltage.Handle();require(!g_voltage.Valid(fake::tick),"zero reference invalidates");
 fake::ref_raw=1500;step(100);g_voltage.Handle();
 rx("\n");g_comm.Handle();control_ready();
 CH9434::ch9434.Init();require(CH9434::ch9434.Ready(),"CH9434 SPI write/read self-check");
 for(unsigned ch=0;ch<4;++ch){rpm_frame(ch,uint16_t(700+ch*70));rpm_frame(ch,uint16_t(700+ch*70));}
 for(unsigned i=0;i<4;++i)CH9434::ch9434.Handle();
 for(unsigned ch=0;ch<4;++ch){require(CH9434::ch9434.RpmValid(ch,fake::tick),"all four RPM channels");require(CH9434::ch9434.GetRPM(ch)==10000+1000*int(ch),"RPM scaling");}
 g_telemetry.TxCompleteISR();step(100);g_telemetry.Handle();
 require(fake::tx.find("CH0:10000 ")!=std::string::npos,"valid RPM reaches wire");
 g_telemetry.TxCompleteISR();
 control_ready();rx(move);g_comm.Handle();
 fake::spi_fail=true;uint32_t started=fake::tick,calls=fake::spi_calls;
 CH9434::ch9434.Handle();require(fake::tick-started<=2&&fake::spi_calls-calls<=1,"fail fast SPI");
 require(!CH9434::ch9434.RpmValid(0,fake::tick),"SPI fault invalidates stale RPM");
 rx(move);g_comm.Handle();require(g_comm.Active() && TIM1->CCR1==1400,"SPI/RPM fault never gates valid actuator control");
 fake::spi_fail=false;
 const uint32_t no_retry=fake::tick;CH9434::ch9434.Handle();
 require(fake::tick==no_retry,"no blocking SPI retry while active");
 step(1000);g_comm.Handle();const uint32_t clocks=fake::clock_writes;
 CH9434::ch9434.Handle();require(CH9434::ch9434.Ready(),"SPI polling recovers without runtime reinitialization");
 require(fake::clock_writes==clocks,"no blocking UART reconfiguration at runtime");
 g_telemetry.TxCompleteISR();step(100);fake::tx_fail=true;g_telemetry.Handle();uint32_t failed=fake::tx_calls;
 fake::tx_fail=false;g_telemetry.Handle();require(fake::tx_calls==failed+1,"failed transmit does not stick busy");
 require(fake::tx.find("CH0:-- ")!=std::string::npos,"invalid RPM reaches wire as --");
 step(101);g_telemetry.Handle();require(fake::tx_aborts==1,"lost TX callback recovers");
 g_telemetry.TxCompleteISR();
 TelemetryService compact;
 fake::tick=100;compact.Handle();
 require(fake::tx.rfind("VOLT:",0)==0&&fake::tx.find("DIAG")==std::string::npos,"normal frames have no blank or diagnostics");
 compact.TxCompleteISR();fake::tick=1000;compact.Handle();
 require(fake::tx.find("DIAG")==std::string::npos && fake::tx.find("RXSTAT")==std::string::npos,"diagnostics disabled by default");
 compact.TxCompleteISR();fake::tick=1100;compact.Handle();
 require(fake::tx.find("DIAG")==std::string::npos,"diagnostics not repeated at 10Hz");
 require(fake::tx.find("SAFETY")==std::string::npos,"no MCU heartbeat protocol");
 fake::tick=1201;compact.Handle();
 require(fake::tx.rfind("\r\nVOLT:",0)==0,"TX abort delimits partial frame");
 compact.TxCompleteISR();fake::tick=1301;compact.Handle();
 require(fake::tx.rfind("VOLT:",0)==0,"resync prefix occurs only once");
 compact.TxCompleteISR();
 // Legacy low2 bench behavior: OFF first; raw commands latch, CTRL alone feeds timeout.
 rx("OFF\n");g_comm.Handle();rx("1 1600\n");g_comm.Handle();
 require(TIM1->CCR1==1600,"first complete command immediately after OFF is not swallowed");
 rx("5 1600\n");g_comm.Handle();
 require(TIM1->CCR1==1600 && TIM8->CCR1==1600,"numeric motor and servo commands execute");
 step(500);g_comm.Handle();require(TIM1->CCR1==1600 && TIM8->CCR1==1600,"standalone raw PWM holds like low2");
 rx("OFF\n");g_comm.Handle();neutral();require(TIM8->CCR1==1500,"OFF stops manual outputs");
 rx("1 -100\n2 9999\n5 9999\n");g_comm.Handle();
 require(TIM1->CCR1==1100 && TIM1->CCR2==1900 && TIM8->CCR1==2500,"legacy numeric bounds match low2");
 rx("OFF\n");g_comm.Handle();
 // OFF plus an unfinished trailing command must still resync before execution.
 rx("OFF\nCTRL:0.5");g_comm.Handle();neutral();
 rx(",0,0,0,0,0,0\n");g_comm.Handle();neutral();
 rx(move);g_comm.Handle();require(TIM1->CCR1==1400,"partial tail after OFF discarded, next full CTRL accepted");
 rx("OFF\n");g_comm.Handle();
 // Match the actual low2 equations, including servo float-to-int truncation.
 for(int i=-100;i<=100;++i) {
  const float v=float(i)/100.f;const float commands[7]={v,-v,v,-v,v,-v,v*.5f};
  g_motion.Apply(commands);
  require(TIM1->CCR1==uint32_t(1500+int32_t(-v*400.f)),"low2 motor formula equivalence");
  require(TIM8->CCR1==uint32_t(1500+int32_t((v*90.f)*(500.f/90.f))),"low2 servo formula equivalence");
  require(TIM8->CCR3==uint32_t(1500+int32_t(std::lround(v*.5f*500.f))),"camera normalized angle conversion");
 }
 g_motion.EmergencyStop();
 TelemetryService sparse;fake::tick=5000;sparse.Handle();
 require(fake::tx.find("RDBG INIT:")!=std::string::npos,"compact RPM summary every five seconds");
 sparse.TxCompleteISR();fake::tick=5100;sparse.Handle();
 require(fake::tx.find("RDBG")==std::string::npos,"normal telemetry does not repeat summary");
 rx("OFF:0.1111\n");g_comm.Handle();neutral();
 require(TIM8->CCR3==1556,"locked camera +10 degree target");
 rx("OFF\n");g_comm.Handle();require(TIM8->CCR3==1556,"plain OFF holds camera");
 rx("CTRL:0.2,0,0,0,0,0,0.1111\n");g_comm.Handle();
 require(TIM8->CCR3==1556 && TIM1->CCR1==1420,"unlock leaves camera angle unchanged");
 step(201);neutral();require(TIM8->CCR3==1556,"independent IRQ deadman holds camera");g_comm.Handle();
 require(TIM8->CCR3==1556,"main-loop timeout holds camera");
 rx("OFF:nan\n");g_comm.Handle();neutral();require(TIM8->CCR3==1556,"bad camera never suppresses OFF or changes angle");
 rx("OFF:-1\n");g_comm.Handle();require(TIM8->CCR3==1000,"camera -90 degrees");
 rx("OFF:1\n");g_comm.Handle();require(TIM8->CCR3==2000,"camera +90 degrees");
 rx("7 2500\n");g_comm.Handle();require(TIM8->CCR3==2000,"manual camera respects +/-90 limit");
 std::cout<<"firmware fault injection passed\n";
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
