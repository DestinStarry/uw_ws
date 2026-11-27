#include "fake_hal.h"
#include "CH9434.h"
#include "RpmDecoder.h"
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
static void check(bool value,const char *why) { if(!value)throw std::runtime_error(why); }
static std::string summary(CH9434 &chip) {
    char line[256];const uint32_t calls=fake::spi_calls;
    check(chip.FormatRpmSummary(line,sizeof(line))>0,"summary bounded");
    check(fake::spi_calls==calls,"formatting never probes SPI");return line;
}
static void frame(uint8_t ch,uint16_t raw) {
    uint8_t p[10]={25,1,2,3,4,5,6,uint8_t(raw>>8),uint8_t(raw),0};
    p[9]=low::RpmDecoder::Crc(p,9);fake::fifo[ch].insert(fake::fifo[ch].end(),p,p+10);
}
int main(){try{
    fake::Reset();CH9434 chip;chip.Init();
    check(chip.Ready(),"proven low2 startup config");
    check(summary(chip).find("INIT:READY TRY:1 ERR:0 CFG:F MASK:1F")!=std::string::npos,"all UART configs verified");
    // Reproduce the user's valid moving ESC frame: raw ERPM A2 -> 2314 RPM at seven pole pairs.
    const uint8_t captured[10]={0x15,0x05,0xC2,0x15,0xA9,0x04,0xAD,0x00,0xA2,0x4B};
    check(low::RpmDecoder::Crc(captured,9)==captured[9],"actual low2 capture CRC");
    for(int i=0;i<2;++i)fake::fifo[1].insert(fake::fifo[1].end(),captured,captured+10);
    chip.Handle();check(chip.RpmValid(1,fake::tick)&&chip.GetRPM(1)==2314,"actual low2 moving capture decoded identically");
    // The original low3 discarded whole 128-byte blocks while a FIFO was >128.
    for(unsigned ch=0;ch<4;++ch)for(unsigned i=0;i<46;++i)frame(ch,uint16_t(700+ch*70));
    for(unsigned i=0;i<4;++i){fake::tick+=10;chip.Handle();}
    for(unsigned ch=0;ch<4;++ch) {
        check(fake::fifo[ch].empty(),"four-channel backlog fully drained");
        check(chip.RpmValid(ch,fake::tick),"backlog does not suppress all valid feedback");
        check(chip.GetRPM(ch)==10000+int(ch)*1000,"all four channels preserve scaling");
    }
    const uint32_t old_clocks=fake::clock_writes;fake::spi_fail=true;
    const uint32_t start=fake::tick,calls=fake::spi_calls;chip.Handle();
    check(fake::tick-start<=2 && fake::spi_calls-calls<=1,"fail-fast SPI preserves loop latency");
    check(!chip.RpmValid(1,fake::tick),"failed SPI invalidates feedback");
    fake::spi_fail=false;fake::tick+=1000;chip.Handle();
    check(chip.Ready()&&fake::clock_writes==old_clocks,"no runtime clock-settling/reconfigure delay");
    frame(1,0);frame(1,0);chip.Handle();
    check(chip.RpmValid(1,fake::tick)&&chip.GetRPM(1)==0,"real zero remains valid");
    fake::tick+=501;check(!chip.RpmValid(1,fake::tick),"feedback expires in 500ms");
    fake::Reset();fake::spi_fail_until=120;CH9434 late;late.Init();
    check(late.Ready()&&fake::clock_writes==1,"late module configured only after successful SPI probe");
    check(summary(late).find("TRY:2 ERR:0 CFG:F")!=std::string::npos,"bounded startup retry succeeds");
    fake::Reset();fake::corrupt_dll=2;CH9434 baud_fault;baud_fault.Init();
    check(!baud_fault.Ready()&&summary(baud_fault).find("TRY:3 ERR:3 CFG:B")!=std::string::npos,"wrong baud divisor rejects startup");
    fake::Reset();fake::fifo_disabled=1;CH9434 fifo_fault;fifo_fault.Init();
    check(!fifo_fault.Ready()&&summary(fifo_fault).find("TRY:3 ERR:3 CFG:D")!=std::string::npos,"read-only IIR identifies missing FIFO enable");
    fake::Reset();fake::spi_stuck=255;CH9434 missing;missing.Init();
    check(!missing.Ready()&&fake::clock_writes==0,"failed SPI never initializes UART clocks");
    check(summary(missing).find("TRY:3 ERR:1 CFG:0 MASK:08")!=std::string::npos,"disconnected module diagnosed");
    fake::tick+=1000;missing.Handle();check(fake::clock_writes==0,"failed boot does not block main loop with reinitialization");
    std::cout<<"CH9434 low2 baseline, hardware capture, backlog and initialization fault tests passed\n";
}catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}
