#include "ControlProtocol.h"
#include "RpmDecoder.h"
#include "TelemetryProtocol.h"
#include <string>
#include <vector>
#include <array>
#include <stdexcept>
#include <iostream>
#include <cstring>
static void require(bool value,const char *reason) { if(!value) throw std::runtime_error(reason); }
static low::Command control(const char *s="CTRL:0,0,0,0,0,0,-0.5") { return low::ParseCommand(s); }
static void commands() {
    require(control().type==low::CommandType::Control,"neutral decode");
    require(low::ParseCommand("CTRL:1,-1,0.2,-0.2,1,-1,0.5").type==low::CommandType::Control,"ranges");
    for(const char *bad:{"CTRL:nan,0,0,0,0,0,0","CTRL:inf,0,0,0,0,0,0",
        "CTRL:1.01,0,0,0,0,0,0","CTRL:1e99,0,0,0,0,0,0","CTRL:1e-999,0,0,0,0,0,0",
        "CTRL:0,0,0,0,0,0,0extra","CTRL:0,0,0,0,0,0,0,0","CTRL:0,0,0,0,0,0",
        "CTRL:0x1p0,0,0,0,0,0,0","CTRL: 0,0,0,0,0,0,0","OFFwhatever","BUZZx","1 1900"})
        require(low::ParseCommand(bad).type==low::CommandType::Invalid,bad);
    low::ControlGuard guard;
    auto moving=control("CTRL:0.1,0,0,0,0,0,0");
    require(!guard.Accept(moving,10,10),"boot must require neutral");
    require(guard.Accept(control(),10,10),"neutral handshake");
    require(guard.Accept(moving,20,20),"active control");
    require(!guard.Expire(220),"200ms boundary");
    require(guard.Expire(221),"timeout stop");
    require(!guard.Accept(moving,222,222),"must neutral after timeout");
    require(!guard.Accept(control(),400,200),"stale queued command rejected");
    require(guard.Accept(control(),UINT32_MAX-30,UINT32_MAX-30),"wrap neutral");
    require(!guard.Expire(50),"wrap recent");
    require(guard.Expire(200),"wrap timeout");
}
static void feed(low::LineInbox &inbox,const std::string &text,uint32_t now=100) {
    for(unsigned char c:text) inbox.Push(c,now);
}
static void framing() {
    low::LineInbox inbox;low::Line line;
    feed(inbox,"CTRL:0,0,");require(!inbox.Pop(line),"partial never a command");
    feed(inbox,"0,0,0,0,0\r\nOFF\nBUZZ\n");
    require(inbox.Pop(line)&&std::string(line.text)=="CTRL:0,0,0,0,0,0,0","split and CRLF");
    require(inbox.Pop(line)&&std::string(line.text)=="OFF","coalesced off");
    require(inbox.Pop(line)&&std::string(line.text)=="BUZZ","coalesced buzz");
    require(!inbox.Pop(line),"no phantom CRLF frame");
    feed(inbox,std::string(128,'x')+"OFF\n");
    require(inbox.TakeFault(),"overlength faults");require(!inbox.Pop(line),"never accept suffix of truncated line");
    feed(inbox,"OFF\n");require(inbox.Pop(line),"recover at next delimiter");
    feed(inbox,"CTRL:",100);feed(inbox,"0\n",201);
    require(inbox.TakeFault()&&!inbox.Pop(line),"slow drip rejected");
    for(unsigned i=0;i<8;++i)feed(inbox,"OFF\n");
    require(inbox.TakeFault(),"queue overflow faults");
    inbox.Reset();feed(inbox,"suffix\nOFF\n");
    require(inbox.Pop(line)&&std::string(line.text)=="OFF","fault flush and delimiter recovery");
    inbox.Push(0,100);require(inbox.TakeFault(),"NUL faults");
    uint32_t rng=42;
    for(unsigned i=0;i<100000;++i) {
        rng=rng*1664525u+1013904223u;inbox.Push(uint8_t(rng>>24),i/10);
        if(inbox.TakeFault())inbox.Reset();
        while(inbox.Pop(line))require(std::strlen(line.text)<low::kLineSize,"bounded output");
    }
}
static std::array<uint8_t,10> packet(uint16_t erpm) {
    std::array<uint8_t,10> p={{25,1,2,3,4,5,6,uint8_t(erpm>>8),uint8_t(erpm),0}};
    p[9]=low::RpmDecoder::Crc(p.data(),9);return p;
}
static void send(low::RpmDecoder &d,const std::array<uint8_t,10> &p,uint32_t now) {
    for(auto b:p)d.Feed(b,now);
}
static void rpm() {
    const char *known="123456789";
    require(low::RpmDecoder::Crc(reinterpret_cast<const uint8_t *>(known),9)==0xf4,"CRC8 known vector");
    low::RpmDecoder d;
    const auto p=packet(700);
    send(d,p,10);require(!d.Valid(10),"one random CRC is insufficient");
    send(d,p,20);require(d.Valid(20)&&d.Rpm()==10000,"mechanical rpm/pole pairs");
    require(d.Valid(520)&&!d.Valid(521),"sample timeout");
    send(d,p,600);send(d,p,610);require(d.Valid(610),"recover after stale");
    auto corrupt=p;corrupt[3]^=0x80;send(d,corrupt,620);
    require(!d.Valid(620),"bad CRC immediately invalidates");
    for(unsigned i=0;i<4;++i)send(d,p,630+i);
    require(d.Valid(634)&&d.Rpm()==10000,"byte slide recovery");
    d.Feed(0x55,640);for(unsigned i=0;i<4;++i)send(d,p,650+i);
    require(d.Valid(654),"inserted byte recovery");
    for(size_t skip=0;skip<10;++skip) {
        low::RpmDecoder e;send(e,p,0);send(e,p,10);
        for(size_t i=0;i<10;++i)if(i!=skip)e.Feed(p[i],20);
        for(unsigned i=0;i<5;++i)send(e,p,30+i);
        require(e.Valid(35)&&e.Rpm()==10000,"all deletion positions recover");
    }
    low::RpmDecoder zero;auto z=packet(0);send(zero,z,1);send(zero,z,2);
    require(zero.Valid(2)&&zero.Rpm()==0,"real zero is valid");
    low::RpmDecoder disconnected;
    for(unsigned i=0;i<100;++i)disconnected.Feed(0,1);
    require(!disconnected.Valid(1),"all-zero noise not a frame");
    low::RpmDecoder independent;require(!independent.Valid(1),"channels independent");
    low::RpmDecoder wrap;send(wrap,p,UINT32_MAX-20);send(wrap,p,UINT32_MAX-10);
    require(wrap.Valid(50),"RPM tick wrap");
    uint32_t rng=17;
    for(unsigned i=0;i<100000;++i){rng=rng*1664525u+1013904223u;d.Feed(uint8_t(rng>>24),i/100);}
    for(unsigned i=0;i<5;++i)send(d,p,2000+i);
    require(d.Valid(2005),"recover after noise stream");
}
static void telemetry() {
    low::TelemetrySnapshot s;char buffer[512];
    auto n=low::FormatTelemetry(buffer,sizeof(buffer),s);
    require(n&&std::string(buffer)=="VOLT:-- CH0:-- CH1:-- CH2:-- CH3:--\r\n","compact startup frame");
    s.voltage.valid=true;s.voltage.value=1525;s.voltage.age=10;
    s.rpm[0].valid=true;s.rpm[0].value=1234;s.rpm[0].age=20;
    s.rpm[1].valid=true;s.rpm[1].value=0;s.rpm[1].age=30;
    n=low::FormatTelemetry(buffer,sizeof(buffer),s);std::string text(buffer);
    require(text=="VOLT:15.25 CH0:1234 CH1:0 CH2:-- CH3:--\r\n","compact values and real zero");
    require(!low::FormatTelemetry(buffer,16,s)&&buffer[0]=='\0',"no truncated frame");
    require(low::FormatDiagnostics(buffer,sizeof(buffer),s)>0,"diagnostic encoding");
    text=buffer;
    require(text.find("DIAG MCU_SEQ:")==0,"diagnostic prefix");
    require(text.find("CH0_VALID:1 CH0_AGE_MS:20")!=std::string::npos,"diagnostic channel validity");
    require(text.find("CH2_AGE_MS:-1")!=std::string::npos,"unknown sample age is -1");
    require(text.find(" CH0:")==std::string::npos&&text.find("VOLT:")==std::string::npos,"diagnostics never refresh measurements");
    require(!low::FormatDiagnostics(buffer,16,s)&&buffer[0]=='\0',"no truncated diagnostic");
    s.sequence=s.now=s.spi_errors=s.rejected=s.stop_reason=UINT32_MAX;
    for(auto &v:s.rpm){v.valid=true;v.value=936214;v.age=UINT32_MAX-1;}
    const auto regular=low::FormatTelemetry(buffer,sizeof(buffer),s);
    require(regular>0&&regular<100,"compact bounded size");
    require(low::FormatDiagnostics(buffer+regular,sizeof(buffer)-regular-2,s)>0,"both worst-case lines and resync fit");
}

int main(int argc,char **argv) {
    try {
        std::string suite=argc>1?argv[1]:"";
        if(suite=="command")commands();else if(suite=="framing")framing();
        else if(suite=="rpm")rpm();else if(suite=="telemetry")telemetry();else return 2;
        std::cout<<suite<<" passed\n";
    }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
