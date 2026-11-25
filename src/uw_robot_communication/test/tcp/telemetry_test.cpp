#include "uw_robot_communication/tcp/telemetry.hpp"
#include <stdexcept>
#include <string>
static void check(bool b) { if (!b) throw std::runtime_error("telemetry check failed"); }
int main() {
 auto v=uw::parse_voltage("Depth:0 VOLT:15.25 TEMP:99 CH0:123 ARMED:1\r\n");
 check(v.present && v.value && *v.value==15.25);
 check(!uw::parse_voltage("TEMP:99 CH0:0 ARMED:1").present);
 for(auto s:{"VOLT:nan","VOLT:inf","VOLT:-1","VOLT:1001","VOLT:15x","VOLT:","VOLT:12 VOLT:13"}) {
 auto bad=uw::parse_voltage(s);check(bad.present && !bad.value); }
 check(!uw::parse_voltage(std::string(1025,'x')).value);
 uw::TimedReading cpu{42.5,10}, voltage{15.25,10}, missing;
 auto out=uw::encode_aux_telemetry(cpu,voltage,11,3,3,true);
 check(out.find("CPU_TEMP:42.50\nCPU_TEMP_VALID:1")!=std::string::npos);
 check(out.find("\nTEMP:42.50\n")!=std::string::npos);
 check(out.find("VOLT:15.25\nVOLT_VALID:1")!=std::string::npos);
 check(out.find("ARMED:")==std::string::npos);
 out=uw::encode_aux_telemetry(cpu,voltage,14,3,3,false);
 check(out.find("VOLT:--\nVOLT_VALID:0")!=std::string::npos);
 check(out.find("\nTEMP:")==std::string::npos);
 check(!missing.valid(11,3));check(!cpu.valid(9,3));
 check(uw::encode_aux_telemetry(missing,missing,11,3,3,true).find("CPU_TEMP_AGE_MS:-1")!=std::string::npos);
 auto channels=uw::parse_rpm("CH0:0 CH1:1234 CH2:-321 CH3:65000 TEMP:99 ARMED:1");
 check(channels[0].value && *channels[0].value==0);
 check(channels[1].value && *channels[1].value==1234);
 check(channels[2].value && *channels[2].value==-321);
 check(channels[3].value && *channels[3].value==65000);
 auto bad=uw::parse_rpm("CH0:12 CH0:13 CH1:nan CH2:12.5 CH3:123x");
 for(const auto &field:bad) check(field.present && !field.value);
 auto partial=uw::parse_rpm("CH1:42 CH4:1 CH00:7 ARMED:1");
 check(!partial[0].present && partial[1].value && !partial[2].present && !partial[3].present);
 check(!uw::parse_rpm("CH0:1000001")[0].value);
 std::array<uw::TimedReading,4> rpm;
 for(size_t i=0;i<4;++i) rpm[i]={channels[i].value,10};
 out=uw::encode_rpm_telemetry(rpm,11,3);
 check(out.find("CH0:0\nCH0_VALID:1")!=std::string::npos);
 check(out.find("CH2:-321\nCH2_VALID:1")!=std::string::npos);
 check(out.find("ARMED:")==std::string::npos);
 out=uw::encode_rpm_telemetry(rpm,14,3);
 check(out.find("CH0:--\nCH0_VALID:0")!=std::string::npos);
 check(out.find("CH3:--\nCH3_VALID:0")!=std::string::npos);
}
