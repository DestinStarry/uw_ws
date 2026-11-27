#include "uw_robot_communication/stm32/protocol.hpp"
#include <iostream>
#include <stdexcept>
static void check(bool v,const char* s) {if(!v) throw std::runtime_error(s);}
int main() {
  uw::Actuators a;a.enabled=true;a.forces<<33,-24.5,16.5,-12.25;
  a.angles<<uw::servo_limit,-uw::servo_limit;
  check(uw::encode_stm32(a)=="CTRL:1.0000,0.5000,-1.0000,-0.5000,1.0000,-1.0000,0.0000\n","physical ordering and normalization");
  a.enabled=false;check(uw::encode_stm32(a)=="OFF:0.0000\n","OFF frame");
  a.camera_angle_deg=10;
  check(uw::encode_stm32(a)=="OFF:0.1111\n","camera operates while motors disabled");
  a.enabled=true;check(uw::encode_stm32(a).find(",0.1111\n")!=std::string::npos,"enabled camera target unchanged");
  a.camera_angle_deg=-90;check(uw::encode_stm32(a).find(",-1.0000\n")!=std::string::npos,"camera lower limit");
  a.camera_angle_deg=91;check(uw::encode_stm32(a)=="OFF\n","invalid camera stops without changing target");
  check(uw::parse_depth("Temp:20 Depth:1.250m DEPTH_MS:100")==1.25,"depth parser");
  check(!uw::parse_depth("DEPTH_VALID:1"),"status must not refresh depth");
  check(!uw::parse_depth("Depth:nan"),"nonfinite depth");
  std::cout<<"hardware_test passed\n";
}
