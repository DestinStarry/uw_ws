#include "uw_robot_communication/tcp/protocol.hpp"
#include <cmath>
#include <stdexcept>
int main() {
  auto ok = uw::parse_command("CMD:0.2,-0.1,0.25,0.1,0,0,1,0,0,0");
  if (!ok) throw std::runtime_error("valid CMD rejected");
  if (std::abs(ok->velocity[0] - .2) > 1e-12 || std::abs(ok->velocity[2] - .25) > 1e-12)
    throw std::runtime_error("CMD velocity parsed wrong");
  if (!ok->arm || ok->estop || ok->depth_toggle || ok->loop_toggle)
    throw std::runtime_error("CMD buttons parsed wrong");
  for (const char *line : {"CMD:nan,0,0,0,0,0,1,0,0,0",
                           "CMD:0,0,0,0,0,0,2,0,0,0",
                           "CMD:0,0,0,0,0,0,1,0,0,0,",
                           "CMD:2,0,0,0,0,0,1,0,0,0",
                           "CMD:0,0,0,2,0,0,1,0,0,0",
                           "CMD:0,0,0,0,0",
                           "JOY:0,0,0,0,0,0,1,0,0,0,0,0"})
    if (uw::parse_command(line)) throw std::runtime_error("malformed CMD accepted");
}
