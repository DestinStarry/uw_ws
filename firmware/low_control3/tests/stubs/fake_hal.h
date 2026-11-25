#pragma once
#include "main.h"
#include <string>
#include <vector>
namespace fake {
extern uint32_t tick,irqmask,spi_calls,tx_calls,tx_aborts;
extern bool spi_fail,adc_fail,tx_fail;
extern std::string tx;
extern uint32_t ref_raw,battery_raw;
extern std::vector<uint8_t> fifo[4];
void Reset();
}
