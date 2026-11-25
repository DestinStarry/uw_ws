/*
 * DepthSensor.cpp
 * 解析 UART1（深度计解算板）收到的深度/温度数据
 * 格式: "Depth:XX.XXm Temp:XX.XXC\r\n"
 */

#include "DepthSensor.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

DepthSensor g_depth;

void DepthSensor::Init()
{
    depth = 0.0f;
    temp  = 0.0f;
    fresh = false;
    last_update_ms = 0;
}

void DepthSensor::Handle()
{
    /* 不需要周期性操作 */
}

void DepthSensor::Receive(uint8_t *data, uint16_t len)
{
    /* 解析 "Depth:XX.XXm Temp:XX.XXC"
     * 注意: nano.specs 下 sscanf %f 不可用(不链接浮点支持), 必须用 strtof */
    (void)len;
    char *p  = (char *)data;
    char *end;
    float d = 0.0f, t = 0.0f;
    bool got_d = false, got_t = false;

    char *dp = strstr(p, "Depth:");
    if (dp) {
        dp += strlen("Depth:");
        d = strtof(dp, &end);
        if (end != dp) got_d = true;
    }

    char *tp = strstr(p, "Temp:");
    if (tp) {
        tp += strlen("Temp:");
        t = strtof(tp, &end);
        if (end != tp) got_t = true;
    }

    if (got_d && got_t) {
        depth = d;
        temp  = t;
        fresh = true;
        last_update_ms = HAL_GetTick();
    }
}
