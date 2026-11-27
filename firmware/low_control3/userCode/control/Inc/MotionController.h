#pragma once
#include "usermain.h"
class MotionController {
public:
    void Init();
    void Handle() {}
    bool ExecuteCTRL(const char *line);
    // Caller validates values and serializes this commit against safety IRQs.
    void Apply(const float values[7]);
    void EmergencyStop();
    void SetCamera(float normalized); // +/-1 = +/-90 degrees, zero = 1500us
    float GetCmd(int i) const { return i >= 0 && i < 7 ? commands_[i] : 0.f; }
private:
    float commands_[7] = {};
};
extern MotionController g_motion;
