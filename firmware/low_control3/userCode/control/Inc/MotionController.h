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
    float GetCmd(int i) const { return i >= 0 && i < 7 ? commands_[i] : 0.f; }
private:
    float commands_[7] = {};
};
extern MotionController g_motion;
