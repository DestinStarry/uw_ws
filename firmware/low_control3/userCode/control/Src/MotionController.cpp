#include "MotionController.h"
#include "Motor.h"
#include "Servo.h"
#include <cmath>
#include "ControlProtocol.h"
MotionController g_motion;
void MotionController::Init() { SetCamera(0); EmergencyStop(); }
bool MotionController::ExecuteCTRL(const char *line) {
    const auto command = low::ParseCommand(line);
    if (command.type != low::CommandType::Control) return false;
    Apply(command.values); return true;
}
void MotionController::Apply(const float values[7]) {
    for (int i = 0; i < 7; ++i) commands_[i] = values[i];
    for (int i = 0; i < 4; ++i) g_motor.SetThrust(i, values[i]);
    for (int i = 0; i < 2; ++i) g_servo.SetAngle(i, values[4+i] * 90.f);
    SetCamera(values[6]);
    // Immediate CCR update; never wait for the next Device scheduler pass.
    g_motor.Handle(); g_servo.Handle();
}
void MotionController::EmergencyStop() {
    for (int i=0;i<6;++i) commands_[i]=0;
    for (int i = 0; i < 4; ++i) g_motor.SetPWM(i, MOTOR_MID);
    for (int i = 0; i < 2; ++i) g_servo.SetPWM(i, SERVO_MID);
    g_motor.Handle(); g_servo.Handle();
}

void MotionController::SetCamera(float normalized) {
    if (!std::isfinite(normalized)) return;
    if (normalized>1.f) normalized=1.f;
    if (normalized<-1.f) normalized=-1.f;
    commands_[6]=normalized;
    g_servo.SetPWM(2, SERVO_MID + int32_t(std::lround(normalized*500.f)));
    // Only channel 7 is updated; motor and steering outputs are untouched.
    TIM8->CCR3=uint32_t(g_servo.data[2]);
}
