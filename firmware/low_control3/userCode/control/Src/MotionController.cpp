#include "MotionController.h"
#include "Motor.h"
#include "Servo.h"
#include "ControlProtocol.h"
MotionController g_motion;
void MotionController::Init() { EmergencyStop(); }
bool MotionController::ExecuteCTRL(const char *line) {
    const auto command = low::ParseCommand(line);
    if (command.type != low::CommandType::Control) return false;
    Apply(command.values); return true;
}
void MotionController::Apply(const float values[7]) {
    for (int i = 0; i < 7; ++i) commands_[i] = values[i];
    for (int i = 0; i < 4; ++i) g_motor.SetThrust(i, values[i]);
    for (int i = 0; i < 2; ++i) g_servo.SetAngle(i, values[4+i] * 90.f);
    g_servo.SetPWM(2, SERVO_MID + int32_t(values[6] * 1000.f));
    // Immediate CCR update; never wait for the next Device scheduler pass.
    g_motor.Handle(); g_servo.Handle();
}
void MotionController::EmergencyStop() {
    for (float &command : commands_) command = 0;
    for (int i = 0; i < 4; ++i) g_motor.SetPWM(i, MOTOR_MID);
    for (int i = 0; i < 3; ++i) g_servo.SetPWM(i, SERVO_MID);
    g_motor.Handle(); g_servo.Handle();
}
