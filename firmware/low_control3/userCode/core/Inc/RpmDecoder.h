#pragma once
#include <cstdint>
#include <cstddef>
namespace low {
class RpmDecoder {
public:
    static const size_t kFrameSize = 10;
    static const uint32_t kTimeoutMs = 500;
    explicit RpmDecoder(uint16_t pole_pairs = 7) : pole_pairs_(pole_pairs) {}
    void Feed(uint8_t byte, uint32_t now);
    void Reset();
    bool Valid(uint32_t now) const;
    int32_t Rpm() const { return rpm_; }
    uint32_t Age(uint32_t now) const { return sampled_ ? uint32_t(now - last_frame_) : UINT32_MAX; }
    uint32_t Frames() const { return frames_; }
    uint32_t RejectedWindows() const { return rejected_; }
    static uint8_t Crc(const uint8_t *data, size_t length);
private:
    uint8_t window_[kFrameSize] = {};
    size_t used_ = 0;
    uint8_t consecutive_ = 0;
    uint16_t pole_pairs_;
    uint32_t last_byte_ = 0, last_frame_ = 0, frames_ = 0, rejected_ = 0;
    int32_t rpm_ = 0;
    bool valid_ = false, sampled_ = false;
};
}
