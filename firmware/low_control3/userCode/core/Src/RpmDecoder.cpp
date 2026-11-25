#include "RpmDecoder.h"
#include <cstring>
namespace low {
uint8_t RpmDecoder::Crc(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80) ? uint8_t((crc << 1) ^ 0x07) : uint8_t(crc << 1);
    }
    return crc;
}
void RpmDecoder::Reset() { used_ = 0; consecutive_ = 0; valid_ = false; }
bool RpmDecoder::Valid(uint32_t now) const {
    return valid_ && sampled_ && uint32_t(now - last_frame_) <= kTimeoutMs;
}
void RpmDecoder::Feed(uint8_t byte, uint32_t now) {
    // Gap is between bytes inside a frame; long gaps also require reacquisition.
    if ((used_ && uint32_t(now - last_byte_) > 50) ||
        (valid_ && sampled_ && uint32_t(now - last_frame_) > kTimeoutMs)) Reset();
    last_byte_ = now;
    window_[used_++] = byte;
    if (used_ < kFrameSize) return;
    bool all_zero = true, all_ff = true;
    for (size_t i = 0; i < kFrameSize; ++i) {
        all_zero = all_zero && window_[i] == 0;
        all_ff = all_ff && window_[i] == 0xff;
    }
    if (pole_pairs_ && !all_zero && !all_ff && Crc(window_, 9) == window_[9]) {
        ++frames_;
        if (consecutive_ < 2) ++consecutive_;
        if (consecutive_ >= 2) {
            const uint16_t erpm = uint16_t((uint16_t(window_[7]) << 8) | window_[8]);
            rpm_ = int32_t(erpm) * 100 / pole_pairs_;
            last_frame_ = now; sampled_ = valid_ = true;
        }
        used_ = 0;
    } else {
        ++rejected_;
        consecutive_ = 0; valid_ = false;
        // No fixed header: slide one byte, then require two CRC-valid frames.
        std::memmove(window_, window_ + 1, kFrameSize - 1);
        used_ = kFrameSize - 1;
    }
}
}
