/// @file serial_port.cpp
/// @brief 非阻塞串口实现；配置与限流策略见头文件注释。
#include "uw_robot_communication/stm32/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <stdexcept>

namespace uw {
namespace {

double monotonic() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

/// 每周期允许的最大读取批次与本批字节数：即使设备洪泛也不能占满执行器。
constexpr int kMaxReadBatches = 8;
constexpr size_t kReadChunk = 512;
/// 单行与缓存上限：超限说明对端异常，直接丢弃而不是无限增长。
constexpr size_t kMaxLine = 1024;
constexpr size_t kMaxRxBuffer = 2048;
/// 写阻塞上限：半帧迟迟写不出去，说明串口/对端已失效。
constexpr double kWriteTimeout = .1;

} // namespace

SerialPort::~SerialPort() { close(); }

void SerialPort::open(const std::string &device) {
  close();
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0)
    throw std::runtime_error("cannot open " + device);

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    close();
    throw std::runtime_error("tcgetattr failed");
  }
  cfmakeraw(&tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close();
    throw std::runtime_error("tcsetattr failed");
  }
  tcflush(fd_, TCIOFLUSH);
}

void SerialPort::close() {
  if (fd_ < 0)
    return;
  // 把已写出一半的帧补完，再显式发 OFF：避免 STM32 停在半条指令上。
  const std::string tail = pending_.substr(offset_) + "\nOFF\n";
  size_t sent = 0;
  const double deadline = monotonic() + kWriteTimeout;
  while (sent < tail.size() && monotonic() < deadline) {
    const ssize_t n = ::write(fd_, tail.data() + sent, tail.size() - sent);
    if (n > 0)
      sent += static_cast<size_t>(n);
    else
      usleep(1000);
  }
  ::close(fd_);
  fd_ = -1;
  pending_.clear();
  rx_.clear();
  offset_ = 0;
  blocked_since_ = 0;
}

void SerialPort::write_line(const std::string &line, double now) {
  if (fd_ < 0)
    return;
  // 只允许一条在途帧：一旦写出前缀，必须完整写完才能切换下一条命令。
  if (offset_ == 0)
    pending_ = line;
  if (pending_.empty()) {
    pending_ = line;
    offset_ = 0;
  }
  const ssize_t n =
      ::write(fd_, pending_.data() + offset_, pending_.size() - offset_);
  if (n > 0) {
    offset_ += static_cast<size_t>(n);
    if (offset_ == pending_.size()) {
      pending_.clear();
      offset_ = 0;
      blocked_since_ = 0;
    }
  } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
    throw std::runtime_error("serial write failed");
  }
  if (!pending_.empty()) {
    if (blocked_since_ == 0)
      blocked_since_ = now;
    if (now - blocked_since_ > kWriteTimeout)
      throw std::runtime_error("serial output blocked >100ms");
  }
}

void SerialPort::read_lines(const LineHandler &on_line) {
  if (fd_ < 0)
    return;
  char bytes[kReadChunk];
  for (int batch = 0; batch < kMaxReadBatches; ++batch) {
    const ssize_t count = ::read(fd_, bytes, sizeof(bytes));
    if (count <= 0) {
      if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
          errno != EINTR)
        throw std::runtime_error("serial read failed");
      break;
    }
    rx_.append(bytes, static_cast<size_t>(count));
    size_t pos;
    while ((pos = rx_.find('\n')) != std::string::npos) {
      const std::string line = rx_.substr(0, pos);
      rx_.erase(0, pos + 1);
      if (line.size() <= kMaxLine)
        on_line(line);
    }
    if (rx_.size() > kMaxRxBuffer)
      rx_.clear();
  }
}

} // namespace uw
