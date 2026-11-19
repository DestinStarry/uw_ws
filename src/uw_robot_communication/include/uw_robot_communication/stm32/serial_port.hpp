/// @file serial_port.hpp
/// @brief 非阻塞串口封装：115200-8N1、按行读取、单帧在途写出。
///
/// 从 ``hardware_node`` 抽出来的原因：打开/关闭、半帧续写、读洪泛限流这三件事
/// 与业务无关，且容易写错；单独成类后节点只保留"协议 + 业务"。
#pragma once

#include <functional>
#include <string>

namespace uw {

/// 115200-8N1 串口（非阻塞）。所有方法都在同一线程调用，内部无锁。
///
/// 行为约定：
/// * ``close()`` 会尽力把未写完的半帧补完，再追加一行 ``OFF``，最多等 100 ms；
/// * ``write_line()`` 每周期只尝试写一次，未写完的部分留到下一周期，
///   超过 100 ms 仍阻塞则抛异常（由节点停机处理）；
/// * ``read_lines()`` 每周期最多读 8 批、每批 512 B，按 ``\n`` 拆行；
///   单行超过 1024 B 丢弃，未成行的缓存超过 2048 B 直接清空，防止设备洪泛。
class SerialPort {
public:
  using LineHandler = std::function<void(const std::string &)>;

  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort &operator=(const SerialPort &) = delete;

  /// 打开并配置设备；失败抛 ``std::runtime_error``。
  void open(const std::string &device);
  bool is_open() const { return fd_ >= 0; }
  /// 关闭：补完半帧 + 发 OFF + close。
  void close();

  /// 发送一行（调用方保证以 ``\n`` 结尾）。``now`` 为单调秒，用于阻塞判定。
  void write_line(const std::string &line, double now);

  /// 读取可用数据并按行回调。
  void read_lines(const LineHandler &on_line);

private:
  int fd_ = -1;
  std::string pending_;  ///< 在途帧（可能只写出一部分）。
  std::string rx_;       ///< 接收缓存（可能包含半行）。
  size_t offset_ = 0;    ///< 在途帧已写出的字节数。
  double blocked_since_ = 0;
};

} // namespace uw
