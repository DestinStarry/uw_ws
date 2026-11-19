/// @file tcp_server.hpp
/// @brief 非阻塞单客户端 TCP 服务端：按行收帧、带上限的发送队列。
///
/// 从 ``tcp_bridge_node`` 抽出来的原因：socket 生命周期、接包/拆包、发送队列上限
/// 与业务无关；节点只保留"文本协议 ↔ ROS 消息"的翻译。
#pragma once

#include <functional>
#include <string>

namespace uw {

/// 单客户端、非阻塞、单线程轮询的 TCP 服务端。
///
/// 行为约定：
/// * 一次只服务一个客户端；``poll()`` 时若没有连接会尝试 ``accept``；
/// * 收到 ``\n`` 才交付一行；未成行缓存超过 2 KB 视为异常并断开；
/// * 发送队列超过 8 KB 视为客户端不读，主动断开（避免拖垮控制链路）；
/// * 任何断开（对端关闭、写失败、队列溢出）都会触发一次断开回调，
///   让节点把"断线"翻译成急停。
class TcpServer {
public:
  using LineHandler = std::function<void(const std::string &)>;
  using DisconnectHandler = std::function<void()>;

  TcpServer() = default;
  ~TcpServer();
  TcpServer(const TcpServer &) = delete;
  TcpServer &operator=(const TcpServer &) = delete;

  /// 绑定并监听；地址/端口非法或绑定失败抛 ``std::runtime_error``。
  void open(const std::string &host, int port);
  void close();

  void set_line_handler(LineHandler handler) { on_line_ = std::move(handler); }
  void set_disconnect_handler(DisconnectHandler handler) {
    on_disconnect_ = std::move(handler);
  }
  bool connected() const { return client_ >= 0; }

  /// 追加重传内容；未连接时丢弃，超限时断开。
  void queue(const std::string &value);
  /// 接受连接、收帧、发送；每周期调用一次。
  void poll();

private:
  void disconnect();

  int server_ = -1;
  int client_ = -1;
  std::string rx_;  ///< 接收缓存（可能包含半行）。
  std::string tx_;  ///< 待发送内容（可能只发出一部分）。
  LineHandler on_line_;
  DisconnectHandler on_disconnect_;
};

} // namespace uw
