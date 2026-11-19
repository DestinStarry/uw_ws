/// @file tcp_server.cpp
/// @brief 非阻塞单客户端 TCP 服务端实现；策略见头文件注释。
#include "uw_robot_communication/tcp/tcp_server.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <stdexcept>

namespace uw {
namespace {

/// 每 5 ms 最多读 4 批，避免输入洪泛占满单线程执行器。
constexpr int kMaxReadBatches = 4;
constexpr size_t kReadChunk = 1024;
/// 未成行缓存上限：超过视为对端异常。
constexpr size_t kMaxRx = 2048;
/// 发送队列上限：超过视为客户端不读，主动断开。
constexpr size_t kMaxTx = 8192;

} // namespace

TcpServer::~TcpServer() { close(); }

void TcpServer::open(const std::string &host, int port) {
  close();
  server_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (server_ < 0)
    throw std::runtime_error("socket failed");

  int yes = 1;
  setsockopt(server_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (port < 1 || port > 65535 ||
      inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1 ||
      bind(server_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0 ||
      listen(server_, 1) < 0) {
    close();
    throw std::runtime_error("invalid address or TCP bind/listen failed");
  }
}

void TcpServer::close() {
  if (client_ >= 0) {
    ::close(client_);
    client_ = -1;
  }
  if (server_ >= 0) {
    ::close(server_);
    server_ = -1;
  }
  rx_.clear();
  tx_.clear();
}

void TcpServer::disconnect() {
  if (client_ >= 0)
    ::close(client_);
  client_ = -1;
  rx_.clear();
  tx_.clear();
  if (on_disconnect_)
    on_disconnect_();
}

void TcpServer::queue(const std::string &value) {
  if (client_ < 0)
    return;
  if (tx_.size() + value.size() > kMaxTx) {
    disconnect();
    return;
  }
  tx_ += value;
}

void TcpServer::poll() {
  if (server_ < 0)
    return;
  if (client_ < 0) {
    client_ = accept4(server_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
    if (client_ < 0)
      return;
    rx_.clear();
    tx_.clear();
  }

  char bytes[kReadChunk];
  for (int batch = 0; batch < kMaxReadBatches; ++batch) {
    const ssize_t n = recv(client_, bytes, sizeof(bytes), 0);
    if (n == 0) {
      disconnect();
      return;
    }
    if (n < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
        disconnect();
      break;
    }
    rx_.append(bytes, static_cast<size_t>(n));
    size_t index;
    while ((index = rx_.find('\n')) != std::string::npos) {
      const std::string line = rx_.substr(0, index);
      rx_.erase(0, index + 1);
      if (on_line_)
        on_line_(line);
    }
    if (rx_.size() > kMaxRx) {
      disconnect();
      return;
    }
  }

  // send 可能只写出一部分，剩余内容留到下一次 poll。
  if (client_ >= 0 && !tx_.empty()) {
    const ssize_t n = send(client_, tx_.data(), tx_.size(), MSG_NOSIGNAL);
    if (n > 0)
      tx_.erase(0, static_cast<size_t>(n));
    else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
             errno != EINTR)
      disconnect();
  }
}

} // namespace uw
