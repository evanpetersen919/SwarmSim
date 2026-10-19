#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

// A small portable wrapper over UDP and TCP sockets, for Windows (Winsock2) and POSIX systems. It
// does only what the transports need: IPv4 on the loopback interface, blocking sockets, and a
// poll with a timeout. No platform header is included here, so including this file never brings
// in the Windows macros.
//
// Setup failures throw NetError with the operating system's error code and text. A timeout and a
// closed connection are normal events on a network and are reported as statuses, not exceptions.
// Wall-clock time is used here, for timeouts. It is allowed in this library and not in core/.
namespace swarm::net {

class NetError : public std::runtime_error {
 public:
  // `code` is errno on POSIX systems and WSAGetLastError() on Windows.
  NetError(const std::string& what, int code);
  int code() const { return code_; }

 private:
  int code_;
};

struct Address {
  std::uint32_t ip = 0x7F000001;  // host byte order; 127.0.0.1
  std::uint16_t port = 0;

  static Address loopback(std::uint16_t port) { return Address{0x7F000001, port}; }
  bool operator==(const Address& other) const { return ip == other.ip && port == other.port; }
  bool operator!=(const Address& other) const { return !(*this == other); }
  std::string to_string() const;  // "127.0.0.1:5000"
};

enum class IoStatus {
  kOk,
  kTimeout,  // nothing arrived within the time given
  kClosed,   // the peer closed or reset the connection
};

// An operating system socket handle, closed when the object is destroyed. Move-only.
class Socket {
 public:
  Socket() = default;
  Socket(const Socket&) = delete;
  Socket& operator=(const Socket&) = delete;
  Socket(Socket&& other) noexcept;
  Socket& operator=(Socket&& other) noexcept;
  ~Socket();

  bool valid() const { return handle_ != kInvalid; }
  void close();

  // The local address the socket is bound to. Throws NetError if it is not bound.
  Address local_address() const;
  // The size the operating system gave the receive buffer, which may differ from the size asked
  // for (Linux doubles it and caps it). For tests and diagnostics.
  int receive_buffer_bytes() const;

  using Handle = std::intptr_t;  // SOCKET on Windows, int on POSIX, widened
  static constexpr Handle kInvalid = -1;
  Handle handle() const { return handle_; }

 protected:
  explicit Socket(Handle handle) : handle_(handle) {}

 private:
  Handle handle_ = kInvalid;
};

class UdpSocket : public Socket {
 public:
  UdpSocket() = default;

  // Binds to 127.0.0.1 on `port`, or on a port the operating system chooses if `port` is 0, and
  // asks for a receive buffer of `receive_buffer` bytes (best effort). Throws NetError.
  static UdpSocket bind_loopback(std::uint16_t port = 0, int receive_buffer = 4 << 20);

  // Sends one datagram. Throws NetError on failure, including use of a socket that is not valid.
  void send_to(const Address& to, const std::uint8_t* data, std::size_t size);

  struct Received {
    IoStatus status = IoStatus::kTimeout;  // kOk or kTimeout
    std::size_t size = 0;                  // bytes stored in the buffer
    bool truncated = false;                // the datagram was larger than the buffer
    Address from;
  };
  // Waits up to `timeout_ms` (0 polls, negative waits without limit) for a datagram and stores it.
  Received receive(std::uint8_t* buffer, std::size_t capacity, int timeout_ms);

 private:
  explicit UdpSocket(Handle h) : Socket(h) {}
};

class TcpStream : public Socket {
 public:
  TcpStream() = default;

  // Connects to `to`, blocking. Throws NetError, for example if nothing is listening.
  static TcpStream connect(const Address& to);

  // Writes all of the bytes, blocking until the operating system has taken them. Returns kClosed
  // if the peer has closed or reset the connection. Throws NetError for other failures.
  IoStatus send_all(const std::uint8_t* data, std::size_t size);

  // Waits up to `timeout_ms` for data and reads what is available, at most `capacity` bytes.
  // kOk with `got` > 0, kTimeout with `got` == 0, or kClosed when the peer has closed.
  IoStatus receive_some(std::uint8_t* buffer, std::size_t capacity, std::size_t& got,
                        int timeout_ms);

  // Reads exactly `size` bytes, waiting up to `timeout_ms` in total. kOk when all arrived,
  // kTimeout if time ran out (`got` says how many bytes were read), kClosed if the peer closed
  // first.
  IoStatus receive_exact(std::uint8_t* buffer, std::size_t size, std::size_t& got, int timeout_ms);

 private:
  friend class TcpListener;
  explicit TcpStream(Handle h) : Socket(h) {}
};

class TcpListener : public Socket {
 public:
  TcpListener() = default;

  // Listens on 127.0.0.1 on `port`, or a port the operating system chooses if `port` is 0.
  static TcpListener listen_loopback(std::uint16_t port = 0, int backlog = 64);

  // Waits up to `timeout_ms` for a connection. Returns nothing on a timeout.
  std::optional<TcpStream> accept(int timeout_ms);

 private:
  explicit TcpListener(Handle h) : Socket(h) {}
};

// Waits up to `timeout_ms` (0 polls, negative waits without limit) until at least one of the
// sockets can be read without blocking, and returns the positions of the ones that can. A listener
// is readable when a connection is waiting, and a socket whose peer closed is readable. Returns an
// empty list on a timeout. Throws NetError on failure.
std::vector<std::size_t> wait_readable(const std::vector<const Socket*>& sockets, int timeout_ms);

}  // namespace swarm::net
