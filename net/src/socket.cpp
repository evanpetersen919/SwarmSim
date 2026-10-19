#include "swarm/net/socket.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <system_error>
#include <utility>

// SWARM_NET_ANALYZE_POSIX makes a Windows machine compile the POSIX branch against stand-in
// headers, so that branch can be checked by a static analyzer where no POSIX headers exist.
#if defined(_WIN32) && !defined(SWARM_NET_ANALYZE_POSIX)
#define SWARM_NET_WINDOWS 1
#endif

#ifdef SWARM_NET_WINDOWS
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#ifdef SWARM_NET_WINDOWS
#ifndef SIO_UDP_CONNRESET
#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)
#endif
#endif

namespace swarm::net {

NetError::NetError(const std::string& what, int code)
    : std::runtime_error(what + ": " + std::system_category().message(code) + " (error " +
                         std::to_string(code) + ")"),
      code_(code) {}

std::string Address::to_string() const {
  return std::to_string((ip >> 24) & 0xFFU) + "." + std::to_string((ip >> 16) & 0xFFU) + "." +
         std::to_string((ip >> 8) & 0xFFU) + "." + std::to_string(ip & 0xFFU) + ":" +
         std::to_string(port);
}

namespace {

using Handle = Socket::Handle;

#ifdef SWARM_NET_WINDOWS
using NativeSocket = SOCKET;
using SockLen = int;
using IoSize = int;
constexpr int kBadHandleCode = WSAENOTSOCK;

int last_error() { return WSAGetLastError(); }
NativeSocket native(Handle h) { return static_cast<NativeSocket>(h); }
Handle widen(NativeSocket s) { return static_cast<Handle>(s); }
bool is_invalid(NativeSocket s) { return s == INVALID_SOCKET; }
void close_native(Handle h) { closesocket(native(h)); }
bool is_closed_error(int code) {
  return code == WSAECONNRESET || code == WSAECONNABORTED || code == WSAESHUTDOWN ||
         code == WSAENOTCONN;
}
bool is_interrupted(int) { return false; }
#else
using NativeSocket = int;
using SockLen = socklen_t;
using IoSize = std::size_t;
constexpr int kBadHandleCode = EBADF;

int last_error() { return errno; }
NativeSocket native(Handle h) { return static_cast<NativeSocket>(h); }
Handle widen(NativeSocket s) { return static_cast<Handle>(s); }
bool is_invalid(NativeSocket s) { return s < 0; }
void close_native(Handle h) { ::close(native(h)); }
bool is_closed_error(int code) {
  return code == ECONNRESET || code == EPIPE || code == ECONNABORTED || code == ENOTCONN;
}
bool is_interrupted(int code) { return code == EINTR; }
#endif

constexpr SockLen kSockaddrLen = static_cast<SockLen>(sizeof(sockaddr_in));
constexpr std::size_t kMaxIoChunk = std::size_t{1} << 20;

#if defined(MSG_NOSIGNAL)
constexpr int kSendFlags = MSG_NOSIGNAL;  // a write to a closed peer is an error, not a signal
#else
constexpr int kSendFlags = 0;
#endif

// Starts Winsock once, on first use. Nothing to do elsewhere.
void ensure_network() {
#ifdef SWARM_NET_WINDOWS
  struct Startup {
    Startup() {
      WSADATA data;
      const int result = WSAStartup(MAKEWORD(2, 2), &data);
      if (result != 0) {
        throw NetError("WSAStartup failed", result);
      }
    }
    ~Startup() { WSACleanup(); }
  };
  static const Startup startup;
#endif
}

void require_valid(Handle h, const char* what) {
  if (h == Socket::kInvalid) {
    throw NetError(what, kBadHandleCode);
  }
}

// Network byte order is built by hand, so there is no dependence on the host's byte order and no
// use of the htons and htonl macros, which some C libraries expand into code that draws
// conversion warnings.
sockaddr_in to_sockaddr(const Address& a) {
  static_assert(sizeof(sockaddr_in::sin_addr) == 4, "an IPv4 address is four bytes");
  sockaddr_in sa{};
  sa.sin_family = AF_INET;
  const unsigned char port[2] = {static_cast<unsigned char>((a.port >> 8) & 0xFFU),
                                 static_cast<unsigned char>(a.port & 0xFFU)};
  std::memcpy(&sa.sin_port, port, sizeof(port));
  const unsigned char ip[4] = {static_cast<unsigned char>((a.ip >> 24) & 0xFFU),
                               static_cast<unsigned char>((a.ip >> 16) & 0xFFU),
                               static_cast<unsigned char>((a.ip >> 8) & 0xFFU),
                               static_cast<unsigned char>(a.ip & 0xFFU)};
  std::memcpy(&sa.sin_addr, ip, sizeof(ip));
  return sa;
}

Address from_sockaddr(const sockaddr_in& sa) {
  unsigned char port[2];
  std::memcpy(port, &sa.sin_port, sizeof(port));
  unsigned char ip[4];
  std::memcpy(ip, &sa.sin_addr, sizeof(ip));
  Address a;
  a.ip = (static_cast<std::uint32_t>(ip[0]) << 24) | (static_cast<std::uint32_t>(ip[1]) << 16) |
         (static_cast<std::uint32_t>(ip[2]) << 8) | static_cast<std::uint32_t>(ip[3]);
  a.port = static_cast<std::uint16_t>((static_cast<unsigned>(port[0]) << 8) | port[1]);
  return a;
}

Handle open_socket(int type) {
  ensure_network();
  const NativeSocket s = ::socket(AF_INET, type, 0);
  if (is_invalid(s)) {
    throw NetError("socket() failed", last_error());
  }
  return widen(s);
}

void bind_to(Handle h, const Address& address) {
  const sockaddr_in sa = to_sockaddr(address);
  if (::bind(native(h), reinterpret_cast<const sockaddr*>(&sa), kSockaddrLen) != 0) {
    const int code = last_error();
    close_native(h);
    throw NetError("bind() to " + address.to_string() + " failed", code);
  }
}

bool set_int_option(Handle h, int level, int name, int value) {
  return ::setsockopt(native(h), level, name, reinterpret_cast<const char*>(&value),
                      static_cast<SockLen>(sizeof(value))) == 0;
}

// The sockets that can be read without blocking, by position. An empty list means a timeout.
std::vector<std::size_t> poll_handles(const std::vector<Handle>& handles, int timeout_ms) {
  std::vector<std::size_t> ready;
  if (handles.empty()) {
    return ready;
  }
#ifdef SWARM_NET_WINDOWS
  std::vector<WSAPOLLFD> fds(handles.size());
  for (std::size_t i = 0; i < handles.size(); ++i) {
    fds[i].fd = native(handles[i]);
    fds[i].events = POLLRDNORM;
    fds[i].revents = 0;
  }
  const int result = WSAPoll(fds.data(), static_cast<ULONG>(fds.size()), timeout_ms);
  if (result == SOCKET_ERROR) {
    throw NetError("WSAPoll failed", last_error());
  }
#else
  std::vector<pollfd> fds(handles.size());
  for (std::size_t i = 0; i < handles.size(); ++i) {
    fds[i].fd = native(handles[i]);
    fds[i].events = POLLIN;
    fds[i].revents = 0;
  }
  for (;;) {
    const int result = ::poll(fds.data(), static_cast<nfds_t>(fds.size()), timeout_ms);
    if (result >= 0) {
      break;
    }
    if (!is_interrupted(last_error())) {
      throw NetError("poll failed", last_error());
    }
  }
#endif
  for (std::size_t i = 0; i < fds.size(); ++i) {
    if (fds[i].revents != 0) {  // data, a connection, a hang-up, or an error: a read will not block
      ready.push_back(i);
    }
  }
  return ready;
}

}  // namespace

// Socket ------------------------------------------------------------------------------------

Socket::Socket(Socket&& other) noexcept : handle_(other.handle_) { other.handle_ = kInvalid; }

Socket& Socket::operator=(Socket&& other) noexcept {
  if (this != &other) {
    close();
    handle_ = other.handle_;
    other.handle_ = kInvalid;
  }
  return *this;
}

Socket::~Socket() { close(); }

void Socket::close() {
  if (handle_ != kInvalid) {
    close_native(handle_);
    handle_ = kInvalid;
  }
}

Address Socket::local_address() const {
  require_valid(handle_, "local_address() on a closed socket");
  sockaddr_in sa{};
  SockLen length = kSockaddrLen;
  if (::getsockname(native(handle_), reinterpret_cast<sockaddr*>(&sa), &length) != 0) {
    throw NetError("getsockname() failed", last_error());
  }
  return from_sockaddr(sa);
}

int Socket::receive_buffer_bytes() const {
  require_valid(handle_, "receive_buffer_bytes() on a closed socket");
  int value = 0;
  SockLen length = static_cast<SockLen>(sizeof(value));
  if (::getsockopt(native(handle_), SOL_SOCKET, SO_RCVBUF, reinterpret_cast<char*>(&value),
                   &length) != 0) {
    throw NetError("getsockopt(SO_RCVBUF) failed", last_error());
  }
  return value;
}

// UdpSocket ---------------------------------------------------------------------------------

UdpSocket UdpSocket::bind_loopback(std::uint16_t port, int receive_buffer) {
  const Handle h = open_socket(SOCK_DGRAM);
#ifdef SWARM_NET_WINDOWS
  // Without this, an ICMP "port unreachable" from an earlier send makes the next receive fail.
  DWORD returned = 0;
  BOOL disable = FALSE;
  WSAIoctl(native(h), SIO_UDP_CONNRESET, &disable, sizeof(disable), nullptr, 0, &returned, nullptr,
           nullptr);
#endif
  set_int_option(h, SOL_SOCKET, SO_RCVBUF, receive_buffer);  // best effort
  set_int_option(h, SOL_SOCKET, SO_SNDBUF, receive_buffer);
  bind_to(h, Address::loopback(port));
  return UdpSocket(h);
}

void UdpSocket::send_to(const Address& to, const std::uint8_t* data, std::size_t size) {
  require_valid(handle(), "send_to() on a closed socket");
  const sockaddr_in sa = to_sockaddr(to);
  const long long sent =
      ::sendto(native(handle()), reinterpret_cast<const char*>(data), static_cast<IoSize>(size), 0,
               reinterpret_cast<const sockaddr*>(&sa), kSockaddrLen);
  if (sent < 0) {
    throw NetError("sendto() " + to.to_string() + " failed", last_error());
  }
  if (static_cast<std::size_t>(sent) != size) {
    throw NetError("sendto() sent part of a datagram", 0);
  }
}

UdpSocket::Received UdpSocket::receive(std::uint8_t* buffer, std::size_t capacity, int timeout_ms) {
  require_valid(handle(), "receive() on a closed socket");
  Received out;
  if (poll_handles({handle()}, timeout_ms).empty()) {
    return out;  // a timeout
  }

  sockaddr_in from{};
  SockLen from_length = kSockaddrLen;
  const std::size_t room = std::min(capacity, kMaxIoChunk);
#ifdef SWARM_NET_WINDOWS
  const int flags = 0;
#else
  const int flags = MSG_TRUNC;  // the return value is then the datagram's real length
#endif
  long long got =
      ::recvfrom(native(handle()), reinterpret_cast<char*>(buffer), static_cast<IoSize>(room),
                 flags, reinterpret_cast<sockaddr*>(&from), &from_length);
#ifdef SWARM_NET_WINDOWS
  if (got < 0 && last_error() == WSAEMSGSIZE) {  // the buffer filled and the rest was dropped
    got = static_cast<long long>(room);
    out.truncated = true;
  }
#else
  if (got >= 0 && static_cast<std::size_t>(got) > room) {
    got = static_cast<long long>(room);
    out.truncated = true;
  }
#endif
  if (got < 0) {
    throw NetError("recvfrom() failed", last_error());
  }
  out.status = IoStatus::kOk;
  out.size = static_cast<std::size_t>(got);
  out.from = from_sockaddr(from);
  return out;
}

// TcpStream ---------------------------------------------------------------------------------

TcpStream TcpStream::connect(const Address& to) {
  const Handle h = open_socket(SOCK_STREAM);
  set_int_option(h, IPPROTO_TCP, TCP_NODELAY, 1);
  const sockaddr_in sa = to_sockaddr(to);
  if (::connect(native(h), reinterpret_cast<const sockaddr*>(&sa), kSockaddrLen) != 0) {
    const int code = last_error();
    close_native(h);
    throw NetError("connect() to " + to.to_string() + " failed", code);
  }
  return TcpStream(h);
}

IoStatus TcpStream::send_all(const std::uint8_t* data, std::size_t size) {
  require_valid(handle(), "send_all() on a closed socket");
  std::size_t sent = 0;
  while (sent < size) {
    const std::size_t chunk = std::min(size - sent, kMaxIoChunk);
    const long long n = ::send(native(handle()), reinterpret_cast<const char*>(data + sent),
                               static_cast<IoSize>(chunk), kSendFlags);
    if (n < 0) {
      const int code = last_error();
      if (is_closed_error(code)) {
        return IoStatus::kClosed;
      }
      if (is_interrupted(code)) {
        continue;
      }
      throw NetError("send() failed", code);
    }
    sent += static_cast<std::size_t>(n);
  }
  return IoStatus::kOk;
}

IoStatus TcpStream::receive_some(std::uint8_t* buffer, std::size_t capacity, std::size_t& got,
                                 int timeout_ms) {
  require_valid(handle(), "receive_some() on a closed socket");
  got = 0;
  if (poll_handles({handle()}, timeout_ms).empty()) {
    return IoStatus::kTimeout;
  }
  const std::size_t room = std::min(capacity, kMaxIoChunk);
  const long long n =
      ::recv(native(handle()), reinterpret_cast<char*>(buffer), static_cast<IoSize>(room), 0);
  if (n > 0) {
    got = static_cast<std::size_t>(n);
    return IoStatus::kOk;
  }
  if (n == 0) {
    return IoStatus::kClosed;  // an orderly close by the peer
  }
  const int code = last_error();
  if (is_closed_error(code)) {
    return IoStatus::kClosed;
  }
  if (is_interrupted(code)) {
    return IoStatus::kTimeout;
  }
  throw NetError("recv() failed", code);
}

IoStatus TcpStream::receive_exact(std::uint8_t* buffer, std::size_t size, std::size_t& got,
                                  int timeout_ms) {
  got = 0;
  const auto start = std::chrono::steady_clock::now();
  while (got < size) {
    int remaining = -1;
    if (timeout_ms >= 0) {
      const auto spent = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - start)
                             .count();
      remaining = static_cast<int>(std::max<long long>(0, timeout_ms - spent));
    }
    std::size_t part = 0;
    const IoStatus status = receive_some(buffer + got, size - got, part, remaining);
    got += part;
    if (status == IoStatus::kClosed) {
      return IoStatus::kClosed;
    }
    if (status == IoStatus::kTimeout && remaining == 0) {
      return IoStatus::kTimeout;
    }
  }
  return IoStatus::kOk;
}

// TcpListener -------------------------------------------------------------------------------

TcpListener TcpListener::listen_loopback(std::uint16_t port, int backlog) {
  const Handle h = open_socket(SOCK_STREAM);
  bind_to(h, Address::loopback(port));
  if (::listen(native(h), backlog) != 0) {
    const int code = last_error();
    close_native(h);
    throw NetError("listen() failed", code);
  }
  return TcpListener(h);
}

std::optional<TcpStream> TcpListener::accept(int timeout_ms) {
  require_valid(handle(), "accept() on a closed socket");
  if (poll_handles({handle()}, timeout_ms).empty()) {
    return std::nullopt;
  }
  const NativeSocket s = ::accept(native(handle()), nullptr, nullptr);
  if (is_invalid(s)) {
    throw NetError("accept() failed", last_error());
  }
  const Handle h = widen(s);
  set_int_option(h, IPPROTO_TCP, TCP_NODELAY, 1);
  return TcpStream(h);
}

// wait_readable -----------------------------------------------------------------------------

std::vector<std::size_t> wait_readable(const std::vector<const Socket*>& sockets, int timeout_ms) {
  std::vector<Handle> handles;
  handles.reserve(sockets.size());
  for (const Socket* socket : sockets) {
    require_valid(socket->handle(), "wait_readable() on a closed socket");
    handles.push_back(socket->handle());
  }
  return poll_handles(handles, timeout_ms);
}

}  // namespace swarm::net
