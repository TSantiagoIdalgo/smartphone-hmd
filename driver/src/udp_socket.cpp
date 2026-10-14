#include "udp_socket.h"

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
using SocketHandle = SOCKET;
static const SocketHandle kInvalid = INVALID_SOCKET;
static void CloseSocket(SocketHandle s) { closesocket(s); }
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
using SocketHandle = int;
static const SocketHandle kInvalid = -1;
static void CloseSocket(SocketHandle s) { close(s); }
#endif

UdpSocket::UdpSocket() : socket_(kInvalid)
{
#if defined(_WIN32)
    WSADATA data;
    WSAStartup(MAKEWORD(2, 2), &data);
#endif
}

UdpSocket::~UdpSocket()
{
    Close();
#if defined(_WIN32)
    WSACleanup();
#endif
}

bool UdpSocket::Open(uint16_t port, int timeout_ms)
{
    Close();
    SocketHandle s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == kInvalid) {
        return false;
    }

#if defined(_WIN32)
    DWORD timeout = static_cast<DWORD>(timeout_ms);
#else
    timeval timeout{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
#endif
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(port);
    if (bind(s, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
        CloseSocket(s);
        return false;
    }
    socket_ = s;
    return true;
}

bool UdpSocket::OpenForSending()
{
    Close();
    SocketHandle s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == kInvalid) {
        return false;
    }
    socket_ = s;
    return true;
}

void UdpSocket::Close()
{
    if (socket_ != kInvalid) {
        CloseSocket(static_cast<SocketHandle>(socket_));
        socket_ = kInvalid;
    }
}

int UdpSocket::Receive(uint8_t* buffer, size_t size, uint32_t* sender_ipv4, uint16_t* sender_port)
{
    if (socket_ == kInvalid) {
        return -1;
    }
    sockaddr_in sender{};
    socklen_t sender_size = sizeof(sender);
    const auto received = recvfrom(static_cast<SocketHandle>(socket_), reinterpret_cast<char*>(buffer),
                                   static_cast<int>(size), 0, reinterpret_cast<sockaddr*>(&sender), &sender_size);
    if (received < 0) {
        return -1;
    }
    if (sender_ipv4) {
        *sender_ipv4 = sender.sin_addr.s_addr;
    }
    if (sender_port) {
        *sender_port = ntohs(sender.sin_port);
    }
    return static_cast<int>(received);
}

bool UdpSocket::SendTo(uint32_t ipv4, uint16_t port, const uint8_t* data, size_t size)
{
    if (socket_ == kInvalid) {
        return false;
    }
    sockaddr_in target{};
    target.sin_family = AF_INET;
    target.sin_addr.s_addr = ipv4;
    target.sin_port = htons(port);
    const auto sent = sendto(static_cast<SocketHandle>(socket_), reinterpret_cast<const char*>(data),
                             static_cast<int>(size), 0, reinterpret_cast<const sockaddr*>(&target), sizeof(target));
    return sent >= 0 && static_cast<size_t>(sent) == size;
}
