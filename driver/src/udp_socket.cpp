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

void UdpSocket::Close()
{
    if (socket_ != kInvalid) {
        CloseSocket(static_cast<SocketHandle>(socket_));
        socket_ = kInvalid;
    }
}

int UdpSocket::Receive(uint8_t* buffer, size_t size)
{
    if (socket_ == kInvalid) {
        return -1;
    }
    const auto received = recv(static_cast<SocketHandle>(socket_), reinterpret_cast<char*>(buffer), static_cast<int>(size), 0);
    return received < 0 ? -1 : static_cast<int>(received);
}
