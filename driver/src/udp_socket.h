#pragma once

#include <cstddef>
#include <cstdint>

// Minimal UDP socket (Winsock on Windows, BSD sockets elsewhere). IPv4 addresses are in network
// byte order.
class UdpSocket {
public:
    UdpSocket();
    ~UdpSocket();
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    // Listens on all interfaces. Receive() waits at most timeout_ms.
    bool Open(uint16_t port, int timeout_ms);
    // Opens a socket that only sends.
    bool OpenForSending();
    void Close();

    // Returns the number of bytes received, or -1 on timeout or error. The sender outputs may be null.
    int Receive(uint8_t* buffer, size_t size, uint32_t* sender_ipv4 = nullptr, uint16_t* sender_port = nullptr);

    bool SendTo(uint32_t ipv4, uint16_t port, const uint8_t* data, size_t size);

private:
#if defined(_WIN32)
    uintptr_t socket_;
#else
    int socket_;
#endif
};
