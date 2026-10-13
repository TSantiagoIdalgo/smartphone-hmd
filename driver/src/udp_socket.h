#pragma once

#include <cstddef>
#include <cstdint>

// Minimal UDP receiver (Winsock on Windows, BSD sockets elsewhere).
class UdpSocket {
public:
    UdpSocket();
    ~UdpSocket();
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    // Listens on all interfaces. Receive() waits at most timeout_ms.
    bool Open(uint16_t port, int timeout_ms);
    void Close();

    // Returns the number of bytes received, or -1 on timeout or error.
    int Receive(uint8_t* buffer, size_t size);

private:
#if defined(_WIN32)
    uintptr_t socket_;
#else
    int socket_;
#endif
};
