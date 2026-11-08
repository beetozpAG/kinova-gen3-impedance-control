#pragma once

// Receptor UDP NO BLOQUEANTE de agarres (grasp_message.hpp).
// poll() se llama en cada ciclo del lazo de control: si no hay datos
// regresa de inmediato. Si llegaron varios, se queda con el ultimo.

#include "grasp_message.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace grasp
{

class GraspReceiver
{
public:
    explicit GraspReceiver(int port)
    {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0)
            throw std::runtime_error("GraspReceiver: no se pudo crear el socket");

        const int flags = ::fcntl(fd_, F_GETFL, 0);
        ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(static_cast<uint16_t>(port));
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            const std::string err = std::strerror(errno);
            ::close(fd_);
            throw std::runtime_error("GraspReceiver: no se pudo abrir el puerto UDP " +
                                     std::to_string(port) + " (" + err + ")");
        }
    }

    ~GraspReceiver()
    {
        if (fd_ >= 0)
            ::close(fd_);
    }

    GraspReceiver(const GraspReceiver&) = delete;
    GraspReceiver& operator=(const GraspReceiver&) = delete;

    // true si llego al menos un mensaje valido en formato; 'out' = el ultimo.
    bool poll(GraspMessage& out)
    {
        bool got = false;
        unsigned char buf[512];
        while (true)
        {
            const ssize_t n = ::recv(fd_, buf, sizeof(buf), MSG_DONTWAIT);
            if (n < 0)
                break;   // EAGAIN: no hay mas datos
            GraspMessage m;
            if (decodeGrasp(buf, static_cast<size_t>(n), m))
            {
                out = m;
                got = true;
            }
            else
            {
                ++bad_packets_;
            }
        }
        return got;
    }

    // Paquetes descartados por tamano/magic/version incorrectos.
    unsigned long badPackets() const { return bad_packets_; }

private:
    int fd_ = -1;
    unsigned long bad_packets_ = 0;
};

}  // namespace grasp
