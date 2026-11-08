// Emisor de prueba de agarres (sin camara ni GPD).
// Tambien sirve de EJEMPLO para el programa de vision: armar un GraspMessage,
// codificarlo con encodeGrasp() y enviarlo por UDP.
//
// Uso:
//   ./grasp_test_sender [x_m y_m z_m] [score] [frame] [host] [puerto]   (METROS)
//   frame: 0 = color, 1 = profundidad
// Por defecto: agarre 0.40 m al frente de la camara de color, approach = +z
// de la camara, score 1, enviado a 127.0.0.1:5005.

#include "grasp_message.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

int main(int argc, char** argv)
{
    grasp::GraspMessage m;
    m.pos_m[0] = argc > 3 ? std::strtof(argv[1], nullptr) : 0.0f;
    m.pos_m[1] = argc > 3 ? std::strtof(argv[2], nullptr) : 0.0f;
    m.pos_m[2] = argc > 3 ? std::strtof(argv[3], nullptr) : 0.40f;
    m.score     = argc > 4 ? std::strtof(argv[4], nullptr) : 1.0f;
    m.frame     = argc > 5 ? static_cast<uint16_t>(std::atoi(argv[5])) : 0;
    const char* host = argc > 6 ? argv[6] : "127.0.0.1";
    const int port   = argc > 7 ? std::atoi(argv[7]) : 5005;

    // Mano de GPD apuntando hacia el frente de la camara:
    // approach = +z, binormal = +x, axis = approach x binormal = +y
    const float approach[3] = {0, 0, 1}, binormal[3] = {1, 0, 0}, axis[3] = {0, 1, 0};
    for (int i = 0; i < 3; ++i)
    {
        m.approach[i] = approach[i];
        m.binormal[i] = binormal[i];
        m.axis[i] = axis[i];
    }
    m.width_m = 0.040f;
    m.seq = static_cast<uint32_t>(std::chrono::steady_clock::now().time_since_epoch().count() & 0xffffffff);
    m.stamp_ns = static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());

    const auto buf = grasp::encodeGrasp(m);

    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, host, &dst.sin_addr) != 1)
    {
        std::fprintf(stderr, "Host invalido: %s\n", host);
        return 1;
    }
    const ssize_t n = ::sendto(fd, buf.data(), buf.size(), 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    ::close(fd);
    if (n != static_cast<ssize_t>(buf.size()))
    {
        std::perror("sendto");
        return 1;
    }
    std::printf("Enviado a %s:%d  pos=(%.3f, %.3f, %.3f) m  score=%.2f  frame=%u  seq=%u\n",
                host, port, m.pos_m[0], m.pos_m[1], m.pos_m[2], m.score, m.frame, m.seq);
    return 0;
}
