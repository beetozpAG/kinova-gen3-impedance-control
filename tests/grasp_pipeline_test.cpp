// Pruebas sin robot ni camara de la integracion con la vision (GPD).

#include "grasp_message.hpp"
#include "grasp_pipeline.hpp"
#include "grasp_receiver.hpp"

#include <arpa/inet.h>
#include <cmath>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace grasp;

namespace
{
int fails = 0;

void check(bool ok, const std::string& name)
{
    std::cout << (ok ? "  OK     " : "  FALLO  ") << name << '\n';
    if (!ok)
        ++fails;
}

GraspMessage goodGrasp()
{
    GraspMessage m;
    m.frame = 0;
    m.seq = 7;
    m.pos_m[2] = 0.40f;                          // 40 cm al frente [m]
    m.approach[2] = 1.0f;                         // approach = +z camara
    m.binormal[0] = 1.0f;                         // binormal = +x
    m.axis[1] = 1.0f;                             // axis = +y
    m.score = 2.5f;
    m.width_m = 0.040f;
    return m;
}

bool near(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol = 1e-6)
{
    return (a - b).norm() < tol;
}

void sendRaw(int port, const void* data, size_t size)
{
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(static_cast<uint16_t>(port));
    ::inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr);
    ::sendto(fd, data, size, 0, reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    ::close(fd);
}
}  // namespace

int main()
{
    const VisionParams vp;

    std::cout << "1) Mensaje\n";
    {
        const GraspMessage m = goodGrasp();
        const auto b = encodeGrasp(m);
        GraspMessage d;
        check(b.size() == 76, "tamano fijo de 76 bytes");
        check(decodeGrasp(b.data(), b.size(), d) && d.seq == 7 && d.pos_m[2] == 0.40f &&
              d.score == 2.5f && d.axis[1] == 1.0f, "codificar y decodificar da lo mismo");
        check(!decodeGrasp(b.data(), 75, d), "rechaza tamano incorrecto");
        auto bad = b;
        bad[0] = 'X';
        check(!decodeGrasp(bad.data(), bad.size(), d), "rechaza magic incorrecto");
        auto v1 = b;
        v1[4] = 1; v1[5] = 0;                       // version 1 = formato viejo en mm
        check(!decodeGrasp(v1.data(), v1.size(), d), "rechaza version 1 (mm)");
    }

    std::cout << "2) Validaciones\n";
    {
        check(validateGrasp(goodGrasp(), vp).empty(), "agarre correcto se acepta");
        auto m = goodGrasp(); m.pos_m[0] = std::nanf("");
        check(!validateGrasp(m, vp).empty(), "rechaza NaN");
        m = goodGrasp(); m.approach[2] = 2.0f;
        check(!validateGrasp(m, vp).empty(), "rechaza eje no unitario");
        m = goodGrasp(); m.axis[1] = -1.0f;
        check(!validateGrasp(m, vp).empty(), "rechaza frame izquierdo");
        m = goodGrasp(); m.score = -1.0f;
        check(!validateGrasp(m, vp).empty(), "rechaza score bajo");
        m = goodGrasp(); m.pos_m[2] = -0.10f;
        check(!validateGrasp(m, vp).empty(), "rechaza agarre detras de la camara");
        m = goodGrasp(); m.pos_m[2] = 2.0f;
        check(!validateGrasp(m, vp).empty(), "rechaza agarre a 2 m");
        m = goodGrasp(); m.frame = 5;
        check(!validateGrasp(m, vp).empty(), "rechaza frame desconocido");
    }

    std::cout << "3) Transformaciones\n";
    {
        // Interfaz en el origen de la base, sin rotacion. Camara de color.
        const GraspTargets t = computeTargets(goodGrasp(), Pose{}, vp);
        const Eigen::Vector3d g(0.0, 0.05639, 0.4 - 0.00305);   // 40 cm + offset de la camara
        check(near(t.grasp_base.p, g), "metros y offset de la camara de color");
        check(near(t.interface_grasp.R.col(2), Eigen::Vector3d::UnitZ()), "tool z = approach");
        check(near(t.interface_grasp.p, g - 0.12 * Eigen::Vector3d::UnitZ()), "interfaz 12 cm antes del agarre");
        check(near(t.interface_pregrasp.p, g - 0.22 * Eigen::Vector3d::UnitZ()), "pre-agarre 10 cm mas atras");
        check(std::abs(t.interface_grasp.R.determinant() - 1.0) < 1e-9, "rotacion de la tool valida (det = 1)");

        // Camara de profundidad: offset distinto.
        auto md = goodGrasp(); md.frame = 1;
        const GraspTargets td = computeTargets(md, Pose{}, vp);
        check(near(td.grasp_base.p, Eigen::Vector3d(0.0275, 0.066, 0.4 - 0.00305)), "offset de la camara de profundidad");

        // Interfaz trasladada y apuntando hacia abajo (180 grados en X).
        Pose iface;
        iface.p = Eigen::Vector3d(0.5, 0.1, 0.6);
        iface.R = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
        const GraspTargets tr = computeTargets(goodGrasp(), iface, vp);
        const Eigen::Vector3d expected = iface.p + iface.R * (vp.color_offset_m + Eigen::Vector3d(0, 0, 0.4));
        check(near(tr.grasp_base.p, expected), "cadena base <- interfaz <- camara <- agarre");
        check(near(tr.interface_grasp.R.col(2), -Eigen::Vector3d::UnitZ()), "con la interfaz hacia abajo, el approach apunta a -Z de la base");
        check(near(tr.interface_grasp.p + 0.12 * tr.interface_grasp.R.col(2), tr.grasp_base.p),
              "interfaz + 12 cm sobre la tool = punto de agarre");
    }

    std::cout << "4) Receptor UDP (127.0.0.1)\n";
    {
        const int port = 50557;
        GraspReceiver rx(port);
        GraspMessage got;
        check(!rx.poll(got), "sin datos regresa de inmediato");

        auto m1 = goodGrasp(); m1.seq = 1;
        auto m2 = goodGrasp(); m2.seq = 2;
        const auto b1 = encodeGrasp(m1), b2 = encodeGrasp(m2);
        sendRaw(port, b1.data(), b1.size());
        sendRaw(port, "basura", 6);
        sendRaw(port, b2.data(), b2.size());
        ::usleep(20000);
        check(rx.poll(got) && got.seq == 2, "recibe y se queda con el ultimo");
        check(rx.badPackets() == 1, "cuenta el paquete invalido");
        check(!rx.poll(got), "despues de leer queda vacio");
    }

    std::cout << (fails ? "\nHAY FALLOS\n" : "\nTODAS LAS PRUEBAS DE VISION: OK\n");
    return fails ? 1 : 0;
}
