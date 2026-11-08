// Prueba de la vision de tornillos SIN mover el robot.
//
//   ./vision_request_test x y z tx ty tz
//
// x y z tx ty tz = pose del TCP tal cual la muestra la app web (m, grados)
// en el momento de la foto. Manda la misma peticion que el paso de vision de
// la secuencia (mismo cliente, include/screw_vision_client.hpp) a
// screw_server y muestra la respuesta: a donde iria el robot en los pasos del
// tornillo (pre-grasp y agarre, con la orientacion de la vision).
//
// No se conecta al robot. screw_server debe estar corriendo:
//   cd ~/Escritorio/kinova_grasp_V2/build_local && ./screw_server ../cfg/gpd_params.cfg

#include "screw_vision_client.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char** argv)
{
    if (argc != 7)
    {
        std::cerr << "Uso: ./vision_request_test x y z tx ty tz   (pose del TCP de la app web, m y grados)\n"
                     "Ej.: ./vision_request_test 0.063 0.552 0.291 0 -180 0\n";
        return 2;
    }

    double pose[6];
    for (int i = 0; i < 6; ++i)
    {
        const std::string a = argv[1 + i];
        const auto r = std::from_chars(a.data(), a.data() + a.size(), pose[i]);
        if (r.ec != std::errc() || r.ptr != a.data() + a.size() || !std::isfinite(pose[i]))
        {
            std::cerr << "Numero invalido: " << a << "\n";
            return 2;
        }
    }

    try
    {
        screw_vision::Client client(5006);
        std::cout << "Buscando screw_server ...\n";
        client.ping(3.0);
        std::cout << "screw_server contesto. Pidiendo foto #1 (GPD puede tardar; cierra sus ventanas) ...\n";

        if (!client.request(1, pose))
        {
            std::cerr << "No se pudo mandar la peticion\n";
            return 1;
        }

        const auto t0 = std::chrono::steady_clock::now();
        while (client.last().seq != 1)
        {
            client.poll();
            if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(180))
            {
                std::cerr << "Sin respuesta en 180 s\n";
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        const screw_vision::Reply& r = client.last();
        if (!r.ok)
        {
            std::cout << "\nVISION FALLO: " << r.text << "\n";
            return 1;
        }
        std::printf("\nTornillo (TCP del mejor agarre, base): x %.4f  y %.4f  z %.4f m  "
                    "(score %.2f, %d agarres)\n",
                    r.tcp_m.x(), r.tcp_m.y(), r.tcp_m.z(), r.score, r.count);
        if (r.has_pose)
        {
            // Inclinacion del eje z de la pinza respecto a la vertical hacia abajo.
            constexpr double D2R = M_PI / 180.0;
            const Eigen::Vector3d z_tool =
                (Eigen::AngleAxisd(r.rpy_deg.z() * D2R, Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(r.rpy_deg.y() * D2R, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(r.rpy_deg.x() * D2R, Eigen::Vector3d::UnitX())) *
                Eigen::Vector3d::UnitZ();
            std::printf("Pre-grasp:          x %.4f  y %.4f  z %.4f m\n",
                        r.pregrasp_m.x(), r.pregrasp_m.y(), r.pregrasp_m.z());
            std::printf("Agarre:             x %.4f  y %.4f  z %.4f m\n",
                        r.tcp_m.x(), r.tcp_m.y(), r.tcp_m.z());
            std::printf("Orientacion (las dos): Rx %.1f  Ry %.1f  Rz %.1f grados, "
                        "inclinacion %.1f grados (limite 20)\n",
                        r.rpy_deg.x(), r.rpy_deg.y(), r.rpy_deg.z(),
                        std::acos(std::clamp(-z_tool.z(), -1.0, 1.0)) / D2R);
        }
        else
        {
            std::printf("El servidor no mando orientacion ni pre-grasp (servidor viejo de "
                        "kinova_grasp): la secuencia la rechaza.\n");
        }
        std::printf("Distancia horizontal a la pose de la foto: %.3f m (limite 0.20)\n",
                    std::hypot(r.tcp_m.x() - pose[0], r.tcp_m.y() - pose[1]));
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
}
