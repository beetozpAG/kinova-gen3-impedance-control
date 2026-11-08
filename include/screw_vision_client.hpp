#pragma once

// Cliente del servidor de vision de tornillos (screw_server.cpp de
// kinova_grasp_V2, o el viejo de kinova_grasp).
//
// Protocolo: texto, un datagrama UDP por mensaje, en localhost.
//   control -> vision:  "PING"                         -> "PONG"
//   control -> vision:  "CAPTURE seq x y z tx ty tz"   pose del TCP (del modelo)
//                                                      (m, grados x-y-z extrinsecos)
//   vision -> control (mejor agarre de GPD, TCP en la base):
//     V2:    "SCREW seq OK x y z tx ty tz px py pz score n"
//            agarre (m), su orientacion (grados x-y-z extrinsecos) y
//            pre-grasp (m, misma orientacion)
//            + " EDGE ax ay az bx by bz": extremos del canto de la placa en
//              el agarre (m, base), o " NOEDGE" (2026-10-20)
//     viejo: "SCREW seq OK x y z score n"   solo la posicion del agarre (m)
//            "SCREW seq FAIL motivo"
//   control -> vision:  "APPROACH seq x y z tx ty tz PLATE|BOLT"   (2026-10-27)
//   vision -> control:  "APPROACH seq OK ESTADO x y z tx ty tz cx cy cz n"
//                       ESTADO MOVE / INVIEW / ATTARGET; pose del EE (TCP) para
//                       que la camara de color quede 30 cm arriba del objeto
//                       (approach_object de Antony) y centroide del objeto
//                       "APPROACH seq FAIL motivo"
//   CAPTURE ... PLATE|BOLT: placa (gpd_plate_params, color de la placa) o perno
//                       (gpd_params, color del perno), como grasps_from_camera --color --filter
//   control -> vision:  "CLOUD seq x y z tx ty tz"     solo la nube, sin GPD (2026-10-20)
//   vision -> control:  "CLOUD seq OK nombre n"        guardo nombre.pcd y nombre_pose.txt
//                                                      (n puntos)
//                       + " HOLE x y z d k": centro del hoyo MAS CHICO de la
//                         placa (base, m), su diametro (m) y k hoyos
//                         encontrados, o " NOHOLE" (2026-10-21)
//                       "CLOUD seq FAIL motivo"
// El servidor contesta a la direccion de donde llego la peticion.
//
// Los numeros se escriben y leen con to_chars/from_chars: no dependen del
// locale (GTK pone el del sistema y con es_MX "%f" usa coma).
//
// poll() y request() no bloquean (se llaman en el lazo de 1 kHz).
// ping() bloquea: llamarlo ANTES del lazo.

#include <Eigen/Dense>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace screw_vision
{

struct Reply
{
    uint32_t seq = 0;      // 0 = todavia no hay respuesta
    bool ok = false;
    Eigen::Vector3d tcp_m = Eigen::Vector3d::Zero();
    bool has_pose = false;   // V2: orientacion y pre-grasp
    Eigen::Vector3d rpy_deg = Eigen::Vector3d::Zero();
    Eigen::Vector3d pregrasp_m = Eigen::Vector3d::Zero();
    double score = 0.0;
    int count = 0;         // agarres que encontro GPD
    // Canto de la placa en el agarre (2026-10-20): los dos extremos en la
    // base [m]. has_edge = false con "NOEDGE" o con un servidor viejo.
    bool has_edge = false;
    Eigen::Vector3d edge_a_m = Eigen::Vector3d::Zero();
    Eigen::Vector3d edge_b_m = Eigen::Vector3d::Zero();
    // Respuesta CLOUD: solo nube; text = nombre del archivo, count = puntos.
    bool is_cloud = false;
    // CLOUD con HOLE: hoyo mas chico de la placa (base [m]) y su diametro [m].
    bool has_hole = false;
    Eigen::Vector3d hole_m = Eigen::Vector3d::Zero();
    double hole_diam_m = 0.0;
    int hole_count = 0;
    // Respuesta APPROACH: tcp_m y rpy_deg = pose objetivo del EE, centroid_m =
    // objeto (base [m]), count = puntos del objeto.
    bool is_approach = false;
    int approach_state = 0;   // 1 MOVE, 2 INVIEW, 3 ATTARGET
    Eigen::Vector3d centroid_m = Eigen::Vector3d::Zero();
    std::string text;      // motivo de la falla
};

namespace detail
{

inline std::vector<std::string_view> split(std::string_view s)
{
    std::vector<std::string_view> out;
    size_t i = 0;
    while (i < s.size())
    {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n'))
            ++i;
        const size_t start = i;
        while (i < s.size() && !(s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n'))
            ++i;
        if (i > start)
            out.push_back(s.substr(start, i - start));
    }
    return out;
}

template <typename T>
bool number(std::string_view s, T& out)
{
    const auto r = std::from_chars(s.data(), s.data() + s.size(), out);
    return r.ec == std::errc() && r.ptr == s.data() + s.size();
}

inline void append(std::string& out, double v)
{
    char buf[32];
    const auto r = std::to_chars(buf, buf + sizeof(buf), v, std::chars_format::fixed, 5);
    out.append(buf, r.ptr);
}

}  // namespace detail

// "CAPTURE seq x y z tx ty tz [OBJ]" (o "CLOUD ..." con cloud_only, o
// "APPROACH ... OBJ" con approach). object: "PLATE", "BOLT" o "" (sin objeto:
// tornillos sin color, como antes). El servidor de kinova_grasp_NEW_VER usa
// la configuracion de GPD y el color de ese objeto (2026-10-26/27).
inline std::string formatRequest(uint32_t seq, const double tool_pose[6], bool cloud_only = false,
                                 const std::string& object = "", bool approach = false)
{
    std::string s = (approach ? "APPROACH " : cloud_only ? "CLOUD " : "CAPTURE ") + std::to_string(seq);
    for (int i = 0; i < 6; ++i)
    {
        s += ' ';
        detail::append(s, tool_pose[i]);
    }
    if (!object.empty() && !cloud_only)
        s += " " + object;
    return s;
}

// false si el texto no es una respuesta valida.
inline bool parseReply(std::string_view text, Reply& out)
{
    const auto t = detail::split(text);
    if (t.size() < 3 || (t[0] != "SCREW" && t[0] != "CLOUD" && t[0] != "APPROACH"))
        return false;

    Reply r;
    r.is_cloud = (t[0] == "CLOUD");
    r.is_approach = (t[0] == "APPROACH");
    if (!detail::number(t[1], r.seq) || r.seq == 0)
        return false;

    if (t[2] == "OK" && r.is_approach)
    {
        if (t.size() != 14)
            return false;
        r.approach_state = t[3] == "MOVE" ? 1 : t[3] == "INVIEW" ? 2 : t[3] == "ATTARGET" ? 3 : 0;
        if (r.approach_state == 0)
            return false;
        double v[9] = {};
        for (size_t i = 0; i < 9; ++i)
            if (!detail::number(t[4 + i], v[i]) || !std::isfinite(v[i]))
                return false;
        if (!detail::number(t[13], r.count))
            return false;
        r.tcp_m = Eigen::Vector3d(v[0], v[1], v[2]);
        r.rpy_deg = Eigen::Vector3d(v[3], v[4], v[5]);
        r.centroid_m = Eigen::Vector3d(v[6], v[7], v[8]);
        r.text = std::string(t[3]);
        r.ok = true;
    }
    else if (t[2] == "OK" && r.is_cloud)
    {
        // 5: sin hoyos (servidor del 2026-10-20); 6: + NOHOLE; 11: + HOLE y 5 numeros
        if ((t.size() != 5 && t.size() != 6 && t.size() != 11) || !detail::number(t[4], r.count))
            return false;
        if (t.size() == 6 && t[5] != "NOHOLE")
            return false;
        if (t.size() == 11)
        {
            double h[4] = {};
            if (t[5] != "HOLE")
                return false;
            for (size_t i = 0; i < 4; ++i)
                if (!detail::number(t[6 + i], h[i]) || !std::isfinite(h[i]))
                    return false;
            if (!detail::number(t[10], r.hole_count))
                return false;
            r.has_hole = true;
            r.hole_m = Eigen::Vector3d(h[0], h[1], h[2]);
            r.hole_diam_m = h[3];
        }
        r.text = std::string(t[3]);
        r.ok = true;
    }
    else if (t[2] == "OK")
    {
        // 8: servidor viejo; 14: con pose; 15: + NOEDGE; 21: + EDGE y 6 numeros
        if (t.size() != 8 && t.size() != 14 && t.size() != 15 && t.size() != 21)
            return false;
        if (t.size() == 15 && t[14] != "NOEDGE")
            return false;
        if (t.size() == 21 && t[14] != "EDGE")
            return false;
        r.has_pose = (t.size() >= 14);
        const size_t nv = r.has_pose ? 9 : 3;
        double v[9] = {};
        for (size_t i = 0; i < nv; ++i)
            if (!detail::number(t[3 + i], v[i]) || !std::isfinite(v[i]))
                return false;
        if (!detail::number(t[3 + nv], r.score) || !detail::number(t[4 + nv], r.count))
            return false;
        r.tcp_m = Eigen::Vector3d(v[0], v[1], v[2]);
        if (r.has_pose)
        {
            r.rpy_deg = Eigen::Vector3d(v[3], v[4], v[5]);
            r.pregrasp_m = Eigen::Vector3d(v[6], v[7], v[8]);
        }
        if (t.size() == 21)
        {
            double e[6] = {};
            for (size_t i = 0; i < 6; ++i)
                if (!detail::number(t[15 + i], e[i]) || !std::isfinite(e[i]))
                    return false;
            r.has_edge = true;
            r.edge_a_m = Eigen::Vector3d(e[0], e[1], e[2]);
            r.edge_b_m = Eigen::Vector3d(e[3], e[4], e[5]);
        }
        r.ok = true;
    }
    else if (t[2] == "FAIL")
    {
        const size_t pos = text.find("FAIL");
        r.text = std::string(text.substr(pos + 4));
        while (!r.text.empty() && (r.text.front() == ' ' || r.text.front() == '\t'))
            r.text.erase(r.text.begin());
        while (!r.text.empty() && (r.text.back() == '\n' || r.text.back() == '\r'))
            r.text.pop_back();
        if (r.text.empty())
            r.text = "sin motivo";
    }
    else
    {
        return false;
    }
    out = r;
    return true;
}


class Client
{
public:
    explicit Client(int server_port, const std::string& server_ip = "127.0.0.1")
    {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0)
            throw std::runtime_error(std::string("vision: socket: ") + std::strerror(errno));

        // Puerto local cualquiera en localhost: el servidor contesta ahi.
        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_port = 0;
        local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (::bind(fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0 ||
            ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL, 0) | O_NONBLOCK) != 0)
        {
            const int err = errno;
            ::close(fd_);
            throw std::runtime_error(std::string("vision: bind: ") + std::strerror(err));
        }

        server_.sin_family = AF_INET;
        server_.sin_port = htons(static_cast<uint16_t>(server_port));
        if (::inet_pton(AF_INET, server_ip.c_str(), &server_.sin_addr) != 1)
        {
            ::close(fd_);
            throw std::runtime_error("vision: IP invalida: " + server_ip);
        }
    }

    ~Client()
    {
        if (fd_ >= 0)
            ::close(fd_);
    }

    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;

    // Espera "PONG" hasta timeout_s. Lanza si el servidor no contesta.
    void ping(double timeout_s)
    {
        using Clock = std::chrono::steady_clock;
        const auto deadline = Clock::now() + std::chrono::duration<double>(timeout_s);
        auto next_send = Clock::now();
        while (Clock::now() < deadline)
        {
            if (Clock::now() >= next_send)
            {
                send("PING");
                next_send = Clock::now() + std::chrono::milliseconds(500);
            }
            std::string msg;
            while (receive(msg))
                if (detail::split(msg) == std::vector<std::string_view>{"PONG"})
                    return;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        throw std::runtime_error(
            "vision: el servidor no contesta en el puerto " +
            std::to_string(ntohs(server_.sin_port)) +
            " (corre en otra terminal, desde kinova_grasp_V2/build_local:  ./screw_server ../cfg/gpd_params.cfg)");
    }

    // Pide una foto (cloud_only: solo la nube, sin GPD). No bloquea. false si
    // no se pudo mandar.
    bool request(uint32_t seq, const double tool_pose[6], bool cloud_only = false,
                 const std::string& object = "", bool approach = false)
    {
        return send(formatRequest(seq, tool_pose, cloud_only, object, approach));
    }

    // Lee las respuestas que hayan llegado. No bloquea.
    void poll()
    {
        std::string msg;
        while (receive(msg))
        {
            Reply r;
            if (parseReply(msg, r))
                last_ = r;
        }
    }

    const Reply& last() const { return last_; }

private:
    bool send(const std::string& text)
    {
        const ssize_t n = ::sendto(fd_, text.data(), text.size(), 0,
                                   reinterpret_cast<const sockaddr*>(&server_), sizeof(server_));
        return n == static_cast<ssize_t>(text.size());
    }

    bool receive(std::string& out)
    {
        char buf[512];
        const ssize_t n = ::recv(fd_, buf, sizeof(buf), 0);
        if (n <= 0)
            return false;   // nada (EAGAIN) o error (p. ej. ECONNREFUSED): se ignora
        out.assign(buf, static_cast<size_t>(n));
        return true;
    }

    int fd_ = -1;
    sockaddr_in server_{};
    Reply last_{};
};

}  // namespace screw_vision
