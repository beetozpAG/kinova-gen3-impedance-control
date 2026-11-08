#pragma once

// Cliente serial del FIXTURE giratorio.
//
// El fixture es un motor DC controlado por un ESP32 con el firmware
// control_posicion_adaptable_friccion.ino (control de posicion adaptable +
// friccion seca). El ESP32 va por USB a la misma PC, 115200 8N1, y recibe
// comandos de texto terminados en '\n':
//
//   STATUS        -> imprime el estado ("Modo         : DETENIDO", ...)
//   ZERO          -> q = 0 en la posicion actual (solo con el motor en STOP)
//   START         -> control de posicion, sostiene la posicion actual
//   POS <grados>  -> nuevo objetivo (rampa de 90 grados/s en el firmware)
//   STOP          -> motor libre
//
// El firmware NO avisa cuando llega al objetivo. Imprime una linea de
// monitor; se aceptan los dos formatos conocidos (cualquier orden de campos):
//   "qd:..,q:..,tau:..,I:..,phi:..,tf:.."   (.ino de Descargas, cada 50 ms, solo en POS)
//   "q:..,qd:..,e:..,i:.."                  (el cargado en el lab, cada 20 ms, tambien en STOP)
// (grados y mA); settled() la usa para decidir que ya llego. Como el monitor
// sale tambien en STOP, NO se usa para deducir el modo.
// Las fallas llegan como "*** FALLA: ..." y dejan el motor en STOP.
//
// Detalles practicos:
//   - Abrir el puerto suele REINICIAR el ESP32 (DTR/RTS). Al reiniciar, el
//     encoder vale 0 donde este el fixture: el fixture debe estar en su
//     posicion original al arrancar. Por eso el puerto se abre una sola vez.
//   - Los numeros se leen y escriben con from_chars/to_chars: no dependen del
//     locale (GTK pone el locale del sistema y con es_MX "%f" usa coma).
//   - poll() y moveTo() no bloquean (se pueden llamar en el lazo de 1 kHz).
//     connect(), zero() y start() esperan respuesta: llamarlos ANTES del lazo.
//   - El usuario necesita permiso para el puerto: grupo 'dialout'.

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <utility>

namespace fixture
{

struct FixtureParams
{
    // Puerto serial del ESP32 (CP2102 / CH340 -> /dev/ttyUSB0 normalmente).
    std::string port = "/dev/ttyUSB0";

    // Rango permitido del fixture [grados]. 0 = posicion original (colgado,
    // donde se queda con el motor apagado). El giro util es hacia NEGATIVO:
    // con este firmware +90 gira hacia el lado equivocado (probado en el lab
    // 2026-10-13).
    double min_deg = -90.0;
    double max_deg = 0.0;

    // Llegada: |q - objetivo| <= tolerance_deg durante settle_s seguidos.
    double tolerance_deg = 1.0;
    double settle_s = 0.30;

    // Una muestra del monitor mas vieja que esto ya no cuenta [s].
    double sample_timeout_s = 0.5;

    // Tiempo maximo para que el ESP32 conteste al conectar (incluye su
    // reinicio al abrir el puerto) [s].
    double connect_timeout_s = 6.0;

    // Tiempo maximo de respuesta a ZERO / START [s].
    double ack_timeout_s = 1.0;
};


enum class Mode
{
    Unknown,
    Off,
    Position,
    Current,
    Break
};

inline const char* modeName(Mode m)
{
    switch (m)
    {
        case Mode::Off:      return "DETENIDO";
        case Mode::Position: return "POSICION";
        case Mode::Current:  return "CORRIENTE";
        case Mode::Break:    return "BREAK";
        default:             return "DESCONOCIDO";
    }
}


// Una linea del monitor del firmware.
struct Sample
{
    bool valid = false;
    double qd_deg = 0.0;    // referencia (con rampa) [grados]
    double q_deg = 0.0;     // posicion medida [grados]
    double e_deg = 0.0;     // error que reporta el firmware (si lo manda) [grados]
    double tau_mA = 0.0;    // accion de control [mA]
    double i_mA = 0.0;      // corriente medida [mA]
    double phi_mA = 0.0;    // parametro adaptable [mA]
    double tf_mA = 0.0;     // compensacion de friccion [mA]
    std::chrono::steady_clock::time_point stamp{};
};


namespace detail
{

// Numero al inicio de 's' (salta espacios). false si no hay numero.
inline bool parseNumber(std::string_view s, double& out)
{
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t'))
        s.remove_prefix(1);
    if (!s.empty() && s.front() == '+')
        s.remove_prefix(1);
    double v = 0.0;
    const auto r = std::from_chars(s.data(), s.data() + s.size(), v);
    if (r.ec != std::errc() || !std::isfinite(v))
        return false;
    out = v;
    return true;
}

inline bool startsWith(std::string_view s, std::string_view prefix)
{
    return s.substr(0, prefix.size()) == prefix;
}

inline bool contains(std::string_view s, std::string_view part)
{
    return s.find(part) != std::string_view::npos;
}

// "qd:1.00,q:2.00,tau:3.0,I:4.0,phi:5.00,tf:6.0" o "q:2.00,qd:1.00,e:1.00,i:4.0"
// -> Sample. Exige q y qd; los demas campos son opcionales.
inline bool parseMonitor(std::string_view line, Sample& out)
{
    bool has_qd = false, has_q = false;
    Sample s;
    while (!line.empty())
    {
        const size_t comma = line.find(',');
        const std::string_view token = line.substr(0, comma);
        line = (comma == std::string_view::npos) ? std::string_view{} : line.substr(comma + 1);

        const size_t colon = token.find(':');
        if (colon == std::string_view::npos)
            return false;
        const std::string_view key = token.substr(0, colon);
        double v = 0.0;
        if (!parseNumber(token.substr(colon + 1), v))
            return false;

        if (key == "qd")       { s.qd_deg = v; has_qd = true; }
        else if (key == "q")   { s.q_deg = v; has_q = true; }
        else if (key == "tau") s.tau_mA = v;
        else if (key == "I" || key == "i") s.i_mA = v;
        else if (key == "e")   s.e_deg = v;
        else if (key == "phi") s.phi_mA = v;
        else if (key == "tf")  s.tf_mA = v;
    }
    if (!has_qd || !has_q)
        return false;
    s.valid = true;
    out = s;
    return true;
}

}  // namespace detail


class FixtureClient
{
public:
    using Clock = std::chrono::steady_clock;

    // Abre y configura el puerto. Lanza std::runtime_error si no puede.
    explicit FixtureClient(FixtureParams params)
        : p_(std::move(params))
    {
        if (!(p_.min_deg < p_.max_deg) || !std::isfinite(p_.min_deg) || !std::isfinite(p_.max_deg))
            throw std::runtime_error("fixture: rango min_deg/max_deg invalido");

        fd_ = ::open(p_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0)
        {
            const int err = errno;
            std::string msg = "fixture: no se pudo abrir " + p_.port + " (" + std::strerror(err) + ")";
            if (err == EACCES)
                msg += ". Agrega tu usuario al grupo dialout: sudo usermod -aG dialout $USER "
                       "y vuelve a iniciar sesion";
            throw std::runtime_error(msg);
        }

        termios tty{};
        if (::tcgetattr(fd_, &tty) != 0)
        {
            ::close(fd_);
            throw std::runtime_error("fixture: " + p_.port + " no es un puerto serial");
        }
        ::cfmakeraw(&tty);
        // 921600: el firmware del ESP32 se recargo con Serial.begin(921600)
        // (2026-10-19; antes 115200).
        ::cfsetispeed(&tty, B921600);
        ::cfsetospeed(&tty, B921600);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;
        if (::tcsetattr(fd_, TCSANOW, &tty) != 0)
        {
            ::close(fd_);
            throw std::runtime_error("fixture: no se pudo configurar " + p_.port);
        }
        ::tcflush(fd_, TCIOFLUSH);
    }

    // Deja el motor libre (STOP) y cierra el puerto.
    ~FixtureClient()
    {
        if (fd_ >= 0)
        {
            stop();
            ::tcdrain(fd_);
            ::close(fd_);
        }
    }

    FixtureClient(const FixtureClient&) = delete;
    FixtureClient& operator=(const FixtureClient&) = delete;

    // Opcional: recibe cada linea que llega del ESP32 (para imprimir).
    std::function<void(const std::string&)> on_line;


    // ---------------- No bloqueantes ----------------

    // Lee lo que haya llegado y actualiza el estado. Nunca espera.
    void poll()
    {
        char buf[256];
        while (true)
        {
            const ssize_t n = ::read(fd_, buf, sizeof(buf));
            if (n <= 0)
                break;   // 0 / EAGAIN: no hay mas datos
            for (ssize_t k = 0; k < n; ++k)
            {
                const char c = buf[k];
                if (c == '\n')
                {
                    handleLine(partial_);
                    partial_.clear();
                }
                else if (c != '\r')
                {
                    if (partial_.size() < 512)   // linea absurda: se trunca
                        partial_.push_back(c);
                }
            }
        }
    }

    // Manda POS <deg>. Lanza si el objetivo esta fuera de rango o el fixture
    // no esta en modo posicion. La llegada se revisa con settled().
    void moveTo(double deg)
    {
        if (!std::isfinite(deg) || deg < p_.min_deg || deg > p_.max_deg)
            throw std::runtime_error("fixture: objetivo " + std::to_string(deg) +
                                     " grados fuera del rango [" + std::to_string(p_.min_deg) +
                                     ", " + std::to_string(p_.max_deg) + "]");
        if (mode_ != Mode::Position)
            throw std::runtime_error(std::string("fixture: moveTo requiere modo POSICION (modo actual: ") +
                                     modeName(mode_) + ")");

        char num[32];
        const auto r = std::to_chars(num, num + sizeof(num), deg, std::chars_format::fixed, 2);
        writeLine("POS " + std::string(num, r.ptr));

        target_deg_ = deg;
        target_acked_ = false;
        move_pending_ = true;
        in_tolerance_ = false;
    }

    // true si: modo posicion, el firmware confirmo el objetivo (linea
    // "Objetivo = X", o el qd del monitor llego a X despues del POS), hay
    // muestras recientes y q y qd llevan settle_s dentro de la tolerancia.
    bool settled() const
    {
        if (mode_ != Mode::Position || !target_acked_ || !sampleFresh() || !in_tolerance_)
            return false;
        return std::chrono::duration<double>(sample_.stamp - in_tolerance_since_).count() >= p_.settle_s;
    }

    bool sampleFresh() const
    {
        return sample_.valid &&
               std::chrono::duration<double>(Clock::now() - sample_.stamp).count() <= p_.sample_timeout_s;
    }

    Mode mode() const { return mode_; }
    const Sample& sample() const { return sample_; }
    double target() const { return target_deg_; }
    bool hasFault() const { return fault_; }
    const std::string& faultText() const { return fault_text_; }
    const FixtureParams& params() const { return p_; }


    // ---------------- Bloqueantes (antes del lazo de control) ----------------

    // Pide STATUS hasta que el ESP32 conteste. Lanza si no contesta o si el
    // firmware reporta un error de arranque (p. ej. INA226).
    void connect()
    {
        const auto deadline = Clock::now() + seconds(p_.connect_timeout_s);
        auto next_status = Clock::now();
        status_seen_ = false;

        while (Clock::now() < deadline)
        {
            if (Clock::now() >= next_status)
            {
                writeLine("STATUS");   // se repite: el primero se pierde si el ESP32 reinicia
                next_status = Clock::now() + std::chrono::milliseconds(1000);
            }
            poll();
            if (!boot_error_.empty())
                throw std::runtime_error("fixture: el ESP32 reporta: " + boot_error_);
            if (status_seen_)
            {
                // Deja llegar el resto del bloque STATUS (q, ganancias, ...).
                pollFor(0.2);
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        throw std::runtime_error("fixture: el ESP32 no contesto en " + p_.port +
                                 " (revisa el cable, el puerto y que el firmware este cargado)");
    }

    // q = 0 en la posicion actual. Solo con el motor en STOP.
    void zero()
    {
        if (mode_ != Mode::Off)
            throw std::runtime_error("fixture: ZERO solo con el motor en STOP");
        command("ZERO", "q = 0 en esta posici");
    }

    // Control de posicion sosteniendo la posicion actual. Borra la falla.
    void start()
    {
        fault_ = false;
        fault_text_.clear();
        command("START", "Control de posici");
        mode_ = Mode::Position;
        target_deg_ = std::numeric_limits<double>::quiet_NaN();
        target_acked_ = false;
        move_pending_ = false;
        in_tolerance_ = false;
    }

    // Motor libre. No lanza (se usa en el destructor y al abortar).
    void stop() noexcept
    {
        try
        {
            writeLine("STOP");
        }
        catch (...)
        {
        }
        mode_ = Mode::Off;
        target_acked_ = false;
        move_pending_ = false;
        in_tolerance_ = false;
    }

    // Lee durante 'seconds' segundos (para esperas cortas fuera del lazo).
    void pollFor(double secs)
    {
        const auto end = Clock::now() + seconds(secs);
        while (Clock::now() < end)
        {
            poll();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

private:
    static Clock::duration seconds(double s)
    {
        return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s));
    }

    void writeLine(const std::string& cmd)
    {
        const std::string data = cmd + "\n";
        size_t sent = 0;
        const auto deadline = Clock::now() + std::chrono::milliseconds(100);
        while (sent < data.size())
        {
            const ssize_t n = ::write(fd_, data.data() + sent, data.size() - sent);
            if (n > 0)
            {
                sent += static_cast<size_t>(n);
                continue;
            }
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
                throw std::runtime_error(std::string("fixture: error al escribir (") + std::strerror(errno) + ")");
            if (Clock::now() > deadline)
                throw std::runtime_error("fixture: no se pudo escribir el comando " + cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // Manda 'cmd' y espera una linea que contenga 'ack'. Lanza si el
    // firmware lo rechaza, si hay falla o si no contesta a tiempo.
    void command(const std::string& cmd, std::string_view ack)
    {
        poll();   // lo viejo se procesa antes de esperar la respuesta
        const unsigned long first = lines_total_;
        writeLine(cmd);

        const auto deadline = Clock::now() + seconds(p_.ack_timeout_s);
        while (Clock::now() < deadline)
        {
            poll();
            for (const auto& [index, line] : recent_)
            {
                if (index < first)
                    continue;
                if (detail::contains(line, ack))
                    return;
                if (isRejection(line))
                    throw std::runtime_error("fixture: " + cmd + " rechazado: " + line);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        throw std::runtime_error("fixture: sin respuesta a " + cmd);
    }

    static bool isRejection(std::string_view line)
    {
        return detail::startsWith(line, "Primero ") ||
               detail::startsWith(line, "Comando no reconocido") ||
               detail::startsWith(line, "Captura en curso") ||
               detail::startsWith(line, "*** FALLA");
    }

    void handleLine(const std::string& line)
    {
        const std::string_view s = line;
        double v = 0.0;

        Sample smp;
        if ((detail::startsWith(s, "qd:") || detail::startsWith(s, "q:")) &&
            detail::parseMonitor(s, smp))
        {
            smp.stamp = Clock::now();
            sample_ = smp;
            // El qd del firmware solo va hacia el objetivo si acepto el POS.
            if (move_pending_ && std::isfinite(target_deg_) &&
                std::abs(smp.qd_deg - target_deg_) <= 0.05)
            {
                target_acked_ = true;
                move_pending_ = false;
            }
            updateTolerance();
            return;   // las lineas del monitor no van a on_line ni a recent_
        }

        if (on_line)
            on_line(line);

        recent_.emplace_back(lines_total_++, line);
        if (recent_.size() > 64)
            recent_.pop_front();

        // POS rechazado por el firmware (no hay espera bloqueante): falla.
        if (move_pending_ && !target_acked_ && isRejection(s) &&
            !detail::startsWith(s, "*** FALLA"))
        {
            fault_ = true;
            fault_text_ = "POS rechazado por el ESP32: " + line;
            move_pending_ = false;
            return;
        }

        if (detail::startsWith(s, "Modo"))
        {
            const size_t colon = s.find(':');
            if (colon != std::string_view::npos)
            {
                const std::string_view m = s.substr(colon + 1);
                if (detail::contains(m, "DETENIDO"))       mode_ = Mode::Off;
                else if (detail::contains(m, "POSICI"))    mode_ = Mode::Position;
                else if (detail::contains(m, "CORRIENTE")) mode_ = Mode::Current;
                else if (detail::contains(m, "BREAK"))     mode_ = Mode::Break;
                status_seen_ = true;
            }
        }
        else if (detail::startsWith(s, "Objetivo = "))
        {
            if (detail::parseNumber(s.substr(11), v) && std::abs(v - target_deg_) < 0.01)
                target_acked_ = true;
            if (target_acked_)
                move_pending_ = false;
        }
        else if (s == "Detenido.")
        {
            mode_ = Mode::Off;
        }
        else if (detail::startsWith(s, "Control de posici"))
        {
            mode_ = Mode::Position;
        }
        else if (detail::startsWith(s, "*** FALLA"))
        {
            fault_ = true;
            fault_text_ = line;
            mode_ = Mode::Off;
            target_acked_ = false;
            move_pending_ = false;
            in_tolerance_ = false;
        }
        else if (detail::startsWith(s, "ERROR:"))
        {
            boot_error_ = line;
        }
    }

    void updateTolerance()
    {
        const bool ok =
            std::isfinite(target_deg_) &&
            std::abs(sample_.q_deg - target_deg_) <= p_.tolerance_deg &&
            std::abs(sample_.qd_deg - target_deg_) <= 0.05;   // la rampa ya termino

        if (ok && !in_tolerance_)
            in_tolerance_since_ = sample_.stamp;
        in_tolerance_ = ok;
    }

    FixtureParams p_;
    int fd_ = -1;
    std::string partial_;

    std::deque<std::pair<unsigned long, std::string>> recent_;
    unsigned long lines_total_ = 0;

    Mode mode_ = Mode::Unknown;
    Sample sample_{};
    bool status_seen_ = false;
    std::string boot_error_;

    double target_deg_ = std::numeric_limits<double>::quiet_NaN();
    bool target_acked_ = false;
    bool move_pending_ = false;   // POS mandado, sin confirmar todavia
    bool in_tolerance_ = false;
    Clock::time_point in_tolerance_since_{};

    bool fault_ = false;
    std::string fault_text_;
};

}  // namespace fixture
