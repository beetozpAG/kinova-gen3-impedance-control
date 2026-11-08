// Prueba del FIXTURE sin el robot (Fase 1).
//
//   ./fixture_test [puerto] [angulo]        por defecto /dev/ttyUSB0 y -90
//   (el giro util es hacia negativo: rango -90 a 0, ver FixtureParams)
//
// Secuencia:
//   1. Abre el puerto y espera a que el ESP32 conteste (STATUS).
//   2. Pide confirmar que el fixture esta en su posicion ORIGINAL.
//   3. STOP -> ZERO -> START (sostiene la posicion).
//   4. Gira a <angulo>, espera a que llegue, lo sostiene 2 s.
//   5. Regresa a 0, espera a que llegue.
//   6. STOP (motor libre) y cierra el puerto.
// Ctrl+C en cualquier momento: STOP y salir.

#include "fixture_client.hpp"

#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>

namespace
{

volatile std::sig_atomic_t stop_requested = 0;

void signalHandler(int)
{
    stop_requested = 1;
}

using Clock = std::chrono::steady_clock;

double secondsSince(Clock::time_point t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

// Espera a que el fixture llegue. false si hubo falla, timeout o Ctrl+C.
bool waitSettled(fixture::FixtureClient& fx, double timeout_s)
{
    const auto t0 = Clock::now();
    double last_print = -1.0;

    while (!stop_requested)
    {
        fx.poll();

        if (fx.hasFault())
        {
            std::cout << "\n  " << fx.faultText() << "\n";
            return false;
        }

        const double t = secondsSince(t0);

        if (fx.settled())
        {
            std::printf("\n  Llego en %.2f s: q = %.2f grados (objetivo %.2f, error %.2f)\n",
                        t, fx.sample().q_deg, fx.target(), fx.sample().q_deg - fx.target());
            return true;
        }

        if (t - last_print >= 0.25)
        {
            last_print = t;
            if (fx.sampleFresh())
                std::printf("\r  t = %4.1f s   qd = %7.2f   q = %7.2f   I = %6.1f mA   ",
                            t, fx.sample().qd_deg, fx.sample().q_deg, fx.sample().i_mA);
            else
                std::printf("\r  t = %4.1f s   (sin datos del monitor)                       ", t);
            std::fflush(stdout);
        }

        if (t > timeout_s)
        {
            std::cout << "\n  TIMEOUT: no llego en " << timeout_s << " s";
            if (!fx.sampleFresh())
                std::cout << " (no llegan lineas del monitor: si se mando MON, mandalo otra vez)";
            std::cout << "\n";
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
}

void holdFor(fixture::FixtureClient& fx, double secs)
{
    const auto t0 = Clock::now();
    while (!stop_requested && !fx.hasFault() && secondsSince(t0) < secs)
    {
        fx.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

}  // namespace


int main(int argc, char** argv)
{
    // Sin SA_RESTART: Ctrl+C tambien interrumpe la espera del teclado.
    struct sigaction sa{};
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    fixture::FixtureParams params;
    if (argc > 1)
        params.port = argv[1];

    double angle = -90.0;
    if (argc > 2)
    {
        const std::string a = argv[2];
        const auto r = std::from_chars(a.data(), a.data() + a.size(), angle);
        if (r.ec != std::errc() || r.ptr != a.data() + a.size() || !std::isfinite(angle) ||
            angle < params.min_deg || angle > params.max_deg)
        {
            std::cerr << "Angulo invalido: " << a << " (rango " << params.min_deg << " a "
                      << params.max_deg << " grados)\n";
            return 2;
        }
    }

    // Tiempo maximo por movimiento: rampa de 90 grados/s + margen.
    const double move_timeout_s = std::abs(angle) / 90.0 + 8.0;

    try
    {
        std::cout << "Abriendo " << params.port << " ...\n";
        fixture::FixtureClient fx(params);
        fx.on_line = [](const std::string& line) {
            if (!line.empty())
                std::cout << "  [ESP32] " << line << "\n";
        };

        std::cout << "Esperando al ESP32 (puede reiniciarse al abrir el puerto) ...\n";
        fx.connect();
        std::cout << "\nESP32 conectado. Modo: " << fixture::modeName(fx.mode()) << "\n";
        fx.on_line = nullptr;   // de aqui en adelante solo lo importante

        std::cout << "\nEl fixture debe estar en su POSICION ORIGINAL (0 grados) y sin nada\n"
                     "que estorbe el giro. Se va a girar a "
                  << angle << " grados y regresar.\n"
                     "Escribe s y Enter para continuar: ";
        std::string answer;
        if (!std::getline(std::cin, answer) || stop_requested || (answer != "s" && answer != "S"))
        {
            std::cout << "Cancelado.\n";
            return 0;
        }

        // Todo lo que no es monitor (respuestas a POS, fallas, ...).
        fx.on_line = [](const std::string& line) {
            if (!line.empty())
                std::cout << "\n  [ESP32] " << line << "\n";
        };

        fx.stop();
        fx.pollFor(0.3);
        fx.zero();
        std::cout << "ZERO: q = 0 aqui.\n";
        fx.start();
        std::cout << "START: control de posicion activo.\n";
        holdFor(fx, 0.5);   // deja llegar las primeras lineas del monitor

        bool ok = true;

        if (!stop_requested && !fx.hasFault())
        {
            std::cout << "\nGirando a " << angle << " grados ...\n";
            fx.moveTo(angle);
            ok = waitSettled(fx, move_timeout_s);
        }

        if (ok && !stop_requested)
        {
            std::cout << "Sosteniendo 2 s ...\n";
            holdFor(fx, 2.0);
            if (fx.hasFault())
            {
                std::cout << "  " << fx.faultText() << "\n";
                ok = false;
            }
        }

        // Regresa a 0 aunque el giro no haya terminado (salvo falla o Ctrl+C).
        if (!stop_requested && !fx.hasFault() && fx.mode() == fixture::Mode::Position)
        {
            std::cout << "\nRegresando a 0 grados ...\n";
            fx.moveTo(0.0);
            ok = waitSettled(fx, move_timeout_s) && ok;
        }

        if (stop_requested)
            std::cout << "\nCtrl+C: deteniendo el motor.\n";

        fx.stop();
        fx.pollFor(0.2);
        std::cout << "STOP: motor libre.\n\n"
                  << (ok && !stop_requested ? "PRUEBA OK\n" : "PRUEBA CON PROBLEMAS (ver arriba)\n");
        return (ok && !stop_requested) ? 0 : 1;
    }
    catch (const std::exception& e)
    {
        // El destructor de FixtureClient ya mando STOP.
        std::cerr << "\nERROR: " << e.what() << "\n";
        return 1;
    }
}
