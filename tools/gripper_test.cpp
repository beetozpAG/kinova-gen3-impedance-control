// Prueba de la pinza Robotiq 2F-85 (Fase 1).
//
//   ./gripper_test
//
// El BRAZO NO SE MUEVE: los 7 actuadores se ponen en modo POSICION (el modo
// en que Kinova los deja al iniciar y al terminar cualquier programa) y se
// les manda siempre la q leida al arrancar. No se usa modo corriente ni el
// modelo de gravedad. Si alguna articulacion se aleja mas de 3 grados de
// esa q, el programa se detiene.
//
// Comandos (escribir y Enter):
//   a          abrir   (posicion 0 %)
//   c          cerrar  (posicion 100 %)
//   p <0-100>  ir a una posicion [%]
//   v <1-100>  velocidad para los siguientes movimientos [%] (inicial 50)
//   f <0-100>  fuerza [%] (inicial: la del wrapper, 1 como en el laboratorio)
//   s          estado
//   q          salir (tambien Ctrl+C)

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include "Matrix.h"
#include "kinova_robot.h"

using namespace matrix;
using kinova_robot::KinovaRobotLowLevel;

namespace
{

constexpr const char* ROBOT_IP = "192.168.1.10";
constexpr const char* USERNAME = "admin";
constexpr const char* PASSWORD = "admin";

constexpr double MAX_JOINT_DEVIATION_RAD = 3.0 * 3.14159265358979323846 / 180.0;

std::atomic_bool stop_requested{false};

void signalHandler(int)
{
    stop_requested = true;
}

// Comandos (teclado -> lazo) y estado (lazo -> teclado).
struct Shared
{
    std::mutex m;
    float pos_cmd = 0.0f;
    float vel_cmd = 0.0f;     // 0 = no mover hasta el primer comando
    float force_cmd = 1.0f;
    bool cmd_pending = false;

    float pos = 0.0f;
    float vel = 0.0f;
    bool detected = false;
    double max_dev_deg = 0.0;
    std::string error;
};

Shared shared;
std::atomic_bool loop_ready{false};
std::atomic_bool loop_done{false};

// Diferencia angular en (-pi, pi].
double angleDiff(double a, double b)
{
    return std::remainder(a - b, 2.0 * 3.14159265358979323846);
}

void controlLoop()
{
    try
    {
        std::cout << "Conectando al Kinova Gen3 en " << ROBOT_IP << " ...\n";
        auto robot = std::make_unique<KinovaRobotLowLevel>(ROBOT_IP, USERNAME, PASSWORD);

        const vec<7> q_hold = robot->getPos();
        for (int i = 0; i < 7; ++i)
        {
            if (!std::isfinite(q_hold[i][0]))
                throw std::runtime_error("q inicial no finita");
        }

        using enum kinova_robot::ControlMode;
        robot->setControlMode({POS, POS, POS, POS, POS, POS, POS});

        {
            std::lock_guard<std::mutex> lock(shared.m);
            shared.detected = robot->gripperDetected;
            shared.pos = robot->gripperPos[0][0];
            shared.pos_cmd = robot->gripperPos[0][0];
            shared.force_cmd = robot->gripperForceCommand;
        }

        if (!robot->gripperDetected)
            throw std::runtime_error("el robot no reporta ninguna pinza (revisa la conexion "
                                     "de la Robotiq y que aparezca en la app web)");

        loop_ready = true;

        while (!stop_requested && !robot->timeout)
        {
            {
                std::lock_guard<std::mutex> lock(shared.m);
                if (shared.cmd_pending)
                {
                    robot->gripperPosCommand[0][0] = shared.pos_cmd;
                    robot->gripperVelCommand[0][0] = shared.vel_cmd;
                    robot->gripperForceCommand = shared.force_cmd;
                    shared.cmd_pending = false;
                }
            }

            robot->sendPos(q_hold);   // espera el siguiente ciclo de 1 ms

            const vec<7> q = robot->getPos();
            double dev = 0.0;
            for (int i = 0; i < 7; ++i)
            {
                if (!std::isfinite(q[i][0]))
                    throw std::runtime_error("q no finita");
                dev = std::max(dev, std::abs(angleDiff(q[i][0], q_hold[i][0])));
            }

            {
                std::lock_guard<std::mutex> lock(shared.m);
                shared.pos = robot->gripperPos[0][0];
                shared.vel = robot->gripperVel[0][0];
                shared.max_dev_deg = std::max(shared.max_dev_deg, dev * 180.0 / 3.14159265358979323846);
            }

            if (dev > MAX_JOINT_DEVIATION_RAD)
                throw std::runtime_error("una articulacion se alejo mas de 3 grados de la posicion inicial");
        }

        if (robot->timeout)
        {
            std::lock_guard<std::mutex> lock(shared.m);
            shared.error = "timeout de comunicacion con el robot";
        }
        // El destructor deja los actuadores en POSICION y sale del modo de bajo nivel.
    }
    catch (const std::exception& e)
    {
        std::lock_guard<std::mutex> lock(shared.m);
        shared.error = e.what();
    }
    catch (...)
    {
        std::lock_guard<std::mutex> lock(shared.m);
        shared.error = "error desconocido";
    }

    stop_requested = true;
    loop_done = true;
}

void printStatus()
{
    std::lock_guard<std::mutex> lock(shared.m);
    std::printf("  Pinza: posicion %.1f %%  velocidad %.1f %%  | comando: pos %.1f %%, vel %.1f %%, "
                "fuerza %.1f %% | desviacion max del brazo %.2f grados\n",
                shared.pos, shared.vel, shared.pos_cmd, shared.vel_cmd, shared.force_cmd,
                shared.max_dev_deg);
}

// Espera a que la pinza termine de moverse (o 4 s) y reporta.
void waitMotion()
{
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));   // que arranque
    int quiet = 0;
    while (!stop_requested && std::chrono::duration<double>(Clock::now() - t0).count() < 4.0)
    {
        float v = 0.0f;
        {
            std::lock_guard<std::mutex> lock(shared.m);
            v = shared.vel;
        }
        quiet = (std::abs(v) < 0.5f) ? quiet + 1 : 0;
        if (quiet >= 10)   // 0.2 s quieta
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    printStatus();
    std::lock_guard<std::mutex> lock(shared.m);
    if (std::abs(shared.pos - shared.pos_cmd) > 3.0f)
        std::cout << "  (no llego al comando: si hay un objeto entre los dedos es lo esperado)\n";
}

bool parseValue(std::istringstream& in, float lo, float hi, float& out)
{
    float v = 0.0f;
    if (!(in >> v) || !std::isfinite(v) || v < lo || v > hi)
    {
        std::cout << "  Valor invalido (rango " << lo << " a " << hi << ")\n";
        return false;
    }
    out = v;
    return true;
}

}  // namespace


int main()
{
    // Sin SA_RESTART: Ctrl+C interrumpe la lectura del teclado.
    struct sigaction sa{};
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    std::cout << "PRUEBA DE PINZA - el brazo queda quieto en modo posicion.\n"
                 "Ten el paro de emergencia a la mano.\n\n";

    std::thread loop(controlLoop);

    while (!loop_ready && !loop_done)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

    float velocity = 50.0f;

    if (loop_ready && !stop_requested)
    {
        std::cout << "\nPinza detectada. Comandos: a (abrir), c (cerrar), p <0-100>, "
                     "v <1-100>, f <0-100>, s, q\n";
        printStatus();

        std::string line;
        while (!stop_requested)
        {
            std::cout << "> " << std::flush;
            if (!std::getline(std::cin, line))
                break;   // Ctrl+D o Ctrl+C
            if (stop_requested)
                break;

            std::istringstream in(line);
            std::string cmd;
            if (!(in >> cmd))
                continue;

            float value = 0.0f;
            bool move = false;

            if (cmd == "a")      { value = 0.0f; move = true; }
            else if (cmd == "c") { value = 100.0f; move = true; }
            else if (cmd == "p") { move = parseValue(in, 0.0f, 100.0f, value); }
            else if (cmd == "v")
            {
                if (parseValue(in, 1.0f, 100.0f, value))
                {
                    velocity = value;
                    std::cout << "  Velocidad = " << velocity << " %\n";
                }
            }
            else if (cmd == "f")
            {
                if (parseValue(in, 0.0f, 100.0f, value))
                {
                    std::lock_guard<std::mutex> lock(shared.m);
                    shared.force_cmd = value;
                    shared.cmd_pending = true;
                    std::cout << "  Fuerza = " << value << " %\n";
                }
            }
            else if (cmd == "s") { printStatus(); }
            else if (cmd == "q") { break; }
            else { std::cout << "  Comando no reconocido\n"; }

            if (move)
            {
                {
                    std::lock_guard<std::mutex> lock(shared.m);
                    shared.pos_cmd = value;
                    shared.vel_cmd = velocity;
                    shared.cmd_pending = true;
                }
                std::cout << "  Moviendo a " << value << " % ...\n";
                waitMotion();
            }
        }
    }

    stop_requested = true;
    loop.join();

    std::lock_guard<std::mutex> lock(shared.m);
    if (!shared.error.empty())
    {
        std::cerr << "\nERROR: " << shared.error << "\n";
        return 1;
    }
    std::cout << "\nFin de la prueba.\n";
    return 0;
}
