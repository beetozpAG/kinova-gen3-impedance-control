#pragma once

// Mensaje UDP de VISION -> CONTROL con un agarre de GPD.
//
// Este archivo lo comparten los dos programas:
//   - vision (GPD):     encodeGrasp() + sendto()
//   - control (Kinova): decodeGrasp() al recibir
//
// Formato fijo de 76 bytes, little-endian (PC x86), sin padding.
// Version 2: posicion y apertura en METROS (version 1 usaba mm y se rechaza).
//
//   off  tipo      campo
//   0    uint32    magic     = 'GRSP'
//   4    uint16    version   = 2
//   6    uint16    frame     0 = sensor de COLOR, 1 = sensor de PROFUNDIDAD
//   8    uint32    seq       numero de deteccion (creciente)
//   12   uint64    stamp_ns  reloj del emisor al tomar la nube (informativo)
//   20   float[3]  pos_m     posicion del agarre en el frame de la camara [m]
//   32   float[3]  approach  eje de aproximacion de GPD (unitario, frame camara)
//   44   float[3]  binormal  eje de cierre de los dedos de GPD (unitario)
//   56   float[3]  axis      eje de la mano de GPD (unitario)
//   68   float     score     puntaje de GPD
//   72   float     width_m   apertura sugerida de la pinza [m]
//
// Los tres ejes forman la matriz de rotacion de la mano de GPD en el frame
// de la camara: R = [approach | binormal | axis] (columnas).

#include <array>
#include <cstdint>
#include <cstring>

namespace grasp
{

constexpr uint32_t GRASP_MAGIC = 0x50535247u;   // bytes 'G' 'R' 'S' 'P'
constexpr uint16_t GRASP_VERSION = 2;   // 2 = metros
constexpr size_t GRASP_MESSAGE_SIZE = 76;

enum class CameraFrame : uint16_t
{
    Color = 0,
    Depth = 1
};

struct GraspMessage
{
    uint16_t frame = 0;
    uint32_t seq = 0;
    uint64_t stamp_ns = 0;
    float pos_m[3] = {};      // [m]
    float approach[3] = {};
    float binormal[3] = {};
    float axis[3] = {};
    float score = 0.0f;
    float width_m = 0.0f;     // [m]
};

using GraspBuffer = std::array<unsigned char, GRASP_MESSAGE_SIZE>;

namespace detail
{
template <typename T>
void put(GraspBuffer& b, size_t off, const T& v) { std::memcpy(b.data() + off, &v, sizeof(T)); }

template <typename T>
void get(const unsigned char* b, size_t off, T& v) { std::memcpy(&v, b + off, sizeof(T)); }
}  // namespace detail

inline GraspBuffer encodeGrasp(const GraspMessage& m)
{
    GraspBuffer b{};
    detail::put(b, 0, GRASP_MAGIC);
    detail::put(b, 4, GRASP_VERSION);
    detail::put(b, 6, m.frame);
    detail::put(b, 8, m.seq);
    detail::put(b, 12, m.stamp_ns);
    for (int i = 0; i < 3; ++i)
    {
        detail::put(b, 20 + 4 * i, m.pos_m[i]);
        detail::put(b, 32 + 4 * i, m.approach[i]);
        detail::put(b, 44 + 4 * i, m.binormal[i]);
        detail::put(b, 56 + 4 * i, m.axis[i]);
    }
    detail::put(b, 68, m.score);
    detail::put(b, 72, m.width_m);
    return b;
}

// Devuelve false si el tamano, magic o version no coinciden.
inline bool decodeGrasp(const unsigned char* data, size_t size, GraspMessage& m)
{
    if (size != GRASP_MESSAGE_SIZE)
        return false;

    uint32_t magic = 0;
    uint16_t version = 0;
    detail::get(data, 0, magic);
    detail::get(data, 4, version);
    if (magic != GRASP_MAGIC || version != GRASP_VERSION)
        return false;

    detail::get(data, 6, m.frame);
    detail::get(data, 8, m.seq);
    detail::get(data, 12, m.stamp_ns);
    for (int i = 0; i < 3; ++i)
    {
        detail::get(data, 20 + 4 * i, m.pos_m[i]);
        detail::get(data, 32 + 4 * i, m.approach[i]);
        detail::get(data, 44 + 4 * i, m.binormal[i]);
        detail::get(data, 56 + 4 * i, m.axis[i]);
    }
    detail::get(data, 68, m.score);
    detail::get(data, 72, m.width_m);
    return true;
}

}  // namespace grasp
