// Imprime en JSON la secuencia de tarea, las poses y los parametros
// principales de include/controller_config.hpp, para la interfaz grafica
// (gui/). No se conecta a nada: solo lee la configuracion compilada.
//
//   ./task_info            -> JSON en la salida estandar

#include "controller_config.hpp"
#include "task_sequence.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

namespace
{

std::string esc(const std::string& s)
{
    std::string o = "\"";
    for (const char c : s)
    {
        if (c == '"' || c == '\\')
            o += '\\';
        if (static_cast<unsigned char>(c) >= 0x20)
            o += c;
    }
    return o + "\"";
}

std::string num(double v)
{
    if (!std::isfinite(v))
        return "null";
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.6g", v);
    return buf;
}

template <typename V>
std::string arr(const V& v, int n)
{
    std::string o = "[";
    for (int i = 0; i < n; ++i)
        o += (i ? ", " : "") + num(v(i));
    return o + "]";
}

const char* typeName(task::StepType t)
{
    switch (t)
    {
        case task::StepType::Move:    return "move";
        case task::StepType::Gripper: return "gripper";
        case task::StepType::Fixture: return "fixture";
        case task::StepType::Wait:    return "wait";
        case task::StepType::Vision:  return "vision";
    }
    return "?";
}

const char* visionTargetName(task::VisionTarget t)
{
    switch (t)
    {
        case task::VisionTarget::PositionDz: return "position_dz";
        case task::VisionTarget::Pregrasp:   return "pregrasp";
        case task::VisionTarget::Grasp:      return "grasp";
        case task::VisionTarget::PlateFixture: return "plate_fixture";
        case task::VisionTarget::Hole:         return "hole";
    }
    return "?";
}

}  // namespace

int main()
{
    const project_config::ControllerConfig cfg{};
    const task::TaskParams& t = cfg.task;

    std::ostringstream o;
    o << "{\n";
    o << "  \"task_enabled\": " << (t.enabled ? "true" : "false") << ",\n";
    o << "  \"first_step\": " << t.first_step << ",\n";
    o << "  \"confirm_each_step\": " << (t.confirm_each_step ? "true" : "false") << ",\n";
    o << "  \"tcp_offset_m\": " << num(t.tcp_offset_m) << ",\n";
    o << "  \"workspace_min_m\": " << arr(t.workspace_min_m, 3) << ",\n";
    o << "  \"workspace_max_m\": " << arr(t.workspace_max_m, 3) << ",\n";
    if (t.low_zone_enabled)
    {
        o << "  \"low_zone_min_m\": " << arr(t.low_zone_min_m, 3) << ",\n";
        o << "  \"low_zone_max_m\": " << arr(t.low_zone_max_m, 3) << ",\n";
    }

    // Describir los pasos con el propio Sequencer (mismo texto que el tablero).
    // Si la configuracion no es valida, describe() no esta disponible.
    std::string error;
    try
    {
        task::TaskParams u = t;
        u.first_step = 1;
        task::Sequencer seq(u);
        o << "  \"steps\": [\n";
        for (size_t i = 0; i < t.steps.size(); ++i)
        {
            const task::Step& s = t.steps[i];
            o << "    {\"n\": " << i + 1 << ", \"type\": " << esc(typeName(s.type))
              << ", \"label\": " << esc(s.label) << ", \"pose\": " << esc(s.pose)
              << ", \"from_vision\": " << (s.from_vision ? "true" : "false")
              << ", \"vision_target\": " << esc(visionTargetName(s.vision_target))
              << ", \"vision_dz_m\": " << num(s.vision_dz_m)
              << ", \"vision_cloud_only\": " << (s.vision_cloud_only ? "true" : "false")
              << ", \"vision_needs_hole\": " << (s.vision_needs_hole ? "true" : "false")
              << ", \"vision_approach\": " << (s.vision_approach ? "true" : "false")
              << ", \"vision_object\": \"" << s.vision_object << "\""
              << ", \"gripper_pct\": " << num(s.gripper_pct)
              << ", \"fixture_deg\": " << num(s.fixture_deg)
              << ", \"wait_s\": " << num(s.wait_s)
              << ", \"describe\": " << esc(seq.describe(i)) << "}"
              << (i + 1 < t.steps.size() ? ",\n" : "\n");
        }
        o << "  ],\n";
    }
    catch (const std::exception& e)
    {
        error = e.what();
        o << "  \"steps\": [],\n";
    }
    o << "  \"error\": " << esc(error) << ",\n";

    o << "  \"poses\": [\n";
    for (size_t i = 0; i < t.poses.size(); ++i)
    {
        const task::TaskPose& p = t.poses[i];
        o << "    {\"name\": " << esc(p.name) << ", \"tcp_m\": " << arr(p.tcp_position_m, 3)
          << ", \"rpy_deg\": " << arr(p.rpy_deg, 3) << ", \"joints_deg\": "
          << (p.hasJoints() ? arr(p.joints_deg, 7) : std::string("null")) << "}"
          << (i + 1 < t.poses.size() ? ",\n" : "\n");
    }
    o << "  ],\n";

    o << "  \"params\": {\n"
      << "    \"stiffness\": " << arr(cfg.stiffness, 6) << ",\n"
      << "    \"damping_factors\": " << arr(cfg.damping_factors, 6) << ",\n"
      << "    \"pose_filter\": " << num(cfg.pose_filter) << ",\n"
      << "    \"reference_max_speed_m_s\": " << num(cfg.reference_max_speed_m_s) << ",\n"
      << "    \"reference_max_angular_speed_deg_s\": " << num(cfg.reference_max_angular_speed_deg_s) << ",\n"
      << "    \"enable_friction_compensation\": " << (cfg.enable_friction_compensation ? "true" : "false") << ",\n"
      << "    \"friction_sign_factor\": " << num(cfg.friction_sign_factor) << ",\n"
      << "    \"Kfric\": " << arr(cfg.Kfric, 7) << ",\n"
      << "    \"enable_nullspace\": " << (cfg.enable_nullspace ? "true" : "false") << ",\n"
      << "    \"nullspace_stiffness\": " << num(cfg.nullspace_stiffness) << ",\n"
      << "    \"nullspace_weights\": " << arr(cfg.nullspace_weights, 7) << ",\n"
      << "    \"enable_j4_limit\": " << (cfg.enable_j4_limit ? "true" : "false") << ",\n"
      << "    \"j4_min_deg\": " << num(cfg.j4_min_deg) << ",\n"
      << "    \"delta_tau_max\": " << num(cfg.delta_tau_max) << ",\n"
      << "    \"update_hz\": " << num(cfg.update_hz) << ",\n"
      << "    \"vision_max_tilt_deg\": " << num(t.vision_max_tilt_deg) << ",\n"
      << "    \"vision_max_xy_m\": " << num(t.vision_max_xy_m) << ",\n"
      << "    \"vision_min_below_m\": " << num(t.vision_min_below_m) << ",\n"
      << "    \"vision_max_pregrasp_m\": " << num(t.vision_max_pregrasp_m) << ",\n"
      << "    \"plate_nominal_tcp_above_edge_m\": " << num(t.plate_nominal_tcp_above_edge_m) << ",\n"
      << "    \"plate_max_skew_deg\": " << num(t.plate_max_skew_deg) << ",\n"
      << "    \"hole_max_shift_m\": " << num(t.hole_max_shift_m) << ",\n"
      << "    \"hole_max_from_photo_m\": " << num(t.hole_max_from_photo_m) << ",\n"
      << "    \"hole_offset_xy_m\": [" << num(t.hole_offset_xy_m.x()) << ", " << num(t.hole_offset_xy_m.y()) << "],\n"
      << "    \"position_tolerance_m\": " << num(t.position_tolerance_m) << ",\n"
      << "    \"orientation_tolerance_deg\": " << num(t.orientation_tolerance_deg) << ",\n"
      << "    \"max_move_distance_m\": " << num(t.max_move_distance_m) << "\n"
      << "  }\n";
    o << "}\n";

    std::cout << o.str();
    return 0;
}
