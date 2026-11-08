// Kinova Gen3 (7 DoF) model: gravity, friction, forward kinematics, pose, Jacobian.
// Copy of KinovaOpenCVCPP2026/Gen3_model.h. Only GetTransform (position),
// GetPose and GetJacobian were changed or commented (see the comments there).
// Manual: Kinova Gen3 user guide r12.0 (Documents/EN-UG-014-Gen3-user-guide-r12.0.pdf).

#include <algorithm>
#include <cmath>

// Kinova Headers
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>

// Mini matrices library
#include "Robot/Matrix.h"

namespace KinovaGen3
{
    using namespace matrix;

    // ----------- GEN3 --- GEAR RATIO AND TORQUE CONSTANTS --------------------------
    // GearRatio/TorqueConstant = Gr/Kt : LARGE actuator (A1, A2, A3, A4)
    const float GroKt_large = 1 / (100 * 0.1124);
    // GearRatio/TorqueConstant = Gr/Kt : SMALL actuator (A5, A6, A7)
    const float GroKt_small = 1 / (100 * 0.0764);
    const mat<7, 1> KTGR = {GroKt_large, GroKt_large, GroKt_large, GroKt_large, GroKt_small, GroKt_small, GroKt_small};
    const mat <7, 1> maxCurrent = {5.0, 5.0, 5.0, 5.0, 3.0, 3.0, 3.0};
    const mat<7,1> maxTau = maxCurrent.hadamard(1/KTGR)*0.9f;
    //----------------------------------------------------------------------
    // ---------- GEN 3 ----- FRICTION COMPENSATION PARAMETERS -----------------------
    const mat<7, 1> sigma0 = {800, 800, 800, 800, 800, 800, 800};
    const mat<7, 1> sigma1 = {48, 60, 48, 48, 28, 28, 28};
    const mat<7, 1> FC = {6.3, 6.8, 6.3, 5.4, 2.6, 2.6, 2.2}; // Coulomb Friction (a0)
    const mat<7, 1> FS = {7.0, 7.8, 7.0, 6.6, 3.8, 3.8, 3.8}; // Stiction (a0 + a1)
    const mat<7, 1> v0 = {0.020, 0.020, 0.020, 0.020, 0.018, 0.010, 0.013};               // Stribeck Velocity (vs)
    // ------------------------------------------------------------------------------                    
  
    // ----------- * GEN 3 * GRAVITY VECTOR EXPRESSION ------------------------------
    mat<7, 1> GravityVector(const mat<7, 1> &q)
    {
        mat<7, 1> g = {};

        // Precalculate sin(q) and cos(q)
        float s2 = sin(q[1][0]);
        float s3 = sin(q[2][0]);
        float s4 = sin(q[3][0]);
        float s5 = sin(q[4][0]);
        float s6 = sin(q[5][0]);
        float s7 = sin(q[6][0]);
        float c2 = cos(q[1][0]);
        float c3 = cos(q[2][0]);
        float c4 = cos(q[3][0]);
        float c5 = cos(q[4][0]);
        float c6 = cos(q[5][0]);
        float c7 = cos(q[6][0]);

        g[0][0] = 0;
        g[1][0] = 0.00050174432 * c2 - 20.166219 * s2 + 0.00050174432 * c2 * c3 + 0.46771198 * c2 * s3 - 8.9622775 * c4 * s2 - 0.000164052 * s2 * s4 + 0.0000066444 * c6 * (c5 * (s2 * s4 - 1.0 * c2 * c3 * c4) + c2 * s3 * s5) + 3.0195944 * s6 * (c5 * (s2 * s4 - 1.0 * c2 * c3 * c4) + c2 * s3 * s5) - 3.0195944 * c6 * (c4 * s2 + c2 * c3 * s4) - 0.0013769 * c7 * (c6 * (c5 * (s2 * s4 - 1.0 * c2 * c3 * c4) + c2 * s3 * s5) + s6 * (c4 * s2 + c2 * c3 * s4)) + 0.0000066444 * s6 * (c4 * s2 + c2 * c3 * s4) + 0.0558698 * c7 * (s5 * (s2 * s4 - 1.0 * c2 * c3 * c4) - 1.0 * c2 * c5 * s3) + 0.0558698 * s7 * (c6 * (c5 * (s2 * s4 - 1.0 * c2 * c3 * c4) + c2 * s3 * s5) + s6 * (c4 * s2 + c2 * c3 * s4)) + 0.0013769 * s7 * (s5 * (s2 * s4 - 1.0 * c2 * c3 * c4) - 1.0 * c2 * c5 * s3) + 0.0000066444 * c5 * (s2 * s4 - 1.0 * c2 * c3 * c4) - 0.0014484792 * s5 * (s2 * s4 - 1.0 * c2 * c3 * c4) + 0.000164052 * c2 * c3 * c4 - 8.9622775 * c2 * c3 * s4 + 0.0014484792 * c2 * c5 * s3 + 0.0000066444 * c2 * s3 * s5;
        g[2][0] = 0.46771198 * c3 * s2 - 0.0013769 * c7 * (c6 * (c3 * s2 * s5 + c4 * c5 * s2 * s3) - 1.0 * s2 * s3 * s4 * s6) - 0.00050174432 * s2 * s3 + 0.0558698 * s7 * (c6 * (c3 * s2 * s5 + c4 * c5 * s2 * s3) - 1.0 * s2 * s3 * s4 * s6) + 0.0000066444 * c6 * (c3 * s2 * s5 + c4 * c5 * s2 * s3) + 3.0195944 * s6 * (c3 * s2 * s5 + c4 * c5 * s2 * s3) - 0.0558698 * c7 * (c3 * c5 * s2 - 1.0 * c4 * s2 * s3 * s5) - 0.0013769 * s7 * (c3 * c5 * s2 - 1.0 * c4 * s2 * s3 * s5) + 8.9622775 * s2 * s3 * s4 + 0.0014484792 * c3 * c5 * s2 - 0.000164052 * c4 * s2 * s3 + 0.0000066444 * c3 * s2 * s5 + 0.0000066444 * c4 * c5 * s2 * s3 - 0.0014484792 * c4 * s2 * s3 * s5 + 3.0195944 * c6 * s2 * s3 * s4 - 0.0000066444 * s2 * s3 * s4 * s6;
        g[3][0] = 0.000164052 * c2 * c4 - 8.9622775 * c2 * s4 - 0.0013769 * c7 * (s6 * (c2 * s4 + c3 * c4 * s2) - 1.0 * c5 * c6 * (c2 * c4 - 1.0 * c3 * s2 * s4)) + 0.0558698 * s7 * (s6 * (c2 * s4 + c3 * c4 * s2) - 1.0 * c5 * c6 * (c2 * c4 - 1.0 * c3 * s2 * s4)) - 3.0195944 * c6 * (c2 * s4 + c3 * c4 * s2) + 0.0000066444 * s6 * (c2 * s4 + c3 * c4 * s2) - 0.0000066444 * c5 * (c2 * c4 - 1.0 * c3 * s2 * s4) + 0.0014484792 * s5 * (c2 * c4 - 1.0 * c3 * s2 * s4) - 0.0000066444 * c5 * c6 * (c2 * c4 - 1.0 * c3 * s2 * s4) - 3.0195944 * c5 * s6 * (c2 * c4 - 1.0 * c3 * s2 * s4) - 0.0558698 * c7 * s5 * (c2 * c4 - 1.0 * c3 * s2 * s4) - 0.0013769 * s5 * s7 * (c2 * c4 - 1.0 * c3 * s2 * s4) - 8.9622775 * c3 * c4 * s2 - 0.000164052 * c3 * s2 * s4;
        g[4][0] = 0.0000066444 * c6 * (s5 * (c2 * s4 + c3 * c4 * s2) + c5 * s2 * s3) + 3.0195944 * s6 * (s5 * (c2 * s4 + c3 * c4 * s2) + c5 * s2 * s3) - 0.0558698 * c7 * (c5 * (c2 * s4 + c3 * c4 * s2) - 1.0 * s2 * s3 * s5) - 0.0013769 * s7 * (c5 * (c2 * s4 + c3 * c4 * s2) - 1.0 * s2 * s3 * s5) + 0.0014484792 * c5 * (c2 * s4 + c3 * c4 * s2) + 0.0000066444 * s5 * (c2 * s4 + c3 * c4 * s2) - 0.0014484792 * s2 * s3 * s5 - 0.0013769 * c6 * c7 * (s5 * (c2 * s4 + c3 * c4 * s2) + c5 * s2 * s3) + 0.0558698 * c6 * s7 * (s5 * (c2 * s4 + c3 * c4 * s2) + c5 * s2 * s3) + 0.0000066444 * c5 * s2 * s3;
        g[5][0] = 0.0558698 * s7 * (s6 * (c5 * (c2 * s4 + c3 * c4 * s2) - 1.0 * s2 * s3 * s5) - 1.0 * c6 * (c2 * c4 - 1.0 * c3 * s2 * s4)) - 0.0013769 * c7 * (s6 * (c5 * (c2 * s4 + c3 * c4 * s2) - 1.0 * s2 * s3 * s5) - 1.0 * c6 * (c2 * c4 - 1.0 * c3 * s2 * s4)) - 3.0195944 * c6 * (c5 * (c2 * s4 + c3 * c4 * s2) - 1.0 * s2 * s3 * s5) + 0.0000066444 * s6 * (c5 * (c2 * s4 + c3 * c4 * s2) - 1.0 * s2 * s3 * s5) - 0.0000066444 * c6 * (c2 * c4 - 1.0 * c3 * s2 * s4) - 3.0195944 * s6 * (c2 * c4 - 1.0 * c3 * s2 * s4);
        g[6][0] = 0.0558698 * s7 * (s5 * (c2 * s4 + c3 * c4 * s2) + c5 * s2 * s3) - 0.0013769 * c7 * (s5 * (c2 * s4 + c3 * c4 * s2) + c5 * s2 * s3) - 0.0013769 * s7 * (c6 * (c5 * (c2 * s4 + c3 * c4 * s2) - 1.0 * s2 * s3 * s5) + s6 * (c2 * c4 - 1.0 * c3 * s2 * s4)) - 0.0558698 * c7 * (c6 * (c5 * (c2 * s4 + c3 * c4 * s2) - 1.0 * s2 * s3 * s5) + s6 * (c2 * c4 - 1.0 * c3 * s2 * s4));

        return g;
    }

    // ------- To transform from INT to Base frame
    // Returns T_base_int: pose of the interface module frame (INT) in the BASE frame.
    // It is the product of the matrices of Table 90 (user guide r12.0, p.221-222):
    //   T_base_int = BT1*(q1) * 1T2*(q2) * ... * 6T7*(q7) * 7T_INT
    // Rotation (3x3): exact product of Table 90.
    // Position (x, y, z): two versions are written, with different dimensions:
    //   Matlab (active):    dimensions of MATLAB's built-in kinovaGen3 model.
    //   Manual (commented): dimensions printed in Table 90.
    //   They differ by about 2 mm. Check with the Kinova web app which one
    //   matches the real robot, and keep that one active.
    // Constants in the position (meters, Table 90):
    //   28481/100000 = 0.15643 + 0.12838  (base to actuator 2, vertical)
    //   147/12500    = 0.00538 + 0.00638  (side offsets at actuators 2 and 3)
    //   10519/25000  = 0.21038 + 0.21038  (actuator 2 to actuator 4)
    //   319/25000    = 0.00638 + 0.00638  (side offsets at actuators 4 and 5)
    //   7859/25000   = 0.20843 + 0.10593  (actuator 4 to actuator 6)
    //   8373/50000   = 0.10593 + 0.06153  (actuator 6 to INT)            Manual
    //   3309/20000   = 0.10593 + 0.05952  (actuator 6 to INT, MATLAB)    Matlab
    //   9/25000      = 0.00018 + 0.00018  (side offsets, only in MATLAB)  Matlab
    mat<4, 4> GetTransform(const mat<7, 1> &q)
    {
        // Precalculate sin(q) and cos(q)
        float s1 = sin(q[0][0]);
        float s2 = sin(q[1][0]);
        float s3 = sin(q[2][0]);
        float s4 = sin(q[3][0]);
        float s5 = sin(q[4][0]);
        float s6 = sin(q[5][0]);
        float s7 = sin(q[6][0]);
        float c1 = cos(q[0][0]);
        float c2 = cos(q[1][0]);
        float c3 = cos(q[2][0]);
        float c4 = cos(q[3][0]);
        float c5 = cos(q[4][0]);
        float c6 = cos(q[5][0]);
        float c7 = cos(q[6][0]);

        // Direct Kinematics
        mat<4, 4> T{};

        T[0][0] = c7 * (s6 * (s4 * (s1 * s3 - c1 * c2 * c3) - c1 * c4 * s2) - c6 * (c5 * (c4 * (s1 * s3 - c1 * c2 * c3) + c1 * s2 * s4) + s5 * (c3 * s1 + c1 * c2 * s3))) + s7 * (s5 * (c4 * (s1 * s3 - c1 * c2 * c3) + c1 * s2 * s4) - c5 * (c3 * s1 + c1 * c2 * s3));
        T[0][1] = s7 * (s6 * (s4 * (s1 * s3 - c1 * c2 * c3) - c1 * c4 * s2) - c6 * (c5 * (c4 * (s1 * s3 - c1 * c2 * c3) + c1 * s2 * s4) + s5 * (c3 * s1 + c1 * c2 * s3))) - c7 * (s5 * (c4 * (s1 * s3 - c1 * c2 * c3) + c1 * s2 * s4) - c5 * (c3 * s1 + c1 * c2 * s3));
        T[0][2] = -c6 * (s4 * (s1 * s3 - c1 * c2 * c3) - c1 * c4 * s2) - s6 * (c5 * (c4 * (s1 * s3 - c1 * c2 * c3) + c1 * s2 * s4) + s5 * (c3 * s1 + c1 * c2 * s3));
        // Matlab:
        // T[0][3] = (10519.0f * c1 * s2) / 25000.0f - (147.0f * s1) / 12500.0f - (319.0f * c3 * s1) / 25000.0f - (3309.0f * c6 * (s4 * (s1 * s3 - c1 * c2 * c3) - c1 * c4 * s2)) / 20000.0f + (9.0f * s5 * (c4 * (s1 * s3 - c1 * c2 * c3) + c1 * s2 * s4)) / 25000.0f - (3309.0f * s6 * (c5 * (c4 * (s1 * s3 - c1 * c2 * c3) + c1 * s2 * s4) + s5 * (c3 * s1 + c1 * c2 * s3))) / 20000.0f - (9.0f * c5 * (c3 * s1 + c1 * c2 * s3)) / 25000.0f - (7859.0f * s4 * (s1 * s3 - c1 * c2 * c3)) / 25000.0f - (319.0f * c1 * c2 * s3) / 25000.0f + (7859.0f * c1 * c4 * s2) / 25000.0f;
        // Manual:
        T[0][3] = (10519.0f * c1 * s2) / 25000.0f - (147.0f * s1) / 12500.0f - (319.0f * c3 * s1) / 25000.0f - (8373.0f * c6 * (s4 * (s1 * s3 - c1 * c2 * c3) - c1 * c4 * s2)) / 50000.0f - (8373.0f * s6 * (c5 * (c4 * (s1 * s3 - c1 * c2 * c3) + c1 * s2 * s4) + s5 * (c3 * s1 + c1 * c2 * s3))) / 50000.0f - (7859.0f * s4 * (s1 * s3 - c1 * c2 * c3)) / 25000.0f - (319.0f * c1 * c2 * s3) / 25000.0f + (7859.0f * c1 * c4 * s2) / 25000.0f;
        T[1][0] = c7 * (s6 * (s4 * (c1 * s3 + c2 * c3 * s1) + c4 * s1 * s2) - c6 * (c5 * (c4 * (c1 * s3 + c2 * c3 * s1) - s1 * s2 * s4) + s5 * (c1 * c3 - c2 * s1 * s3))) + s7 * (s5 * (c4 * (c1 * s3 + c2 * c3 * s1) - s1 * s2 * s4) - c5 * (c1 * c3 - c2 * s1 * s3));
        T[1][1] = s7 * (s6 * (s4 * (c1 * s3 + c2 * c3 * s1) + c4 * s1 * s2) - c6 * (c5 * (c4 * (c1 * s3 + c2 * c3 * s1) - s1 * s2 * s4) + s5 * (c1 * c3 - c2 * s1 * s3))) - c7 * (s5 * (c4 * (c1 * s3 + c2 * c3 * s1) - s1 * s2 * s4) - c5 * (c1 * c3 - c2 * s1 * s3));
        T[1][2] = -c6 * (s4 * (c1 * s3 + c2 * c3 * s1) + c4 * s1 * s2) - s6 * (c5 * (c4 * (c1 * s3 + c2 * c3 * s1) - s1 * s2 * s4) + s5 * (c1 * c3 - c2 * s1 * s3));
        // Matlab:
        // T[1][3] = -(10519.0f * s1 * s2) / 25000.0f - (319.0f * c1 * c3) / 25000.0f - (147.0f * c1) / 12500.0f - (3309.0f * c6 * (s4 * (c1 * s3 + c2 * c3 * s1) + c4 * s1 * s2)) / 20000.0f + (9.0f * s5 * (c4 * (c1 * s3 + c2 * c3 * s1) - s1 * s2 * s4)) / 25000.0f - (3309.0f * s6 * (c5 * (c4 * (c1 * s3 + c2 * c3 * s1) - s1 * s2 * s4) + s5 * (c1 * c3 - c2 * s1 * s3))) / 20000.0f - (9.0f * c5 * (c1 * c3 - c2 * s1 * s3)) / 25000.0f - (7859.0f * s4 * (c1 * s3 + c2 * c3 * s1)) / 25000.0f + (319.0f * c2 * s1 * s3) / 25000.0f - (7859.0f * c4 * s1 * s2) / 25000.0f;
        // Manual:
        T[1][3] = (319.0f * c2 * s1 * s3) / 25000.0f - (319.0f * c1 * c3) / 25000.0f - (10519.0f * s1 * s2) / 25000.0f - (8373.0f * c6 * (s4 * (c1 * s3 + c2 * c3 * s1) + c4 * s1 * s2)) / 50000.0f - (8373.0f * s6 * (c5 * (c4 * (c1 * s3 + c2 * c3 * s1) - s1 * s2 * s4) + s5 * (c1 * c3 - c2 * s1 * s3))) / 50000.0f - (7859.0f * s4 * (c1 * s3 + c2 * c3 * s1)) / 25000.0f - (147.0f * c1) / 12500.0f - (7859.0f * c4 * s1 * s2) / 25000.0f;
        T[2][0] = s7 * (s5 * (c2 * s4 + c3 * c4 * s2) + c5 * s2 * s3) - c7 * (c6 * (c5 * (c2 * s4 + c3 * c4 * s2) - s2 * s3 * s5) + s6 * (c2 * c4 - c3 * s2 * s4));
        T[2][1] = -c7 * (s5 * (c2 * s4 + c3 * c4 * s2) + c5 * s2 * s3) - s7 * (c6 * (c5 * (c2 * s4 + c3 * c4 * s2) - s2 * s3 * s5) + s6 * (c2 * c4 - c3 * s2 * s4));
        T[2][2] = c6 * (c2 * c4 - c3 * s2 * s4) - s6 * (c5 * (c2 * s4 + c3 * c4 * s2) - s2 * s3 * s5);
        // Matlab:
        // T[2][3] = (10519.0f * c2) / 25000.0f + (7859.0f * c2 * c4) / 25000.0f + (319.0f * s2 * s3) / 25000.0f - (3309.0f * s6 * (c5 * (c2 * s4 + c3 * c4 * s2) - s2 * s3 * s5)) / 20000.0f + (3309.0f * c6 * (c2 * c4 - c3 * s2 * s4)) / 20000.0f + (9.0f * s5 * (c2 * s4 + c3 * c4 * s2)) / 25000.0f - (7859.0f * c3 * s2 * s4) / 25000.0f + (9.0f * c5 * s2 * s3) / 25000.0f + 28481.0f / 100000.0f;
        // Manual:
        T[2][3] = (10519.0f * c2) / 25000.0f + (7859.0f * c2 * c4) / 25000.0f + (319.0f * s2 * s3) / 25000.0f - (8373.0f * s6 * (c5 * (c2 * s4 + c3 * c4 * s2) - s2 * s3 * s5)) / 50000.0f + (8373.0f * c6 * (c2 * c4 - c3 * s2 * s4)) / 50000.0f - (7859.0f * c3 * s2 * s4) / 25000.0f + 28481.0f / 100000.0f;
        T[3][0] = 0.0f;
        T[3][1] = 0.0f;
        T[3][2] = 0.0f;
        T[3][3] = 1.0f;

        return T;

    }

    // ------- Robotiq 2F-85 tool offset (Kinova Gen3 user guide r12.0, Table 28):
    // Transform from interface module frame (INT) to gripper frame:
    // X = 0, Y = 0, Z = 0.120 m, thetaX = thetaY = thetaZ = 0
    const float TOOL_OFFSET_Z_2F85 = 0.120f;

    // ------- To transform from the gripper frame (TOOL) to Base frame
    // T_base_tool = T_base_int * T_int_tool
    mat<4, 4> GetToolTransform(const mat<7, 1> &q)
    {
        // INT -> TOOL: pure translation along Z_INT, no rotation
        mat<4, 4> T_int_tool{};
        T_int_tool[0][0] = 1.0f;
        T_int_tool[1][1] = 1.0f;
        T_int_tool[2][2] = 1.0f;
        T_int_tool[2][3] = TOOL_OFFSET_Z_2F85;
        T_int_tool[3][3] = 1.0f;

        return GetTransform(q) * T_int_tool;
    }



    // ----------- * GEN 3 * POSE ----------------------------------------------------------------
    mat<7, 1> GetPose(const mat<7, 1> &q, const mat<7, 1> &lastPose = mat<7, 1>{})
    {
        mat<7, 1> pose{};
        mat<4, 4> T = GetTransform(q);  


        // Unit Quaternion obtention algorithm
        // Richard A. Spurrier 1978
        // Trace of Direct Kinematics Rotation Matrix
        float r_trace = trace(select<0, 3, 0, 3>(T));
        float w = r_trace;
        float x = 0;
        float y = 0;
        float z = 0;
        // if(trace > 0) w = 0;
        float i = std::max({T[0][0], T[1][1], T[2][2], w});
        if (i == w)
        {
            w = sqrt(r_trace + 1) / 2;
            x = (T[2][1] - T[1][2]) / (4.0f * w);
            y = (T[0][2] - T[2][0]) / (4.0f * w);
            z = (T[1][0] - T[0][1]) / (4.0f * w);
        }
        if (i == T[0][0])
        {
            x = sqrt((T[0][0] / 2) + ((1 - r_trace) / 4));
            w = (T[2][1] - T[1][2]) / (4.0f * x);
            y = (T[1][0] + T[0][1]) / (4.0f * x);
            z = (T[2][0] + T[0][2]) / (4.0f * x);
        }
        if (i == T[1][1])
        {
            y = sqrt((T[1][1] / 2) + ((1 - r_trace) / 4));
            w = (T[0][2] - T[2][0]) / (4.0f * y);
            z = (T[2][1] + T[1][2]) / (4.0f * y);
            x = (T[0][1] + T[1][0]) / (4.0f * y);
        }
        if (i == T[2][2])
        {
            z = sqrt((T[2][2] / 2) + ((1 - r_trace) / 4));
            w = (T[1][0] - T[0][1]) / (4.0f * z);
            x = (T[0][2] + T[2][0]) / (4.0f * z);
            y = (T[1][2] + T[2][1]) / (4.0f * z);
        }

        pose[0][0] = T[0][3]; // x
        pose[1][0] = T[1][3]; // y
        pose[2][0] = T[2][3]; // z
        pose[3][0] = w;       // eta
        pose[4][0] = x;       // beta_1
        pose[5][0] = y;       // beta_2
        pose[6][0] = z;       // beta_3

        if (lastPose[0][0] != 0 && lastPose[1][0] != 0 && lastPose[2][0] != 0 && lastPose[3][0] != 0 && lastPose[4][0] != 0 && lastPose[5][0] != 0 && lastPose[6][0] != 0)
        {
            auto innerProd = trans(lastPose.select<3,7,0,1>()) * pose.select<3,7,0,1>();
            if (innerProd[0][0] < 0)
            {
                for (unsigned int i = 3; i < 7; i++)
                {
                    pose[i][0] = -pose[i][0];
                }
            }
        }
        return pose;
    }
    
    // ----------- * GEN 3 * JACOBIAN v23-07-09----------------------------------------------------------------
    // Rows 1-3: linear velocity, rows 4-6: angular velocity (joint z axes in BASE).
    // Note: rows 1-3 are the exact derivatives for a point 0.174 m after INT
    // along Z_INT (1707/5000 = 0.1674 + 0.174), with the rounded values of
    // Table 93 (user guide r12.0, p.226). It is NOT the INT point of GetTransform,
    // nor the Robotiq 2F-85 TOOL point (0.120 m, Table 28, p.92).
    // The 0.174 m is not in the manual r12.0.
    mat<6, 7> GetJacobian(const mat<7, 1> &q)
    {
        // 6 rows , 7 colums
        mat<6, 7> jacobian = {};
        // Precalculate sin(q) and cos(q)
        double s1 = sin(q[0][0]);
        double s2 = sin(q[1][0]);
        double s3 = sin(q[2][0]);
        double s4 = sin(q[3][0]);
        double s5 = sin(q[4][0]);
        double s6 = sin(q[5][0]);
        //double s7 = sin(q[6][0]);
        double c1 = cos(q[0][0]);
        double c2 = cos(q[1][0]);
        double c3 = cos(q[2][0]);
        double c4 = cos(q[3][0]);
        double c5 = cos(q[4][0]);
        double c6 = cos(q[5][0]);
        //double c7 = cos(q[6][0]);

        // 1st row
        jacobian[0][0] = (8.0f*c2*s1*s3)/625.0f - (8.0f*c1*c3)/625.0f - (263.0f*s1*s2)/625.0f - (1707.0f*c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2))/5000.0f - (1707.0f*s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))/5000.0f - (3143.0f*s4*(c1*s3 + c2*c3*s1))/10000.0f - (59*c1)/5000.0f - (3143.0f*c4*s1*s2)/10000.0f; 
        jacobian[0][1] = c1*((263.0f*c2)/625.0f + (3143.0f*c2*c4)/10000.0f + (8.0f*s2*s3)/625.0f - (1707.0f*s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5))/5000.0f + (1707.0f*c6*(c2*c4 - c3*s2*s4))/5000.0f - (3143.0f*c3*s2*s4)/10000.0f);
        jacobian[0][2] = s1*s2*((263.0f*c2)/1250.0f + (3143.0f*c2*c4)/10000.0f + (8.0f*s2*s3)/625.0f - (1707.0f*s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5))/5000.0f + (1707.0f*c6*(c2*c4 - c3*s2*s4))/5000.0f - (3143.0f*c3*s2*s4)/10000.0f) - c2*((8.0f*c1*c3)/625.0f + (263.0f*s1*s2)/1250.0f + (1707.0f*c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))/5000.0f + (3143.0f*s4*(c1*s3 + c2*c3*s1))/10000.0f - (8.0f*c2*s1*s3)/625.0f + (3143.0f*c4*s1*s2)/10000.0f);        
        jacobian[0][3] = (c1*c3 - c2*s1*s3)*((3143.0f*c2*c4)/10000.0f + (4.0f*s2*s3)/625.0f - (1707.0f*s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5))/5000.0f + (1707.0f*c6*(c2*c4 - c3*s2*s4))/5000.0f - (3143.0f*c3*s2*s4)/10000.0f) - s2*s3*((4.0f*c1*c3)/625.0f + (1707.0f*c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))/5000.0f + (3143.0f*s4*(c1*s3 + c2*c3*s1))/10000.0f - (4.0f*c2*s1*s3)/625.0f + (3143.0f*c4*s1*s2)/10000.0f);
        jacobian[0][4] = (s4*(c1*s3 + c2*c3*s1) + c4*s1*s2)*((1059.0f*c2*c4)/10000.0f - (1707.0f*s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5))/5000.0f + (1707.0f*c6*(c2*c4 - c3*s2*s4))/5000.0f - (1059.0f*c3*s2*s4)/10000.0f) - (c2*c4 - c3*s2*s4)*((1707.0f*c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))/5000.0f + (1059.0f*s4*(c1*s3 + c2*c3*s1))/10000.0f + (1059.0f*c4*s1*s2)/10000.0f);
        jacobian[0][5] = (s5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))*((1707.0f*s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5))/5000.0f - (1707.0f*c6*(c2*c4 - c3*s2*s4))/5000.0f) - (s5*(c2*s4 + c3*c4*s2) + c5*s2*s3)*((1707.0f*c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))/5000.0f);
        jacobian[0][6] = 0;

        // 2nd row
        jacobian[1][0] = (59*s1)/5000.0f - (263.0f*c1*s2)/625.0f + (8.0f*c3*s1)/625.0f + (1707.0f*c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))/5000.0f + (3143.0f*s4*(s1*s3 - c1*c2*c3))/10000.0f + (8.0f*c1*c2*s3)/625.0f - (3143.0f*c1*c4*s2)/10000.0f;
        jacobian[1][1] = -s1*((263.0f*c2)/625.0f + (3143.0f*c2*c4)/10000.0f + (8.0f*s2*s3)/625.0f - (1707.0f*s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5))/5000.0f + (1707.0f*c6*(c2*c4 - c3*s2*s4))/5000.0f - (3143.0f*c3*s2*s4)/10000.0f);
        jacobian[1][2] = c2*((8.0f*c3*s1)/625.0f - (263.0f*c1*s2)/1250.0f + (1707.0f*c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))/5000.0f + (3143.0f*s4*(s1*s3 - c1*c2*c3))/10000.0f + (8.0f*c1*c2*s3)/625.0f - (3143.0f*c1*c4*s2)/10000.0f) + c1*s2*((263.0f*c2)/1250.0f + (3143.0f*c2*c4)/10000.0f + (8.0f*s2*s3)/625.0f - (1707.0f*s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5))/5000.0f + (1707.0f*c6*(c2*c4 - c3*s2*s4))/5000.0f - (3143.0f*c3*s2*s4)/10000.0f);
        jacobian[1][3] = s2*s3*((4.0f*c3*s1)/625.0f + (1707.0f*c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))/5000.0f + (3143.0f*s4*(s1*s3 - c1*c2*c3))/10000.0f + (4.0f*c1*c2*s3)/625.0f - (3143.0f*c1*c4*s2)/10000.0f) - (c3*s1 + c1*c2*s3)*((3143.0f*c2*c4)/10000.0f + (4.0f*s2*s3)/625.0f - (1707.0f*s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5))/5000.0f + (1707.0f*c6*(c2*c4 - c3*s2*s4))/5000.0f - (3143.0f*c3*s2*s4)/10000.0f);
        jacobian[1][4] = (c2*c4 - c3*s2*s4)*((1707.0f*c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))/5000.0f + (1059.0f*s4*(s1*s3 - c1*c2*c3))/10000.0f - (1059.0f*c1*c4*s2)/10000.0f) - (s4*(s1*s3 - c1*c2*c3) - c1*c4*s2)*((1059.0f*c2*c4)/10000.0f - (1707.0f*s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5))/5000.0f + (1707.0f*c6*(c2*c4 - c3*s2*s4))/5000.0f - (1059.0f*c3*s2*s4)/10000.0f);
        jacobian[1][5] = (s5*(c2*s4 + c3*c4*s2) + c5*s2*s3)*((1707.0f*c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))/5000.0f) - (s5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) - c5*(c3*s1 + c1*c2*s3))*((1707.0f*s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5))/5000.0f - (1707.0f*c6*(c2*c4 - c3*s2*s4))/5000.0f);
        jacobian[1][6] = 0;

        // 3rd row
        jacobian[2][0] = 0;
        jacobian[2][1] = c1*((4.0f*s1)/625.0f - (263.0f*c1*s2)/625.0f + (8.0f*c3*s1)/625.0f + (1707.0f*c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))/5000.0f + (3143.0f*s4*(s1*s3 - c1*c2*c3))/10000.0f + (8.0f*c1*c2*s3)/625.0f - (3143.0f*c1*c4*s2)/10000.0f) - s1*((4.0f*c1)/625.0f + (8.0f*c1*c3)/625.0f + (263.0f*s1*s2)/625.0f + (1707.0f*c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))/5000.0f + (3143.0f*s4*(c1*s3 + c2*c3*s1))/10000.0f - (8.0f*c2*s1*s3)/625.0f + (3143.0f*c4*s1*s2)/10000.0f);
        jacobian[2][2] = c1*s2*((8.0f*c1*c3)/625.0f + (263.0f*s1*s2)/1250.0f + (1707.0f*c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))/5000.0f + (3143.0f*s4*(c1*s3 + c2*c3*s1))/10000.0f - (8.0f*c2*s1*s3)/625.0f + (3143.0f*c4*s1*s2)/10000.0f) + s1*s2*((8.0f*c3*s1)/625.0f - (263.0f*c1*s2)/1250.0f + (1707.0f*c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))/5000.0f + (3143.0f*s4*(s1*s3 - c1*c2*c3))/10000.0f + (8.0f*c1*c2*s3)/625.0f - (3143.0f*c1*c4*s2)/10000.0f);
        jacobian[2][3] = (c1*c3 - c2*s1*s3)*((4.0f*c3*s1)/625.0f + (1707.0f*c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))/5000.0f + (3143.0f*s4*(s1*s3 - c1*c2*c3))/10000.0f + (4.0f*c1*c2*s3)/625.0f - (3143.0f*c1*c4*s2)/10000.0f) - (c3*s1 + c1*c2*s3)*((4.0f*c1*c3)/625.0f + (1707.0f*c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))/5000.0f + (3143.0f*s4*(c1*s3 + c2*c3*s1))/10000.0f - (4.0f*c2*s1*s3)/625.0f + (3143.0f*c4*s1*s2)/10000.0f);
        jacobian[2][4] = (s4*(c1*s3 + c2*c3*s1) + c4*s1*s2)*((1707.0f*c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))/5000.0f + (1059.0f*s4*(s1*s3 - c1*c2*c3))/10000.0f - (1059.0f*c1*c4*s2)/10000.0f) - (s4*(s1*s3 - c1*c2*c3) - c1*c4*s2)*((1707.0f*c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))/5000.0f + (1059.0f*s4*(c1*s3 + c2*c3*s1))/10000.0f + (1059.0f*c4*s1*s2)/10000.0f);
        jacobian[2][5] = ((1707.0f*c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3)))/5000.0f)*(s5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) - c5*(c3*s1 + c1*c2*s3)) - (s5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))*((1707.0f*c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2))/5000.0f + (1707.0f*s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3)))/5000.0f);
        jacobian[2][6] = 0;

        // 4th row
        jacobian[3][0] = 0;
        jacobian[3][1] = s1;
        jacobian[3][2] = -c1*s2;
        jacobian[3][3] = c3*s1 + c1*c2*s3;
        jacobian[3][4] = s4*(s1*s3 - c1*c2*c3) - c1*c4*s2;
        jacobian[3][5] = c5*(c3*s1 + c1*c2*s3) - s5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4);
        jacobian[3][6] = c6*(s4*(s1*s3 - c1*c2*c3) - c1*c4*s2) + s6*(c5*(c4*(s1*s3 - c1*c2*c3) + c1*s2*s4) + s5*(c3*s1 + c1*c2*s3));

        // 5th row
        jacobian[4][0] = 0;
        jacobian[4][1] = c1;
        jacobian[4][2] = s1*s2;
        jacobian[4][3] = c1*c3 - c2*s1*s3;
        jacobian[4][4] = s4*(c1*s3 + c2*c3*s1) + c4*s1*s2;
        jacobian[4][5] = c5*(c1*c3 - c2*s1*s3) - s5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4);
        jacobian[4][6] = c6*(s4*(c1*s3 + c2*c3*s1) + c4*s1*s2) + s6*(c5*(c4*(c1*s3 + c2*c3*s1) - s1*s2*s4) + s5*(c1*c3 - c2*s1*s3));

        // 6th row
        jacobian[5][0] = -1;
        jacobian[5][1] = 0;
        jacobian[5][2] = -c2;
        jacobian[5][3] = -s2*s3;
        jacobian[5][4] = c3*s2*s4 - c2*c4;
        jacobian[5][5] = - s5*(c2*s4 + c3*c4*s2) - c5*s2*s3;
        jacobian[5][6] = s6*(c5*(c2*s4 + c3*c4*s2) - s2*s3*s5) - c6*(c2*c4 - c3*s2*s4);

        return jacobian;
    }  

}
