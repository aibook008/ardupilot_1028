/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
 */
/*
  DF-200 propeller thrust/power lookup tables.

  TEST-ONLY code, DF-200 SITL only (see Motor::ThrustModel in SIM_Motor.h).
  Data source: docs/电机电池.docx
    - table 0: 60in lift prop (outer MOT_5..8), static bench test, 15 points
    - table 1: 40in cruise prop (inner MOT_1..4), wind tunnel, RPM x wind
      speed (0/2.9/10.1/20/30/40/50 m/s), 63 points (duplicate rows averaged)

  Thrust converted from gram-force to newtons (x 9.80665/1000), power from
  kW to W. Tables calibrated at ~398 V; voltage sag is not modelled.
  Density scaling (rho/1.225) is applied by the caller.

  Why this exists: the momentum-disc calc_thrust() gives negative thrust at
  AS=50 with hover command (Vout < Vin) and squares small per-motor velin
  differences into a same-sign Mx positive feedback (see
  docs/vtol_df200.md §11.10 and docs/df200_motor_thrust_derivation.md).
*/

#pragma once

#include <AP_Math/AP_Math.h>

namespace SITL {
namespace DF200MotorTable {

// 60in lift prop, static (docs/电机电池.docx 表0). RPM 1551..3181.
static const float prop60_rpm[15] = {
    1551, 1680, 1895, 2023, 2175, 2294, 2386, 2470,
    2604, 2759, 2874, 2939, 3035, 3111, 3181,
};
static const float prop60_thrust_N[15] = {
    278.3f, 343.3f, 444.3f, 510.5f, 596.1f, 645.6f, 697.0f, 748.6f,
    863.6f, 970.5f, 1066.8f, 1137.8f, 1222.7f, 1280.9f, 1379.2f,
};
static const float prop60_power_W[15] = {
    3048, 3938, 5768, 7102, 8931, 10347, 11694, 13199,
    15734, 19062, 21810, 23879, 26638, 29168, 32043,
};
// shaft torque [N·m], docx table 0 torque column (measured). Signed by the
// caller's yaw_factor sign — see prop60_torque()/prop40_torque() below.
// Cross-check: docs/数据库_v2.xlsx 动力参数 sheet gives Q=32.00 N·m at
// 2019 rpm, matching the interp of this column (18.77..33.53 over
// 1551..2023 rpm) to <0.2 N·m.
static const float prop60_torque_Nm[15] = {
    18.77f, 22.39f, 29.07f, 33.53f, 39.22f, 43.08f, 46.81f, 51.04f,
    57.71f, 65.99f, 72.48f, 77.60f, 83.83f, 89.55f, 96.21f,
};

// 40in cruise prop, one branch per wind speed (docs/电机电池.docx 表1).
struct Prop40Entry {
    float rpm;
    float thrust_N;
    float power_W;
    float torque_Nm;
};

static const Prop40Entry prop40_v0[] = {
    {2000, 169.90f, 2910, 13.87f}, {2200, 205.84f, 3900, 16.93f}, {2400, 242.65f, 5090, 20.27f},
    {2600, 283.39f, 6530, 24.00f}, {2800, 328.08f, 8210, 28.00f}, {3000, 374.22f, 10130, 32.23f},
    {3200, 425.36f, 12480, 37.23f}, {3400, 478.19f, 15120, 42.48f}, {3600, 534.83f, 18130, 48.08f},
};
static const Prop40Entry prop40_v29[] = {
    {1000, 44.85f, 380, 3.59f}, {1500, 97.24f, 1230, 7.85f}, {2000, 168.67f, 2910, 13.91f},
    {2500, 261.83f, 5750, 21.97f}, {3000, 376.91f, 10150, 32.32f}, {3500, 506.15f, 16580, 45.25f},
    {4000, 652.75f, 26000, 62.08f},
};
static const Prop40Entry prop40_v10[] = {
    {2000, 155.74f, 3000, 14.31f}, {2200, 191.67f, 3980, 17.29f}, {2400, 230.05f, 5170, 20.56f},
    {2600, 271.49f, 6570, 24.14f}, {2800, 317.58f, 8210, 28.02f}, {3000, 367.56f, 10150, 32.32f},
    {3200, 419.64f, 12400, 37.02f}, {3400, 474.09f, 14990, 42.11f}, {3600, 532.37f, 18100, 48.02f},
};
static const Prop40Entry prop40_v20[] = {
    {2200, 151.70f, 3985, 17.30f}, {2400, 189.63f, 5230, 20.82f}, {2600, 230.87f, 6690, 24.58f},
    {2800, 275.86f, 8400, 28.66f}, {3000, 325.70f, 10400, 33.10f}, {3200, 378.13f, 12620, 37.66f},
    {3400, 432.85f, 15170, 42.60f}, {3600, 493.10f, 18070, 47.94f}, {3800, 558.22f, 21400, 53.77f},
};
static const Prop40Entry prop40_v30[] = {
    {2400, 123.49f, 4260, 16.95f}, {2600, 162.89f, 5840, 21.45f}, {2800, 207.02f, 7690, 26.22f},
    {3000, 255.27f, 9780, 31.13f}, {3200, 307.28f, 12180, 36.35f}, {3400, 363.00f, 14850, 41.71f},
    {3600, 423.01f, 17870, 47.40f}, {3800, 486.50f, 21220, 53.32f}, {4000, 553.86f, 24960, 59.58f},
};
static const Prop40Entry prop40_v40[] = {
    {2400, 40.23f, 1880, 7.49f}, {2600, 75.58f, 3460, 12.71f}, {2800, 115.16f, 5300, 18.07f},
    {3000, 159.08f, 7420, 23.62f}, {3200, 207.34f, 9890, 29.52f}, {3400, 261.36f, 12750, 35.80f},
    {3600, 318.04f, 15890, 42.15f}, {3800, 381.23f, 19440, 48.85f}, {4000, 448.29f, 23350, 55.75f},
};
static const Prop40Entry prop40_v50[] = {
    {2800, 17.78f, 1360, 4.63f}, {3000, 55.70f, 3380, 10.75f}, {3200, 98.26f, 5730, 17.10f},
    {3400, 144.75f, 8430, 23.68f}, {3600, 196.70f, 11550, 30.64f}, {3800, 255.10f, 15140, 38.06f},
    {4000, 319.19f, 19110, 45.62f}, {4200, 390.18f, 23530, 53.49f},
};

struct Prop40Branch {
    float v;                     // wind speed m/s
    const Prop40Entry *entries;
    uint8_t n;
};

static const Prop40Branch prop40_branches[] = {
    { 0.0f, prop40_v0,  sizeof(prop40_v0)/sizeof(prop40_v0[0]) },
    { 2.9f, prop40_v29, sizeof(prop40_v29)/sizeof(prop40_v29[0]) },
    {10.0f, prop40_v10, sizeof(prop40_v10)/sizeof(prop40_v10[0]) },
    {20.0f, prop40_v20, sizeof(prop40_v20)/sizeof(prop40_v20[0]) },
    {30.0f, prop40_v30, sizeof(prop40_v30)/sizeof(prop40_v30[0]) },
    {40.0f, prop40_v40, sizeof(prop40_v40)/sizeof(prop40_v40[0]) },
    {50.0f, prop40_v50, sizeof(prop40_v50)/sizeof(prop40_v50[0]) },
};

// interpolate a value vs rpm in a sorted rpm[]/val[] pair.
// below the table: quadratic through origin (thrust ~ rpm^2, smooth at cmd=0);
// above the table: clamp to the last value.
static inline float interp_by_rpm(const float *rpm, const float *val, uint8_t n, float r)
{
    if (r <= 0.0f) {
        return 0.0f;
    }
    if (r <= rpm[0]) {
        return val[0] * sq(r / rpm[0]);
    }
    if (r >= rpm[n-1]) {
        return val[n-1];
    }
    for (uint8_t i=1; i<n; i++) {
        if (r <= rpm[i]) {
            const float f = (r - rpm[i-1]) / (rpm[i] - rpm[i-1]);
            return val[i-1] + f * (val[i] - val[i-1]);
        }
    }
    return val[n-1];
}

// thrust [N] from one 40in wind-speed branch
static inline float branch_thrust(const Prop40Branch &b, float rpm)
{
    if (rpm <= 0.0f) {
        return 0.0f;
    }
    const Prop40Entry &e0 = b.entries[0];
    if (rpm <= e0.rpm) {
        return e0.thrust_N * sq(rpm / e0.rpm);
    }
    const Prop40Entry &en = b.entries[b.n-1];
    if (rpm >= en.rpm) {
        return en.thrust_N;
    }
    for (uint8_t i=1; i<b.n; i++) {
        const Prop40Entry &e = b.entries[i];
        if (rpm <= e.rpm) {
            const Prop40Entry &p = b.entries[i-1];
            const float f = (rpm - p.rpm) / (e.rpm - p.rpm);
            return p.thrust_N + f * (e.thrust_N - p.thrust_N);
        }
    }
    return en.thrust_N;
}

// power [W] from one 40in wind-speed branch
static inline float branch_power(const Prop40Branch &b, float rpm)
{
    if (rpm <= 0.0f) {
        return 0.0f;
    }
    const Prop40Entry &e0 = b.entries[0];
    if (rpm <= e0.rpm) {
        return e0.power_W * sq(rpm / e0.rpm);
    }
    const Prop40Entry &en = b.entries[b.n-1];
    if (rpm >= en.rpm) {
        return en.power_W;
    }
    for (uint8_t i=1; i<b.n; i++) {
        const Prop40Entry &e = b.entries[i];
        if (rpm <= e.rpm) {
            const Prop40Entry &p = b.entries[i-1];
            const float f = (rpm - p.rpm) / (e.rpm - p.rpm);
            return p.power_W + f * (e.power_W - p.power_W);
        }
    }
    return en.power_W;
}

// 40in cruise prop thrust [N] at (rpm, wind speed). v clamped to [0,50].
static inline float prop40_thrust(float rpm, float v)
{
    v = constrain_float(v, 0.0f, 50.0f);
    const uint8_t nb = sizeof(prop40_branches)/sizeof(prop40_branches[0]);
    if (v <= prop40_branches[0].v) {
        return branch_thrust(prop40_branches[0], rpm);
    }
    if (v >= prop40_branches[nb-1].v) {
        return branch_thrust(prop40_branches[nb-1], rpm);
    }
    for (uint8_t i=1; i<nb; i++) {
        if (v <= prop40_branches[i].v) {
            const float f = (v - prop40_branches[i-1].v) /
                            (prop40_branches[i].v - prop40_branches[i-1].v);
            const float t0 = branch_thrust(prop40_branches[i-1], rpm);
            const float t1 = branch_thrust(prop40_branches[i], rpm);
            return t0 + f * (t1 - t0);
        }
    }
    return branch_thrust(prop40_branches[nb-1], rpm);
}

// 40in cruise prop shaft power [W] at (rpm, wind speed)
static inline float prop40_power(float rpm, float v)
{
    v = constrain_float(v, 0.0f, 50.0f);
    const uint8_t nb = sizeof(prop40_branches)/sizeof(prop40_branches[0]);
    if (v <= prop40_branches[0].v) {
        return branch_power(prop40_branches[0], rpm);
    }
    if (v >= prop40_branches[nb-1].v) {
        return branch_power(prop40_branches[nb-1], rpm);
    }
    for (uint8_t i=1; i<nb; i++) {
        if (v <= prop40_branches[i].v) {
            const float f = (v - prop40_branches[i-1].v) /
                            (prop40_branches[i].v - prop40_branches[i-1].v);
            const float p0 = branch_power(prop40_branches[i-1], rpm);
            const float p1 = branch_power(prop40_branches[i], rpm);
            return p0 + f * (p1 - p0);
        }
    }
    return branch_power(prop40_branches[nb-1], rpm);
}

// 60in lift prop static thrust [N]
static inline float prop60_thrust(float rpm)
{
    return interp_by_rpm(prop60_rpm, prop60_thrust_N,
                         sizeof(prop60_rpm)/sizeof(prop60_rpm[0]), rpm);
}

// 60in lift prop shaft power [W]
static inline float prop60_power(float rpm)
{
    return interp_by_rpm(prop60_rpm, prop60_power_W,
                         sizeof(prop60_rpm)/sizeof(prop60_rpm[0]), rpm);
}

/*
  Signed reaction torque [N·m] lookup.

  Sign convention: the caller passes the motor's yaw_factor exactly as it
  appears in the frame json (df200.json motor*_yaw, +/-, e.g. +9/-9 ...
  normalized to +1/-1), so the sign of the returned torque is consistent
  with the stock `yaw_factor * command * yaw_scale` formula: yaw_factor
  encodes the physical CW/CCW installation direction of that propeller,
  the lookup only replaces the (previously proxy-scaled) magnitude.

  Replaces the stock proxy 0.05*diagonal_size*thrust*command, which for
  DF-200 over-estimates the reaction torque by ~15-38x (see
  docs/df200_model_alignment_plan.md §2-D and docs/df200_motor_data.md).
*/

// 60in lift prop, static: signed shaft torque [N·m]
static inline float prop60_torque(float rpm, float yaw_factor)
{
    return yaw_factor * interp_by_rpm(prop60_rpm, prop60_torque_Nm,
                                      sizeof(prop60_rpm)/sizeof(prop60_rpm[0]), rpm);
}

// signed torque [N·m] from one 40in wind-speed branch
static inline float branch_torque(const Prop40Branch &b, float rpm, float yaw_factor)
{
    if (rpm <= 0.0f) {
        return 0.0f;
    }
    const Prop40Entry &e0 = b.entries[0];
    if (rpm <= e0.rpm) {
        return yaw_factor * e0.torque_Nm * sq(rpm / e0.rpm);
    }
    const Prop40Entry &en = b.entries[b.n-1];
    if (rpm >= en.rpm) {
        return yaw_factor * en.torque_Nm;
    }
    for (uint8_t i=1; i<b.n; i++) {
        const Prop40Entry &e = b.entries[i];
        if (rpm <= e.rpm) {
            const Prop40Entry &p = b.entries[i-1];
            const float f = (rpm - p.rpm) / (e.rpm - p.rpm);
            return yaw_factor * (p.torque_Nm + f * (e.torque_Nm - p.torque_Nm));
        }
    }
    return yaw_factor * en.torque_Nm;
}

// 40in cruise prop, signed shaft torque [N·m] at (rpm, wind speed).
// v clamped to [0,50], same as thrust/power.
static inline float prop40_torque(float rpm, float v, float yaw_factor)
{
    v = constrain_float(v, 0.0f, 50.0f);
    const uint8_t nb = sizeof(prop40_branches)/sizeof(prop40_branches[0]);
    if (v <= prop40_branches[0].v) {
        return branch_torque(prop40_branches[0], rpm, yaw_factor);
    }
    if (v >= prop40_branches[nb-1].v) {
        return branch_torque(prop40_branches[nb-1], rpm, yaw_factor);
    }
    for (uint8_t i=1; i<nb; i++) {
        if (v <= prop40_branches[i].v) {
            const float f = (v - prop40_branches[i-1].v) /
                            (prop40_branches[i].v - prop40_branches[i-1].v);
            const float q0 = branch_torque(prop40_branches[i-1], rpm, yaw_factor);
            const float q1 = branch_torque(prop40_branches[i], rpm, yaw_factor);
            return q0 + f * (q1 - q0);
        }
    }
    return branch_torque(prop40_branches[nb-1], rpm, yaw_factor);
}

} // namespace DF200MotorTable
} // namespace SITL
