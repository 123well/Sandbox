// params.hpp — THE PARAMETER TABLE (single source of truth), per BUILD_SPEC.md.
// Phase 0: WORLD block is live; TORSO/LEGS/JOINT/ACTUATION/EYE blocks are declared now
// and consumed from Phase 1 onward. Values frozen verbatim from the spec.
#pragma once

#include <cstdint>

namespace params {

// ---------------------------------------------------------------- WORLD
inline constexpr float gravity_x        = 0.0f;    // m/s^2, real vector (runtime-toggleable)
inline constexpr float gravity_y        = -9.81f;  // m/s^2
inline constexpr double fixed_dt        = 1.0 / 60.0;   // s, accumulator stepped EXACTLY once
inline constexpr int32_t vel_iterations = 12;
inline constexpr int32_t pos_iterations = 6;
inline constexpr float ground_y         = 0.0f;    // m
// arena: static walls, friction 0.55, restitution 0.0
inline constexpr float wall_friction       = 0.55f;
inline constexpr float wall_restitution    = 0.0f;
// ice patch: static, friction 0.01, restitution 0.0 (deliberate low-friction prop)
inline constexpr float ice_friction        = 0.01f;
inline constexpr float ice_restitution     = 0.0f;
// grip ground patch used by harness friction tests (Phase 0 assert 0.6)
inline constexpr float grip_ground_friction = 0.85f;

// determinism: seeded RNG constant (any spawn randomness must use it and be logged)
inline constexpr uint64_t rng_seed_default = 0xC0FFEE123ULL;

// rest thresholds (constant thresholds only — no time-based state)
inline constexpr float rest_linvel_threshold  = 0.05f;  // m/s
inline constexpr float rest_angvel_threshold  = 0.10f;  // rad/s

// ---------------------------------------------------------------- TORSO (disc) — Phase 1+
inline constexpr float torso_R          = 0.30f;   // m
inline constexpr float torso_density    = 1.0f;    // kg/m^2 (Box2D computes mass/inertia)
inline constexpr float torso_friction   = 0.90f;
inline constexpr float torso_restitution= 0.0f;
inline constexpr float torso_lin_damp   = 0.30f;
inline constexpr float torso_ang_damp   = 1.00f;
inline constexpr float spawn_center_x   = 0.0f;
inline constexpr float spawn_center_y   = 0.55f;   // above settle height; gravity settles tripod

// ---------------------------------------------------------------- LEGS — Phase 1+
inline constexpr float femur_len        = 0.35f;   // hip->knee, m
inline constexpr float tibia_len        = 0.25f;   // knee->foot, m  (L_total = 0.60)
inline constexpr float leg_total_len    = 0.60f;
inline constexpr float femur_density    = 0.8f;
inline constexpr float femur_friction   = 0.70f;
inline constexpr float tibia_density    = 0.6f;
inline constexpr float tibia_friction   = 0.70f;
inline constexpr float foot_radius      = 0.05f;
inline constexpr float foot_density     = 0.5f;
inline constexpr float foot_friction    = 1.00f;   // solid fixture
inline constexpr float limb_lin_damp    = 1.50f;
inline constexpr float limb_ang_damp    = 3.00f;
// PROPORTION GUARD: L_total / R_torso = 2.0. FORBID R_torso > 0.35 or L_total < 0.50.
static_assert(torso_R <= 0.35f, "PROPORTION GUARD: R_torso must not exceed 0.35");
static_assert(leg_total_len >= 0.50f, "PROPORTION GUARD: L_total must be >= 0.50");

// joint limits, degrees, zero = radial-outward rest pose — Phase 1+
inline constexpr float hip_side_lower_deg  = -100.0f, hip_side_upper_deg = 30.0f;
inline constexpr float hip_down_lower_deg  = -25.0f,  hip_down_upper_deg = 25.0f;
inline constexpr float hip_up_lower_deg    = -30.0f,  hip_up_upper_deg   = 120.0f;
inline constexpr float knee_lower_deg      = -15.0f,  knee_upper_deg     = 120.0f;

// ---------------------------------------------------------------- ACTUATION — Phase 2+
inline constexpr float peak_torque_hip  = 4.5f;   // N*m
inline constexpr float peak_torque_knee = 3.0f;   // N*m
inline constexpr float vmax_hip         = 3.0f;   // rad/s
inline constexpr float vmax_knee        = 4.0f;   // rad/s
inline constexpr float torque_deadband  = 0.02f;  // N*m per joint; below => deliver EXACTLY 0
inline constexpr float hold_torque_multiplier = 10.0f; // torso-hold gate ONLY, never live limbs

// ---------------------------------------------------------------- EYE — Phase 1+
inline constexpr int32_t eye_n_rays   = 5;
inline constexpr float eye_fan_fov_deg = 60.0f;   // centered on forward
inline constexpr float eye_max_range_m = 3.0f;    // export EYE_MAX_RANGE

// ---------------------------------------------------------------- STUBS (core defines this)
inline constexpr float stub_energy_start    = 1.0f;  // core defines this
inline constexpr float stub_integrity_start = 1.0f;  // core defines this

} // namespace params

// Seam constant exported for consumers (Phase 4 grep requirement):
#define EYE_MAX_RANGE 3.0f
