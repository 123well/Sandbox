ROLE: You are a senior C++ physics-engine engineer. Build, FROM SCRATCH, a 2D rigid-body
sandbox for a future embodied-AI experiment, in STRICT PHASES. You are in a FRESH context:
everything you need is in this prompt. Do not assume any prior attempt.

CORE PRINCIPLE — PHASE GATING: You must complete Phase 0 and make its acceptance binary PASS
(machine exit code 0, real printed numbers) BEFORE writing any spider code. Then Phase 1, etc.
A phase may not start until the previous phase's acceptance binary passes. Within a phase, once
a module passes its test, FREEZE it: later phases may add to its file but must NOT modify the
passing logic. If a later fix would require editing a frozen passing module, STOP and report
the conflict instead of silently editing. Never overwrite verified behavior.

GLOBAL LAWS (every phase, no exceptions):
- NO MAGIC FORCES. Every acceleration traces to exactly one of: gravity, contact normal,
  contact friction, joint torque, linear/angular damping. The ONLY sanctioned non-obvious force
  is the torso static-friction hold in Phase 3, gated on a real ground contact + zero delivered
  torque. Per-frame SetTransform/SetLinearVelocity/SetAngularVelocity in the control path,
  hardcoded velocities, "helpful" nudges = BUG.
- RIGID BODIES ONLY. No soft-body, no particles, no deformation, no liquid.
- REAL PHYSICS VALUES. Mass/inertia from density+shape (Box2D computes them; no hand-set mass).
  Friction/restitution are real per-fixture coefficients. Forces/impulses in real N / N·s.
  Joint motors in real N·m and rad/s. Gravity is a real vector.
- RAW SEAM. SensorSnapshot/RayHit emit instantaneous, un-smoothed SI values. No EMA, no
  low-pass, no cross-step buffer, no derivative stored in the struct. The core (written later
  by a human) owns all Δ and all surprise. This is load-bearing: smoothing here destroys the
  curiosity signal.
- DETERMINISM. Fixed 1/60 s step via accumulator, world stepped EXACTLY once per dt. Same input
  => byte-identical SensorSnapshot streams and identical COM trajectory. Any spawn randomness
  uses a seeded RNG and is logged. Rest/hold policies use constant thresholds only (no
  time-based state, no counters that depend on history).
- SCOPE. This build is PHYSICS + STANCE + ACTUATION + HOLD + SEAM + DELETION ONLY. Do NOT add
  extra scene-editor features beyond the deletion tool; editor expansion is a LATER pass after
  the engine is locked. Do NOT implement any neural network / learning rule / gradient / loss /
  optimizer. pain/reward/energy/integrity are STUBS clearly marked "core defines this".
- STACK: C++17. Box2D v2.4.x via FetchContent (single source of truth for motion). raylib
  (default X11 backend, runs under XWayland; do NOT force native Wayland, do NOT add
  wlroots/wayland-client). CMake. Linux/Arch. Entry point int main(), no windows.h/WinMain/MSVC
  pragmas. Link: -lraylib -lGL -lm -lpthread -ldl -lrt -lX11. Build:
  cmake -B build && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc).
  Acceptance wired to ctest. ALL files inside sandbox/, logs in sandbox/logs/, NEVER /tmp.

FILE LAYOUT (create exactly this):
  sandbox/
    CMakeLists.txt
    README.md
    PHYSICS_AUDIT.md            (filled per phase with measured numbers)
    src/
      main.cpp                  (accumulator loop, render, input dispatch)
      params.hpp                (THE PARAMETER TABLE below, as constexpr, single source)
      world.hpp/.cpp            (b2World wrapper, ground, walls, ice, contact listener, step)
      bodies.hpp/.cpp           (ObjType/ObjMeta, spawn helpers, userData, filters, CCD flags)
      spider.hpp/.cpp           (torso+4 legs+joints+eye+killLeg+die+postStep hold)
      eye.hpp/.cpp              (5-ray rim eye, raycast callback, self-exclusion filter)
      sensors.hpp/.cpp          (SensorSnapshot producer, RAW, stub pain/reward/energy)
      motors.hpp/.cpp           (MotorCommand = cmdTorque, applyMotorCommand torque-saturation)
      editor.hpp/.cpp           (grab-throw, spawn keys, delete tool, hover highlight, gravity
                                 toggle, explosion, pause, single-step)
      logging.hpp/.cpp          (CSV/JSONL to sandbox/logs/run.csv, per-step fields)
      acceptance.cpp            (standalone test binary: runs every phase's asserts, prints
                                 numbers, exit 0 iff ALL pass; wired to ctest)
    assets/                     (empty, reserved)
    logs/                       (runtime)

=== PARAMETER TABLE (put in params.hpp as constexpr; these are CHOSEN to guarantee a stable
     tripod stance and spider-like proportions — do not invent different ones without re-running
     the Phase 1 stance test) ===
WORLD:
  gravity_default        = (0, -9.81)      // side view, real vector, runtime-toggleable
  fixed_dt               = 1/60
  vel_iterations         = 12
  pos_iterations         = 6
  ground_y               = 0
  arena                  = static walls, friction 0.55, restitution 0.0
  ice_patch              = static, friction 0.01, restitution 0.0   (deliberate low-friction prop)
TORSO (disc):
  shape                  = circle
  R_torso                = 0.30            // metres
  density                = 1.0
  friction               = 0.90
  restitution            = 0.0
  linear_damping         = 0.30
  angular_damping        = 1.00
  spawn_center           = (0.0, 0.55)     // above settle height; gravity settles it into tripod
  // STANCE WINDOW (why these numbers): a radial 4-leg tripod needs torso-center height h with
  //   side-leg hip (at height h) able to reach ground  =>  h <= L_total = 0.60
  //   up-leg hip (at height h+R) CANNOT reach ground    =>  h+R > L_total  =>  h > 0.30
  //   down-leg hip (at height h-R) reaches ground easily (h-R < 0.60 always here)
  // => stable tripod for any h in (0.30, 0.60]. Spawn 0.55 lands inside; solver settles to ~0.45-0.55.
LEGS (4, radial outward at LOCAL azimuths 0,90,180,270 deg; 0=front/right,90=up,180=back/left,270=down):
  femur_len              = 0.35            // hip->knee
  tibia_len              = 0.25            // knee->foot  (L_total = 0.60)
  femur_density          = 0.8 ; friction 0.70 ; restitution 0.0
  tibia_density          = 0.6 ; friction 0.70 ; restitution 0.0
  foot                   = circle r=0.05, density 0.5, friction 1.00 (solid) + separate
                           isSensor ground-contact fixture (no impulse, contact count only)
  all limb bodies        = bullet=true (CCD)
  limb linear_damping    = 1.50 ; angular_damping = 3.00   // limbs must settle, not swing forever
  // PROPORTION GUARD: L_total / R_torso = 2.0. FORBID R_torso > 0.35 or L_total < 0.50 — that
  // reproduces the "big ball, tiny legs" failure where side legs cannot reach the ground.
JOINT LIMITS (degrees, zero = radial-outward rest pose; measured from that rest):
  side-leg hip  (az 0,180): lower = -100, upper = +30   // -100 lets femur hang past straight-down
                                                       // => foot reaches ground at zero torque
  down-leg hip  (az 270)  : lower = -25,  upper = +25   // already points down at rest; small splay
  up-leg hip    (az 90)   : lower = -30,  upper = +120  // can never reach ground (hip too high)
  all knees               : lower = -15,  upper = +120  // bends for future manipulation
  // TUNING RULE (only if Phase 1 stance test fails): widen the side-leg hip LOWER limit by 5°
  // increments and re-run the test until feet contact. Do NOT change topology, do NOT disable
  // collisions, do NOT script the pose. The analytic guarantee above should make this unnecessary.
ACTUATION (per joint, limb properties NOT commands):
  peak_torque_hip        = 4.5 N·m
  peak_torque_knee       = 3.0 N·m
  vmax_hip               = 3.0 rad/s       // actuator velocity ceiling (Hill-like roll-off)
  vmax_knee              = 4.0 rad/s
  torque_deadband        = 0.02 N·m        // per-joint; below => deliver EXACTLY 0 (anti-tremor)
  hold_torque_multiplier = 10              // used ONLY for torso hold gate, NEVER on live limbs
EYE (single, fixed at FRONT rim az=0, rotates with body, NO independent rotation):
  origin                 = torsoPos + R_torso * forward(torsoAngle)
  n_rays                 = 5 ; fan_fov      = 60 deg (centered on forward)
  max_range              = 3.0 m           // export EYE_MAX_RANGE; ray distance exported BOTH
                                           // raw metres AND normalized [0,1]
  ray filter             = EXCLUDE every body with ObjMeta.isAgentPart==true (the eye must never
                           see the spider's own torso/limbs/feet — else self-legs block the fan)
SENSOR/MOTOR SEAM (field names/units fixed, raw):
  SensorSnapshot: proprio[16] (4 legs x hipAngle,hipAngVel,kneeAngle,kneeAngVel; rad, rad/s),
    footContact[4] (0..1), linVel(b2Vec2 m/s), angVel(rad/s), tilt(rad), eye[5] RayHit
    (distance_m raw + distance_norm[0,1] + type + rgb), energy, integrity, pain(stub), reward(stub).
  RayHit: { float distance_m; float distance_norm; ObjType type; float r,g,b; bool hit; }
  MotorCommand: cmdTorque[8] (N·m, [hip,knee]x4, signed). applyMotorCommand maps to
    torque-saturated revolute motor: enableMotor=true, maxMotorTorque=clamp(|cmd|,0,peak),
    motorSpeed=sign(cmd)*vmax; NEVER SetMotorSpeed as a velocity command in the control path;
    NEVER ApplyTorqueToCenter for actuation (actuation is per-joint).
STUBS (mark "core defines this"): energy starts 1.0, drains by base + spike/motor cost;
  integrity starts 1.0, damaged by hazard/explosion; pain = w1*energyDeficit + w2*max(0,dInteg/dt)
  + w3*|tilt| + w4*deadLegCount + w5*stuck (stuck = motorEffort*(1-min(1,|vel|/vref)));
  reward = -dpain + denergy (NO forward-directional term — sandbox must not bake a hidden goal).

=== PHASE 0 — ENGINE HARNESS (NO SPIDER). Gate: acceptance.cpp Phase-0 block exit 0. ===
Build world/ground/walls/ice, accumulator step, contact listener, damping, CCD, friction,
seeded-RNG determinism, logging. Proxy test scene (no spider): falling box, bouncing ball,
two colliding circles, hinge pendulum, 60 m/s projectile vs thin wall, sliding box on ice.
ASSERTS (print numbers):
  0.1 falling box from rest: y-drop after t s ≈ 0.5*9.81*t² within 2% (real gravity).
  0.2 bouncing ball: rebound speed / impact speed ≈ restitution within 5% (real restitution).
  0.3 two circles head-on: total momentum conserved within 1e-3 (real collision response).
  0.4 hinge pendulum: amplitude decays monotonically under angular_damping (no perpetual swing).
  0.5 projectile bullet=true stopped by thin static wall (no tunneling); same with bullet=false
      MUST tunnel (proves CCD flag is what stops it, not luck).
  0.6 box on ice (mu 0.01) slides and decelerates per friction; on ground (mu 0.85) stops fast.
  0.7 two identical seeded runs => byte-identical logged trajectories.
If any fail: fix the harness ONLY (no spider yet). Record numbers in PHYSICS_AUDIT.md Phase 0.

=== PHASE 1 — SPIDER BODY, NO ACTUATION, NO HOLD. Gate: Phase-1 block exit 0. ===
Build torso disc + 4 radial legs + joints (collideConnected=true on ALL hip/knee) + foot sensors
+ eye, per the table. Spawn at (0,0.55), MotorCommand=0, gravity on, run 2.0 s.
ASSERTS (print numbers):
  1.1 no fixture overlap at t=0 (spawn margin); adjacent segments touch-not-overlap at rest.
  1.2 TRIPOD: at steady state foot-ground contacts >= 3; up-leg foot strictly above ground
      (free); COM-x inside contact convex hull with margin >= 0.05 m each side.
  1.3 settles: |torso v|,|torso w| below rest thresholds within <= 1.5 s; topple flag (|angle|>60°
      or contacts<2) = false.
  1.4 limbs hang FREE to their angle limits under gravity+damping (at_limit flag true & stable);
      NO limb frozen mid-air (there is no hold yet, so this must be pure gravity).
  1.5 collideConnected real: sweep two legs into each other => mutual push, no pass-through,
      no explosion. Confirm collideConnected=true on every joint and NOT reverted.
  1.6 delete an agent limb via killLeg => DestroyJoint, limb remains as solid debris that still
      collides with ground/torso/other legs (nothing passes through nothing).
If 1.2/1.3 fail: apply the TUNING RULE (widen side-leg hip lower limit 5° steps, re-run). Do NOT
change topology, do NOT disable collisions, do NOT script pose. Freeze Phase-1 modules on pass.

=== PHASE 2 — ACTUATION (TORQUE). Gate: Phase-2 block exit 0. ===
Wire cmdTorque + torque-saturated motors + per-joint deadband + peak/vmax. No hold yet.
ASSERTS (print numbers):
  2.1 friction-only: on ice (mu~0.01) a fixed 2 N·m hip torque => |COM dx| < 0.05 m over 4 s;
      on grip ground same torque => |COM dx| > 0.10 m over 4 s.
  2.2 no self-propulsion: zero-g airborne flail => |COM dx|,|COM dy| < 0.08 m (internal torques
      cannot propel the disc; residual is solver position-projection noise, report it).
  2.3 controllability probe: seeded fixed pseudo-random torque pattern ("wiggle", same seed every
      run) for 5 s on grip ground => (i) net COM displacement > 0 (body responds to torque);
      (ii) NO rest/hold snapping while any delivered torque != 0 (commands honored); (iii) no
      NaN/explosion. This separates "no policy yet" (fine, core comes later) from "dead control
      path" (bug).
  2.4 grep: no SetMotorSpeed velocity command in control path; no ApplyTorqueToCenter actuation;
      cmdTorque field present.
Freeze Phase-2 modules on pass.

=== PHASE 3 — TORSO HOLD (SCOPED) + JITTER QUANTIFIED. Gate: Phase-3 block exit 0. ===
Add the contact-gated torso static-friction hold in spider postStep. RULES:
  - engage IFF (every live joint's DELIVERED torque == 0, i.e. all commands inside their
    per-joint deadband — NOT a sum-of-commands test) AND (>=1 foot in ground contact) AND
    (no external impulse this step).
  - when engaged: cancel ONLY torso linear+angular momentum via equal-and-opposite impulse
    (= -m*v, -I*w). Live joints output ZERO torque (NO holding torque on limbs — limbs must hang
    free; that is the metabolic cost of spare limbs). Dead/amputated limbs stay solid debris.
  - release on ANY nonzero delivered torque OR an external impulse on the torso from a body that
    is NOT its own limb/foot and NOT the static ground (b2ContactListener normal impulse by
    fixture category; exclude self + ground). Constant thresholds, no leaky counter, no
    time-based state, nothing exposed in SensorSnapshot.
ASSERTS (print numbers):
  3.1 zero torque on ground 2000 steps => torso COM-x drift < 1e-3 m (continuous hold, tighter
      than a counter-gated one).
  3.2 limbs reach angle limits and STAY (at_limit stable); none frozen mid-air by holding torque.
  3.3 metabolic cost: hold an upper limb up (active torque) 2 s then zero => limb falls to limit
      AND log shows energy drained FASTER during hold than during free hang.
  3.4 not invulnerable: explosion on a resting agent RELEASES the hold and moves the body.
  3.5 mid-air zero torque => hold NOT engaged (no ground contact).
  3.6 jitter measured: at rest max limb |w| < 0.05 rad/s and torso |v| < 0.01 m/s steady-state;
      both decay below thresholds within <= 1.5 s. Mitigation = iterations/geometry/limits/
      damping ONLY; collideConnected NOT reverted; NO holding torque re-added to limbs.
Freeze Phase-3 modules on pass.

=== PHASE 4 — SEAM + DELETION + FINAL. Gate: full acceptance.cpp exit 0. ===
Finalize raw SensorSnapshot/RayHit (instantaneous SI; ray both raw m + normalized; EYE_MAX_RANGE
exported; no smoothing/EMA/derivative/Δ in struct — grep sensors.cpp for
ema|moving_average|lowpass|smooth|filter|derivative|delta => only prohibition comments),
MotorCommand seam, readSensors/applyMotorCommand, deletion tool (right-click delete under cursor
+ yellow hover; free props removed cleanly via DestroyBody [Box2D kills attached joints], refs
purged, isBodyAlive guarded; agent limb -> killLeg debris; torso -> die ragdoll, knees intact;
readSensors zeroed raw snapshot after death, postStep early exit).
ASSERTS:
  4.1 determinism: two fresh worlds, identical seeded input => byte-identical SensorSnapshot
      streams member-by-member (incl. ray hits) and identical COM trajectory in run.csv.
  4.2 raw seam intact (grep as above); SI units; ray dual export; EYE_MAX_RANGE exported.
  4.3 deletion: delete each class (food/hazard/predator/terrain/wall/debris) no crash no
      dangling pointer; delete agent limb => debris; delete torso => ragdoll; hover highlight.
  4.4 eye self-exclusion: rays never hit isAgentPart bodies; a camouflaged predator (type=None,
      bg rgb) behind the agent is invisible until the body turns (d(distance)/dt is the only cue).
  4.5 build passes; no files outside sandbox/; no NN/torch/onnx/backprop/loss/optimizer tokens;
      no control-path SetTransform/SetLinearVelocity/SetAngularVelocity; Phase-0..3 frozen
      modules unmodified.

=== VERIFICATION REPORT (mandatory, in PHYSICS_AUDIT.md) ===
For EACH phase and EACH assert id above: [id] | KEEP/REBUILD-or-BUILT | test performed |
MEASURED NUMBERS (paste real stdout, not paraphrase) | pass criterion | PASS/FAIL.
End with a summary table and the full acceptance.cpp stdout. Do NOT claim PASS without numbers
from an actual run. acceptance.cpp must exit non-zero on any failed assert so "pass" is
machine-checkable, not self-reported.

=== FINAL DELIVERABLES ===
- CMake project building & running on Linux/Arch (raylib X11 under XWayland), opening the
  sandbox window with the spider + god-editor + HUD (energy, integrity, pain, reward, tilt,
  per-foot contact, 5 rays fanning from FRONT rim) + per-step CSV log in sandbox/logs/run.csv.
- acceptance.cpp + ctest wiring; PHYSICS_AUDIT.md filled with numbers; README (build/run on
  Linux, key bindings, the SensorSnapshot/MotorCommand seam where the human core plugs in, and a
  note that the core is intentionally NOT included).
- Visible proof on launch: with MotorCommand=0 the spider drops and settles into a stable TRIPOD
  (>=3 contacts, up leg free) and does NOT creep; sweeping two legs gives a real mutual push;
  amputating a leg leaves dangling debris; an explosion knocks the resting body (hold releases).
