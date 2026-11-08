// bodies.hpp — ObjType/ObjMeta, spawn helpers, userData, filter flags, CCD flags.
// Phase 0: types + generic dynamic-body spawn helpers (proxy scene). NO spider here.
#pragma once

#include <box2d/box2d.h>
#include <cstdint>

namespace sandbox {

// Category of every body in the world. Phase 4's eye/RayHit exports this type.
enum class ObjType : int32_t {
    None = 0,
    Ground,
    Wall,
    Ice,
    GripGround,
    Box,        // generic dynamic prop
    Ball,       // generic dynamic disc
    Projectile, // fast bullet-flagged body
    Debris,     // former agent limb after killLeg/delete
    Food,       // reserved (later phases)
    Hazard,     // reserved (later phases)
    Predator,   // reserved (later phases)
    Torso,      // reserved (Phase 1)
    Femur,      // reserved (Phase 1)
    Tibia,      // reserved (Phase 1)
    Foot,       // reserved (Phase 1)
};

// Per-body metadata stored in b2Body::GetUserData().
// NOTE (RAW SEAM law): this struct carries instantaneous identity/contact info only.
// No smoothing, no derivatives, no cross-step buffers.
struct ObjMeta {
    ObjType type        = ObjType.None;
    bool    isAgentPart = false;  // eye self-exclusion filter (Phase 1+)
    int32_t id          = -1;     // stable id for logging/deletion bookkeeping
};

// Contact-filter groups: sensor fixtures live in their own group so foot sensors can
// report ground contact without colliding with anything (Phase 1+); harmless in Phase 0.
inline constexpr int16_t kGroupSolid  = 1;
inline constexpr int16_t kGroupSensor = 2;

// ---------------------------------------------------------------- spawn helpers
// All masses/inertias come from density + shape (Box2D computes them; never hand-set).

b2Body* spawnStaticBox(b2WorldId world, const b2Vec2& center, float hx, float hy,
                       float friction, float restitution, ObjType type, int32_t id);

b2Body* spawnDynamicBox(b2WorldId world, const b2Vec2& center, float hx, float hy,
                        float density, float friction, float restitution,
                        float linearDamping, float angularDamping, bool bullet,
                        ObjType type, int32_t id);

b2Body* spawnDynamicDisc(b2WorldId world, const b2Vec2& center, float radius,
                         float density, float friction, float restitution,
                         float linearDamping, float angularDamping, bool bullet,
                         ObjType type, int32_t id);

ObjType metaType(const b2Body& body);
bool    metaIsAgentPart(const b2Body& body);

} // namespace sandbox
