// world.hpp — b2World wrapper: ground, arena walls, ice/grip patches, contact listener,
// fixed-dt accumulator step. Phase 0 harness; frozen after acceptance 0.1-0.7 pass.
#pragma once

#include <box2d/box2d.h>
#include <cstdint>
#include <vector>

#include "bodies.hpp"

namespace sandbox {

// Instantaneous record of one contact event (RAW — no smoothing, no Δ stored).
struct ContactEvent {
    int32_t idA;        // ObjMeta.id of fixture body A (-1 if none)
    int32_t idB;
    ObjType typeA;
    ObjType typeB;
    float normalImpulse; // N*s, instantaneous value at end-step
    float tangentImpulse;
    bool  touching;
};

// Counts ground contacts per body id (foot-contact sensing in Phase 1+; generic now).
class World {
public:
    explicit World(b2Vec2 gravity);
    ~World();

    World(const World&) = delete;
    World& operator=(const World&) = delete;

    // --- construction helpers (idempotent scene assembly from the parameter table) ---
    void buildArena();               // long ground strip + static walls (mu 0.55)
    void addIcePatch(float cx, float halfW);    // static top surface, mu 0.01
    void addGripPatch(float cx, float halfW);   // static top surface, mu 0.85

    // --- stepping (DETERMINISM law: world stepped EXACTLY once per fixed dt) ---
    // Accumulator API for main.cpp (real-time feed):
    void update(double realDtSeconds);          // consumes accumulator, calls step() 0..n times
    // Exact API for acceptance tests (no wall-clock involved):
    void step();                                 // exactly one b2_Step of params::fixed_dt

    int64_t stepCount() const { return step_count_; }
    double  simTime()   const { return double(step_count_) * params_fixed_dt(); }

    b2WorldId raw() const { return world_id_; }

    // Per-body cumulative external impulse on the torso side etc. kept minimal here:
    // expose the last-step contact events (cleared each step start) for audit/logging.
    const std::vector<ContactEvent>& lastStepEvents() const { return events_this_step_; }

    // Ground-contact count per body id as of the current step (touching manifold points).
    int groundContactsFor(int32_t id) const;

private:
    void beginContact(b2ContactId contact);
    void endContact(b2ContactId contact);
    void preSolve(b2ContactId contact, const b2Manifold* oldManifold);

    static void trampolineBegin(b2ContactId c, void* ud);
    static void trampolineEnd(b2ContactId c, void* ud);
    static void trampolinePreSolve(b2ContactId c, const b2Manifold* old, void* ud);
    static void trampolinePostSolve(b2ContactId c, const b2ContactImpulse* imp, void* ud);

    b2WorldId world_id_;
    b2BodyId  ground_body_{};
    double  accumulator_ = 0.0;
    int64_t step_count_  = 0;

    std::vector<ContactEvent> events_this_step_;
    std::vector<ContactEvent> pending_events_;     // filled during solve
    std::vector<b2ContactId>  active_contacts_;    // ids seen this step (for postSolve capture)

    // touching-state map for groundContactsFor(): bodyId -> count of touching ground contacts
    struct TouchKey { b2BodyId a; b2BodyId b; };
    std::vector<std::pair<TouchKey, bool>> touch_state_;
};

double params_fixed_dt();  // exposed so main/acceptance share the constant source

} // namespace sandbox
