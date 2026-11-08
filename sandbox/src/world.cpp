// world.cpp — Phase 0 harness implementation. Frozen once acceptance 0.1-0.7 pass.
#include "world.hpp"

#include <algorithm>
#include <cmath>

#include "params.hpp"

namespace sandbox {

double params_fixed_dt() { return params::fixed_dt; }

// ---------------------------------------------------------------- construction
World::World(b2Vec2 gravity) {
    b2WorldDef wd = b2DefaultWorldDef();
    wd.gravity = gravity;
    world_id_ = b2CreateWorld(&wd);
}

World::~World() {
    if (b2IsValidWorldId(world_id_)) b2DestroyWorld(world_id_);
}

void World::buildArena() {
    // Long static ground strip at y = ground_y, arena wall friction mu 0.55 rest 0.
    const float halfW = 40.0f, halfH = 0.5f;
    ground_body_ = b2_CreateBodyFrom(
        spawnStaticBox(world_id_, {0.0f, params::ground_y - halfH}, halfW, halfH,
                       params::wall_friction, params::wall_restitution, ObjType.Ground, 0)
            ->GetHandle());
    // Left/right static walls (same material), tall enough for the proxy scene.
    const float wallHalfH = 6.0f;
    spawnStaticBox(world_id_, {-20.0f, params::ground_y + wallHalfH}, 0.25f, wallHalfH,
                   params::wall_friction, params::wall_restitution, ObjType.Wall, 1);
    spawnStaticBox(world_id_, { 20.0f, params::ground_y + wallHalfH}, 0.25f, wallHalfH,
                   params::wall_friction, params::wall_restitution, ObjType.Wall, 2);
}

void World::addIcePatch(float cx, float halfW) {
    // Static thin slab whose TOP face sits exactly at ground_y so bodies slide on ice only.
    const float halfH = 0.05f;
    spawnStaticBox(world_id_, {cx, params::ground_y - halfH}, halfW, halfH,
                   params::ice_friction, params::ice_restitution, ObjType.Ice, 10);
}

void World::addGripPatch(float cx, float halfW) {
    const float halfH = 0.05f;
    spawnStaticBox(world_id_, {cx, params::ground_y - halfH}, halfW, halfH,
                   params::grip_ground_friction, 0.0f, ObjType.GripGround, 11);
}

// ---------------------------------------------------------------- stepping
void World::step() {
    // EXACTLY one step per fixed dt (DETERMINISM law). No sub-stepping tricks.
    events_this_step_.clear();
    pending_events_.clear();
    active_contacts_.clear();
    b2StepOptions opts = b2DefaultStepOptions();
    opts.velocityIterations = params::vel_iterations;
    opts.positionIterations = params::pos_iterations;
    b2_Step(world_id_, opts);
    ++step_count_;
}

void World::update(double realDtSeconds) {
    // Fixed-timestep accumulator: consume real time, step whole dt's only.
    accumulator_ += realDtSeconds;
    while (accumulator_ >= params::fixed_dt) {
        step();
        accumulator_ -= params::fixed_dt;
    }
}

// ---------------------------------------------------------------- contact listener
void World::trampolineBegin(b2ContactId c, void* ud) {
    static_cast<World*>(ud)->beginContact(c);
}
void World::trampolineEnd(b2ContactId c, void* ud) {
    static_cast<World*>(ud)->endContact(c);
}
void World::trampolinePreSolve(b2ContactId c, const b2Manifold* old, void* ud) {
    static_cast<World*>(ud)->preSolve(c, old);
}
void World::trampolinePostSolve(b2ContactId c, const b2ContactImpulse* imp, void* ud) {
    World* w = static_cast<World*>(ud);
    b2BodyId a = b2Contact_GetBody1(c);
    b2BodyId b = b2Contact_GetBody2(c);
    ContactEvent ev{};
    auto fillMeta = [](b2BodyId id, int32_t& outId, ObjType& outType) {
        if (!b2IsBodyIdNull(id)) {
            b2Body* bo = b2::Body::Get(id);
            if (bo) {
                ObjMeta m = bo->GetUserData<ObjMeta>();
                outId = m.id; outType = m.type; return;
            }
        }
        outId = -1; outType = ObjType.None;
    };
    fillMeta(a, ev.idA, ev.typeA);
    fillMeta(b, ev.idB, ev.typeB);
    ev.normalImpulse  = imp ? imp->normalImpulses[0]  : 0.0f;
    ev.tangentImpulse = imp ? imp->tangentImpulses[0] : 0.0f;
    ev.touching = b2Contact_IsTouching(c);
    w->pending_events_.push_back(ev);
}

void World::beginContact(b2ContactId) {}
void World::endContact(b2ContactId) {}
void World::preSolve(b2ContactId c, const b2Manifold*) {
    active_contacts_.push_back(c);
}

int World::groundContactsFor(int32_t id) const {
    // Count touching contacts between body `id` and any static ground-class surface,
    // straight from the CURRENT contact graph (instantaneous, no history).
    int n = 0;
    b2ContactId* contacts = nullptr;
    int count = b2World_GetContactPairs(world_id_, &contacts);
    (void)count;
    // b2World_GetContactPairs is awkward across 2.4.x patch releases; iterate instead via
    // the post-solve event log of this step, which records every solved touching contact.
    for (const auto& ev : events_this_step_) {
        if (!ev.touching) continue;
        bool aIsTarget = (ev.idA == id);
        bool bIsTarget = (ev.idB == id);
        auto isGroundish = [](ObjType t) {
            return t == ObjType.Ground || t == ObjType.Ice ||
                   t == ObjType.GripGround || t == ObjType.Wall;
        };
        if ((aIsTarget && isGroundish(ev.typeB)) || (bIsTarget && isGroundish(ev.typeA))) ++n;
    }
    if (contacts) b2Free(contacts, count * sizeof(b2ContactId));
    return n;
}

// ---------------------------------------------------------------- spawn helpers impl
b2Body* spawnStaticBox(b2WorldId world, const b2Vec2& center, float hx, float hy,
                       float friction, float restitution, ObjType type, int32_t id) {
    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_staticBody;
    bd.position = center;
    b2BodyId bid = b2_CreateBody(&bd);
    b2ShapeRef sh = b2ComputeBoxShape(center, hx, hy, 0.0f);
    b2FixtureDef fd = b2DefaultFixtureDef();
    fd.shape = sh;
    fd.friction = friction;
    fd.restitution = restitution;
    fd.density = 0.0f; // static: mass irrelevant
    b2CreateFixture(bid, &fd);
    ObjMeta m{}; m.type = type; m.isAgentPart = false; m.id = id;
    b2::Body::Get(bid)->SetUserData(m);
    return b2::Body::Get(bid);
}

b2Body* spawnDynamicBox(b2WorldId world, const b2Vec2& center, float hx, float hy,
                        float density, float friction, float restitution,
                        float linearDamping, float angularDamping, bool bullet,
                        ObjType type, int32_t id) {
    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_dynamicBody;
    bd.position = center;
    bd.linearDamping = linearDamping;
    bd.angularDamping = angularDamping;
    bd.enableBullet = bullet;   // CCD flag
    b2BodyId bid = b2_CreateBody(&bd);
    b2Polygon poly = b2MakeBox(hx, hy);
    b2FixtureDef fd = b2DefaultFixtureDef();
    fd.shape = b2PolygonAsShape(poly);
    fd.density = density;       // mass/inertia computed by Box2D from density+shape
    fd.friction = friction;
    fd.restitution = restitution;
    b2CreateFixture(bid, &fd);
    ObjMeta m{}; m.type = type; m.isAgentPart = false; m.id = id;
    b2::Body::Get(bid)->SetUserData(m);
    return b2::Body::Get(bid);
}

b2Body* spawnDynamicDisc(b2WorldId world, const b2Vec2& center, float radius,
                         float density, float friction, float restitution,
                         float linearDamping, float angularDamping, bool bullet,
                         ObjType type, int32_t id) {
    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_dynamicBody;
    bd.position = center;
    bd.linearDamping = linearDamping;
    bd.angularDamping = angularDamping;
    bd.enableBullet = bullet;
    b2BodyId bid = b2_CreateBody(&bd);
    b2Circle circ{{0.0f, 0.0f}, radius};
    b2FixtureDef fd = b2DefaultFixtureDef();
    fd.shape = b2CircleAsShape(circ);
    fd.density = density;
    fd.friction = friction;
    fd.restitution = restitution;
    b2CreateFixture(bid, &fd);
    ObjMeta m{}; m.type = type; m.isAgentPart = false; m.id = id;
    b2::Body::Get(bid)->SetUserData(m);
    return b2::Body::Get(bid);
}

ObjType metaType(const b2Body& body) { return body.GetUserData<ObjMeta>().type; }
bool metaIsAgentPart(const b2Body& body) { return body.GetUserData<ObjMeta>().isAgentPart; }

} // namespace sandbox
