/*
 DYNAMICENGINE3D
 AI-Assisted Soft-Body Physics for Unity3D
 By: Elitmers
*/

#include "RigidBodyInstance.h"
#include "SoftBodyInstance.h"
#include "Collisions.h"
#include "Error.h"

#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>

// =========================================================================
// Numerical stability guards (root cause of the TimeScale crash).
//
// At small TimeScale the engine is fed a very small `dt` (e.g. 0.00025 at
// TimeScale = 0.1). XPBD's alphaTilde = compliance / dt^2 then explodes,
// the per-iteration correction ratio collapses to ~0.994, and after 24
// iterations per FixedUpdate the constraint is barely solved - positions
// drift, sqrt() gets fed NaN, and the NaN propagates to every connected
// node in one solve pass. By the time FinalizePositions' IsFinite() guard
// runs at substep-end, every node is already poisoned.
//
// Fix: clamp the native sub_dt to a safe floor. If the caller asks for a
// sub_dt below the floor, we substitute the floor and rescale gravity so
// the *physical* gravity impulse applied this substep is unchanged.
// We also detect NaN/Inf in any active soft body's positions between
// substeps and recover by snapping predicted/current back to previous.
// =========================================================================

// 1/600 s = ~1.67 ms. Below this, single-precision XPBD math starts losing
// meaningful bits and the per-iteration correction ratio
// (alphaTilde / (wSum + alphaTilde)) collapses to ~1.0, meaning the
// constraint solver effectively stops solving.
static constexpr float kMinSafeSubDt = 1.0f / 600.0f;

struct RegisteredBody
{
    bool isRigid = false;
    RigidBodyInstance* rigid = nullptr;
    SoftBodyInstance* soft = nullptr;
    int id = 0;

    float restitution = 0.5f, staticFriction = 0.5f, slidingFriction = 0.4f;
    float radius = 0.01f;
    int layer = 0, collisionMask = ~0;
    bool active = false;
};

static std::vector<RegisteredBody>       g_Registry;
static std::unordered_map<void*, size_t> g_HandleToEntry;

static std::vector<NativeSoftBodyData> g_FrameDescs;
static std::vector<RegisteredBody*>    g_FrameDescOwners; // entry behind each descriptor (wake plumbing)
static std::vector<SoftBodyInstance*>  g_FrameSoft;
static std::vector<RigidBodyInstance*> g_FrameRigid;       // all active rigids (collision layer input)
static std::vector<RigidBodyInstance*> g_FrameRigidDynamic;
static std::vector<NodePairExclusion>  g_CrossBodyExclusionScratch;

static RegisteredBody* FindEntry(void* handle)
{
    auto it = g_HandleToEntry.find(handle);
    return it == g_HandleToEntry.end() ? nullptr : &g_Registry[it->second];
}

EXPORT void Registry_Unregister(void* handle);

EXPORT void Registry_Reset()
{
    g_Registry.clear();
    g_HandleToEntry.clear();
    g_FrameDescs.clear();
    g_FrameDescOwners.clear();
    g_FrameSoft.clear();
    g_FrameRigid.clear();
    g_FrameRigidDynamic.clear();
    g_CrossBodyExclusionScratch.clear();
    Collisions_ResetPersistentCollisionState();
}

EXPORT int Registry_RegisterRigid(RigidBodyInstance* body, int id)
{
    if (!body)
    {
        Error_SetError(ErrorCode::NullHandle);
        return -1;
    }
    if (FindEntry(body)) Registry_Unregister(body);

    RegisteredBody e; e.isRigid = true; e.rigid = body; e.id = id;
    // instanceId is what the rigid layer reads for contact events; keep it in sync.
    body->instanceId = id;
    g_Registry.push_back(e);
    g_HandleToEntry[body] = g_Registry.size() - 1;
    return (int)g_Registry.size() - 1;
}

EXPORT int Registry_RegisterSoft(SoftBodyInstance* body, const void* faces, int faceCount, int id)
{
    if (!body)
    {
        Error_SetError(ErrorCode::NullHandle);
        return -1;
    }

    if (FindEntry(body)) Registry_Unregister(body);

    RegisteredBody e; e.isRigid = false; e.soft = body; e.id = id;
    g_Registry.push_back(e);
    g_HandleToEntry[body] = g_Registry.size() - 1;

    // Face topology arrives at registration (Unity's NativeFaceData); face collision needs it.
    if (faces && faceCount > 0)
    {
        const FaceData* src = static_cast<const FaceData*>(faces);
        body->faces.assign(src, src + faceCount);
    }

    return (int)g_Registry.size() - 1;
}

EXPORT void Registry_Unregister(void* handle)
{
    auto it = g_HandleToEntry.find(handle);
    if (it == g_HandleToEntry.end())
    {
        Error_SetError(ErrorCode::UnknownHandle);
        return;
    }

    size_t idx = it->second, last = g_Registry.size() - 1;
    if (idx != last)
    {
        void* movedHandle = g_Registry[last].isRigid
            ? (void*)g_Registry[last].rigid : (void*)g_Registry[last].soft;
        g_Registry[idx] = g_Registry[last];
        g_HandleToEntry[movedHandle] = idx;
    }
    g_Registry.pop_back();
    // Erase by key, not by iterator: the insert above can rehash and invalidate `it`.
    g_HandleToEntry.erase(handle);
}

EXPORT void Registry_UploadFrameMetadata(const BodyFrameMetadata* meta, int count)
{
    if (!meta || count <= 0) { Error_SetError(ErrorCode::InvalidArgument); return; }

    bool sawUnknown = false;
    for (int i = 0; i < count; ++i)
    {
        RegisteredBody* e = FindEntry(meta[i].body);
        if (!e) { sawUnknown = true; continue; }

        e->restitution = meta[i].restitution;
        e->staticFriction = meta[i].staticFriction;
        e->slidingFriction = meta[i].slidingFriction;
        e->radius = meta[i].radius;
        e->layer = meta[i].layer;
        e->collisionMask = meta[i].collisionMask;
        e->active = meta[i].active != 0;

        if (e->isRigid)
        {
            // The rigid layer reads material/layer state off the body, not the entry — sync it.
            e->rigid->isKinematic = meta[i].isKinematic != 0;
            e->rigid->freezeRotation = meta[i].freezeRotation != 0;
            e->rigid->restitution = meta[i].restitution;
            e->rigid->friction = meta[i].slidingFriction;
            e->rigid->layer = meta[i].layer;
            e->rigid->collisionLayerMask = meta[i].collisionMask;
            RigidBody_SetLinearDamping(e->rigid, meta[i].linearDamping);
            RigidBody_SetAngularDamping(e->rigid, meta[i].angularDamping);
        }
    }

    if (sawUnknown) Error_SetError(ErrorCode::UnknownHandle);
}

EXPORT void Registry_UpdateFaces(void* body, const void* faces, int faceCount)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    RegisteredBody* e = FindEntry(body);
    if (!e) { Error_SetError(ErrorCode::UnknownHandle); return; }
    if (e->isRigid) { Error_SetError(ErrorCode::InvalidArgument); return; } // rigid faces are set once via RigidBody_SetFaceCollisionShape
    if (!faces || faceCount <= 0) { e->soft->faces.clear(); return; }
    const FaceData* src = static_cast<const FaceData*>(faces);
    e->soft->faces.assign(src, src + faceCount);
}

static void RebuildFrameLists()
{
    // Soft bodies only: rigids have their own collision layer and never enter the node cloud.
    g_FrameDescs.clear();
    g_FrameDescOwners.clear();
    g_FrameSoft.clear();
    g_FrameRigid.clear();
    g_FrameRigidDynamic.clear();
    g_CrossBodyExclusionScratch.clear();

    for (RegisteredBody& e : g_Registry)
    {
        if (!e.active) continue;

        if (e.isRigid)
        {
            g_FrameRigid.push_back(e.rigid);
            if (!e.rigid->isKinematic) g_FrameRigidDynamic.push_back(e.rigid);
            continue;
        }

        SoftBodyInstance* sb = e.soft;
        NativeSoftBodyData d{};
        d.restitution = e.restitution; d.staticFriction = e.staticFriction;
        d.slidingFriction = e.slidingFriction; d.radius = e.radius;
        d.layer = e.layer; d.collisionLayerMask = e.collisionMask; d.id = e.id;
        d.currentX = sb->currentX.data();     d.currentY = sb->currentY.data();     d.currentZ = sb->currentZ.data();
        d.predictedX = sb->predictedX.data(); d.predictedY = sb->predictedY.data(); d.predictedZ = sb->predictedZ.data();
        d.previousX = sb->previousX.data();   d.previousY = sb->previousY.data();   d.previousZ = sb->previousZ.data();
        d.masses = sb->masses.data(); d.isPinned = sb->isPinned.data();
        d.faces = sb->faces.data();
        d.faceCount = (int)sb->faces.size();
        d.nodeCount = (int)sb->currentX.size();
        d.isSleeping = sb->isSleeping ? 1 : 0;

        // World AABB over current nodes: Collisions_RigidSoftContacts broadphase reads it first.
        Vector3 bMn = V3(1e30f, 1e30f, 1e30f), bMx = V3(-1e30f, -1e30f, -1e30f);
        for (int n = 0; n < d.nodeCount; ++n)
        {
            bMn = Vector3{ MinF(bMn.x, d.currentX[n]), MinF(bMn.y, d.currentY[n]), MinF(bMn.z, d.currentZ[n]), 0.0f };
            bMx = Vector3{ MaxF(bMx.x, d.currentX[n]), MaxF(bMx.y, d.currentY[n]), MaxF(bMx.z, d.currentZ[n]), 0.0f };
        }
        if (d.nodeCount <= 0) { bMn = V3Zero(); bMx = V3Zero(); }
        d.boundsMin = Vector3{ bMn.x - d.radius, bMn.y - d.radius, bMn.z - d.radius, 0.0f };
        d.boundsMax = Vector3{ bMx.x + d.radius, bMx.y + d.radius, bMx.z + d.radius, 0.0f };
        g_FrameSoft.push_back(sb);

        if (d.nodeCount > 0) { g_FrameDescs.push_back(d); g_FrameDescOwners.push_back(&e); }

        for (int idx : sb->CrossBodyBeams())
        {
            const BeamData& beam = sb->beams[idx];
            if (!beam.isActive || beam.isBroken) continue;
            RegisteredBody* other = FindEntry(beam.bodyB);
            if (!other || !other->active) continue;
            g_CrossBodyExclusionScratch.push_back({ e.id, beam.nodeA, other->id, beam.nodeB });
        }
    }

    Collisions_SetCrossBodyExclusions(g_CrossBodyExclusionScratch.data(), (int)g_CrossBodyExclusionScratch.size());
}

EXPORT void Engine_Step(float dt, int substeps, int iterations, float gravity,
    float collisionEpsilon, float faceNodeCellSize,
    SubstepCallback onSubstep, ContactEventsCallback onContactEvents)
{
    if (dt <= 0.0f) { Error_SetError(ErrorCode::InvalidDt); return; }
    if (substeps <= 0) { Error_SetError(ErrorCode::InvalidSubstep); return; }

    float sub_dt = dt / (float)substeps;
    float gravityScale = 1.0f;
    float effectiveSubDt = sub_dt;
    if (sub_dt < kMinSafeSubDt)
    {
        gravityScale = (sub_dt / kMinSafeSubDt);
        effectiveSubDt = kMinSafeSubDt;
    }
    const float effectiveGravity = gravity * gravityScale * gravityScale;
    const float epsilon = collisionEpsilon / (float)substeps;
    const int rounds = iterations < 1 ? 1 : iterations;

    // Snapshot force/torque before the substep loop — RigidBody_Step clears them each substep.
    RigidBody_ClearContactDebug();
    for (RegisteredBody& e : g_Registry)
    {
        if (!e.isRigid || !e.rigid) continue;
        e.rigid->debugFrameForce = e.rigid->force;
        e.rigid->debugFrameTorque = e.rigid->torque;
    }

    for (int s = 0; s < substeps; ++s)
    {
        RebuildFrameLists();
        const bool isFirstSubstep = (s == 0);

        if (!g_FrameSoft.empty())
            SoftBody_IntegrateBatch(g_FrameSoft.data(), (int)g_FrameSoft.size(), effectiveSubDt, effectiveGravity, isFirstSubstep);

        for (RigidBodyInstance* r : g_FrameRigidDynamic)
            RigidBody_Step(r, effectiveSubDt, effectiveGravity);

        // Broadphase → narrowphase → sequential-impulse solver, after RigidBody_Step.
        if (!g_FrameRigid.empty())
            Collisions_RigidStep(g_FrameRigid.data(), (int)g_FrameRigid.size(), effectiveSubDt, 4);

        for (int round = 0; round < rounds; ++round)
        {
            if (!g_FrameSoft.empty())
                // Pass sub_dt instead of effectiveSubDt to prevent artificial stiffness from kMinSafeSubDt
                SoftBody_SolveConstraintsBatch(g_FrameSoft.data(), (int)g_FrameSoft.size(), sub_dt, 1);

            if (round == 0 && onSubstep) onSubstep(sub_dt);

            if (round == rounds - 1)
            {
                // Rigid-soft coupling; runs before the soft-only resolve so later passes see it.
                if (!g_FrameRigid.empty() && !g_FrameDescs.empty())
                    for (RigidBodyInstance* r : g_FrameRigid)
                        for (NativeSoftBodyData& d : g_FrameDescs)
                            Collisions_RigidSoftContacts(r, &d, effectiveSubDt);

                if (!g_FrameDescs.empty())
                    Collisions_ResolveAll(g_FrameDescs.data(), (int)g_FrameDescs.size(),
                        effectiveSubDt, epsilon, faceNodeCellSize, round == 0, true);

                // Drain wake flags before the next substep's RebuildFrameLists discards them.
                for (size_t i = 0; i < g_FrameDescs.size(); ++i)
                {
                    if (!g_FrameDescs[i].wakeRequested) continue;
                    RegisteredBody* e = g_FrameDescOwners[i];
                    if (e->soft->isSleeping) SoftBody_Wake(e->soft);
                }

                if (onContactEvents) onContactEvents();
            }
        }

        if (!g_FrameSoft.empty())
        {
            SoftBody_ApplyPlasticityBatch(g_FrameSoft.data(), (int)g_FrameSoft.size(), effectiveSubDt);
            SoftBody_FinalizeBatch(g_FrameSoft.data(), (int)g_FrameSoft.size(), effectiveSubDt);
            for (SoftBodyInstance* sb : g_FrameSoft)
                SoftBody_CaptureRenderPose(sb);
        }

        // -- NaN/Inf recovery sweep --------------------------------
        // If anything diverged this substep (despite the clamping above),
        // snap poisoned nodes back to their previous position. This is the
        // last line of defence - the integrator and solver both have their
        // own guards, but a single NaN in a cross-body beam can still slip
        // through if the partner body is mid-solve.
        for (SoftBodyInstance* sb : g_FrameSoft)
        {
            if (!sb) continue;
            const int n = (int)sb->currentX.size();
            for (int i = 0; i < n; ++i)
            {
                if (!std::isfinite(sb->currentX[i]) || !std::isfinite(sb->currentY[i]) || !std::isfinite(sb->currentZ[i]) ||
                    !std::isfinite(sb->predictedX[i]) || !std::isfinite(sb->predictedY[i]) || !std::isfinite(sb->predictedZ[i]))
                {
                    sb->currentX[i] = sb->previousX[i];
                    sb->currentY[i] = sb->previousY[i];
                    sb->currentZ[i] = sb->previousZ[i];
                    sb->predictedX[i] = sb->previousX[i];
                    sb->predictedY[i] = sb->previousY[i];
                    sb->predictedZ[i] = sb->previousZ[i];
                }
            }
        }
    }
}