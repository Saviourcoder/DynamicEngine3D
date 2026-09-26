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

        // A cross-body beam is a rigid distance constraint between two specific nodes.
        // Letting node-node collision fight that same pair every substep is the instability
        // you're seeing — so tell the collision layer to skip these pairs entirely.
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

    const float sub_dt = dt / (float)substeps;
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
            SoftBody_IntegrateBatch(g_FrameSoft.data(), (int)g_FrameSoft.size(), sub_dt, gravity, isFirstSubstep);

        for (RigidBodyInstance* r : g_FrameRigidDynamic)
            RigidBody_Step(r, sub_dt, gravity);

        // Broadphase → narrowphase → sequential-impulse solver, after RigidBody_Step.
        if (!g_FrameRigid.empty())
            Collisions_RigidStep(g_FrameRigid.data(), (int)g_FrameRigid.size(), sub_dt, 4);

        for (int round = 0; round < rounds; ++round)
        {
            if (!g_FrameSoft.empty())
                SoftBody_SolveConstraintsBatch(g_FrameSoft.data(), (int)g_FrameSoft.size(), sub_dt, 1);

            if (round == 0 && onSubstep) onSubstep(sub_dt);

            if (round == rounds - 1)
            {
                // Rigid-soft coupling; runs before the soft-only resolve so later passes see it.
                if (!g_FrameRigid.empty() && !g_FrameDescs.empty())
                    for (RigidBodyInstance* r : g_FrameRigid)
                        for (NativeSoftBodyData& d : g_FrameDescs)
                            Collisions_RigidSoftContacts(r, &d, sub_dt);

                if (!g_FrameDescs.empty())
                    Collisions_ResolveAll(g_FrameDescs.data(), (int)g_FrameDescs.size(),
                        sub_dt, epsilon, faceNodeCellSize, round == 0, true);

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
            SoftBody_ApplyPlasticityBatch(g_FrameSoft.data(), (int)g_FrameSoft.size(), sub_dt);
            SoftBody_FinalizeBatch(g_FrameSoft.data(), (int)g_FrameSoft.size(), sub_dt);
            for (SoftBodyInstance* sb : g_FrameSoft)
                SoftBody_CaptureRenderPose(sb);
        }
    }
}