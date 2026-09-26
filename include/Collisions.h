/*
 DYNAMICENGINE3D
 AI-Assisted 3D Physics Engine
 By: Elitmers
*/

#pragma once

#include "Export.h"
#include "EngineTypes.h"
#include "RigidBodyInstance.h"
#include <vector>

class SoftBodyInstance;
struct ContactEvent
{
    int   idA;
    int   idB;
    float px, py, pz;  // world contact point (approximate)
    float nx, ny, nz;  // contact normal, points from B toward A
    float normalSpeed; // relative closing speed along the normal
    float tx, ty, tz;  // tangential (sliding) velocity in world space
};

constexpr int COLLISIONS_STATIC_CONTACT_ID = -1;

EXPORT void RemoveStaticBody(int id);
EXPORT void AddStaticBox(int id, float* center, float* size, float* rot, float margin, float sf, float slf, float res, int layer);
EXPORT void AddStaticSphere(int id, float* center, float radius, float margin, float sf, float slf, float res, int layer);
EXPORT void AddStaticCapsule(int id, float* center, float radius, float height, float* axis, float margin, float sf, float slf, float res, int layer);
EXPORT void AddStaticMesh(int id, float* center, float* verts, int vertCount, int* indices, int indexCount, float margin, float sf, float slf, float res, int layer);
EXPORT void AddStaticTerrain(int id, float* center, float* size, float* heights, int width, int length, float margin, float sf, float slf, float res, int layer);
EXPORT void ProcessDirtyStatics();
EXPORT void Collisions_SetIgnoredPairs(const NativeInt2* pairs, int count);
EXPORT int  Collisions_GetContactEventCount();
EXPORT void Collisions_GetContactEvents(ContactEvent* outEvents, int maxCount);
EXPORT void Collisions_BeginFrame();
EXPORT void Collisions_ResetPersistentCollisionState();
EXPORT void ClearStaticColliders();

EXPORT bool Collisions_Raycast(Vector3 origin, Vector3 direction, float maxDistance, int ignoreId,
    int* outHitId, Vector3* outPoint, Vector3* outNormal, int* outFaceIndex);
constexpr int WORLD_HIT_NONE = 0;
constexpr int WORLD_HIT_STATIC = 1;
constexpr int WORLD_HIT_SOFTBODY = 2;

EXPORT bool World_Raycast(
    Vector3 origin, Vector3 direction, float maxDistance,
    int ignoreStaticId, SoftBodyInstance* ignoreSoftBody,
    SoftBodyInstance** softBodies, int softBodyCount, float nodeRadius,
    int* outHitType, int* outStaticId, SoftBodyInstance** outSoftBody, int* outNodeIndex,
    Vector3* outPoint, Vector3* outNormal, int* outFaceIndex, float* outDistance);
EXPORT void Collisions_ResolveAll(
    NativeSoftBodyData* bodies, int bodyCount,
    float dt, float epsilon, float faceNodeCellSize, bool applyRestitution,
    bool shouldResolve);

// ---- Rigid-body collision layer entry points (Collisions.cpp) ----
// Internal to the DLL; not part of the C# ABI.

// One rigid solver step: broadphase → narrowphase → sequential impulses.
void Collisions_RigidStep(RigidBodyInstance** bodies, int count, float dt, int solverIterations);

// Rigid vs one soft body; returns the contact count.
int Collisions_RigidSoftContacts(RigidBodyInstance* rigid, NativeSoftBodyData* soft, float dt);

// World-space AABB of a body's collision shape at its current pose.
void Collisions_RigidShapeWorldAabb(const RigidBodyInstance* body, Vector3& outMin, Vector3& outMax);

struct NodePairExclusion { int idA; int nodeA; int idB; int nodeB; };
void Collisions_SetCrossBodyExclusions(const NodePairExclusion* pairs, int count);

// Shape authoring / queries, called from C# (P/Invoke ABI — names are fixed).
EXPORT void RigidBody_SetCollisionShape2(RigidBodyInstance* body, int type,
    const Vector3* vertices, int vertCount,
    const int* faceIndices, int faceCount, float skinRadius);
EXPORT int RigidBody_GetShapeVertexCount(const RigidBodyInstance* body);
EXPORT int RigidBody_GetShapeVertices(const RigidBodyInstance* body, Vector3* outVerts, int maxCount);