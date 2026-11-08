/*
 DYNAMICENGINE3D
 AI-Assisted Soft-Body Physics for Unity3D
 By: Elitmers
*/

#pragma once
#include "Export.h"
#include "EngineTypes.h"
#include "MathUtils.h"
#include "Constants.h"
#include <vector>
#include <cstdint>

// ---- Rigid collision geometry (RigidCollision layer) ----
// Local-space relative to the COM; world positions come from the body pose on demand.

enum class RigidShapeType : int
{
    Box = 0,
    Sphere = 1,
    Capsule = 2,
    Trimesh = 3,
};

struct RigidCollisionShape
{
    RigidShapeType type = RigidShapeType::Trimesh;

    // Box
    Vector3 boxHalfExtents = { 0,0,0,0 };  // half-extents, local space

    // Sphere
    float sphereRadius = 0.0f;

    // Capsule (axis = local Y, height = half-length of the segment part)
    float capsuleRadius = 0.0f;
    float capsuleHeight = 0.0f;

    // Trimesh (and the mesh vertices for the legacy upload path)
    std::vector<Vector3> vertices;           // local-space, relative to COM
    std::vector<int>     faces;              // flattened triples: a0,b0,c0, a1,b1,c1, ...
    float                skinRadius = 0.05f; // node collision skin (rigid-soft + debug viz)

    // Local AABB over all geometry; the broadphase transforms it with the pose each step.
    Vector3 localAabbMin = { 0,0,0,0 };
    Vector3 localAabbMax = { 0,0,0,0 };

    bool Empty() const;
    int  FaceCount() const { return (int)faces.size() / 3; }
    void RecomputeLocalAabb();
};

struct RigidBodyInstance
{
    // Linear state
    Vector3   position = { 0,0,0,0 };
    Vector3   prevPosition = { 0,0,0,0 };
    Vector3   linearVelocity = { 0,0,0,0 };
    Vector3   force = { 0,0,0,0 };

    // Angular state
    Quaternion orientation = { 0,0,0,1 };
    Vector3    angularVelocity = { 0,0,0,0 };
    Vector3    torque = { 0,0,0,0 };

    // Split-impulse (pseudo-velocity) accumulators: penetration recovery goes here, never
    // into linearVelocity/angularVelocity, so positional correction cannot add energy.
    Vector3    pseudoLinearVelocity = { 0,0,0,0 };
    Vector3    pseudoAngularVelocity = { 0,0,0,0 };

    // Debug snapshot captured before the substep loop clears the accumulators (Engine_Step start).
    Vector3   debugFrameForce = { 0,0,0,0 };
    Vector3   debugFrameTorque = { 0,0,0,0 };

    // Inertia (local-space diagonal; full world-space tensor computed per step)
    Vector3 localInertia = { 1,1,1,0 };  // diagonal of I_local
    Vector3 invLocalInertia = { 1,1,1,0 };  // 1/localInertia per component

    float mass = 1.0f;
    float invMass = 1.0f;

    float linearDamping = 0.02f;
    float angularDamping = 0.05f;

    bool isKinematic = false;
    bool isActive = true;

    int layer = 0;
    int collisionLayerMask = ~0;

    // For layer/collision filtering on the C# side
    int instanceId = 0;

    bool canSleep = true;
    bool isSleeping = false;
    float sleepTimer = 0.0f;
    float sleepThreshold = 0.01f;
    float sleepTime = 1.0f;

    bool freezeRotation = false;

    Quaternion prevOrientation = QIdentity();
    float      restitution     = 0.0f;
    float      friction        = 0.5f;

    // This IS the rigid layer's collision representation; the old proxy-node arrays are deleted.
    RigidCollisionShape shape;
};

// Helpers
EXPORT RigidBodyInstance* RigidBody_Create();
EXPORT void RigidBody_Destroy(RigidBodyInstance* body);

EXPORT void RigidBody_SetMass(RigidBodyInstance* body, float mass);
EXPORT void RigidBody_SetBoxInertia(RigidBodyInstance* body, float w, float h, float d);
EXPORT void RigidBody_SetSphereInertia(RigidBodyInstance* body, float radius);

EXPORT void RigidBody_SetPosition(RigidBodyInstance* body, Vector3 pos);
EXPORT void RigidBody_SetOrientation(RigidBodyInstance* body, Quaternion q);
EXPORT void RigidBody_SetLinearVelocity(RigidBodyInstance* body, Vector3 v);
EXPORT void RigidBody_SetAngularVelocity(RigidBodyInstance* body, Vector3 w);

EXPORT Vector3    RigidBody_GetPosition(RigidBodyInstance* body);
EXPORT Quaternion RigidBody_GetOrientation(RigidBodyInstance* body);
EXPORT Vector3    RigidBody_GetLinearVelocity(RigidBodyInstance* body);
EXPORT Vector3    RigidBody_GetAngularVelocity(RigidBodyInstance* body);

EXPORT void RigidBody_AddForce(RigidBodyInstance* body, Vector3 worldForce);
EXPORT void RigidBody_AddTorque(RigidBodyInstance* body, Vector3 worldTorque);
EXPORT void RigidBody_AddForceAtPoint(RigidBodyInstance* body, Vector3 worldForce, Vector3 worldPoint);
EXPORT void RigidBody_AddImpulse(RigidBodyInstance* body, Vector3 impulse);
EXPORT void RigidBody_AddAngularImpulse(RigidBodyInstance* body, Vector3 impulse);
EXPORT void RigidBody_ClearForces(RigidBodyInstance* body);
EXPORT void RigidBody_SetLinearDamping(RigidBodyInstance* body, float d);
EXPORT void RigidBody_SetAngularDamping(RigidBodyInstance* body, float d);
EXPORT void RigidBody_SetKinematic(RigidBodyInstance* body, bool kinematic);
EXPORT void RigidBody_Step(RigidBodyInstance* body, float dt, float gravity);
EXPORT void RigidBody_StepAll(RigidBodyInstance** bodies, int count, float dt, float gravity);

EXPORT void RigidBody_Wake(RigidBodyInstance* body);
EXPORT void RigidBody_Sleep(RigidBodyInstance* body);
EXPORT int  RigidBody_IsSleeping(RigidBodyInstance* body);
EXPORT void RigidBody_SetSleepThreshold(RigidBodyInstance* body, float threshold);
EXPORT void RigidBody_SetSleepTime(RigidBodyInstance* body, float time);
EXPORT void RigidBody_SetCanSleep(RigidBodyInstance* body, int canSleep);
EXPORT void RigidBody_SetFreezeRotation(RigidBodyInstance* body, int freeze);

// Legacy C# upload path; converts to the flattened-int face layout.
EXPORT void RigidBody_SetCollisionShape(RigidBodyInstance* body, const Vector3* vertices, int vertexCount, const FaceData* faces, int faceCount, float nodeRadius);

EXPORT void RigidBody_SetFaceCollisionShape(RigidBodyInstance* body,
    const Vector3* localOffsets, int vertCount,
    const int* faceIndices, int faceCount, float radius);

// Primitive shape uploads for the rigid collision layer.
EXPORT void RigidBody_SetBoxCollider(RigidBodyInstance* body, float w, float h, float d);
EXPORT void RigidBody_SetSphereCollider(RigidBodyInstance* body, float radius);
EXPORT void RigidBody_SetCapsuleCollider(RigidBodyInstance* body, float radius, float height);

EXPORT int RigidBody_GetCollisionNodeCount(RigidBodyInstance* body);
EXPORT Vector3 RigidBody_GetCollisionNode(RigidBodyInstance* body, int index);

EXPORT int RigidBody_GetFaceCount(RigidBodyInstance* body);
EXPORT void RigidBody_GetFaces(RigidBodyInstance* body, FaceData* outFaces, int maxCount);

// Rigid-rigid contact debug recording; runs on the Engine_Step calling thread.
void RecordRigidContactDebug(int idA, int idB, const Vector3& point, const Vector3& normal,
    float impulse, float penetration);

// Rigid-rigid contact debug buffer
struct RigidContactDebug
{
    int   idA;           // native id of body A (-1 if unregistered)
    int   idB;           // native id of body B (-1 if unregistered)
    float px, py, pz;    // world contact point
    float nx, ny, nz;    // contact normal, points from A toward B
    float impulse;       // summed normal impulse across solver iterations (N*s)
    float penetration;   // penetration depth at solve time (m)
};

EXPORT int  RigidBody_GetRigidContactDebugCount();
EXPORT bool RigidBody_GetRigidContactDebug(int index, RigidContactDebug* outContact);
EXPORT Vector3 RigidBody_GetDebugForce(RigidBodyInstance* body);
EXPORT Vector3 RigidBody_GetDebugTorque(RigidBodyInstance* body);
Vector3 ApplyWorldInvInertia(const RigidBodyInstance* b, Vector3 worldTorque);

void RigidBody_ClearContactDebug();
