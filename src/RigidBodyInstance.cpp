/*
 DYNAMICENGINE3D
 AI-Assisted Soft-Body Physics for Unity3D
 By: Elitmers
*/

#include "RigidBodyInstance.h"
#include "Collisions.h"
#include "Error.h"
#include <cmath>
#include <vector>
#include <algorithm>

// ---- internal helpers ----

// Shared with Collisions.cpp (declared in RigidBodyInstance.h) � do not redefine there.
Vector3 ApplyWorldInvInertia(const RigidBodyInstance* b, Vector3 worldTorque)
{
    Vector3 localTau = QRotate(QConj(b->orientation), worldTorque);
    Vector3 localAlpha = {
        localTau.x * b->invLocalInertia.x,
        localTau.y * b->invLocalInertia.y,
        localTau.z * b->invLocalInertia.z,
        0.0f
    };
    return QRotate(b->orientation, localAlpha);
}

// Rigid-rigid contact debug recording; runs on the Engine_Step thread, so no atomics.
constexpr int MAX_RIGID_CONTACT_DEBUG = 256;
static RigidContactDebug g_RigidContactDebug[MAX_RIGID_CONTACT_DEBUG];
static int g_RigidContactDebugCount = 0;

void RecordRigidContactDebug(
    int idA, int idB, const Vector3& point, const Vector3& normal,
    float impulse, float penetration)
{
    if (g_RigidContactDebugCount >= MAX_RIGID_CONTACT_DEBUG) return;

    RigidContactDebug& e = g_RigidContactDebug[g_RigidContactDebugCount++];
    e.idA = idA; e.idB = idB;
    e.px = point.x; e.py = point.y; e.pz = point.z;
    e.nx = normal.x; e.ny = normal.y; e.nz = normal.z;
    e.impulse = impulse;
    e.penetration = penetration;
}

void RigidBody_ClearContactDebug()
{
    g_RigidContactDebugCount = 0;
}

EXPORT int RigidBody_GetRigidContactDebugCount()
{
    return g_RigidContactDebugCount;
}

EXPORT bool RigidBody_GetRigidContactDebug(int index, RigidContactDebug* outContact)
{
    if (!outContact) { Error_SetError(ErrorCode::InvalidArgument); return false; }
    if (index < 0 || index >= g_RigidContactDebugCount) { Error_SetError(ErrorCode::InvalidNodeIndex); return false; }
    *outContact = g_RigidContactDebug[index];
    return true;
}

EXPORT Vector3 RigidBody_GetDebugForce(RigidBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return { 0,0,0,0 }; }
    return body->debugFrameForce;
}

EXPORT Vector3 RigidBody_GetDebugTorque(RigidBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return { 0,0,0,0 }; }
    return body->debugFrameTorque;
}

// ---- exports ----

EXPORT RigidBodyInstance* RigidBody_Create()
{
    return new RigidBodyInstance();
}

EXPORT void RigidBody_Destroy(RigidBodyInstance* body)
{
    delete body;
}

EXPORT void RigidBody_SetMass(RigidBodyInstance* body, float mass)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (mass <= 0.0f) { Error_SetError(ErrorCode::InvalidArgument); return; }
    body->mass = mass;
    body->invMass = 1.0f / mass;
}

EXPORT void RigidBody_SetBoxInertia(RigidBodyInstance* body, float w, float h, float d)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    float m = body->mass;
    float ix = (1.0f / 12.0f) * m * (h * h + d * d);
    float iy = (1.0f / 12.0f) * m * (w * w + d * d);
    float iz = (1.0f / 12.0f) * m * (w * w + h * h);
    body->localInertia = { ix, iy, iz, 0.0f };
    body->invLocalInertia = { 1.0f / ix, 1.0f / iy, 1.0f / iz, 0.0f };
}

EXPORT void RigidBodyBridge_SetKinematicPoses(const KinematicPoseUpload* poses, int count)
{
    if (!poses || count <= 0) { Error_SetError(ErrorCode::InvalidArgument); return; }

    bool sawNullBody = false;
    for (int i = 0; i < count; ++i)
    {
        RigidBodyInstance* b = static_cast<RigidBodyInstance*>(poses[i].body);
        if (!b) { sawNullBody = true; continue; }

        // Keep the pre-move pose as the sweep source, or CCD collapses to zero and tunnels.
        const Vector3 oldPos = b->position;
        const Quaternion oldRot = b->orientation;

        RigidBody_SetPosition(b, V3(poses[i].px, poses[i].py, poses[i].pz));
        RigidBody_SetOrientation(b, Quaternion{ poses[i].qx, poses[i].qy, poses[i].qz, poses[i].qw });

        b->prevPosition = oldPos;
        b->prevOrientation = oldRot;
    }

    if (sawNullBody) Error_SetError(ErrorCode::NullHandle);
}

EXPORT void RigidBody_SetSphereInertia(RigidBodyInstance* body, float radius)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (radius <= 0.0f) { Error_SetError(ErrorCode::InvalidArgument); return; }
    float i = (2.0f / 5.0f) * body->mass * radius * radius;
    body->localInertia = { i, i, i, 0.0f };
    body->invLocalInertia = { 1.0f / i, 1.0f / i, 1.0f / i, 0.0f };
}

EXPORT void RigidBody_SetPosition(RigidBodyInstance* body, Vector3 pos)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->position = body->prevPosition = pos;
}

EXPORT void RigidBody_SetOrientation(RigidBodyInstance* body, Quaternion q)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->orientation = QNormalize(q);
}

EXPORT void RigidBody_SetLinearVelocity(RigidBodyInstance* body, Vector3 v)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->linearVelocity = v;
}

EXPORT void RigidBody_SetAngularVelocity(RigidBodyInstance* body, Vector3 w)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->angularVelocity = w;
}

EXPORT Vector3 RigidBody_GetPosition(RigidBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return { 0,0,0,0 }; }
    return body->position;
}

EXPORT Quaternion RigidBody_GetOrientation(RigidBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return QIdentity(); }
    return body->orientation;
}

EXPORT Vector3 RigidBody_GetLinearVelocity(RigidBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return { 0,0,0,0 }; }
    return body->linearVelocity;
}

EXPORT Vector3 RigidBody_GetAngularVelocity(RigidBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return { 0,0,0,0 }; }
    return body->angularVelocity;
}

EXPORT void RigidBody_AddForce(RigidBodyInstance* body, Vector3 worldForce)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (body->isKinematic) return; // kinematic bodies ignore forces by design, not an error

    RigidBody_Wake(body);

    body->force = Add(body->force, worldForce);
}

EXPORT void RigidBody_AddTorque(RigidBodyInstance* body, Vector3 worldTorque)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (body->isKinematic) return;

    RigidBody_Wake(body);

    body->torque = Add(body->torque, worldTorque);
}

EXPORT void RigidBody_AddForceAtPoint(RigidBodyInstance* body, Vector3 worldForce, Vector3 worldPoint)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (body->isKinematic) return;

    RigidBody_Wake(body);

    body->force = Add(body->force, worldForce);
    Vector3 r = Sub(worldPoint, body->position);
    body->torque = Add(body->torque, Cross(r, worldForce));
}

EXPORT void RigidBody_AddImpulse(RigidBodyInstance* body, Vector3 impulse)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (body->isKinematic) return;

    RigidBody_Wake(body);

    body->linearVelocity = Add(body->linearVelocity, Scale(impulse, body->invMass));
}

EXPORT void RigidBody_AddAngularImpulse(RigidBodyInstance* body, Vector3 impulse)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (body->isKinematic) return;

    RigidBody_Wake(body);

    body->angularVelocity = Add(body->angularVelocity, ApplyWorldInvInertia(body, impulse));
}

EXPORT void RigidBody_ClearForces(RigidBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->force = { 0,0,0,0 };
    body->torque = { 0,0,0,0 };
}

EXPORT void RigidBody_SetLinearDamping(RigidBodyInstance* body, float d)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->linearDamping = d;
}

EXPORT void RigidBody_SetAngularDamping(RigidBodyInstance* body, float d)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->angularDamping = d;
}

EXPORT void RigidBody_SetKinematic(RigidBodyInstance* body, bool kinematic)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->isKinematic = kinematic;
}

EXPORT void RigidBody_Wake(RigidBodyInstance* body) {
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->isSleeping = false; body->sleepTimer = 0.0f;
}
EXPORT void RigidBody_Sleep(RigidBodyInstance* body) {
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->isSleeping = true;
}
EXPORT int RigidBody_IsSleeping(RigidBodyInstance* body) {
    if (!body) { Error_SetError(ErrorCode::NullHandle); return 0; }
    return body->isSleeping ? 1 : 0;
}
EXPORT void RigidBody_SetSleepThreshold(RigidBodyInstance* body, float threshold) {
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->sleepThreshold = threshold;
}
EXPORT void RigidBody_SetSleepTime(RigidBodyInstance* body, float time) {
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->sleepTime = time;
}
EXPORT void RigidBody_SetCanSleep(RigidBodyInstance* body, int canSleep) {
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->canSleep = (canSleep != 0);
}

EXPORT void RigidBody_Step(RigidBodyInstance* body, float dt, float gravity)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (dt <= 0.0f) { Error_SetError(ErrorCode::InvalidDt); return; }
    if (!body->isActive || body->isKinematic) return; // expected skip, not an error

    if (body->canSleep) {
        float speed = Length(body->linearVelocity) + Length(body->angularVelocity) * 0.1f; // combined metric
        if (speed < body->sleepThreshold) {
            body->sleepTimer += dt;
            if (body->sleepTimer >= body->sleepTime)
                body->isSleeping = true;
        }
        else {
            body->sleepTimer = 0.0f;
            body->isSleeping = false;
        }
    }

    if (body->isSleeping)
        return;

    // --- Linear ---
    Vector3 gravForce = { 0.0f, -gravity * body->mass, 0.0f, 0.0f };
    Vector3 totalForce = Add(body->force, gravForce);
    Vector3 linearAcc = Scale(totalForce, body->invMass);

    float linearDrag = std::exp(-body->linearDamping * dt);
    body->linearVelocity.x = body->linearVelocity.x * linearDrag + linearAcc.x * dt;
    body->linearVelocity.y = body->linearVelocity.y * linearDrag + linearAcc.y * dt;
    body->linearVelocity.z = body->linearVelocity.z * linearDrag + linearAcc.z * dt;

    body->prevPosition = body->position;
    body->prevOrientation = body->orientation;
    body->position.x += body->linearVelocity.x * dt;
    body->position.y += body->linearVelocity.y * dt;
    body->position.z += body->linearVelocity.z * dt;

    // --- Angular ---
    Vector3 localOmega = QRotate(QConj(body->orientation), body->angularVelocity);

    Vector3 gyro = ApplyWorldInvInertia(body, Cross(body->angularVelocity,
        QRotate(body->orientation,
            { body->localInertia.x * localOmega.x,
              body->localInertia.y * localOmega.y,
              body->localInertia.z * localOmega.z, 0.0f })));

    Vector3 alpha = ApplyWorldInvInertia(body, body->torque);
    alpha = Sub(alpha, gyro);

    float angularDrag = std::exp(-body->angularDamping * dt);
    body->angularVelocity.x = body->angularVelocity.x * angularDrag + alpha.x * dt;
    body->angularVelocity.y = body->angularVelocity.y * angularDrag + alpha.y * dt;
    body->angularVelocity.z = body->angularVelocity.z * angularDrag + alpha.z * dt;

    float wx = body->angularVelocity.x * 0.5f * dt;
    float wy = body->angularVelocity.y * 0.5f * dt;
    float wz = body->angularVelocity.z * 0.5f * dt;

    Quaternion q = body->orientation;
    Quaternion dq = {
        wx * q.w + wy * q.z - wz * q.y,
        wy * q.w + wz * q.x - wx * q.z,
        wz * q.w + wx * q.y - wy * q.x,
       -wx * q.x - wy * q.y - wz * q.z
    };
    body->orientation = QNormalize({
        q.x + dq.x, q.y + dq.y, q.z + dq.z, q.w + dq.w
        });

    // Clear accumulated forces
    RigidBody_ClearForces(body);
}

EXPORT void RigidBody_StepAll(RigidBodyInstance** bodies, int count, float dt, float gravity)
{
    if (!bodies || count <= 0) { Error_SetError(ErrorCode::InvalidArgument); return; }
    for (int i = 0; i < count; ++i)
        RigidBody_Step(bodies[i], dt, gravity);
}

EXPORT void RigidBody_SetFreezeRotation(RigidBodyInstance* body, int freeze)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->freezeRotation = (freeze != 0);
}

// Legacy shape upload: stores the mesh as this body's Trimesh collision geometry.
EXPORT void RigidBody_SetFaceCollisionShape(RigidBodyInstance* body,
    const Vector3* localOffsets, int vertCount,
    const int* faceIndices, int faceCount, float radius)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }

    RigidCollisionShape& s = body->shape;
    s.type = RigidShapeType::Trimesh;
    if (localOffsets && vertCount > 0)
        s.vertices.assign(localOffsets, localOffsets + vertCount);
    else
        s.vertices.clear();
    if (faceIndices && faceCount > 0)
        s.faces.assign(faceIndices, faceIndices + (size_t)faceCount * 3);
    else
        s.faces.clear();
    s.skinRadius = radius;
    s.RecomputeLocalAabb();
}

// Legacy C# entry point: SetFaceCollisionShape but taking FaceData faces and a node radius.
EXPORT void RigidBody_SetCollisionShape(RigidBodyInstance* body,
    const Vector3* vertices, int vertexCount,
    const FaceData* faces, int faceCount, float nodeRadius)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }

    RigidCollisionShape& s = body->shape;
    s.type = RigidShapeType::Trimesh;
    s.skinRadius = nodeRadius;

    s.faces.clear();
    if (faces && faceCount > 0)
    {
        s.faces.resize((size_t)faceCount * 3);
        for (int i = 0; i < faceCount; ++i)
        {
            s.faces[(size_t)i * 3 + 0] = faces[i].nodeA;
            s.faces[(size_t)i * 3 + 1] = faces[i].nodeB;
            s.faces[(size_t)i * 3 + 2] = faces[i].nodeC;
        }
    }

    if (!vertices || vertexCount <= 0)
    {
        s.vertices.clear();
    }
    else
    {
        s.vertices.assign(vertices, vertices + vertexCount);
    }
    s.RecomputeLocalAabb();
}


EXPORT int RigidBody_GetCollisionNodeCount(RigidBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return 0; }
    return (int)body->shape.vertices.size();
}

EXPORT Vector3 RigidBody_GetCollisionNode(RigidBodyInstance* body, int index)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return { 0,0,0,0 }; }
    if (index < 0 || index >= (int)body->shape.vertices.size()) { Error_SetError(ErrorCode::InvalidNodeIndex); return { 0,0,0,0 }; }
    return Add(body->position, QRotate(body->orientation, body->shape.vertices[index]));
}

EXPORT int RigidBody_GetFaceCount(RigidBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return 0; }
    return body->shape.FaceCount();
}

EXPORT void RigidBody_GetFaces(RigidBodyInstance* body, FaceData* outFaces, int maxCount)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (!outFaces || maxCount <= 0) { Error_SetError(ErrorCode::InvalidArgument); return; }
    const int count = body->shape.FaceCount();
    const int n = count < maxCount ? count : maxCount;
    for (int i = 0; i < n; ++i)
    {
        outFaces[i].nodeA = body->shape.faces[(size_t)i * 3 + 0];
        outFaces[i].nodeB = body->shape.faces[(size_t)i * 3 + 1];
        outFaces[i].nodeC = body->shape.faces[(size_t)i * 3 + 2];
    }
}

// ---------------------------------------------------------------------------
// Collision-shape support; the old proxy-node collider pathway is deleted, not disabled.
// ---------------------------------------------------------------------------

bool RigidCollisionShape::Empty() const
{
    switch (type)
    {
    case RigidShapeType::Box:     return boxHalfExtents.x <= 0.0f || boxHalfExtents.y <= 0.0f || boxHalfExtents.z <= 0.0f;
    case RigidShapeType::Sphere:  return sphereRadius <= 0.0f;
    case RigidShapeType::Capsule: return capsuleRadius <= 0.0f;
    case RigidShapeType::Trimesh: return vertices.empty();
    }
    return true;
}

void RigidCollisionShape::RecomputeLocalAabb()
{
    Vector3 mn = V3(1e30f, 1e30f, 1e30f);
    Vector3 mx = V3(-1e30f, -1e30f, -1e30f);

    auto include = [&](const Vector3& p)
        {
            mn = Vector3{ MinF(mn.x, p.x), MinF(mn.y, p.y), MinF(mn.z, p.z), 0.0f };
            mx = Vector3{ MaxF(mx.x, p.x), MaxF(mx.y, p.y), MaxF(mx.z, p.z), 0.0f };
        };

    switch (type)
    {
    case RigidShapeType::Box:
    {
        include(boxHalfExtents);
        include(Scale(boxHalfExtents, -1.0f));
        break;
    }
    case RigidShapeType::Sphere:
    {
        include(V3(sphereRadius, sphereRadius, sphereRadius));
        include(V3(-sphereRadius, -sphereRadius, -sphereRadius));
        break;
    }
    case RigidShapeType::Capsule:
    {
        const float h = capsuleRadius + capsuleHeight;
        include(V3(capsuleRadius, h, capsuleRadius));
        include(V3(-capsuleRadius, -h, -capsuleRadius));
        break;
    }
    case RigidShapeType::Trimesh:
    {
        for (const Vector3& p : vertices) include(p);
        if (vertices.empty()) { mn = V3Zero(); mx = V3Zero(); }
        break;
    }
    }

    localAabbMin = mn;
    localAabbMax = mx;
}

EXPORT void RigidBody_SetBoxCollider(RigidBodyInstance* body, float w, float h, float d)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (w <= 0.0f || h <= 0.0f || d <= 0.0f) { Error_SetError(ErrorCode::InvalidArgument); return; }
    body->shape.type = RigidShapeType::Box;
    body->shape.boxHalfExtents = { w * 0.5f, h * 0.5f, d * 0.5f, 0.0f };
    body->shape.RecomputeLocalAabb();
}

EXPORT void RigidBody_SetSphereCollider(RigidBodyInstance* body, float radius)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (radius <= 0.0f) { Error_SetError(ErrorCode::InvalidArgument); return; }
    body->shape.type = RigidShapeType::Sphere;
    body->shape.sphereRadius = radius;
    body->shape.RecomputeLocalAabb();
}

EXPORT void RigidBody_SetCapsuleCollider(RigidBodyInstance* body, float radius, float height)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (radius <= 0.0f || height < 0.0f) { Error_SetError(ErrorCode::InvalidArgument); return; }
    body->shape.type = RigidShapeType::Capsule;
    body->shape.capsuleRadius = radius;
    // Store full height (StaticColliderData semantics); capsuleHeight is the segment half-length.
    body->shape.capsuleHeight = height * 0.5f;
    body->shape.RecomputeLocalAabb();
}