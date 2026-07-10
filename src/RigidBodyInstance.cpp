/*
 DYNAMICENGINE3D
 AI-Assisted Soft-Body Physics for Unity3D
 By: Elitmers
*/

#include "RigidBodyInstance.h"
#include <cmath>

// ---- internal helpers ----

static Vector3 ApplyWorldInvInertia(const RigidBodyInstance* b, Vector3 worldTorque)
{
    // alpha_world = R * (I_local^-1 * (R^T * tau_world))
    Quaternion qInv = { -b->orientation.x, -b->orientation.y, -b->orientation.z, b->orientation.w };
    Vector3 localTau = QRotate(qInv, worldTorque);
    Vector3 localAlpha = {
        localTau.x * b->invLocalInertia.x,
        localTau.y * b->invLocalInertia.y,
        localTau.z * b->invLocalInertia.z,
        0.0f
    };
    return QRotate(b->orientation, localAlpha);
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
    if (!body || mass <= 0.0f) return;
    body->mass = mass;
    body->invMass = 1.0f / mass;
}

// Box inertia: I = (1/12) * m * (h²+d², w²+d², w²+h²)
EXPORT void RigidBody_SetBoxInertia(RigidBodyInstance* body, float w, float h, float d)
{
    if (!body) return;
    float m = body->mass;
    float ix = (1.0f / 12.0f) * m * (h * h + d * d);
    float iy = (1.0f / 12.0f) * m * (w * w + d * d);
    float iz = (1.0f / 12.0f) * m * (w * w + h * h);
    body->localInertia = { ix, iy, iz, 0.0f };
    body->invLocalInertia = { 1.0f / ix, 1.0f / iy, 1.0f / iz, 0.0f };
}

// Sphere inertia: I = (2/5) * m * r²
EXPORT void RigidBody_SetSphereInertia(RigidBodyInstance* body, float radius)
{
    if (!body) return;
    float i = (2.0f / 5.0f) * body->mass * radius * radius;
    body->localInertia = { i, i, i, 0.0f };
    body->invLocalInertia = { 1.0f / i, 1.0f / i, 1.0f / i, 0.0f };
}

EXPORT void RigidBody_SetPosition(RigidBodyInstance* body, Vector3 pos)
{
    if (!body) return;
    body->position = body->prevPosition = pos;
}

EXPORT void RigidBody_SetOrientation(RigidBodyInstance* body, Quaternion q)
{
    if (!body) return;
    body->orientation = QNormalize(q);
}

EXPORT void RigidBody_SetLinearVelocity(RigidBodyInstance* body, Vector3 v)
{
    if (!body) return;
    body->linearVelocity = v;
}

EXPORT void RigidBody_SetAngularVelocity(RigidBodyInstance* body, Vector3 w)
{
    if (!body) return;
    body->angularVelocity = w;
}

EXPORT Vector3 RigidBody_GetPosition(RigidBodyInstance* body)
{
    if (!body) return { 0,0,0,0 };
    return body->position;
}

EXPORT Quaternion RigidBody_GetOrientation(RigidBodyInstance* body)
{
    if (!body) return QIdentity();
    return body->orientation;
}

EXPORT Vector3 RigidBody_GetLinearVelocity(RigidBodyInstance* body)
{
    if (!body) return { 0,0,0,0 };
    return body->linearVelocity;
}

EXPORT Vector3 RigidBody_GetAngularVelocity(RigidBodyInstance* body)
{
    if (!body) return { 0,0,0,0 };
    return body->angularVelocity;
}

EXPORT void RigidBody_AddForce(RigidBodyInstance* body, Vector3 worldForce)
{
    if (!body || body->isKinematic) return;
    body->force = Add(body->force, worldForce);
}

EXPORT void RigidBody_AddTorque(RigidBodyInstance* body, Vector3 worldTorque)
{
    if (!body || body->isKinematic) return;
    body->torque = Add(body->torque, worldTorque);
}

EXPORT void RigidBody_AddForceAtPoint(RigidBodyInstance* body, Vector3 worldForce, Vector3 worldPoint)
{
    if (!body || body->isKinematic) return;
    body->force = Add(body->force, worldForce);
    Vector3 r = Sub(worldPoint, body->position);
    body->torque = Add(body->torque, Cross(r, worldForce));
}

EXPORT void RigidBody_AddImpulse(RigidBodyInstance* body, Vector3 impulse)
{
    if (!body || body->isKinematic) return;
    body->linearVelocity = Add(body->linearVelocity, Scale(impulse, body->invMass));
}

EXPORT void RigidBody_AddAngularImpulse(RigidBodyInstance* body, Vector3 impulse)
{
    if (!body || body->isKinematic) return;
    body->angularVelocity = Add(body->angularVelocity, ApplyWorldInvInertia(body, impulse));
}

EXPORT void RigidBody_ClearForces(RigidBodyInstance* body)
{
    if (!body) return;
    body->force = { 0,0,0,0 };
    body->torque = { 0,0,0,0 };
}

EXPORT void RigidBody_SetLinearDamping(RigidBodyInstance* body, float d)
{
    if (body) body->linearDamping = d;
}

EXPORT void RigidBody_SetAngularDamping(RigidBodyInstance* body, float d)
{
    if (body) body->angularDamping = d;
}

EXPORT void RigidBody_SetKinematic(RigidBodyInstance* body, bool kinematic)
{
    if (body) body->isKinematic = kinematic;
}

EXPORT void RigidBody_Step(RigidBodyInstance* body, float dt, float gravity)
{
    if (!body || !body->isActive || body->isKinematic || dt <= 0.0f) return;

    // --- Linear ---
    Vector3 gravForce = { 0.0f, -gravity * body->mass, 0.0f, 0.0f };
    Vector3 totalForce = Add(body->force, gravForce);
    Vector3 linearAcc = Scale(totalForce, body->invMass);

    float linearDrag = std::exp(-body->linearDamping * dt);
    body->linearVelocity.x = body->linearVelocity.x * linearDrag + linearAcc.x * dt;
    body->linearVelocity.y = body->linearVelocity.y * linearDrag + linearAcc.y * dt;
    body->linearVelocity.z = body->linearVelocity.z * linearDrag + linearAcc.z * dt;

    body->prevPosition = body->position;
    body->position.x += body->linearVelocity.x * dt;
    body->position.y += body->linearVelocity.y * dt;
    body->position.z += body->linearVelocity.z * dt;

    // --- Angular ---
    // 1. Calculate local angular velocity first
    Quaternion qInv = { -body->orientation.x, -body->orientation.y, -body->orientation.z, body->orientation.w };
    Vector3 localOmega = QRotate(qInv, body->angularVelocity);

    // 2. Calculate gyroscopic acceleration
    Vector3 gyro = ApplyWorldInvInertia(body, Cross(body->angularVelocity,
        QRotate(body->orientation,
            { body->localInertia.x * localOmega.x,
              body->localInertia.y * localOmega.y,
              body->localInertia.z * localOmega.z, 0.0f })));

    // 3. Calculate torque acceleration and subtract gyroscopic term
    Vector3 alpha = ApplyWorldInvInertia(body, body->torque);
    alpha = Sub(alpha, gyro);

    float angularDrag = std::exp(-body->angularDamping * dt);
    body->angularVelocity.x = body->angularVelocity.x * angularDrag + alpha.x * dt;
    body->angularVelocity.y = body->angularVelocity.y * angularDrag + alpha.y * dt;
    body->angularVelocity.z = body->angularVelocity.z * angularDrag + alpha.z * dt;

    // Integrate orientation: q += 0.5 * [omega, 0] * q * dt
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
    if (!bodies || count <= 0) return;
    for (int i = 0; i < count; ++i)
        RigidBody_Step(bodies[i], dt, gravity);
}