/*
 DYNAMICENGINE3D
 AI-Assisted Soft-Body Physics for Unity3D
 By: Elitmers
*/

#pragma once
#include "EngineTypes.h"
#include "MathUtils.h"
#include "Constants.h"
#include <vector>
#include <cstdint>

#if defined(_WIN32)
#define EXPORT extern "C" __declspec(dllexport)
#else
#define EXPORT extern "C" __attribute__((visibility("default")))
#endif

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