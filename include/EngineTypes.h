/*
 DYNAMICENGINE3D
 AI-Assisted Soft-Body Physics for Unity3D
 By: Elitmers
*/

#pragma once

#include <stdint.h>

   // Core math types (16-byte aligned for SIMD-friendly layout)
struct alignas(16) Vector3
{
    float x, y, z, w; // 16 bytes
};

struct alignas(16) Quaternion
{
    float x, y, z, w; // 16 bytes
};

struct FaceData
{
    int32_t nodeA;
    int32_t nodeB;
    int32_t nodeC;
};

struct NativeInt2 { int x; int y; };

struct alignas(16) NativeSoftBodyData {
    float* predictedX;
    float* predictedY;
    float* predictedZ;
    float* currentX;
    float* currentY;
    float* currentZ;
    float* previousX;
    float* previousY;
    float* previousZ;
    float* masses;
    uint8_t* isPinned;
    FaceData* faces;
    int nodeCount;
    int faceCount;
    float radius;
    float restitution;
    float staticFriction;
    float slidingFriction;
    int layer;
    int collisionLayerMask;
    int id;
    
    // 12 bytes padding to match C# layout
    float _pad0;
    float _pad1;
    float _pad2;

    Vector3 boundsMin;
    Vector3 boundsMax;
};

struct SoftBodyInstance;

// Physics primitives
struct BeamData
{
    int32_t nodeA;
    int32_t nodeB;
    float   compliance;
    float   damping;
    float   restLength;
    float   originalRestLength;
    float   plasticityThreshold;
    float   plasticityRate;
    float   maxDeformation;
    float   minLength;
    float   maxLength;
    float   strength;
    float   lagrangeMultiplier;
    float   forceEMA;
    // Edge sliding specific
    int32_t slidingNode;
    int32_t edgeNodeA;
    int32_t edgeNodeB;
    float   targetPerpDistance;
    Vector3 initialPerpDir;
    // Flags
    int32_t isActive;
    int32_t isCrossBody;
    int32_t isBroken;
    int32_t isEdgeSliding;
    SoftBodyInstance* bodyA;
    SoftBodyInstance* bodyB;
};

// Static collider types
enum ColliderType : int32_t
{
    COL_SPHERE = 0,
    COL_BOX = 1,
    COL_CAPSULE = 2,
    COL_TRIANGLE = 3
};

// 16-byte aligned to keep Vector3/Quaternion fields naturally aligned
struct alignas(16) StaticColliderData
{
    int32_t    type;                 // 4 bytes
    float      _pad0[3];             // pad to 16-byte boundary
    Vector3    center;               // 16 bytes
    Vector3    size;                 // 16 bytes (box half-size = size * 0.5)
    Vector3    axis;                 // 16 bytes (capsule axis)
    Quaternion rotation;             // 16 bytes (box rotation)
    Vector3    v0;                   // 16 bytes (triangle vertex 0)
    Vector3    v1;                   // 16 bytes (triangle vertex 1)
    Vector3    v2;                   // 16 bytes (triangle vertex 2)
    float      radius;               // 4 bytes (sphere/capsule radius)
    float      height;               // 4 bytes (capsule height)
    float      staticFriction;       // 4 bytes
    float      slidingFriction;      // 4 bytes
    float      restitution;          // 4 bytes
    int32_t    layer;                // 4 bytes
    float      _pad1[2];             // pad to 16-byte boundary
};

// 16-byte aligned, matches C# NativeInfluenceData with explicit 8-byte padding between (nodeIndex, weight) and localOffset
struct alignas(16) InfluenceData
{
    int32_t nodeIndex;     // 4 bytes
    float   weight;        // 4 bytes
    float   _pad0;         // 4 bytes
    float   _pad1;         // 4 bytes
    Vector3 localOffset;   // 16 bytes (aligned to offset 16)
};

struct CollisionPair
{
    int32_t nodeA;
    int32_t nodeB;
};

struct alignas(16) Matrix4x4
{
    float m[16]; // column-major: m[col*4 + row]
};