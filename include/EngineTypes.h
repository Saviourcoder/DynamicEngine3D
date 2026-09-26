/*
 DYNAMICENGINE3D
 AI-Assisted 3D Physics Engine
 By: Elitmers
*/

#pragma once

#include <stdint.h>
#include <cstddef>

// Core math types. Must NOT be alignas(16)
struct Vector3
{
    float x, y, z, w;
};

struct Quaternion
{
    float x, y, z, w;
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
    int32_t isSleeping;
    int32_t wakeRequested;
    uint8_t _pad0[4];    // Explicit pad so boundsMin starts cleanly at offset 144
    Vector3 boundsMin;    // Offset 144
    Vector3 boundsMax;    // Offset 160
};                        // Total size = 176 bytes (see static_assert below)

class SoftBodyInstance;

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
    float   _pad0[2];       // explicit pad: Vector3 used to be alignas(16)
    Vector3 initialPerpDir;
    // Flags
    int32_t isActive;
    int32_t isCrossBody;
    int32_t isBroken;
    int32_t isEdgeSliding;
    SoftBodyInstance* bodyA;
    SoftBodyInstance* bodyB;
};

// World-space distance constraint: tethers a soft-body node to an immovable
// point in world space. Mirrors the XPBD math of SolveCrossBodyBeam, but with
// body B replaced by a static anchor (wB = 0), so only nodeA moves.
//
// Stored on the owning SoftBodyInstance::worldConstraints. bodyA is set
// natively by SoftBody_AddWorldSpaceConstraint (not marshalled from C#).
struct alignas(16) WorldSpaceConstraint
{
    Vector3            worldAnchor;            // offset 0   - immovable anchor ("body B")
    int32_t            nodeA;                  // offset 16  - node index inside bodyA
    float              compliance;             // offset 20  - XPBD compliance (m/N)
    float              damping;                // offset 24  - velocity damping along grad
    float              restLength;             // offset 28  - target |anchor - nodeA|
    float              minLength;              // offset 32  - lower clamp, <=0 => disabled
    float              maxLength;              // offset 36  - upper clamp, <=0 => disabled
    float              strength;               // offset 40  - break force, <=0 => unbreakable
    float              lagrangeMultiplier;     // offset 44  - XPBD multiplier (per-substep)
    float              forceEMA;               // offset 48  - smoothed force for break logic
    int32_t            isActive;               // offset 52
    int32_t            isBroken;               // offset 56
    float              _pad0[1];               // offset 60  - explicit pad so bodyA is 8-aligned
    SoftBodyInstance* bodyA;                  // offset 64  - owning body (8 bytes on 64-bit)
};  // sizeof = 80 (alignas(16) rounds the 72-byte payload up to 80)

// Static collider types
enum ColliderType : int32_t
{
    COL_SPHERE = 0,
    COL_BOX = 1,
    COL_CAPSULE = 2,
    COL_TRIANGLE = 3,
    COL_TERRAIN = 4
};

// 16-byte aligned to keep Vector3/Quaternion fields naturally aligned
struct alignas(16) StaticColliderData
{
    int32_t    type;
    float      _pad0[3];             // pad to 16-byte boundary
    Vector3    center;
    Vector3    size;                 // box half-size = size * 0.5
    Vector3    axis;                 // capsule axis
    Quaternion rotation;             // box rotation
    Vector3    v0;                   // triangle vertex 0
    Vector3    v1;                   // triangle vertex 1
    Vector3    v2;                   // triangle vertex 2
    float      radius;               // sphere/capsule radius
    float      height;               // capsule height
    float      staticFriction;
    float      slidingFriction;
    float      restitution;
    int32_t    layer;
    int32_t    edgeBoundaryMask;     // triangles only
    float      _pad1[1];             // pad to 16-byte boundary

    // Terrain specific data
    float* terrainHeights;
    int32_t    terrainWidth;
    int32_t    terrainLength;
};

// Matches C# NativeInfluenceData: explicit padding puts localOffset at offset 16.
struct alignas(16) InfluenceData
{
    int32_t nodeIndex;
    float   weight;
    float   _pad0;
    float   _pad1;
    Vector3 localOffset;   // must land at offset 16
};

struct BodyFrameMetadata
{
    void* body;
    float   restitution;
    float   staticFriction;
    float   slidingFriction;
    float   radius;
    float   linearDamping;
    float   angularDamping;
    int     layer;
    int     collisionMask;
    uint8_t isKinematic;
    uint8_t freezeRotation;
    uint8_t active;
};

struct KinematicPoseUpload
{
    void* body;
    float px, py, pz;
    float qx, qy, qz, qw;
};

// Matches C# NativeBeamInit (Structs.cs); a batch shares one body.
struct NativeBeamInit
{
    int32_t nodeA;
    int32_t nodeB;
    float   compliance;
    float   damping;
    float   restLength;
    float   minLength;
    float   maxLength;
    float   strength;             // <= 0 → +inf natively
    float   plasticityThreshold;
    float   plasticityRate;
    float   maxDeformation;
    int32_t isEdgeSliding;
    int32_t edgeNodeA;
    int32_t edgeNodeB;
    int32_t slidingNode;
};

// Engine_Step callbacks (Registry.cpp): per-substep, and after contact resolution.
typedef void (*SubstepCallback)(float dt);
typedef void (*ContactEventsCallback)();

// Interop layout guards.
static_assert(sizeof(Vector3) == 16 && sizeof(Quaternion) == 16, "math types must stay 16 bytes");
static_assert(sizeof(BeamData) == 128 && offsetof(BeamData, initialPerpDir) == 80, "BeamData layout changed");
static_assert(
    sizeof(NativeSoftBodyData) == 176 &&
    offsetof(NativeSoftBodyData, boundsMin) == 144,
    "NativeSoftBodyData layout changed"
    );
static_assert(sizeof(StaticColliderData) == 176 && offsetof(StaticColliderData, center) == 16, "StaticColliderData layout changed");
static_assert(sizeof(InfluenceData) == 32 && offsetof(InfluenceData, localOffset) == 16, "InfluenceData layout changed");
static_assert(sizeof(NativeBeamInit) == 60 && offsetof(NativeBeamInit, isEdgeSliding) == 44, "NativeBeamInit layout changed");
static_assert(sizeof(WorldSpaceConstraint) == 80, "WorldSpaceConstraint layout changed");
static_assert(offsetof(WorldSpaceConstraint, worldAnchor) == 0, "WorldSpaceConstraint worldAnchor offset changed");
static_assert(offsetof(WorldSpaceConstraint, nodeA) == 16, "WorldSpaceConstraint nodeA offset changed");
static_assert(offsetof(WorldSpaceConstraint, bodyA) == 64, "WorldSpaceConstraint bodyA offset changed");