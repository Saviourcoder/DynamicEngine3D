/*
 DYNAMICENGINE3D
 AI-Assisted 3D Physics Engine
 By: Elitmers
*/

#pragma once
#include "Export.h"
#include "EngineTypes.h"
#include "MathUtils.h"
#include "Constants.h"
#include <vector>
#include <cmath>
#include <algorithm>

// Physics kernels implemented in Solver.cpp
extern "C" EXPORT void Integrate(float* cx, float* cy, float* cz, float* px, float* py, float* pz,
    float* predX, float* predY, float* predZ, const uint8_t* pinned, int count, float dt, float damping, float gravity);

extern "C" EXPORT void ApplyPressure(float* predX, float* predY, float* predZ,
    const float* cx, const float* cy, const float* cz, const float* masses, const uint8_t* pinned,
    int count, float pressure, float dt);

extern "C" EXPORT void SolveConstraints(float* predX, float* predY, float* predZ,
    const float* px, const float* py, const float* pz,
    const BeamData* beams, float* multipliers,
    const float* invMasses, const uint8_t* isPinned,
    const float* alphaTildeCache, int beamCount, float dt);

extern "C" EXPORT void SolveConstraintsColored(float* predX, float* predY, float* predZ,
    const float* px, const float* py, const float* pz,
    const BeamData* beams, float* multipliers,
    const float* invMasses, const uint8_t* isPinned,
    const float* alphaTildeCache,
    const int* colorOffsets, const int* colorBeamIndices, int numColors,
    int beamCount, float dt);

extern "C" EXPORT void FinalizePositions(float* cx, float* cy, float* cz, float* px, float* py, float* pz,
    const float* predX, const float* predY, const float* predZ, const uint8_t* pinned, int count);

extern "C" EXPORT void ApplyPlasticity(const float* predX, const float* predY, const float* predZ,
    BeamData* beams, int beamCount, float dt);

extern "C" EXPORT void UpdateNodeRotations(Quaternion* currentRotations,
    const float* cx, const float* cy, const float* cz,
    const int* neighborIndices, const int* neighborOffsets,
    const Vector3* restOffsetsCache, int nodeCount);

void SolveCrossBodyBeam(BeamData& beam, float dt);
void SolveEdgeSliding(BeamData& beam, float dt);

// World-space distance constraint (anchor <-> soft-body node).
// Mirrors SolveCrossBodyBeam's XPBD math but treats the anchor as an
// immovable body B (wB = 0). See WorldSpaceConstraint in EngineTypes.h.
void SolveWorldSpaceConstraint(WorldSpaceConstraint& c, float dt);

// Solves internal (colored/parallel above a size threshold, serial below it)
void SolveInterleavedConstraints(SoftBodyInstance** bodies, int count, float dt, int iterations);

class SoftBodyInstance;

struct MotorState
{
    std::vector<int>  axisIndices;
    SoftBodyInstance* baseBody = nullptr;   // woken alongside this body while the motor runs

    float targetRate = 0.0f;
    float maxTorque = 0.0f;
    float propGain = 100.0f;
    bool  enabled = false;
    bool  configured = false;

    Vector3 cachedAxisDirection = { 0.0f, 1.0f, 0.0f, 0.0f };
    bool    axisDirectionInitialized = false;

    float currentAngularVelocity = 0.0f;
};

class SoftBodyInstance
{
public:
    std::vector<float> currentX; std::vector<float> currentY; std::vector<float> currentZ;
    std::vector<float> previousX; std::vector<float> previousY; std::vector<float> previousZ;
    std::vector<float> predictedX; std::vector<float> predictedY; std::vector<float> predictedZ;
    std::vector<float> initialX; std::vector<float> initialY; std::vector<float> initialZ;

    std::vector<Quaternion> currentRot;
    std::vector<float> masses;
    std::vector<float> invMasses;
    std::vector<uint8_t> isPinned;
    std::vector<BeamData> beams;
    std::vector<float> multipliers;
    std::vector<float> alphaTildeCache;
    float internalPressure = 0.0f;
    std::vector<FaceData> faces;
    std::vector<int> nbrOffsets;
    std::vector<int> nbrData;
    std::vector<Vector3> restOffsetsCache;

    // Internal beams grouped by color (CSR, same layout as nbrOffsets/nbrData).
    std::vector<int> colorOffsets;
    std::vector<int> colorBeamIndices;
    std::vector<int> crossBodyBeams;
    bool             crossBodyBeamsDirty = true;

    // World-space distance constraints tethering this body's nodes to fixed
    // world anchors. Stored separately from beams so the existing cross-body
    // coloring / neighbor caches are unaffected.
    std::vector<WorldSpaceConstraint> worldConstraints;

    float compliance = 1e-3f;
    float defaultDamping = 0.0f;
    float worldRestLengthScale = 1.0f;

    bool canSleep = true;
    bool isSleeping = false;
    float sleepTimer = 0.0f;
    float sleepThreshold = 0.01f;  // m/s - below this velocity considered stationary
    float sleepTime = 1.0f;        // seconds of stillness before sleeping

    MotorState motor;

    int  refNodeForward = 0, refNodeBack = 0, refNodeLeft = 0, refNodeRight = 0;
    Quaternion referenceRotationOffset = { 0.0f, 0.0f, 0.0f, 1.0f };
    Quaternion lastValidRotation = { 0.0f, 0.0f, 0.0f, 1.0f };
    Vector3    centerToPivotLocal = { 0.0f, 0.0f, 0.0f, 0.0f };
    bool       renderBasisValid = false;

    Vector3    prevCenter = { 0.0f, 0.0f, 0.0f, 0.0f };
    Vector3    currCenter = { 0.0f, 0.0f, 0.0f, 0.0f };
    Quaternion prevRotation = { 0.0f, 0.0f, 0.0f, 1.0f };
    Quaternion currRotation = { 0.0f, 0.0f, 0.0f, 1.0f };

    void SyncMultipliers();
    void RebuildNeighborCache();
    void RebuildBeamColoring();
    void ResetLagrangeMultipliers();

    // Cross-body beam indices; lazy rebuild on first use after a topology change.
    const std::vector<int>& CrossBodyBeams();
};

// Lifetime

EXPORT SoftBodyInstance* SoftBody_Create();
EXPORT void SoftBody_Destroy(SoftBodyInstance* body);


// Setup

EXPORT void SoftBody_ClearNodes(SoftBodyInstance* body);
EXPORT void SoftBody_SetupNodes(SoftBodyInstance* body, Vector3* worldPositions, int count);
EXPORT void SoftBody_SetNodeMass(SoftBodyInstance* body, int index, float mass);
EXPORT void SoftBody_SetNodePinned(SoftBodyInstance* body, int index, int pinned);
EXPORT int  SoftBody_IsNodePinned(SoftBodyInstance* body, int index);
EXPORT void SoftBody_GenerateBeamsFromDistance(SoftBodyInstance* body, float connectionDist);
EXPORT int  SoftBody_AddBeam(SoftBodyInstance* bodyA, int nodeA, SoftBodyInstance* bodyB, int nodeB, float compliance, float damping, float restLength, float minL, float maxL, float strength, float plasticityThreshold, float plasticityRate, float maxDeformation, int isEdgeSliding, int edgeA, int edgeB, int slidingNode);
EXPORT void SoftBody_AddBeamBatch(SoftBodyInstance* body, const NativeBeamInit* beams, int count);
EXPORT void SoftBody_SetBeamCompliance(SoftBodyInstance* body, int index, float compliance, float damping);
EXPORT void SoftBody_SetBeamRestLength(SoftBodyInstance* body, int beamIndex, float length);
EXPORT void SoftBody_SetBeamLimits(SoftBodyInstance* body, int beamIndex, float minLength, float maxLength, float strength);
EXPORT void SoftBody_SetBeamEdgeData(SoftBodyInstance* body, int beamIndex, float targetPerpDistance, float dirX, float dirY, float dirZ);
EXPORT void SoftBody_UpdateBeamBulk(SoftBodyInstance* body, float compliance, float damping);
EXPORT void SoftBody_RemoveCrossBodyBeams(SoftBodyInstance* body, SoftBodyInstance* target);
EXPORT int  SoftBody_GetBeamCount(SoftBodyInstance* body);
EXPORT int  SoftBody_IsBeamBroken(SoftBodyInstance* body, int beamIndex);

// World-space distance constraints (anchor <-> soft-body node).
// Mirrors the sb-sb cross-body API, but bodyB is replaced by a fixed world
// anchor point. The anchor is treated as immovable (wB = 0).
EXPORT int  SoftBody_AddWorldSpaceConstraint(SoftBodyInstance* body, int nodeA, Vector3 worldAnchor, float compliance, float damping, float restLength, float minLength, float maxLength, float strength);
EXPORT void SoftBody_RemoveWorldSpaceConstraint(SoftBodyInstance* body, int index);
EXPORT void SoftBody_RemoveAllWorldSpaceConstraints(SoftBodyInstance* body);
EXPORT void SoftBody_SetWorldSpaceAnchor(SoftBodyInstance* body, int index, Vector3 worldAnchor);
EXPORT void SoftBody_SetWorldSpaceCompliance(SoftBodyInstance* body, int index, float compliance, float damping);
EXPORT void SoftBody_SetWorldSpaceRestLength(SoftBodyInstance* body, int index, float restLength);
EXPORT void SoftBody_SetWorldSpaceLimits(SoftBodyInstance* body, int index, float minLength, float maxLength, float strength);
EXPORT int  SoftBody_GetWorldSpaceConstraintCount(SoftBodyInstance* body);
EXPORT int  SoftBody_IsWorldSpaceConstraintBroken(SoftBodyInstance* body, int index);
EXPORT void SoftBody_SetInternalPressure(SoftBodyInstance* body, float pressure);
EXPORT void SoftBody_SetFaces(SoftBodyInstance* body, const int* triangles, int triangleCount);
EXPORT int  SoftBody_GetFaceCount(SoftBodyInstance* body);


// Per-node access

EXPORT int  SoftBody_GetNodeCount(SoftBodyInstance* body);
EXPORT void SoftBody_GetPositions(SoftBodyInstance* body, Vector3* outPositions);
EXPORT void SoftBody_GetRotations(SoftBodyInstance* body, Quaternion* outRotations);
EXPORT Vector3 SoftBody_GetPredictedPos(SoftBodyInstance* body, int index);
EXPORT Vector3 SoftBody_GetCurrentPos(SoftBodyInstance* body, int index);
EXPORT void SoftBody_SetPredictedPos(SoftBodyInstance* body, int index, Vector3 pos);
EXPORT void SoftBody_SetCurrentPos(SoftBodyInstance* body, int index, Vector3 pos);
EXPORT void SoftBody_SetPreviousPos(SoftBodyInstance* body, int index, Vector3 pos);
EXPORT void SoftBody_ApplyWorldForceToNode(SoftBodyInstance* body, int nodeIndex, Vector3 worldForce, float dt);
EXPORT Vector3 SoftBody_GetLinearVelocity(SoftBodyInstance* body, float dt);

EXPORT void* SoftBody_GetPredictedX(SoftBodyInstance* b);
EXPORT void* SoftBody_GetPredictedY(SoftBodyInstance* b);
EXPORT void* SoftBody_GetPredictedZ(SoftBodyInstance* b);
EXPORT void* SoftBody_GetCurrentX(SoftBodyInstance* b);
EXPORT void* SoftBody_GetCurrentY(SoftBodyInstance* b);
EXPORT void* SoftBody_GetCurrentZ(SoftBodyInstance* b);
EXPORT void* SoftBody_GetPreviousX(SoftBodyInstance* b);
EXPORT void* SoftBody_GetPreviousY(SoftBodyInstance* b);
EXPORT void* SoftBody_GetPreviousZ(SoftBodyInstance* b);
EXPORT void* SoftBody_GetMasses(SoftBodyInstance* b);
EXPORT void* SoftBody_GetPinned(SoftBodyInstance* b);

// Sleeping

EXPORT void SoftBody_Wake(SoftBodyInstance* body);
EXPORT void SoftBody_Sleep(SoftBodyInstance* body);
EXPORT int  SoftBody_IsSleeping(SoftBodyInstance* body);
EXPORT void SoftBody_SetSleepThreshold(SoftBodyInstance* body, float threshold);
EXPORT void SoftBody_SetSleepTime(SoftBodyInstance* body, float time);
EXPORT void SoftBody_SetCanSleep(SoftBodyInstance* body, int canSleep);

// Motor

EXPORT void SoftBody_ConfigureMotor(SoftBodyInstance* body, SoftBodyInstance* baseBody, const int* axisIndices, int axisCount, float propGain);
EXPORT void SoftBody_ClearMotor(SoftBodyInstance* body);
EXPORT void SoftBody_SetMotorDrive(SoftBodyInstance* body, float targetRate, float maxTorque, int enabled);
EXPORT float SoftBody_GetMotorAngularVelocity(SoftBodyInstance* body);
EXPORT Vector3 SoftBody_GetMotorAxisDirection(SoftBodyInstance* body);
EXPORT Vector3 SoftBody_GetMotorAxisCenter(SoftBodyInstance* body);
EXPORT float SoftBody_EvaluateMotor(SoftBodyInstance* body, Vector3 axisCenter, Vector3 axisDirection, float targetRate, float maxTorque, float propGain, float dt, int applyTorque);


// Render pose 

EXPORT void SoftBody_SetRenderBasis(SoftBodyInstance* body, int forwardNode, int backNode, int leftNode, int rightNode, Quaternion rotationOffset, Vector3 centerToPivotLocal);
EXPORT void SoftBody_CaptureRenderPose(SoftBodyInstance* body);
EXPORT void SoftBody_GetRenderPose(SoftBodyInstance* body, float alpha, Vector3* outPosition, Quaternion* outRotation);
EXPORT Vector3 SoftBody_GetCenter(SoftBodyInstance* body);
EXPORT Quaternion SoftBody_GetBasisRotation(SoftBodyInstance* body);
EXPORT Vector3 SoftBody_GetInterpolatedNodePosition(SoftBodyInstance* body, int index, float alpha);


// Simulation steps

EXPORT void SoftBody_UpdateNodeRotations(SoftBodyInstance* body);
EXPORT void SoftBody_CheckAndBreakConstraints(SoftBodyInstance* body, float dt);
EXPORT void SoftBody_SolveCrossBodyStep(SoftBodyInstance* body, float dt, int iterations);
EXPORT void SoftBody_SolveWorldSpaceStep(SoftBodyInstance* body, float dt, int iterations);
EXPORT void SoftBody_FinalizeStep(SoftBodyInstance* body);
EXPORT void SoftBody_IntegrateBatch(SoftBodyInstance** bodies, int count, float dt, float gravity, int isFirstSubstep);
EXPORT void SoftBody_SolveConstraintsBatch(SoftBodyInstance** bodies, int count, float dt, int iterations);
EXPORT void SoftBody_ApplyPlasticityBatch(SoftBodyInstance** bodies, int count, float dt);
EXPORT void SoftBody_FinalizeBatch(SoftBodyInstance** bodies, int count, float dt);
EXPORT void SoftBody_UpdateSleepIslands(SoftBodyInstance** bodies, int count, float dt);

void SoftBody_StepMotor(SoftBodyInstance* body, float dt);


// Engine-wide

EXPORT void Physics_SetWorkerThreads(int threads);