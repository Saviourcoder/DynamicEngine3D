#pragma once
#include "EngineTypes.h"
#include "MathUtils.h"
#include "Constants.h"
#include <vector>
#include <cmath>
#include <algorithm>

#ifdef _OPENMP
#include <omp.h>
#endif

#if defined(_WIN32)
#define EXPORT extern "C" __declspec(dllexport)
#else
#define EXPORT extern "C" __attribute__((visibility("default")))
#endif

// External physics functions implemented
extern "C" void Integrate(Vector3*, Vector3*, Vector3*, const uint8_t*, int, float, float, float);
extern "C" void ApplyPressure(Vector3*, const Vector3*, const float*, const uint8_t*, int, float, float);
extern "C" void ApplyBeamDampingForces(Vector3*, const Vector3*, const BeamData*, const float*, const uint8_t*, int, float);
extern "C" void SolveConstraints(Vector3*, const BeamData*, float*, const float*, const uint8_t*, int, float);
extern "C" void FinalizePositions(Vector3*, Vector3*, const Vector3*, const uint8_t*, int);

class SoftBodyInstance
{
public:
    std::vector<Vector3> currentPos;
    std::vector<Vector3> previousPos;
    std::vector<Vector3> predictedPos;
    std::vector<Vector3> initialPos;
    std::vector<Quaternion> currentRot;
    std::vector<float> masses;
    std::vector<uint8_t> isPinned;
    std::vector<BeamData> beams;
    std::vector<float> multipliers;
    float internalPressure = 0.0f;

    // Caches for rotation job
    std::vector<int> nbrOffsets;
    std::vector<int> nbrData;

    float compliance = 1e-3f;
    float defaultDamping = 0.3f;
    float worldRestLengthScale = 1.0f;

    void SyncMultipliers();
    void RebuildNeighborCache();
};

// EXPORT Function Declarations
EXPORT SoftBodyInstance* SoftBody_Create();
EXPORT void SoftBody_Destroy(SoftBodyInstance* body);
EXPORT void SoftBody_GetPositions(SoftBodyInstance* body, Vector3* outPositions);
EXPORT void SoftBody_GetRotations(SoftBodyInstance* body, Quaternion* outRotations);
EXPORT void SoftBody_ClearNodes(SoftBodyInstance* body);
EXPORT void SoftBody_SetupNodes(SoftBodyInstance* body, Vector3* worldPositions, int count);
EXPORT void SoftBody_GenerateBeamsFromDistance(SoftBodyInstance* body, float connectionDist);
EXPORT void SoftBody_SetBeamCompliance(SoftBodyInstance* body, int index, float compliance, float damping);
EXPORT void SoftBody_ApplyWorldForceToNode(SoftBodyInstance* body, int nodeIndex, Vector3 worldForce, float dt);
EXPORT void SoftBody_ResetLagrangeMultipliers(SoftBodyInstance* body);
EXPORT void SoftBody_CheckAndBreakConstraints(SoftBodyInstance* body, float dt);
EXPORT void SoftBody_SolveCrossBodyStep(SoftBodyInstance* body, float dt);
EXPORT void SoftBody_UpdateNodeRotations(SoftBodyInstance* body);
EXPORT void SoftBody_SetNodePinned(SoftBodyInstance* body, int index, bool pinned);
EXPORT void SoftBody_SetPreviousPos(SoftBodyInstance* body, int index, Vector3 pos);
EXPORT Vector3 SoftBody_GetPredictedPos(SoftBodyInstance* body, int index);
EXPORT void SoftBody_SetBeamRestLength(SoftBodyInstance* body, int beamIndex, float length);
EXPORT int SoftBody_AddBeam(SoftBodyInstance* bodyA, int nodeA, SoftBodyInstance* bodyB, int nodeB, float compliance, float damping, float restLength, float minL, float maxL, float strength, float plasticityThreshold, float plasticityRate, float maxDeformation, int isEdgeSliding, int edgeA, int edgeB, int slidingNode);
EXPORT void SoftBody_UpdateBeamBulk(SoftBodyInstance* body, float compliance, float damping);
EXPORT void SoftBody_SetPredictedPos(SoftBodyInstance* body, int index, Vector3 pos);
EXPORT Vector3 SoftBody_GetCurrentPos(SoftBodyInstance* body, int index);
EXPORT void SoftBody_SetCurrentPos(SoftBodyInstance* body, int index, Vector3 pos);
EXPORT void SoftBody_SetNodeMass(SoftBodyInstance* body, int index, float mass);
EXPORT void SoftBody_SetPredictedPosBulk(SoftBodyInstance* body, Vector3* positions, int count);
EXPORT void SoftBody_SetCurrentPosBulk(SoftBodyInstance* body, Vector3* positions, int count);
EXPORT void SoftBody_SetPreviousPosBulk(SoftBodyInstance* body, Vector3* positions, int count);
EXPORT void SoftBody_StepPhysics(SoftBodyInstance* body, float dt, int constraintIters, float gravity);
EXPORT void SoftBody_FinalizeStep(SoftBodyInstance* body);
EXPORT void* SoftBody_GetPredicted(SoftBodyInstance* b);
EXPORT void* SoftBody_GetCurrent(SoftBodyInstance* b);
EXPORT void* SoftBody_GetPrevious(SoftBodyInstance* b);
EXPORT void* SoftBody_GetMasses(SoftBodyInstance* b);
EXPORT void* SoftBody_GetPinned(SoftBodyInstance* b);