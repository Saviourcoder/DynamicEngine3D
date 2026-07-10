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
extern "C" void Integrate(float* cx, float* cy, float* cz, float* px, float* py, float* pz, float* predX, float* predY, float* predZ, const uint8_t* pinned, int count, float dt, float damping, float gravity);
extern "C" void ApplyPressure(float* predX, float* predY, float* predZ, const float* cx, const float* cy, const float* cz, const float* masses, const uint8_t* pinned, int count, float pressure, float dt);
extern "C" void ApplyBeamDampingForces(float* predX, float* predY, float* predZ, const float* px, const float* py, const float* pz, const BeamData* beams, const float* masses, const uint8_t* pinned, int count, float dt);
extern "C" void SolveConstraints(float* predX, float* predY, float* predZ, const BeamData* beams, float* multipliers, const float* masses, const uint8_t* pinned, int count, float dt);
extern "C" void FinalizePositions(float* cx, float* cy, float* cz, float* px, float* py, float* pz, const float* predX, const float* predY, const float* predZ, const uint8_t* pinned, int count);
extern "C" void ApplyPlasticity(const float* predX, const float* predY, const float* predZ, BeamData* beams, int beamCount, float dt);

class SoftBodyInstance
{
public:
    std::vector<float> currentX; std::vector<float> currentY; std::vector<float> currentZ;
    std::vector<float> previousX; std::vector<float> previousY; std::vector<float> previousZ;
    std::vector<float> predictedX; std::vector<float> predictedY; std::vector<float> predictedZ;
    std::vector<float> initialX; std::vector<float> initialY; std::vector<float> initialZ;

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
EXPORT void SoftBody_SolveCrossBodyStep(SoftBodyInstance* body, float dt, int iterations);
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

// Bulk setters will receive separate X, Y, Z arrays now
EXPORT void SoftBody_SetPredictedPosBulk(SoftBodyInstance* body, float* xs, float* ys, float* zs, int count);
EXPORT void SoftBody_SetCurrentPosBulk(SoftBodyInstance* body, float* xs, float* ys, float* zs, int count);
EXPORT void SoftBody_SetPreviousPosBulk(SoftBodyInstance* body, float* xs, float* ys, float* zs, int count);

EXPORT void SoftBody_StepPhysics(SoftBodyInstance* body, float dt, int constraintIters, float gravity);
EXPORT void SoftBody_FinalizeStep(SoftBodyInstance* body);

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