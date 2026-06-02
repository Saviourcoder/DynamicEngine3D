/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝
*/
#include "EngineTypes.h"
#include "MathUtils.h"
#include "Constants.h"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

#if defined(_WIN32)
#define EXPORT extern "C" __declspec(dllexport)
#else
#define EXPORT extern "C" __attribute__((visibility("default")))
#endif

// Integrate 
EXPORT void Integrate(Vector3* currentPos, Vector3* prevPos, Vector3* predictedPos,
    const uint8_t* isPinned, int nodeCount, float dt, float damping, float gravity)
{
    if (nodeCount <= 0) return;

    float dtSq = dt * dt;
    float drag = std::exp(-damping * dt);

#pragma omp parallel for schedule(static)
    for (int i = 0; i < nodeCount; ++i)
    {
        if (isPinned[i])
        {
            predictedPos[i] = currentPos[i];
            continue;
        }

        const Vector3& pos = currentPos[i];
        const Vector3& prev = prevPos[i];

        float vx = (pos.x - prev.x) * drag;
        float vy = (pos.y - prev.y) * drag;
        float vz = (pos.z - prev.z) * drag;

        predictedPos[i] = { pos.x + vx,
                            pos.y + vy - (gravity * dtSq),
                            pos.z + vz,
                            0.0f };
    }
}

// SolveConstraints
EXPORT void SolveConstraints(Vector3* predictedPos, const BeamData* beams, float* multipliers,
    const float* masses, const uint8_t* isPinned, int beamCount, float dt)
{
    if (beamCount <= 0) return;

    float invDtSq = 1.0f / (dt * dt);

    for (int i = 0; i < beamCount; ++i)
    {
        const BeamData& beam = beams[i];
        if (!beam.isActive || beam.isCrossBody) continue;

        Vector3& pA = predictedPos[beam.nodeA];
        Vector3& pB = predictedPos[beam.nodeB];

        float dx = pA.x - pB.x, dy = pA.y - pB.y, dz = pA.z - pB.z;
        float len = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-7f) continue;

        float wA = isPinned[beam.nodeA] ? 0.0f : 1.0f / masses[beam.nodeA];
        float wB = isPinned[beam.nodeB] ? 0.0f : 1.0f / masses[beam.nodeB];
        float wSum = wA + wB;
        if (wSum <= 0.0f) continue;

        float alphaTilde = beam.compliance * invDtSq;
        float curLambda = multipliers[i];
        float deltaLambda = -(len - beam.restLength + alphaTilde * curLambda) / (wSum + alphaTilde);

        multipliers[i] = curLambda + deltaLambda;

        float correctionMag = deltaLambda / len;
        float cx = dx * correctionMag, cy = dy * correctionMag, cz = dz * correctionMag;

        if (!isPinned[beam.nodeA]) { pA.x += wA * cx; pA.y += wA * cy; pA.z += wA * cz; }
        if (!isPinned[beam.nodeB]) { pB.x -= wB * cx; pB.y -= wB * cy; pB.z -= wB * cz; }
    }
}

// ApplyBeamDampingForces
EXPORT void ApplyBeamDampingForces(Vector3* predictedPos, const Vector3* prevPos,
    const BeamData* beams, const float* masses,
    const uint8_t* isPinned, int beamCount, float dt)
{
    if (beamCount <= 0 || dt <= 0.0f) return;

    float invDt = 1.0f / dt;

    for (int i = 0; i < beamCount; ++i)
    {
        const BeamData& beam = beams[i];
        if (!beam.isActive || beam.damping <= 0.0f || beam.isCrossBody) continue;

        int a = beam.nodeA, b = beam.nodeB;
        Vector3 posA = predictedPos[a], posB = predictedPos[b];
        Vector3 delta = Sub(posA, posB);
        float dist = Length(delta);
        if (dist < 1e-7f) continue;

        Vector3 dir = Scale(delta, 1.0f / dist);
        Vector3 velA = Scale(Sub(posA, prevPos[a]), invDt);
        Vector3 velB = Scale(Sub(posB, prevPos[b]), invDt);
        float dampF = Dot(Sub(velA, velB), dir) * beam.damping;
        Vector3 impulse = Scale(dir, dampF * dt * dt);

        if (!isPinned[a]) { float im = 1.0f / masses[a]; predictedPos[a].x -= impulse.x * im; predictedPos[a].y -= impulse.y * im; predictedPos[a].z -= impulse.z * im; }
        if (!isPinned[b]) { float im = 1.0f / masses[b]; predictedPos[b].x += impulse.x * im; predictedPos[b].y += impulse.y * im; predictedPos[b].z += impulse.z * im; }
    }
}

// ApplyPressure 
EXPORT void ApplyPressure(Vector3* predictedPos, const Vector3* currentPos,
    const float* masses, const uint8_t* isPinned, int nodeCount,
    float internalPressure, float dt)
{
    if (internalPressure <= 0.0f || nodeCount <= 0) return;

    // Sequential reduction too small to be worth parallelising
    Vector3 center = V3Zero();
    int activeNodes = 0;
    for (int i = 0; i < nodeCount; ++i)
    {
        center.x += currentPos[i].x;
        center.y += currentPos[i].y;
        center.z += currentPos[i].z;
        if (!isPinned[i]) ++activeNodes;
    }
    float invN = 1.0f / (float)nodeCount;
    center.x *= invN; center.y *= invN; center.z *= invN;
    if (activeNodes == 0) return;

    float forcePerNode = internalPressure * PhysicsConstants::PRESSURE_SCALE_FACTOR / (float)activeNodes;
    float dt2 = dt * dt;

    // Safe to parallelise: each i writes only prevPos[i]
#pragma omp parallel for schedule(static)
    for (int i = 0; i < nodeCount; ++i)
    {
        if (isPinned[i]) continue;

        Vector3 dir = Sub(currentPos[i], center);
        float   dist = Length(dir);
        if (dist < PhysicsConstants::MIN_CENTER_DISTANCE) continue;

        float m = masses[i];
        if (m <= 0.0f) continue;

        dir = Scale(dir, 1.0f / dist);
        float accelMag = forcePerNode / m;

        predictedPos[i].x += dir.x * accelMag * dt2;
        predictedPos[i].y += dir.y * accelMag * dt2;
        predictedPos[i].z += dir.z * accelMag * dt2;
    }
}

// ApplyPlasticity 
EXPORT void ApplyPlasticity(const Vector3* predictedPos, BeamData* beams, int beamCount, float dt)
{
    if (beamCount <= 0) return;

#pragma omp parallel for schedule(static)
    for (int i = 0; i < beamCount; ++i)
    {
        BeamData& beam = beams[i];
        if (!beam.isActive || beam.isCrossBody) continue;

        float curLen = Length(Sub(predictedPos[beam.nodeA], predictedPos[beam.nodeB]));
        if (curLen < PhysicsConstants::MIN_BEAM_LENGTH) continue;

        float strain = (curLen - beam.restLength) / beam.restLength;
        if (AbsF(strain) <= beam.plasticityThreshold) continue;

        float excessStrain = (AbsF(strain) - beam.plasticityThreshold) * SignF(strain);
        float plasticDef = excessStrain * beam.plasticityRate * dt;

        beam.restLength += plasticDef * beam.restLength;

        float minL = beam.originalRestLength * (1.0f - beam.maxDeformation);
        float maxL = beam.originalRestLength * (1.0f + beam.maxDeformation);
        beam.restLength = beam.restLength < minL ? minL : (beam.restLength > maxL ? maxL : beam.restLength);
    }
}


// CheckAndBreakConstraints
EXPORT void CheckAndBreakConstraints(BeamData* beams, int beamCount,
    const Vector3* predictedPos,
    const float* multipliers,
    const float* strengths,
    float dt)
{
    if (beamCount <= 0 || dt <= 0.0f) return;

    float invDt2 = 1.0f / (dt * dt);

#pragma omp parallel for schedule(static)
    for (int i = 0; i < beamCount; ++i)
    {
        BeamData& beam = beams[i];
        if (!beam.isActive || !beam.isCrossBody) continue;

        float lenSq = LengthSq(Sub(predictedPos[beam.nodeA], predictedPos[beam.nodeB]));
        constexpr float EPS2 = PhysicsConstants::CONSTRAINT_EPSILON * PhysicsConstants::CONSTRAINT_EPSILON;
        if (lenSq < EPS2) continue;

        float strength = strengths[i];
        if (strength <= 0.0f || !std::isfinite(strength)) continue;

        if (std::fabs(multipliers[i]) * invDt2 > strength)
            beam.isActive = 0;
    }
}

// FinalizePositions
EXPORT void FinalizePositions(Vector3* currentPos, Vector3* prevPos,
    const Vector3* predictedPos,
    const uint8_t* isPinned, int nodeCount)
{
    if (nodeCount <= 0) return;

#pragma omp parallel for schedule(static)
    for (int i = 0; i < nodeCount; ++i)
    {
        prevPos[i] = currentPos[i];
        if (!isPinned[i])
            currentPos[i] = predictedPos[i];
    }
}

// UpdateNodeRotations 
EXPORT void UpdateNodeRotations(Quaternion* currentRotations,
    const Quaternion* initialRotations,
    const Vector3* currentPos, const Vector3* initialLocalPos,
    const int* neighborIndices, const int* neighborOffsets,
    int nodeCount)
{
    if (nodeCount <= 0) return;

#pragma omp parallel for schedule(dynamic, 4)
    for (int i = 0; i < nodeCount; ++i)
    {
        int start = neighborOffsets[i];
        int end = neighborOffsets[i + 1];

        if (end <= start) { currentRotations[i] = initialRotations[i]; continue; }

        constexpr int MAX_NEIGHBORS = 64;
        int neighborCount = end - start;
        if (neighborCount > MAX_NEIGHBORS) neighborCount = MAX_NEIGHBORS;
        Vector3 restOffsets[MAX_NEIGHBORS];
        for (int n = 0; n < neighborCount; ++n)
            restOffsets[n] = Sub(initialLocalPos[neighborIndices[start + n]], initialLocalPos[i]);

        Quaternion q = currentRotations[i];
        Vector3    ci = currentPos[i];

        for (int it = 0; it < PhysicsConstants::MAX_ROTATION_ITERATIONS; ++it)
        {
            Vector3 omega = V3Zero();
            float   denom = 0.0f;

            for (int n = 0; n < neighborCount; ++n)
            {
                int     j = neighborIndices[start + n];
                Vector3 cij = Sub(currentPos[j], ci);
                Vector3 rij = QRotate(q, restOffsets[n]);

                omega.x += rij.y * cij.z - rij.z * cij.y;
                omega.y += rij.z * cij.x - rij.x * cij.z;
                omega.z += rij.x * cij.y - rij.y * cij.x;
                denom += Dot(rij, cij) + Dot(rij, rij);
            }

            if (AbsF(denom) < PhysicsConstants::ROTATION_CONVERGENCE_EPSILON) break;
            float invDen = 1.0f / denom;
            omega.x *= invDen; omega.y *= invDen; omega.z *= invDen;

            float w = Length(omega);
            if (w < PhysicsConstants::MIN_OMEGA_MAGNITUDE) break;

            q = QNormalize(QMul(QAxisAngle(Scale(omega, 1.0f / w), w), q));
        }
        currentRotations[i] = q;
    }
}

// SolveCrossBody
EXPORT void SolveCrossBody(Vector3* predictedPos,
    const BeamData* beams, float* multipliers,
    const float* masses, const uint8_t* isPinned,
    int beamCount, float dt)
{
    if (beamCount <= 0) return;

    float invDtSq = 1.0f / (dt * dt);

    for (int i = 0; i < beamCount; ++i)
    {
        const BeamData& beam = beams[i];
        if (!beam.isActive || !beam.isCrossBody) continue;

        Vector3& pA = predictedPos[beam.nodeA];
        Vector3& pB = predictedPos[beam.nodeB];

        float dx = pA.x - pB.x, dy = pA.y - pB.y, dz = pA.z - pB.z;
        float len = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-7f) continue;

        float wA = isPinned[beam.nodeA] ? 0.0f : 1.0f / masses[beam.nodeA];
        float wB = isPinned[beam.nodeB] ? 0.0f : 1.0f / masses[beam.nodeB];
        float wSum = wA + wB;
        if (wSum <= 0.0f) continue;

        float alphaTilde = beam.compliance * invDtSq;
        float curLambda = multipliers[i];
        float deltaLambda = -(len - beam.restLength + alphaTilde * curLambda) / (wSum + alphaTilde);

        multipliers[i] = curLambda + deltaLambda;

        float correctionMag = deltaLambda / len;
        float cx = dx * correctionMag, cy = dy * correctionMag, cz = dz * correctionMag;

        if (!isPinned[beam.nodeA]) { pA.x += wA * cx; pA.y += wA * cy; pA.z += wA * cz; }
        if (!isPinned[beam.nodeB]) { pB.x -= wB * cx; pB.y -= wB * cy; pB.z -= wB * cz; }
    }
}

EXPORT void StepBodyPreFinalize(
    Vector3* currentPos,
    Vector3* prevPos,
    Vector3* predictedPos,
    float* masses,
    const uint8_t* isPinned,
    BeamData* beams,
    float* multipliers,
    float* restLengths,
    const float* originalRestLengths,
    const float* plasticityThresholds,
    const float* plasticityRates,
    const float* maxDeformations,
    int           nodeCount,
    int           beamCount,
    float         dt,
    float         damping,
    float         gravity,
    int           constraintIters,
    float         internalPressure,
    float         worldRestLengthScale)
{
    if (nodeCount <= 0) return;

    // 1. Verlet integrate with gravity and velocity damping
    Integrate(currentPos, prevPos, predictedPos, isPinned, nodeCount, dt, damping, gravity);

    // 2. Internal pressure 
    if (internalPressure > 0.0f)
        ApplyPressure(predictedPos, currentPos, masses, isPinned, nodeCount, internalPressure, dt);

    // 3. Beam velocity damping
    if (beamCount > 0)
        ApplyBeamDampingForces(predictedPos, prevPos, beams, masses, isPinned, beamCount, dt);

    // 4. Reset XPBD Lagrange multipliers for this substep
    if (beamCount > 0)
        std::memset(multipliers, 0, beamCount * sizeof(float));

    // 5. XPBD constraint solve — sequential
    for (int iter = 0; iter < constraintIters; ++iter)
        SolveConstraints(predictedPos, beams, multipliers, masses, isPinned, beamCount, dt);

    // 6. Plasticity — updates restLengths
    if (beamCount > 0)
        ApplyPlasticity(predictedPos, beams, beamCount, dt);
}