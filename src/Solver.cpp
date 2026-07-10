/*
 DYNAMICENGINE3D
 AI-Assisted Soft-Body Physics for Unity3D
 By: Elitmers
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
EXPORT void Integrate(float* cx, float* cy, float* cz, float* px, float* py, float* pz, float* predX, float* predY, float* predZ,
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
            predX[i] = cx[i]; predY[i] = cy[i]; predZ[i] = cz[i];
            continue;
        }

        float vx = (cx[i] - px[i]) * drag;
        float vy = (cy[i] - py[i]) * drag;
        float vz = (cz[i] - pz[i]) * drag;

        predX[i] = cx[i] + vx;
        predY[i] = cy[i] + vy - (gravity * dtSq);
        predZ[i] = cz[i] + vz;
    }
}

// SolveConstraints
EXPORT void SolveConstraints(float* predX, float* predY, float* predZ, const BeamData* beams, float* multipliers,
    const float* masses, const uint8_t* isPinned, int beamCount, float dt)
{
    if (beamCount <= 0) return;

    float invDtSq = 1.0f / (dt * dt);

    for (int i = 0; i < beamCount; ++i)
    {
        const BeamData& beam = beams[i];
        if (!beam.isActive || beam.isCrossBody) continue;

        int a = beam.nodeA;
        int b = beam.nodeB;

        float dx = predX[a] - predX[b], dy = predY[a] - predY[b], dz = predZ[a] - predZ[b];
        float len = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-7f) continue;

        float wA = isPinned[a] ? 0.0f : 1.0f / masses[a];
        float wB = isPinned[b] ? 0.0f : 1.0f / masses[b];
        float wSum = wA + wB;
        if (wSum <= 0.0f) continue;

        float alphaTilde = beam.compliance * invDtSq;
        float curLambda = multipliers[i];
        float deltaLambda = -(len - beam.restLength + alphaTilde * curLambda) / (wSum + alphaTilde);

        multipliers[i] = curLambda + deltaLambda;

        float correctionMag = deltaLambda / len;
        float cx_corr = dx * correctionMag, cy_corr = dy * correctionMag, cz_corr = dz * correctionMag;

        if (!isPinned[a]) { predX[a] += wA * cx_corr; predY[a] += wA * cy_corr; predZ[a] += wA * cz_corr; }
        if (!isPinned[b]) { predX[b] -= wB * cx_corr; predY[b] -= wB * cy_corr; predZ[b] -= wB * cz_corr; }
    }
}

// ApplyBeamDampingForces
EXPORT void ApplyBeamDampingForces(float* predX, float* predY, float* predZ, const float* px, const float* py, const float* pz,
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
        Vector3 posA = { predX[a], predY[a], predZ[a], 0.0f };
        Vector3 posB = { predX[b], predY[b], predZ[b], 0.0f };
        Vector3 delta = Sub(posA, posB);
        float dist = Length(delta);
        if (dist < 1e-7f) continue;

        Vector3 dir = Scale(delta, 1.0f / dist);
        Vector3 p_a = { px[a], py[a], pz[a], 0.0f };
        Vector3 p_b = { px[b], py[b], pz[b], 0.0f };
        Vector3 velA = Scale(Sub(posA, p_a), invDt);
        Vector3 velB = Scale(Sub(posB, p_b), invDt);
        float dampF = Dot(Sub(velA, velB), dir) * beam.damping;
        Vector3 impulse = Scale(dir, dampF * dt * dt);

        if (!isPinned[a]) { float im = 1.0f / masses[a]; predX[a] -= impulse.x * im; predY[a] -= impulse.y * im; predZ[a] -= impulse.z * im; }
        if (!isPinned[b]) { float im = 1.0f / masses[b]; predX[b] += impulse.x * im; predY[b] += impulse.y * im; predZ[b] += impulse.z * im; }
    }
}

// ApplyPressure 
EXPORT void ApplyPressure(float* predX, float* predY, float* predZ, const float* cx, const float* cy, const float* cz,
    const float* masses, const uint8_t* isPinned, int nodeCount,
    float internalPressure, float dt)
{
    if (internalPressure <= 0.0f || nodeCount <= 0) return;

    Vector3 center = V3Zero();
    int activeNodes = 0;
    for (int i = 0; i < nodeCount; ++i)
    {
        center.x += cx[i];
        center.y += cy[i];
        center.z += cz[i];
        if (!isPinned[i]) ++activeNodes;
    }
    float invN = 1.0f / (float)nodeCount;
    center.x *= invN; center.y *= invN; center.z *= invN;
    if (activeNodes == 0) return;

    float forcePerNode = internalPressure * PhysicsConstants::PRESSURE_SCALE_FACTOR / (float)activeNodes;
    float dt2 = dt * dt;

#pragma omp parallel for schedule(static) if(nodeCount > 4096 && !omp_in_parallel())
    for (int i = 0; i < nodeCount; ++i)
    {
        if (isPinned[i]) continue;

        Vector3 posI = { cx[i], cy[i], cz[i], 0.0f };
        Vector3 dir = Sub(posI, center);
        float   dist = Length(dir);
        if (dist < PhysicsConstants::MIN_CENTER_DISTANCE) continue;

        float m = masses[i];
        if (m <= 0.0f) continue;

        dir = Scale(dir, 1.0f / dist);
        float accelMag = forcePerNode / m;

        predX[i] += dir.x * accelMag * dt2;
        predY[i] += dir.y * accelMag * dt2;
        predZ[i] += dir.z * accelMag * dt2;
    }
}

// ApplyPlasticity 
EXPORT void ApplyPlasticity(const float* predX, const float* predY, const float* predZ, BeamData* beams, int beamCount, float dt)
{
    if (beamCount <= 0) return;

#pragma omp parallel for schedule(static) if(beamCount > 4096 && !omp_in_parallel())
    for (int i = 0; i < beamCount; ++i)
    {
        BeamData& beam = beams[i];
        if (!beam.isActive || beam.isCrossBody) continue;

        float dx = predX[beam.nodeA] - predX[beam.nodeB];
        float dy = predY[beam.nodeA] - predY[beam.nodeB];
        float dz = predZ[beam.nodeA] - predZ[beam.nodeB];
        float curLen = std::sqrt(dx*dx + dy*dy + dz*dz);
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

// FinalizePositions
EXPORT void FinalizePositions(float* cx, float* cy, float* cz, float* px, float* py, float* pz,
    const float* predX, const float* predY, const float* predZ,
    const uint8_t* isPinned, int nodeCount)
{
    if (nodeCount <= 0) return;

#pragma omp parallel for schedule(static) if(nodeCount > 4096 && !omp_in_parallel())
    for (int i = 0; i < nodeCount; ++i)
    {
        px[i] = cx[i]; py[i] = cy[i]; pz[i] = cz[i];
        if (!isPinned[i])
        {
            cx[i] = predX[i]; cy[i] = predY[i]; cz[i] = predZ[i];
        }
    }
}

// UpdateNodeRotations 
EXPORT void UpdateNodeRotations(Quaternion* currentRotations,
    const Quaternion* initialRotations,
    const float* cx, const float* cy, const float* cz, 
    const float* ix, const float* iy, const float* iz,
    const int* neighborIndices, const int* neighborOffsets,
    int nodeCount)
{
    if (nodeCount <= 0) return;

#pragma omp parallel for schedule(static)
    for (int i = 0; i < nodeCount; ++i)
    {
        int start = neighborOffsets[i];
        int end = neighborOffsets[i + 1];

        if (end <= start) { currentRotations[i] = initialRotations[i]; continue; }

        constexpr int MAX_NEIGHBORS = 64;
        int neighborCount = end - start;
        if (neighborCount > MAX_NEIGHBORS) neighborCount = MAX_NEIGHBORS;
        Vector3 restOffsets[MAX_NEIGHBORS];
        Vector3 cijCache[MAX_NEIGHBORS];
        Vector3 initial_i = { ix[i], iy[i], iz[i], 0.0f };
        Vector3 ci = { cx[i], cy[i], cz[i], 0.0f };

        for (int n = 0; n < neighborCount; ++n)
        {
            int j = neighborIndices[start + n];
            Vector3 initial_j = { ix[j], iy[j], iz[j], 0.0f };
            restOffsets[n] = Sub(initial_j, initial_i);

            Vector3 cj = { cx[j], cy[j], cz[j], 0.0f };
            cijCache[n] = Sub(cj, ci);   // cache it here, not inside the loop
        }

        Quaternion q = currentRotations[i];

        for (int it = 0; it < PhysicsConstants::MAX_ROTATION_ITERATIONS; ++it)
        {
            float qx = q.x, qy = q.y, qz = q.z, qw = q.w;
            float x2 = qx + qx, y2 = qy + qy, z2 = qz + qz;
            float xx = qx * x2, xy = qx * y2, xz = qx * z2;
            float yy = qy * y2, yz = qy * z2, zz = qz * z2;
            float wx = qw * x2, wy = qw * y2, wz = qw * z2;

            float m00 = 1.0f - (yy + zz), m01 = xy - wz, m02 = xz + wy;
            float m10 = xy + wz, m11 = 1.0f - (xx + zz), m12 = yz - wx;
            float m20 = xz - wy, m21 = yz + wx, m22 = 1.0f - (xx + yy);

            Vector3 omega = V3Zero();
            float   denom = 0.0f;

            for (int n = 0; n < neighborCount; ++n)
            {
                const Vector3& off = restOffsets[n];
                Vector3 rij = {
                    m00 * off.x + m01 * off.y + m02 * off.z,
                    m10 * off.x + m11 * off.y + m12 * off.z,
                    m20 * off.x + m21 * off.y + m22 * off.z,
                    0.0f
                };

                const Vector3& cij = cijCache[n];
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

            // first-order exponential map: sin(x/2) ≈ x/2, cos(x/2) ≈ 1 for small x
            Quaternion dq = { omega.x * 0.5f, omega.y * 0.5f, omega.z * 0.5f, 1.0f };
            q = QNormalize(QMul(dq, q));
        }
        currentRotations[i] = q;
    }
}