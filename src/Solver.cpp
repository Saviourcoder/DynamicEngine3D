/*
 DYNAMICENGINE3D
 AI-Assisted 3D Physics Engine
 By: Elitmers
*/

#include "Export.h"
#include "EngineTypes.h"
#include "MathUtils.h"
#include "Constants.h"
#include "SoftBodyInstance.h"
#include "Error.h"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <vector>

#include "ThreadPool.h"

// Verlet-style position prediction, four nodes per SSE block.

extern "C" EXPORT void Integrate(float* cx, float* cy, float* cz, float* px, float* py, float* pz, float* predX, float* predY, float* predZ,
    const uint8_t* isPinned, int nodeCount, float dt, float damping, float gravity)
{
    if (nodeCount <= 0) return; // nothing to integrate this call, not an error
    if (!cx || !cy || !cz || !px || !py || !pz || !predX || !predY || !predZ || !isPinned)
    {
        Error_SetError(ErrorCode::InvalidArgument);
        return;
    }
    if (dt <= 0.0f)
    {
        Error_SetError(ErrorCode::InvalidDt);
        return;
    }

    const float dtSq = dt * dt;
    const float drag = std::exp(-damping * dt);
    const float gravityStep = gravity * dtSq;

    const int blocks = nodeCount >> 2;

    if (blocks > 0)
    {
        const __m128 vDrag = _mm_set1_ps(drag);
        const __m128 vGravity = _mm_set1_ps(gravityStep);

        // 1024 blocks = 4096 nodes, the same threshold the other passes use.
        constexpr int kMinIntegrateBlocks = 1024;

        GetGlobalPool().parallel_for(blocks, [&](int b)
            {
                const int i = b << 2;

                int32_t pinnedBytes;
                std::memcpy(&pinnedBytes, isPinned + i, sizeof(pinnedBytes));
                // All bits set in lanes whose node is free to move, zero where pinned.
                const __m128 moves = _mm_castsi128_ps(_mm_cmpeq_epi32(
                    _mm_cvtepu8_epi32(_mm_cvtsi32_si128(pinnedBytes)), _mm_setzero_si128()));

                const __m128 x = _mm_loadu_ps(cx + i);
                const __m128 y = _mm_loadu_ps(cy + i);
                const __m128 z = _mm_loadu_ps(cz + i);

                const __m128 vx = _mm_and_ps(_mm_mul_ps(_mm_sub_ps(x, _mm_loadu_ps(px + i)), vDrag), moves);
                const __m128 vy = _mm_and_ps(_mm_mul_ps(_mm_sub_ps(y, _mm_loadu_ps(py + i)), vDrag), moves);
                const __m128 vz = _mm_and_ps(_mm_mul_ps(_mm_sub_ps(z, _mm_loadu_ps(pz + i)), vDrag), moves);

                _mm_storeu_ps(predX + i, _mm_add_ps(x, vx));
                _mm_storeu_ps(predY + i, _mm_sub_ps(_mm_add_ps(y, vy), _mm_and_ps(vGravity, moves)));
                _mm_storeu_ps(predZ + i, _mm_add_ps(z, vz));
            }, kMinIntegrateBlocks);
    }

    for (int i = blocks << 2; i < nodeCount; ++i)
    {
        if (isPinned[i])
        {
            predX[i] = cx[i]; predY[i] = cy[i]; predZ[i] = cz[i];
            continue;
        }

        predX[i] = cx[i] + (cx[i] - px[i]) * drag;
        predY[i] = cy[i] + (cy[i] - py[i]) * drag - gravityStep;
        predZ[i] = cz[i] + (cz[i] - pz[i]) * drag;
    }
}

// Per-beam XPBD distance constraint; safe to call concurrently for beams sharing no node.
static inline void SolveOneBeam(int i, float* predX, float* predY, float* predZ,
    const float* px, const float* py, const float* pz,
    const BeamData* beams, float* multipliers,
    const float* invMasses, const uint8_t* isPinned,
    const float* alphaTildeCache)
{
    const BeamData& beam = beams[i];
    if (!beam.isActive || beam.isCrossBody) return;

    int a = beam.nodeA;
    int b = beam.nodeB;

    float dx = predX[a] - predX[b], dy = predY[a] - predY[b], dz = predZ[a] - predZ[b];
    float lenSq = dx * dx + dy * dy + dz * dz;
    if (lenSq < 1e-14f) return;

    float invLen = 1.0f / std::sqrt(lenSq);   // one sqrt, one divide, total
    float len = lenSq * invLen;             // recovers len for free (lenSq/len == len)

    float wA = invMasses[a];
    float wB = invMasses[b];
    float wSum = wA + wB;
    if (wSum <= 0.0f) return;

    float alphaTilde = alphaTildeCache[i];
    float curLambda = multipliers[i];

    float dirX = dx * invLen, dirY = dy * invLen, dirZ = dz * invLen;

    float vxA = (predX[a] - px[a]) - (predX[b] - px[b]);
    float vyA = (predY[a] - py[a]) - (predY[b] - py[b]);
    float vzA = (predZ[a] - pz[a]) - (predZ[b] - pz[b]);

    float relVelNormal = (vxA * dirX + vyA * dirY + vzA * dirZ);

    // Unconditionally stable XPBD implicit damping (Macklin et al. 2016):
    // Damping factor enters the denominator as (1 + gamma) * wSum, guaranteeing
    // that attenuation is strictly bounded in [0, 1) and can never overshoot or explode.
    float gamma = beam.damping > 0.0f ? beam.damping : 0.0f;
    float denom = (1.0f + gamma) * wSum + alphaTilde;
    if (denom <= 0.0f) return;

    float deltaLambda = -(len - beam.restLength + alphaTilde * curLambda + gamma * relVelNormal) / denom;
    multipliers[i] = curLambda + deltaLambda;

    float correctionMag = deltaLambda * invLen;
    float cx_corr = dx * correctionMag;
    float cy_corr = dy * correctionMag;
    float cz_corr = dz * correctionMag;

    if (!isPinned[a]) { predX[a] += wA * cx_corr; predY[a] += wA * cy_corr; predZ[a] += wA * cz_corr; }
    if (!isPinned[b]) { predX[b] -= wB * cx_corr; predY[b] -= wB * cy_corr; predZ[b] -= wB * cz_corr; }
}

extern "C" EXPORT void SolveConstraints(float* predX, float* predY, float* predZ,
    const float* px, const float* py, const float* pz,
    const BeamData* beams, float* multipliers,
    const float* invMasses, const uint8_t* isPinned,
    const float* alphaTildeCache, int beamCount, float dt)
{
    (void)dt; // alphaTilde is precomputed per substep in alphaTildeCache

    if (beamCount <= 0) return; // no beams this call, not an error
    if (!predX || !predY || !predZ || !px || !py || !pz || !beams || !multipliers || !invMasses || !isPinned || !alphaTildeCache)
    {
        Error_SetError(ErrorCode::InvalidArgument);
        return;
    }

    for (int i = 0; i < beamCount; ++i)
        SolveOneBeam(i, predX, predY, predZ, px, py, pz, beams, multipliers, invMasses, isPinned, alphaTildeCache);
}

extern "C" EXPORT void SolveConstraintsColored(float* predX, float* predY, float* predZ,
    const float* px, const float* py, const float* pz,
    const BeamData* beams, float* multipliers,
    const float* invMasses, const uint8_t* isPinned,
    const float* alphaTildeCache,
    const int* colorOffsets, const int* colorBeamIndices, int numColors,
    int beamCount, float dt)
{
    (void)dt;          // alphaTilde is already precomputed
    (void)beamCount;   // not needed when we have the color ranges

    if (numColors <= 0) return; // nothing to solve, not an error
    if (!predX || !predY || !predZ || !px || !py || !pz || !beams || !multipliers ||
        !invMasses || !isPinned || !alphaTildeCache || !colorOffsets || !colorBeamIndices)
    {
        Error_SetError(ErrorCode::InvalidArgument);
        return;
    }

    // Each color group has no shared nodes, so beams within one color parallelize safely.
    for (int c = 0; c < numColors; ++c)
    {
        const int start = colorOffsets[c];
        const int end = colorOffsets[c + 1];
        const int count = end - start;
        if (count <= 0) continue;

        // Small color groups are faster serial; larger ones benefit from the pool.
        constexpr int kParallelColorThreshold = 64;

        if (count < kParallelColorThreshold)
        {
            for (int i = start; i < end; ++i)
            {
                const int beamIdx = colorBeamIndices[i];
                SolveOneBeam(beamIdx, predX, predY, predZ,
                    px, py, pz,
                    beams, multipliers,
                    invMasses, isPinned,
                    alphaTildeCache);
            }
        }
        else
        {
            GetGlobalPool().parallel_for(count, [&](int local)
                {
                    const int beamIdx = colorBeamIndices[start + local];
                    SolveOneBeam(beamIdx, predX, predY, predZ,
                        px, py, pz,
                        beams, multipliers,
                        invMasses, isPinned,
                        alphaTildeCache);
                }, 32);
        }
    }
}

void SolveInterleavedConstraints(
    SoftBodyInstance** bodies,
    int count,
    float dt,
    int iterations)
{
    if (count <= 0 || iterations <= 0)
        return;

    const float invDtSq = 1.0f / (dt * dt);

    // Only bodies with at least this many beams get the colored + parallel path.
    constexpr int kMinBeamsForColoredParallel = 1024;

    // Prepare alpha cache + coloring (still parallel across bodies)

    GetGlobalPool().parallel_for(count, [&](int i)
        {
            SoftBodyInstance* body = bodies[i];
            if (!body || body->beams.empty() || body->isSleeping)
                return;

            if (body->alphaTildeCache.size() != body->beams.size())
                body->alphaTildeCache.resize(body->beams.size());

            for (size_t b = 0; b < body->beams.size(); ++b)
                body->alphaTildeCache[b] = body->beams[b].compliance * invDtSq;

            // Lazy rebuild of coloring (only needed if we might go parallel)
            if (body->colorOffsets.empty() &&
                static_cast<int>(body->beams.size()) >= kMinBeamsForColoredParallel)
            {
                body->RebuildBeamColoring();
            }
        }, 4);

    // Sort bodies into colored vs cheap-serial once, outside the iteration loop.

    struct ColoredBody
    {
        SoftBodyInstance* body;
        int numColors;
    };
    std::vector<ColoredBody> coloredBodies;
    std::vector<SoftBodyInstance*> serialBodies;
    coloredBodies.reserve(count);
    serialBodies.reserve(count);

    int maxColors = 0;
    for (int i = 0; i < count; ++i)
    {
        SoftBodyInstance* body = bodies[i];
        if (!body || body->beams.empty() || body->isSleeping)
            continue;

        const int beamCount = static_cast<int>(body->beams.size());
        const bool useColoredParallel =
            beamCount >= kMinBeamsForColoredParallel &&
            body->colorOffsets.size() > 1;

        if (useColoredParallel)
        {
            const int numColors = static_cast<int>(body->colorOffsets.size() - 1);
            coloredBodies.push_back({ body, numColors });
            if (numColors > maxColors) maxColors = numColors;
        }
        else
        {
            serialBodies.push_back(body);
        }
    }

    int serialBeamCount = 0;
    for (SoftBodyInstance* body : serialBodies)
        serialBeamCount += static_cast<int>(body->beams.size());

    struct WaveJob
    {
        SoftBodyInstance* body;
        int beamIdx;
    };

    std::vector<std::vector<WaveJob>> waveJobs(maxColors);
    for (int c = 0; c < maxColors; ++c)
    {
        size_t total = 0;
        for (const ColoredBody& cb : coloredBodies)
        {
            if (c < cb.numColors)
                total += static_cast<size_t>(std::max(0, cb.body->colorOffsets[c + 1] - cb.body->colorOffsets[c]));
        }
        waveJobs[c].reserve(total);

        for (const ColoredBody& cb : coloredBodies)
        {
            if (c >= cb.numColors) continue;
            const int start = cb.body->colorOffsets[c];
            const int end = cb.body->colorOffsets[c + 1];
            for (int k = start; k < end; ++k)
                waveJobs[c].push_back({ cb.body, cb.body->colorBeamIndices[k] });
        }
    }

    constexpr int kParallelColorThreshold = 64;

    // Constraint iterations
    for (int iter = 0; iter < iterations; ++iter)
    {
        auto solveSerialBody = [&](SoftBodyInstance* body)
            {
                SolveConstraints(
                    body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(),
                    body->currentX.data(), body->currentY.data(), body->currentZ.data(),
                    body->beams.data(), body->multipliers.data(),
                    body->invMasses.data(), body->isPinned.data(),
                    body->alphaTildeCache.data(),
                    static_cast<int>(body->beams.size()), dt);
            };

        if (serialBodies.size() > 1 && serialBeamCount >= kParallelColorThreshold)
        {
            GetGlobalPool().parallel_for(static_cast<int>(serialBodies.size()), [&](int i)
                {
                    solveSerialBody(serialBodies[i]);
                }, 1);
        }
        else
        {
            for (SoftBodyInstance* body : serialBodies)
                solveSerialBody(body);
        }

        for (const std::vector<WaveJob>& jobs : waveJobs)
        {
            const int total = static_cast<int>(jobs.size());
            if (total <= 0) continue;

            auto solveJob = [&](int i)
                {
                    const WaveJob& job = jobs[i];
                    SoftBodyInstance* body = job.body;
                    SolveOneBeam(job.beamIdx,
                        body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(),
                        body->currentX.data(), body->currentY.data(), body->currentZ.data(),
                        body->beams.data(), body->multipliers.data(),
                        body->invMasses.data(), body->isPinned.data(),
                        body->alphaTildeCache.data());
                };

            if (total < kParallelColorThreshold)
            {
                for (int i = 0; i < total; ++i)
                    solveJob(i);
            }
            else
            {
                GetGlobalPool().parallel_for(total, solveJob, 32);
            }
        }

        // 2. Cross-body constraints (serial)

        for (int i = 0; i < count; ++i)
        {
            SoftBodyInstance* body = bodies[i];
            if (!body)
                continue;

            for (int index : body->CrossBodyBeams())
            {
                BeamData& beam = body->beams[index];

                if (!beam.isActive)
                    continue;

                // bodyA is the canonical owner
                if (beam.bodyA != body)
                    continue;

                if (!beam.bodyB)
                    continue;

                if (beam.isEdgeSliding)
                    SolveEdgeSliding(beam, dt);
                else
                    SolveCrossBodyBeam(beam, dt);
            }
        }

        // 3. World-space constraints (serial - only 1 body involved per constraint).
        // Mirrors the cross-body block above but iterates each body's
        // worldConstraints vector. The anchor is immovable (wB = 0), so
        // only nodeA is ever moved - no risk of double-solving across bodies.
        for (int i = 0; i < count; ++i)
        {
            SoftBodyInstance* body = bodies[i];
            if (!body || body->worldConstraints.empty() || body->isSleeping)
                continue;

            for (WorldSpaceConstraint& c : body->worldConstraints)
            {
                if (!c.isActive)
                    continue;
                SolveWorldSpaceConstraint(c, dt);
            }
        }
    }
}

extern "C" EXPORT void ApplyPressure(float* predX, float* predY, float* predZ, const float* cx, const float* cy, const float* cz,
    const float* masses, const uint8_t* isPinned, int nodeCount,
    float internalPressure, float dt)
{
    (void)cx; (void)cy; (void)cz;
    if (internalPressure <= 0.0f || nodeCount <= 0) return; // no pressure to apply, not an error
    if (!predX || !predY || !predZ || !masses || !isPinned)
    {
        Error_SetError(ErrorCode::InvalidArgument);
        return;
    }

    Vector3 center = V3Zero();
    int activeNodes = 0;

    constexpr int kParallelThreshold = 512;
    if (nodeCount < kParallelThreshold)
    {
        for (int i = 0; i < nodeCount; ++i)
        {
            center.x += predX[i]; center.y += predY[i]; center.z += predZ[i];
            if (!isPinned[i]) ++activeNodes;
        }
    }
    else
    {
        int numChunks = (int)std::min<int>(nodeCount, std::max(1, GetGlobalPool().pooledThreads() + 1));
        static thread_local std::vector<Vector3> partialCenters;
        static thread_local std::vector<int> partialActive;
        partialCenters.resize(numChunks);
        partialActive.resize(numChunks);
        int chunkSize = (nodeCount + numChunks - 1) / numChunks;

        GetGlobalPool().parallel_for(numChunks, [&](int c)
            {
                int start = c * chunkSize;
                int end = std::min(start + chunkSize, nodeCount);
                Vector3 sum = V3Zero();
                int active = 0;
                for (int i = start; i < end; ++i)
                {
                    sum.x += predX[i]; sum.y += predY[i]; sum.z += predZ[i];
                    if (!isPinned[i]) ++active;
                }
                partialCenters[c] = sum;
                partialActive[c] = active;
            });

        for (int c = 0; c < numChunks; ++c)
        {
            center.x += partialCenters[c].x;
            center.y += partialCenters[c].y;
            center.z += partialCenters[c].z;
            activeNodes += partialActive[c];
        }
    }

    float invN = 1.0f / (float)nodeCount;
    center.x *= invN; center.y *= invN; center.z *= invN;
    if (activeNodes == 0) return;

    float forcePerNode = internalPressure * PhysicsConstants::PRESSURE_SCALE_FACTOR / (float)activeNodes;
    float dt2 = dt * dt;

    GetGlobalPool().parallel_for(nodeCount, [&](int i)
        {
            if (isPinned[i]) return;

            Vector3 posI = { predX[i], predY[i], predZ[i], 0.0f };
            Vector3 dir = Sub(posI, center);
            float   dist = Length(dir);
            if (dist < PhysicsConstants::MIN_CENTER_DISTANCE) return;

            float m = masses[i];
            if (m <= 0.0f) return;

            dir = Scale(dir, 1.0f / dist);
            float accelMag = forcePerNode / m;

            predX[i] += dir.x * accelMag * dt2;
            predY[i] += dir.y * accelMag * dt2;
            predZ[i] += dir.z * accelMag * dt2;
        }, 4096);
}


extern "C" EXPORT void ApplyPlasticity(const float* predX, const float* predY, const float* predZ, BeamData* beams, int beamCount, float dt)
{
    if (beamCount <= 0) return; // no beams this call, not an error
    if (!predX || !predY || !predZ || !beams)
    {
        Error_SetError(ErrorCode::InvalidArgument);
        return;
    }

    constexpr float MAX_PLASTICITY_DT = 1.0f / 120.0f;
    if (dt > MAX_PLASTICITY_DT) dt = MAX_PLASTICITY_DT;

    GetGlobalPool().parallel_for(beamCount, [&](int i)
        {
            BeamData& beam = beams[i];
            if (!beam.isActive || beam.isCrossBody) return;

            float dx = predX[beam.nodeA] - predX[beam.nodeB];
            float dy = predY[beam.nodeA] - predY[beam.nodeB];
            float dz = predZ[beam.nodeA] - predZ[beam.nodeB];
            float curLen = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (curLen < PhysicsConstants::MIN_BEAM_LENGTH) return;

            float strain = (curLen - beam.restLength) / beam.restLength;
            if (AbsF(strain) <= beam.plasticityThreshold) return;

            float excessStrain = (AbsF(strain) - beam.plasticityThreshold) * SignF(strain);
            float plasticDef = excessStrain * beam.plasticityRate * dt;

            beam.restLength += plasticDef * beam.restLength;

            float minDeformFrac = std::max(0.01f, 1.0f - beam.maxDeformation);
            float minL = std::max(PhysicsConstants::MIN_BEAM_LENGTH, beam.originalRestLength * minDeformFrac);
            float maxL = beam.originalRestLength * (1.0f + std::max(0.0f, beam.maxDeformation));
            beam.restLength = std::clamp(beam.restLength, minL, maxL);
        }, 4096);
}

// FinalizePositions
extern "C" EXPORT void FinalizePositions(float* cx, float* cy, float* cz, float* px, float* py, float* pz,
    const float* predX, const float* predY, const float* predZ,
    const uint8_t* isPinned, int nodeCount)
{
    if (nodeCount <= 0) return; // nothing to finalize, not an error
    if (!cx || !cy || !cz || !px || !py || !pz || !predX || !predY || !predZ || !isPinned)
    {
        Error_SetError(ErrorCode::InvalidArgument);
        return;
    }

    GetGlobalPool().parallel_for(nodeCount, [&](int i)
        {
            px[i] = cx[i]; py[i] = cy[i]; pz[i] = cz[i];

            cx[i] = IsFinite(predX[i]) ? predX[i] : px[i];
            cy[i] = IsFinite(predY[i]) ? predY[i] : py[i];
            cz[i] = IsFinite(predZ[i]) ? predZ[i] : pz[i];
        }, 4096);
}

extern "C" EXPORT void UpdateNodeRotations(Quaternion* currentRotations,
    const float* cx, const float* cy, const float* cz,
    const int* neighborIndices, const int* neighborOffsets,
    const Vector3* restOffsetsCache,
    int nodeCount)
{
    if (nodeCount <= 0) return; // nothing to update, not an error
    if (!currentRotations || !cx || !cy || !cz || !neighborIndices || !neighborOffsets || !restOffsetsCache)
    {
        Error_SetError(ErrorCode::InvalidArgument);
        return;
    }

    constexpr int kMinRotationBlocks = 4096;

    GetGlobalPool().parallel_for(nodeCount, [&](int i)
        {
            int start = neighborOffsets[i];
            int end = neighborOffsets[i + 1];

            if (end <= start) { currentRotations[i] = QIdentity(); return; }

            constexpr int MAX_NEIGHBORS = 64;
            int neighborCount = end - start;
            if (neighborCount > MAX_NEIGHBORS) neighborCount = MAX_NEIGHBORS;

            const Vector3* restOffsets = restOffsetsCache + start;
            Vector3 ci = { cx[i], cy[i], cz[i], 0.0f };

            // 1. O(N) Precomputation: Build Covariance Matrix A & rest length sum
            float A00 = 0, A01 = 0, A02 = 0;
            float A10 = 0, A11 = 0, A12 = 0;
            float A20 = 0, A21 = 0, A22 = 0;
            float restLenSqSum = 0;

            for (int n = 0; n < neighborCount; ++n)
            {
                int j = neighborIndices[start + n];
                float cx_j = cx[j] - ci.x;
                float cy_j = cy[j] - ci.y;
                float cz_j = cz[j] - ci.z;

                const Vector3& off = restOffsets[n];

                A00 += off.x * cx_j; A01 += off.x * cy_j; A02 += off.x * cz_j;
                A10 += off.y * cx_j; A11 += off.y * cy_j; A12 += off.y * cz_j;
                A20 += off.z * cx_j; A21 += off.z * cy_j; A22 += off.z * cz_j;

                restLenSqSum += off.x * off.x + off.y * off.y + off.z * off.z;
            }

            Quaternion q = currentRotations[i];

            // 2. O(1) Iterative Solver
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

                // Derive omega directly from M = R * A (where M12 = m10*A02 + m11*A12 + m12*A22, etc)
                Vector3 omega = {
                    (m10 * A02 + m11 * A12 + m12 * A22) - (m20 * A01 + m21 * A11 + m22 * A21), // M12 - M21
                    (m20 * A00 + m21 * A10 + m22 * A20) - (m00 * A02 + m01 * A12 + m02 * A22), // M20 - M02
                    (m00 * A01 + m01 * A11 + m02 * A21) - (m10 * A00 + m11 * A10 + m12 * A20), // M01 - M10
                    0.0f
                };

                // denom = Trace(M) + restLenSqSum
                float denom = (m00 * A00 + m01 * A10 + m02 * A20) +
                    (m10 * A01 + m11 * A11 + m12 * A21) +
                    (m20 * A02 + m21 * A12 + m22 * A22) + restLenSqSum;

                if (AbsF(denom) < PhysicsConstants::ROTATION_CONVERGENCE_EPSILON) break;

                float invDen = 1.0f / denom;
                omega.x *= invDen; omega.y *= invDen; omega.z *= invDen;

                constexpr float MAX_OMEGA = 2.0f;
                const float MIN_OMEGA_SQ = PhysicsConstants::MIN_OMEGA_MAGNITUDE * PhysicsConstants::MIN_OMEGA_MAGNITUDE;
                constexpr float MAX_OMEGA_SQ = MAX_OMEGA * MAX_OMEGA;

                float wSq = omega.x * omega.x + omega.y * omega.y + omega.z * omega.z;
                if (wSq < MIN_OMEGA_SQ) break;
                if (wSq > MAX_OMEGA_SQ)
                {
                    float w = std::sqrt(wSq);
                    float scale = MAX_OMEGA / w;
                    omega.x *= scale; omega.y *= scale; omega.z *= scale;
                }

                Quaternion dq = { omega.x * 0.5f, omega.y * 0.5f, omega.z * 0.5f, 1.0f };
                q = QNormalize(QMul(dq, q));
            }
            currentRotations[i] = q;
        }, kMinRotationBlocks);
}