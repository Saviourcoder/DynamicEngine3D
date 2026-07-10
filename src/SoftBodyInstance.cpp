/*
 DYNAMICENGINE3D
 AI-Assisted Soft-Body Physics for Unity3D
 By: Elitmers
*/

#include "SoftBodyInstance.h"
#include <unordered_map>
#include <omp.h>
#include <chrono>

extern "C" {
    struct EngineStats {
        int totalNodes;
        int totalBeams;
        int totalFaces;
        float integrateTimeMs;
        float constraintTimeMs;
        float collisionTimeMs;
        float fps;
    };

    EngineStats g_Stats = { 0 };
    static auto g_LastFrameTime = std::chrono::high_resolution_clock::now();
    static int g_FrameCount = 0;
    static float g_TimeAccum = 0.0f;

    EXPORT void Engine_GetStats(EngineStats* outStats) {
        if (outStats) *outStats = g_Stats;
    }

    EXPORT void Engine_ClearFrameStats() {
        g_Stats.integrateTimeMs = 0;
        g_Stats.constraintTimeMs = 0;
        g_Stats.collisionTimeMs = 0;
        g_Stats.totalNodes = 0;
        g_Stats.totalBeams = 0;
        g_Stats.totalFaces = 0;

        // Calculate DLL-side FPS
        auto now = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float>(now - g_LastFrameTime).count();
        g_LastFrameTime = now;

        g_TimeAccum += dt;
        g_FrameCount++;
        if (g_TimeAccum >= 1.0f) {
            g_Stats.fps = g_FrameCount / g_TimeAccum;
            g_FrameCount = 0;
            g_TimeAccum = 0.0f;
        }
    }
}

// Forward decls from Solver.cpp
extern "C" void Integrate(float* cx, float* cy, float* cz, float* px, float* py, float* pz, float* predX, float* predY, float* predZ, const uint8_t* isPinned, int nodeCount, float dt, float damping, float gravity);
extern "C" void SolveConstraints(float* predX, float* predY, float* predZ, const BeamData* beams, float* multipliers, const float* masses, const uint8_t* isPinned, int beamCount, float dt);
extern "C" void ApplyBeamDampingForces(float* predX, float* predY, float* predZ, const float* px, const float* py, const float* pz, const BeamData* beams, const float* masses, const uint8_t* isPinned, int beamCount, float dt);
extern "C" void ApplyPressure(float* predX, float* predY, float* predZ, const float* cx, const float* cy, const float* cz, const float* masses, const uint8_t* isPinned, int nodeCount, float internalPressure, float dt);
extern "C" void ApplyPlasticity(const float* predX, const float* predY, const float* predZ, BeamData* beams, int beamCount, float dt);
extern "C" void FinalizePositions(float* cx, float* cy, float* cz, float* px, float* py, float* pz, const float* predX, const float* predY, const float* predZ, const uint8_t* isPinned, int nodeCount);
extern "C" void UpdateNodeRotations(Quaternion* currentRotations, const Quaternion* initialRotations, const float* cx, const float* cy, const float* cz, const float* ix, const float* iy, const float* iz, const int* neighborIndices, const int* neighborOffsets, int nodeCount);

void SoftBodyInstance::SyncMultipliers()
{
    multipliers.assign(beams.size(), 0.0f);
}

void SoftBodyInstance::RebuildNeighborCache()
{
    int nodeCount = (int)currentX.size();
    std::vector<int> counts(nodeCount, 0);
    for (const auto& b : beams)
    {
        if (b.isCrossBody || !b.isActive) continue;
        if (b.nodeA >= nodeCount || b.nodeB >= nodeCount || b.nodeA < 0 || b.nodeB < 0) continue;
        counts[b.nodeA]++;
        counts[b.nodeB]++;
    }

    nbrOffsets.resize(nodeCount + 1, 0);
    int total = 0;
    for (int i = 0; i < nodeCount; i++)
    {
        nbrOffsets[i] = total;
        total += counts[i];
    }
    nbrOffsets[nodeCount] = total;

    nbrData.resize(total);
    std::vector<int> cursors(nodeCount, 0);
    for (const auto& b : beams)
    {
        if (b.nodeA >= nodeCount || b.nodeB >= nodeCount || b.nodeA < 0 || b.nodeB < 0) continue;
        if (b.isCrossBody || !b.isActive) continue;
        nbrData[nbrOffsets[b.nodeA] + cursors[b.nodeA]++] = b.nodeB;
        nbrData[nbrOffsets[b.nodeB] + cursors[b.nodeB]++] = b.nodeA;
    }
}

// Helper Internal Functions for Cross-Body Math
void SolveCrossBodyBeam(BeamData& beam, float dt)
{
    SoftBodyInstance* bA = beam.bodyA;
    SoftBodyInstance* bB = beam.bodyB;
    if (!bA || !bB) return;

    if (beam.nodeA < 0 || beam.nodeA >= (int)bA->predictedX.size() ||
        beam.nodeB < 0 || beam.nodeB >= (int)bB->predictedX.size())
        return;

    float pAx = bA->predictedX[beam.nodeA], pAy = bA->predictedY[beam.nodeA], pAz = bA->predictedZ[beam.nodeA];
    float pBx = bB->predictedX[beam.nodeB], pBy = bB->predictedY[beam.nodeB], pBz = bB->predictedZ[beam.nodeB];
    float dx = pBx - pAx, dy = pBy - pAy, dz = pBz - pAz;
    float currentLenSq = dx * dx + dy * dy + dz * dz;

    if (currentLenSq < PhysicsConstants::MIN_BEAM_LENGTH * PhysicsConstants::MIN_BEAM_LENGTH) return;
    float currentLen = std::sqrt(currentLenSq);

    float invLen = 1.0f / currentLen;
    float gradX = dx * invLen, gradY = dy * invLen, gradZ = dz * invLen;
    float targetLen = beam.restLength;

    if (beam.minLength > 0.0f && currentLen < beam.minLength) targetLen = beam.minLength;
    else if (beam.maxLength > 0.0f && currentLen > beam.maxLength) targetLen = beam.maxLength;
    else if (beam.minLength > 0.0f || beam.maxLength > 0.0f) return;

    float C = currentLen - targetLen;
    float effectiveCompliance = beam.compliance / (dt * dt);

    bool pinA = bA->isPinned[beam.nodeA];
    bool pinB = bB->isPinned[beam.nodeB];
    float wA = pinA ? 0.0f : 1.0f / bA->masses[beam.nodeA];
    float wB = pinB ? 0.0f : 1.0f / bB->masses[beam.nodeB];
    float wSum = wA + wB;

    if (wSum <= 0.0f) return;

    float deltaLambda = -(C + effectiveCompliance * beam.lagrangeMultiplier) / (wSum + effectiveCompliance);

    beam.lagrangeMultiplier += deltaLambda;
    float corrX = gradX * deltaLambda, corrY = gradY * deltaLambda, corrZ = gradZ * deltaLambda;

    if (!pinA)
    {
        bA->predictedX[beam.nodeA] -= corrX * wA;
        bA->predictedY[beam.nodeA] -= corrY * wA;
        bA->predictedZ[beam.nodeA] -= corrZ * wA;
    }
    if (!pinB)
    {
        bB->predictedX[beam.nodeB] += corrX * wB;
        bB->predictedY[beam.nodeB] += corrY * wB;
        bB->predictedZ[beam.nodeB] += corrZ * wB;
    }
}

void SolveEdgeSliding(BeamData& beam, float dt)
{
    SoftBodyInstance* sBody = reinterpret_cast<SoftBodyInstance*>(beam.bodyA);
    SoftBodyInstance* eBody = reinterpret_cast<SoftBodyInstance*>(beam.bodyB);
    if (!sBody || !eBody) return;

    if (beam.slidingNode < 0 || beam.slidingNode >= (int)sBody->predictedX.size() ||
        beam.edgeNodeA < 0 || beam.edgeNodeA >= (int)eBody->predictedX.size() ||
        beam.edgeNodeB < 0 || beam.edgeNodeB >= (int)eBody->predictedX.size())
        return;

    Vector3 nodePos = { sBody->predictedX[beam.slidingNode], sBody->predictedY[beam.slidingNode], sBody->predictedZ[beam.slidingNode], 0.0f };
    Vector3 edgePosA = { eBody->predictedX[beam.edgeNodeA], eBody->predictedY[beam.edgeNodeA], eBody->predictedZ[beam.edgeNodeA], 0.0f };
    Vector3 edgePosB = { eBody->predictedX[beam.edgeNodeB], eBody->predictedY[beam.edgeNodeB], eBody->predictedZ[beam.edgeNodeB], 0.0f };

    Vector3 edgeVec = Sub(edgePosB, edgePosA);
    float edgeLen = Length(edgeVec);
    if (edgeLen < PhysicsConstants::MIN_BEAM_LENGTH) return;

    Vector3 edgeDir = Scale(edgeVec, 1.0f / edgeLen);
    Vector3 nodeToEdgeStart = Sub(nodePos, edgePosA);
    float t = Dot(nodeToEdgeStart, edgeDir);

    Vector3 closestPt = Add(edgePosA, Scale(edgeDir, t));
    Vector3 perpVec = Sub(nodePos, closestPt);
    float perpDist = Length(perpVec);

    if (perpDist < PhysicsConstants::MIN_BEAM_LENGTH) return;

    Vector3 perpDir = Scale(perpVec, 1.0f / perpDist);

    if (LengthSq(beam.initialPerpDir) > 0.0f && Dot(perpDir, beam.initialPerpDir) < 0.0f) {
        perpDir = Neg(perpDir);
        perpDist = -perpDist;
    }

    float constraint = perpDist - beam.targetPerpDistance;
    float dampedCompliance = beam.compliance + (beam.damping + PhysicsConstants::CROSS_BODY_CONSTRAINT_DAMPING) * dt;

    bool pinNode = sBody->isPinned[beam.slidingNode];
    bool pinEdgeA = eBody->isPinned[beam.edgeNodeA];
    bool pinEdgeB = eBody->isPinned[beam.edgeNodeB];

    if (pinNode && pinEdgeA && pinEdgeB) return;

    float massNode = sBody->masses[beam.slidingNode];
    float massEdgeA = eBody->masses[beam.edgeNodeA];
    float massEdgeB = eBody->masses[beam.edgeNodeB];

    float weightA = 1.0f - (t / edgeLen);
    float weightB = t / edgeLen;

    float wNode = pinNode ? 0.0f : 1.0f / massNode;
    float wEdgeA = pinEdgeA ? 0.0f : (1.0f / massEdgeA) * weightA * weightA;
    float wEdgeB = pinEdgeB ? 0.0f : (1.0f / massEdgeB) * weightB * weightB;
    float wSum = wNode + wEdgeA + wEdgeB;

    if (wSum <= 0.0f) return;

    float effectiveCompliance = dampedCompliance / (dt * dt);

    float deltaLambda = -(constraint + effectiveCompliance * beam.lagrangeMultiplier) / (wSum + effectiveCompliance);

    beam.lagrangeMultiplier += deltaLambda;

    if (!pinNode) {
        sBody->predictedX[beam.slidingNode] += perpDir.x * (1.0f / massNode) * deltaLambda;
        sBody->predictedY[beam.slidingNode] += perpDir.y * (1.0f / massNode) * deltaLambda;
        sBody->predictedZ[beam.slidingNode] += perpDir.z * (1.0f / massNode) * deltaLambda;
    }
    if (!pinEdgeA) {
        eBody->predictedX[beam.edgeNodeA] -= perpDir.x * (1.0f / massEdgeA) * deltaLambda * weightA;
        eBody->predictedY[beam.edgeNodeA] -= perpDir.y * (1.0f / massEdgeA) * deltaLambda * weightA;
        eBody->predictedZ[beam.edgeNodeA] -= perpDir.z * (1.0f / massEdgeA) * deltaLambda * weightA;
    }
    if (!pinEdgeB) {
        eBody->predictedX[beam.edgeNodeB] -= perpDir.x * (1.0f / massEdgeB) * deltaLambda * weightB;
        eBody->predictedY[beam.edgeNodeB] -= perpDir.y * (1.0f / massEdgeB) * deltaLambda * weightB;
        eBody->predictedZ[beam.edgeNodeB] -= perpDir.z * (1.0f / massEdgeB) * deltaLambda * weightB;
    }
}

// Export Functions

EXPORT SoftBodyInstance* SoftBody_Create()
{
    return new SoftBodyInstance();
}

EXPORT void SoftBody_Destroy(SoftBodyInstance* body)
{
    if (body) delete body;
}

EXPORT void SoftBody_GetPositions(SoftBodyInstance* body, Vector3* outPositions)
{
    if (!body || !outPositions) return;
    int count = (int)body->currentX.size();
    for (int i = 0; i < count; ++i) {
        outPositions[i] = { body->currentX[i], body->currentY[i], body->currentZ[i], 0.0f };
    }
}

EXPORT void SoftBody_GetRotations(SoftBodyInstance* body, Quaternion* outRotations)
{
    if (!body || !outRotations) return;
    std::copy(body->currentRot.begin(), body->currentRot.end(), outRotations);
}

EXPORT void SoftBody_ClearNodes(SoftBodyInstance* body)
{
    if (!body) return;
    body->currentX.clear(); body->currentY.clear(); body->currentZ.clear();
    body->previousX.clear(); body->previousY.clear(); body->previousZ.clear();
    body->predictedX.clear(); body->predictedY.clear(); body->predictedZ.clear();
    body->initialX.clear(); body->initialY.clear(); body->initialZ.clear();
    body->currentRot.clear();
    body->masses.clear();
    body->isPinned.clear();
    body->beams.clear();
}

EXPORT void SoftBody_SetupNodes(SoftBodyInstance* body, Vector3* worldPositions, int count)
{
    if (!body || !worldPositions || count <= 0) return;

    body->currentX.resize(count); body->currentY.resize(count); body->currentZ.resize(count);
    body->previousX.resize(count); body->previousY.resize(count); body->previousZ.resize(count);
    body->predictedX.resize(count); body->predictedY.resize(count); body->predictedZ.resize(count);
    body->initialX.resize(count); body->initialY.resize(count); body->initialZ.resize(count);

    for (int i = 0; i < count; ++i) {
        body->currentX[i] = body->previousX[i] = body->predictedX[i] = body->initialX[i] = worldPositions[i].x;
        body->currentY[i] = body->previousY[i] = body->predictedY[i] = body->initialY[i] = worldPositions[i].y;
        body->currentZ[i] = body->previousZ[i] = body->predictedZ[i] = body->initialZ[i] = worldPositions[i].z;
    }

    body->currentRot.resize(count, QIdentity());
    body->masses.resize(count, PhysicsConstants::DEFAULT_NODE_MASS);
    body->isPinned.resize(count, false);
}

EXPORT void SoftBody_GenerateBeamsFromDistance(SoftBodyInstance* body, float connectionDist)
{
    if (!body) return;
    float minDistSq = PhysicsConstants::MIN_BEAM_LENGTH * PhysicsConstants::MIN_BEAM_LENGTH;
    float maxDistSq = connectionDist * connectionDist;
    int count = (int)body->currentX.size();

    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            float dx = body->currentX[i] - body->currentX[j];
            float dy = body->currentY[i] - body->currentY[j];
            float dz = body->currentZ[i] - body->currentZ[j];
            float distSq = dx * dx + dy * dy + dz * dz;
            if (distSq <= maxDistSq && distSq > minDistSq) {
                float dist = std::sqrt(distSq);
                BeamData b = {};
                b.nodeA = i;
                b.nodeB = j;
                b.compliance = body->compliance;
                b.damping = body->defaultDamping;
                b.restLength = dist;
                b.originalRestLength = dist;
                b.isActive = 1;
                b.isCrossBody = 0;
                body->beams.push_back(b);
            }
        }
    }
    body->RebuildNeighborCache();
    body->SyncMultipliers();
}

EXPORT void SoftBody_SetBeamCompliance(SoftBodyInstance* body, int index, float compliance, float damping)
{
    if (!body || index < 0 || index >= (int)body->beams.size()) return;
    body->beams[index].compliance = compliance;
    body->beams[index].damping = damping;
}

EXPORT void SoftBody_ApplyWorldForceToNode(SoftBodyInstance* body, int nodeIndex, Vector3 worldForce, float dt)
{
    if (!body) return;
    if (nodeIndex < 0 || nodeIndex >= (int)body->masses.size() || body->isPinned[nodeIndex]) return;

    float mass = body->masses[nodeIndex];
    if (mass <= 0.0f) return;

    float ax = worldForce.x / mass;
    float ay = worldForce.y / mass;
    float az = worldForce.z / mass;

    float vx = (body->currentX[nodeIndex] - body->previousX[nodeIndex]) / dt;
    float vy = (body->currentY[nodeIndex] - body->previousY[nodeIndex]) / dt;
    float vz = (body->currentZ[nodeIndex] - body->previousZ[nodeIndex]) / dt;

    vx += ax * dt;
    vy += ay * dt;
    vz += az * dt;

    body->previousX[nodeIndex] = body->currentX[nodeIndex] - vx * dt;
    body->previousY[nodeIndex] = body->currentY[nodeIndex] - vy * dt;
    body->previousZ[nodeIndex] = body->currentZ[nodeIndex] - vz * dt;
}

EXPORT void SoftBody_ResetLagrangeMultipliers(SoftBodyInstance* body)
{
    if (body) std::fill(body->multipliers.begin(), body->multipliers.end(), 0.0f);
}

EXPORT void SoftBody_CheckAndBreakConstraints(SoftBodyInstance* body, float dt)
{
    if (!body) return;
    if (dt <= 0.0f) return;

    constexpr float TIME_CONSTANT = 0.15f;   // ~150 ms window for "sustained"
    constexpr float BURST_MULT = 4.0f;   // tolerate short spikes up to 4� strength

    float alpha = 1.0f - std::exp(-dt / TIME_CONSTANT);
    float dtSq = dt * dt;

    for (auto& beam : body->beams) {
        if (!beam.isActive || !beam.isCrossBody || beam.isBroken) continue;
        if (beam.strength <= 0.0f || !std::isfinite(beam.strength)) continue;

        float forceMag = AbsF(beam.lagrangeMultiplier) / dtSq;

        // EMA low-pass: smooths out single-substep spikes
        beam.forceEMA = (1.0f - alpha) * beam.forceEMA + alpha * forceMag;

        bool sustainedOverload = beam.forceEMA > beam.strength;
        bool catastrophicBurst = forceMag > beam.strength * BURST_MULT;

        if (sustainedOverload || catastrophicBurst) {
            beam.isBroken = 1;
            beam.isActive = 0;
        }
    }
}

EXPORT void SoftBody_SetBeamEdgeData(SoftBodyInstance* body, int beamIndex,
    float targetPerpDistance, float dirX, float dirY, float dirZ)
{
    if (!body || beamIndex < 0 || beamIndex >= (int)body->beams.size()) return;
    BeamData& b = body->beams[beamIndex];
    b.targetPerpDistance = targetPerpDistance;
    b.initialPerpDir = { dirX, dirY, dirZ };
}

EXPORT void SoftBody_SolveCrossBodyStep(SoftBodyInstance* body, float dt, int iterations)
{
    if (!body) return;
    if (iterations < 1) iterations = 1;

    for (int it = 0; it < iterations; ++it)
    {
        for (auto& beam : body->beams)
        {
            if (!beam.isActive || !beam.isCrossBody) continue;
            if (beam.bodyA != body) continue; //only bodyA will solve the beam. This prevents double solving
            if (beam.bodyB == 0) continue;

            if (beam.isEdgeSliding)
            {
                SolveEdgeSliding(beam, dt);
            }
            else
            {
                SolveCrossBodyBeam(beam, dt);
            }
        }
    }
}

EXPORT void SoftBody_UpdateNodeRotations(SoftBodyInstance* body)
{
    if (!body) return;
    int nodeCount = (int)body->currentX.size();
    if (nodeCount == 0) return;

    if (body->nbrOffsets.empty()) body->RebuildNeighborCache();

    // Use the refactored UpdateNodeRotations from Solver.cpp to apply SoA effectively
    std::vector<Quaternion> initialRots(nodeCount, QIdentity());
    ::UpdateNodeRotations(body->currentRot.data(), initialRots.data(),
        body->currentX.data(), body->currentY.data(), body->currentZ.data(),
        body->initialX.data(), body->initialY.data(), body->initialZ.data(),
        body->nbrData.data(), body->nbrOffsets.data(), nodeCount);
}

EXPORT void SoftBody_SetNodePinned(SoftBodyInstance* body, int index, bool pinned)
{
    if (!body) return;
    if (index >= 0 && index < (int)body->isPinned.size()) body->isPinned[index] = pinned;
}

EXPORT void SoftBody_SetPreviousPos(SoftBodyInstance* body, int index, Vector3 pos)
{
    if (!body) return;
    if (index >= 0 && index < (int)body->previousX.size()) {
        body->previousX[index] = pos.x; body->previousY[index] = pos.y; body->previousZ[index] = pos.z;
    }
}

EXPORT Vector3 SoftBody_GetPredictedPos(SoftBodyInstance* body, int index)
{
    if (!body) return { 0,0,0,0 };
    if (index >= 0 && index < (int)body->predictedX.size()) return { body->predictedX[index], body->predictedY[index], body->predictedZ[index], 0.0f };
    return { 0,0,0,0 };
}

EXPORT void SoftBody_SetBeamRestLength(SoftBodyInstance* body, int beamIndex, float length)
{
    if (!body) return;
    if (beamIndex >= 0 && beamIndex < (int)body->beams.size()) body->beams[beamIndex].restLength = length;
}

EXPORT int SoftBody_AddBeam(
    SoftBodyInstance* bodyA, int nodeA,
    SoftBodyInstance* bodyB, int nodeB,
    float compliance, float damping, float restLength,
    float minL, float maxL, float strength, float plasticityThreshold, float plasticityRate, float maxDeformation,
    int isEdgeSliding, int edgeA, int edgeB, int slidingNode)
{
    if (!bodyA) return -1;
    BeamData b = {};
    b.nodeA = nodeA;
    b.nodeB = nodeB;
    b.compliance = compliance;
    b.damping = damping;
    b.restLength = restLength;
    b.originalRestLength = restLength;
    b.minLength = minL;
    b.maxLength = maxL;
    b.strength = strength;
    b.plasticityThreshold = plasticityThreshold;
    b.plasticityRate = plasticityRate;
    b.maxDeformation = maxDeformation;
    b.forceEMA = 0.0f;
    b.isCrossBody = (bodyA == bodyB) ? 0 : 1;
    b.isActive = 1;
    b.isBroken = 0;
    b.bodyA = bodyA;
    b.bodyB = bodyB;
    b.isEdgeSliding = isEdgeSliding ? 1 : 0;
    b.edgeNodeA = edgeA;
    b.edgeNodeB = edgeB;
    b.slidingNode = slidingNode;

    bodyA->beams.push_back(b);
    bodyA->multipliers.push_back(0.0f);
    return (int)bodyA->beams.size() - 1;
}

EXPORT void SoftBody_UpdateBeamBulk(SoftBodyInstance* body, float compliance, float damping)
{
    if (!body) return;
    body->compliance = compliance;
    body->defaultDamping = damping;
    for (auto& b : body->beams)
    {
        b.compliance = compliance;
        b.damping = damping;
    }
}

EXPORT void SoftBody_SetPredictedPos(SoftBodyInstance* body, int index, Vector3 pos)
{
    if (!body) return;
    if (index >= 0 && index < (int)body->predictedX.size()) {
        body->predictedX[index] = pos.x; body->predictedY[index] = pos.y; body->predictedZ[index] = pos.z;
    }
}

EXPORT Vector3 SoftBody_GetCurrentPos(SoftBodyInstance* body, int index)
{
    if (!body) return { 0.0f, 0.0f, 0.0f, 0.0f };
    if (index >= 0 && index < (int)body->currentX.size())
        return { body->currentX[index], body->currentY[index], body->currentZ[index], 0.0f };
    return { 0.0f, 0.0f, 0.0f, 0.0f };
}

EXPORT void SoftBody_SetCurrentPos(SoftBodyInstance* body, int index, Vector3 pos)
{
    if (!body) return;
    if (index >= 0 && index < (int)body->currentX.size()) {
        body->currentX[index] = pos.x; body->currentY[index] = pos.y; body->currentZ[index] = pos.z;
    }
}

EXPORT void SoftBody_SetNodeMass(SoftBodyInstance* body, int index, float mass)
{
    if (!body) return;
    if (index >= 0 && index < (int)body->masses.size())
        body->masses[index] = mass;
}

EXPORT void SoftBody_SetInternalPressure(SoftBodyInstance* body, float pressure)
{
    if (!body) return;
    body->internalPressure = pressure;
}

EXPORT void SoftBody_SetPredictedPosBulk(SoftBodyInstance* body, float* xs, float* ys, float* zs, int count)
{
    if (!body || !xs || !ys || !zs || count <= 0) return;
    if (count == (int)body->predictedX.size()) {
        std::copy(xs, xs + count, body->predictedX.begin());
        std::copy(ys, ys + count, body->predictedY.begin());
        std::copy(zs, zs + count, body->predictedZ.begin());
    }
}

EXPORT void SoftBody_SetCurrentPosBulk(SoftBodyInstance* body, float* xs, float* ys, float* zs, int count)
{
    if (!body || !xs || !ys || !zs || count <= 0) return;
    if (count == (int)body->currentX.size()) {
        std::copy(xs, xs + count, body->currentX.begin());
        std::copy(ys, ys + count, body->currentY.begin());
        std::copy(zs, zs + count, body->currentZ.begin());
    }
}

EXPORT void SoftBody_SetPreviousPosBulk(SoftBodyInstance* body, float* xs, float* ys, float* zs, int count)
{
    if (!body || !xs || !ys || !zs || count <= 0) return;
    if (count == (int)body->previousX.size()) {
        std::copy(xs, xs + count, body->previousX.begin());
        std::copy(ys, ys + count, body->previousY.begin());
        std::copy(zs, zs + count, body->previousZ.begin());
    }
}

EXPORT void SoftBody_StepPhysics(SoftBodyInstance* body, float dt, int constraintIters, float gravity)
{
    if (!body) return;
    if (body->currentX.empty() || dt <= 0.0f) return;

    int nodeCount = (int)body->currentX.size();
    int beamCount = (int)body->beams.size();

    float* masses = body->masses.data();
    const uint8_t* pinned = body->isPinned.data();

    // Accumulate counts (Thread-safe)
#pragma omp atomic
    g_Stats.totalNodes += nodeCount;
#pragma omp atomic
    g_Stats.totalBeams += beamCount;

    // --- Profile Integration ---
    auto t0 = std::chrono::high_resolution_clock::now();
    Integrate(body->currentX.data(), body->currentY.data(), body->currentZ.data(),
        body->previousX.data(), body->previousY.data(), body->previousZ.data(),
        body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(),
        pinned, nodeCount, dt, body->defaultDamping, gravity);
    auto t1 = std::chrono::high_resolution_clock::now();

    float intTime = std::chrono::duration<float, std::milli>(t1 - t0).count();
#pragma omp atomic
    g_Stats.integrateTimeMs += intTime;

    if (body->internalPressure > 0.0f)
        ApplyPressure(body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(),
            body->currentX.data(), body->currentY.data(), body->currentZ.data(),
            masses, pinned, nodeCount, body->internalPressure, dt);

    if (beamCount > 0)
        ApplyBeamDampingForces(body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(),
            body->previousX.data(), body->previousY.data(), body->previousZ.data(),
            body->beams.data(), masses, pinned, beamCount, dt);

    std::fill(body->multipliers.begin(), body->multipliers.end(), 0.0f);
    for (auto& beam : body->beams)
    {
        if (beam.isCrossBody)
            beam.lagrangeMultiplier = 0.0f;
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    for (int iter = 0; iter < constraintIters; ++iter)
        SolveConstraints(body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(),
            body->beams.data(), body->multipliers.data(), masses, pinned, beamCount, dt);
    auto t3 = std::chrono::high_resolution_clock::now();

    float constTime = std::chrono::duration<float, std::milli>(t3 - t2).count();
#pragma omp atomic
    g_Stats.constraintTimeMs += constTime;

    if (beamCount > 0)
    {
        ApplyPlasticity(body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(), body->beams.data(), beamCount, dt);
    }
}

EXPORT void SoftBody_FinalizeStep(SoftBodyInstance* body)
{
    if (!body) return;
    FinalizePositions(body->currentX.data(), body->currentY.data(), body->currentZ.data(),
        body->previousX.data(), body->previousY.data(), body->previousZ.data(),
        body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(),
        body->isPinned.data(), (int)body->currentX.size());
}

EXPORT void* SoftBody_GetPredictedX(SoftBodyInstance* b) { return b ? b->predictedX.data() : nullptr; }
EXPORT void* SoftBody_GetPredictedY(SoftBodyInstance* b) { return b ? b->predictedY.data() : nullptr; }
EXPORT void* SoftBody_GetPredictedZ(SoftBodyInstance* b) { return b ? b->predictedZ.data() : nullptr; }

EXPORT void* SoftBody_GetCurrentX(SoftBodyInstance* b) { return b ? b->currentX.data() : nullptr; }
EXPORT void* SoftBody_GetCurrentY(SoftBodyInstance* b) { return b ? b->currentY.data() : nullptr; }
EXPORT void* SoftBody_GetCurrentZ(SoftBodyInstance* b) { return b ? b->currentZ.data() : nullptr; }

EXPORT void* SoftBody_GetPreviousX(SoftBodyInstance* b) { return b ? b->previousX.data() : nullptr; }
EXPORT void* SoftBody_GetPreviousY(SoftBodyInstance* b) { return b ? b->previousY.data() : nullptr; }
EXPORT void* SoftBody_GetPreviousZ(SoftBodyInstance* b) { return b ? b->previousZ.data() : nullptr; }

EXPORT void* SoftBody_GetMasses(SoftBodyInstance* b) { return b ? b->masses.data() : nullptr; }
EXPORT void* SoftBody_GetPinned(SoftBodyInstance* b) { return b ? b->isPinned.data() : nullptr; }

EXPORT void SoftBody_RemoveCrossBodyBeams(SoftBodyInstance* body, SoftBodyInstance* target)
{
    if (!body || !target) return;
    auto& beams = body->beams;
    beams.erase(std::remove_if(beams.begin(), beams.end(),
        [target](const BeamData& b) {
            return b.isCrossBody && (b.bodyA == target || b.bodyB == target);
        }), beams.end());
}

EXPORT void SoftBody_StepPhysicsBatch(SoftBodyInstance** bodies, int count,
    float dt, int constraintIters, float gravity)
{
    if (count <= 0 || dt <= 0.0f) return;

    // ── Phase 1: Integrate all bodies in parallel ────────────────────────────
    // Each body is independent here, so full parallelism is safe.
    // Pressure, beam damping, and multiplier resets all belong to this phase —
    // they must complete for every body before any constraints run.
    auto t0 = std::chrono::high_resolution_clock::now();

#pragma omp parallel for schedule(static, 1) if(count > 4 && !omp_in_parallel())
    for (int i = 0; i < count; ++i)
    {
        SoftBodyInstance* body = bodies[i];
        if (!body || body->currentX.empty()) continue;

        int nodeCount = (int)body->currentX.size();
        int beamCount = (int)body->beams.size();
        const float* masses = body->masses.data();
        const uint8_t* pinned = body->isPinned.data();

#pragma omp atomic
        g_Stats.totalNodes += nodeCount;
#pragma omp atomic
        g_Stats.totalBeams += beamCount;

        Integrate(body->currentX.data(), body->currentY.data(), body->currentZ.data(),
            body->previousX.data(), body->previousY.data(), body->previousZ.data(),
            body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(),
            pinned, nodeCount, dt, body->defaultDamping, gravity);

        if (body->internalPressure > 0.0f)
            ApplyPressure(body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(),
                body->currentX.data(), body->currentY.data(), body->currentZ.data(),
                masses, pinned, nodeCount, body->internalPressure, dt);

        if (beamCount > 0)
            ApplyBeamDampingForces(body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(),
                body->previousX.data(), body->previousY.data(), body->previousZ.data(),
                body->beams.data(), masses, pinned, beamCount, dt);

        // Reset Lagrange multipliers once per substep, not per iteration.
        std::fill(body->multipliers.begin(), body->multipliers.end(), 0.0f);
        for (auto& beam : body->beams)
            if (beam.isCrossBody) beam.lagrangeMultiplier = 0.0f;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    g_Stats.integrateTimeMs += std::chrono::duration<float, std::milli>(t1 - t0).count();

    // ── Phase 2: Global constraint iterations ────────────────────────────────
    // Internal and cross-body beams now solve together within each iteration.
    // Previously, all internal iterations finished before any cross-body work
    // began, so corrections never converged together.
    //
    // Step 2a is parallel: bodies share no data through internal beams.
    // Step 2b is sequential: each cross-body beam writes to two bodies,
    //   making it unsafe to parallelize without fine-grained locking.
    auto t2 = std::chrono::high_resolution_clock::now();

    for (int iter = 0; iter < constraintIters; ++iter)
    {
        // 2a. Internal beams — each body is self-contained, full parallelism.
#pragma omp parallel for schedule(static, 1) if(count > 4 && !omp_in_parallel())
        for (int i = 0; i < count; ++i)
        {
            SoftBodyInstance* body = bodies[i];
            if (!body || body->beams.empty()) continue;

            SolveConstraints(body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(),
                body->beams.data(), body->multipliers.data(),
                body->masses.data(), body->isPinned.data(),
                (int)body->beams.size(), dt);
        }

        // 2b. Cross-body beams — sequential; each beam is owned by bodyA
        //   and solved exactly once per iteration to prevent double-counting.
        for (int i = 0; i < count; ++i)
        {
            SoftBodyInstance* body = bodies[i];
            if (!body) continue;

            for (auto& beam : body->beams)
            {
                if (!beam.isActive || !beam.isCrossBody) continue;
                if (beam.bodyA != body) continue; // bodyA is the canonical owner
                if (!beam.bodyB) continue;

                if (beam.isEdgeSliding)
                    SolveEdgeSliding(beam, dt);
                else
                    SolveCrossBodyBeam(beam, dt);
            }
        }
    }

    auto t3 = std::chrono::high_resolution_clock::now();
    g_Stats.constraintTimeMs += std::chrono::duration<float, std::milli>(t3 - t2).count();

    // ── Phase 3: Plasticity (parallel; only reads predicted positions) ────────
#pragma omp parallel for schedule(static, 1) if(count > 4 && !omp_in_parallel())
    for (int i = 0; i < count; ++i)
    {
        SoftBodyInstance* body = bodies[i];
        if (!body || body->beams.empty()) continue;

        ApplyPlasticity(body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(),
            body->beams.data(), (int)body->beams.size(), dt);
    }
}

EXPORT void SoftBody_FinalizeBatch(SoftBodyInstance** bodies, int count, float dt)
{
#pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < count; ++i)
        if (bodies[i]) SoftBody_CheckAndBreakConstraints(bodies[i], dt);

#pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < count; ++i)
        if (bodies[i])
        {
            SoftBody_FinalizeStep(bodies[i]);
            SoftBody_UpdateNodeRotations(bodies[i]);
        }
}

EXPORT void Physics_SetWorkerThreads(int threads)
{
    if (threads > 0) omp_set_num_threads(threads);
}