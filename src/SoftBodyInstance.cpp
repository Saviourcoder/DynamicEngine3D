/*
 DYNAMICENGINE3D
 AI-Assisted 3D Physics Engine
 By: Elitmers
*/

#include "SoftBodyInstance.h"
#include "EngineStats.h"
#include "Error.h"
#include <unordered_map>
#include <algorithm>
#include "ThreadPool.h"
#include <chrono>

extern "C" {

    EngineStats g_Stats = { 0 };
    static auto g_LastFrameTime = std::chrono::high_resolution_clock::now();
    static int g_FrameCount = 0;
    static float g_TimeAccum = 0.0f;

    EXPORT void Engine_GetStats(EngineStats* outStats) {
        if (!outStats) { Error_SetError(ErrorCode::InvalidArgument); return; }
        g_Stats.threadPoolSize = GetGlobalPool().pooledThreads();
        g_Stats.peakThreadsUsedThisFrame = GetGlobalPool().peakThreadsUsedThisFrame();
        g_Stats.parallelDispatchesThisFrame = GetGlobalPool().parallelDispatchesThisFrame();
        g_Stats.serialDispatchesThisFrame = GetGlobalPool().serialDispatchesThisFrame();
        *outStats = g_Stats;
    }

    EXPORT void Engine_ClearFrameStats() {
        g_Stats.integrateTimeMs = 0;
        g_Stats.constraintTimeMs = 0;
        g_Stats.collisionTimeMs = 0;
        g_Stats.staticCollisionTimeMs = 0;
        g_Stats.nodeNodeCollisionTimeMs = 0;
        g_Stats.faceNodeCollisionTimeMs = 0;
        g_Stats.deformTimeMs = 0;
        g_Stats.totalNodes = 0;
        g_Stats.totalBeams = 0;
        g_Stats.totalFaces = 0;

        g_Stats.sapPairCount = 0;
        g_Stats.staticCandidateChecks = 0;
        g_Stats.staticContactsResolved = 0;
        g_Stats.staticGridRebuilt = 0;
        g_Stats.nodeNodeContactsResolved = 0;
        g_Stats.faceNodeCandidatesChecked = 0;
        g_Stats.faceNodeContactsResolved = 0;

        g_Stats.rigidFaceDetectionTimeMs = 0;
        g_Stats.rigidResolutionTimeMs = 0;
        g_Stats.rigidSolveTimeMs = 0;

        GetGlobalPool().resetFrameStats();

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

void SoftBodyInstance::SyncMultipliers()
{
    multipliers.assign(beams.size(), 0.0f);
}

const std::vector<int>& SoftBodyInstance::CrossBodyBeams()
{
    if (crossBodyBeamsDirty)
    {
        crossBodyBeams.clear();
        for (size_t i = 0; i < beams.size(); ++i)
            if (beams[i].isCrossBody) crossBodyBeams.push_back((int)i);
        crossBodyBeamsDirty = false;
    }
    return crossBodyBeams;
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
    restOffsetsCache.resize(total);
    std::vector<int> cursors(nodeCount, 0);
    for (const auto& b : beams)
    {
        if (b.nodeA >= nodeCount || b.nodeB >= nodeCount || b.nodeA < 0 || b.nodeB < 0) continue;
        if (b.isCrossBody || !b.isActive) continue;

        Vector3 initialA = { initialX[b.nodeA], initialY[b.nodeA], initialZ[b.nodeA], 0.0f };
        Vector3 initialB = { initialX[b.nodeB], initialY[b.nodeB], initialZ[b.nodeB], 0.0f };

        int slotA = nbrOffsets[b.nodeA] + cursors[b.nodeA]++;
        nbrData[slotA] = b.nodeB;
        restOffsetsCache[slotA] = Sub(initialB, initialA);

        int slotB = nbrOffsets[b.nodeB] + cursors[b.nodeB]++;
        nbrData[slotB] = b.nodeA;
        restOffsetsCache[slotB] = Sub(initialA, initialB);
    }
}

void SoftBodyInstance::RebuildBeamColoring()
{
    colorOffsets.clear();
    colorBeamIndices.clear();

    int nodeCount = (int)currentX.size();
    if (nodeCount == 0 || beams.empty()) return;

    std::vector<int> beamColor(beams.size(), -1);
    std::vector<std::vector<int>> nodeUsedColors(nodeCount);

    int maxColor = -1;
    for (size_t i = 0; i < beams.size(); ++i)
    {
        const BeamData& b = beams[i];
        if (!b.isActive || b.isCrossBody) continue;
        if (b.nodeA < 0 || b.nodeA >= nodeCount || b.nodeB < 0 || b.nodeB >= nodeCount) continue;

        std::vector<int>& usedA = nodeUsedColors[b.nodeA];
        std::vector<int>& usedB = nodeUsedColors[b.nodeB];

        int color = 0;
        while (std::find(usedA.begin(), usedA.end(), color) != usedA.end() ||
            std::find(usedB.begin(), usedB.end(), color) != usedB.end())
            ++color;

        beamColor[i] = color;
        usedA.push_back(color);
        usedB.push_back(color);
        if (color > maxColor) maxColor = color;
    }

    if (maxColor < 0) return; // nothing colorable (all cross-body / inactive)

    int numColors = maxColor + 1;
    std::vector<int> counts(numColors, 0);
    for (int c : beamColor) if (c >= 0) counts[c]++;

    colorOffsets.resize(numColors + 1, 0);
    int total = 0;
    for (int c = 0; c < numColors; ++c) { colorOffsets[c] = total; total += counts[c]; }
    colorOffsets[numColors] = total;

    colorBeamIndices.resize(total);
    std::vector<int> cursors(numColors, 0);
    for (size_t i = 0; i < beams.size(); ++i)
    {
        int c = beamColor[i];
        if (c < 0) continue;
        colorBeamIndices[colorOffsets[c] + cursors[c]++] = (int)i;
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
    float wA = bA->invMasses[beam.nodeA];
    float wB = bB->invMasses[beam.nodeB];
    float wSum = wA + wB;

    if (wSum <= 0.0f) return;

    float velAx = bA->predictedX[beam.nodeA] - bA->currentX[beam.nodeA];
    float velAy = bA->predictedY[beam.nodeA] - bA->currentY[beam.nodeA];
    float velAz = bA->predictedZ[beam.nodeA] - bA->currentZ[beam.nodeA];
    float velBx = bB->predictedX[beam.nodeB] - bB->currentX[beam.nodeB];
    float velBy = bB->predictedY[beam.nodeB] - bB->currentY[beam.nodeB];
    float velBz = bB->predictedZ[beam.nodeB] - bB->currentZ[beam.nodeB];

    float relVelAlongGrad = (velBx - velAx) * gradX + (velBy - velAy) * gradY + (velBz - velAz) * gradZ;

    // Unconditionally stable XPBD implicit damping (Macklin et al. 2016):
    float gamma = beam.damping > 0.0f ? beam.damping : 0.0f;
    float denom = (1.0f + gamma) * wSum + effectiveCompliance;
    if (denom <= 0.0f) return;

    float deltaLambda = -(C + effectiveCompliance * beam.lagrangeMultiplier + gamma * relVelAlongGrad) / denom;
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
    Vector3 refDir = beam.initialPerpDir; // fallback for degenerate cases only
    {
        Vector3 nodePosCur = { sBody->currentX[beam.slidingNode], sBody->currentY[beam.slidingNode], sBody->currentZ[beam.slidingNode], 0.0f };
        Vector3 edgeACur = { eBody->currentX[beam.edgeNodeA], eBody->currentY[beam.edgeNodeA], eBody->currentZ[beam.edgeNodeA], 0.0f };
        Vector3 edgeBCur = { eBody->currentX[beam.edgeNodeB], eBody->currentY[beam.edgeNodeB], eBody->currentZ[beam.edgeNodeB], 0.0f };

        Vector3 edgeVecCur = Sub(edgeBCur, edgeACur);
        float edgeLenCur = Length(edgeVecCur);
        if (edgeLenCur > PhysicsConstants::MIN_BEAM_LENGTH)
        {
            Vector3 edgeDirCur = Scale(edgeVecCur, 1.0f / edgeLenCur);
            Vector3 toCur = Sub(nodePosCur, edgeACur);
            float tCur = Dot(toCur, edgeDirCur);
            Vector3 closestCur = Add(edgeACur, Scale(edgeDirCur, tCur));
            Vector3 perpCur = Sub(nodePosCur, closestCur);
            float perpLenCur = Length(perpCur);
            if (perpLenCur > PhysicsConstants::MIN_BEAM_LENGTH)
                refDir = Scale(perpCur, 1.0f / perpLenCur);
        }
    }

    if (LengthSq(refDir) > 0.0f && Dot(perpDir, refDir) < 0.0f) {
        perpDir = Neg(perpDir);
        perpDist = -perpDist;
    }

    float absPerp = AbsF(perpDist);
    float targetDist = beam.targetPerpDistance;

    if (beam.minLength > 0.0f || beam.maxLength > 0.0f) {
        targetDist = absPerp;
        if (beam.minLength > 0.0f && absPerp < beam.minLength) targetDist = beam.minLength;
        if (beam.maxLength > 0.0f && absPerp > beam.maxLength) targetDist = beam.maxLength;

        if (perpDist < 0.0f) targetDist = -targetDist;
    }

    float constraint = perpDist - targetDist;
    if (AbsF(constraint) < 1e-5f) return; // Completely within sliding limits, applying no force.

    float totalDamping = MaxF(beam.damping + PhysicsConstants::CROSS_BODY_CONSTRAINT_DAMPING, 0.0f);
    float dampedCompliance = (beam.compliance * dt) / (dt + beam.compliance * totalDamping);

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

    if (wNode > 0.0f)
    {
        float predictedPerpDist = perpDist + wNode * deltaLambda;
        if ((perpDist > 0.0f) != (predictedPerpDist > 0.0f))
        {
            const float epsilon = 1e-4f;
            float safeTarget = perpDist > 0.0f ? epsilon : -epsilon;
            deltaLambda = (safeTarget - perpDist) / wNode;
        }
    }

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

// World-space distance constraint: anchor <-> soft-body node.
// XPBD math mirrors SolveCrossBodyBeam exactly, with the anchor playing the
// role of an immovable body B (wB = 0, pinB = true). Only nodeA is ever moved.
void SolveWorldSpaceConstraint(WorldSpaceConstraint& c, float dt)
{
    SoftBodyInstance* bA = c.bodyA;
    if (!bA) return;

    if (c.nodeA < 0 || c.nodeA >= (int)bA->predictedX.size()) return;

    // Lazy invMasses sizing in case a constraint was added between solves.
    int nodeCount = (int)bA->currentX.size();
    if ((int)bA->invMasses.size() != nodeCount)
    {
        bA->invMasses.resize(nodeCount);
        for (int n = 0; n < nodeCount; ++n)
            bA->invMasses[n] = bA->isPinned[n] ? 0.0f : 1.0f / bA->masses[n];
    }

    float pAx = bA->predictedX[c.nodeA], pAy = bA->predictedY[c.nodeA], pAz = bA->predictedZ[c.nodeA];
    // World anchor is an immovable point B.
    float pBx = c.worldAnchor.x, pBy = c.worldAnchor.y, pBz = c.worldAnchor.z;

    float dx = pBx - pAx, dy = pBy - pAy, dz = pBz - pAz;
    float currentLenSq = dx * dx + dy * dy + dz * dz;

    if (currentLenSq < PhysicsConstants::MIN_BEAM_LENGTH * PhysicsConstants::MIN_BEAM_LENGTH) return;
    float currentLen = std::sqrt(currentLenSq);

    float invLen = 1.0f / currentLen;
    float gradX = dx * invLen, gradY = dy * invLen, gradZ = dz * invLen;
    float targetLen = c.restLength;

    // Same min/max clamp semantics as SolveCrossBodyBeam:
    //   - if both limits are <=0 -> pure rest-length target
    //   - if either limit is set and currentLen is within [min,max] -> no correction
    if (c.minLength > 0.0f && currentLen < c.minLength) targetLen = c.minLength;
    else if (c.maxLength > 0.0f && currentLen > c.maxLength) targetLen = c.maxLength;
    else if (c.minLength > 0.0f || c.maxLength > 0.0f) return;

    float C = currentLen - targetLen;

    float effectiveCompliance = c.compliance / (dt * dt);

    bool pinA = bA->isPinned[c.nodeA];
    float wA = bA->invMasses[c.nodeA];
    float wB = 0.0f;                 // anchor is immovable
    float wSum = wA + wB;

    if (wSum <= 0.0f) return;

    // Velocity damping: anchor velocity is zero, so relVel = -velA.
    float velAx = bA->predictedX[c.nodeA] - bA->currentX[c.nodeA];
    float velAy = bA->predictedY[c.nodeA] - bA->currentY[c.nodeA];
    float velAz = bA->predictedZ[c.nodeA] - bA->currentZ[c.nodeA];
    float relVelAlongGrad = -(velAx * gradX + velAy * gradY + velAz * gradZ);

    // Unconditionally stable XPBD implicit damping (Macklin et al. 2016):
    float gamma = c.damping > 0.0f ? c.damping : 0.0f;
    float denom = (1.0f + gamma) * wSum + effectiveCompliance;
    if (denom <= 0.0f) return;

    float deltaLambda = -(C + effectiveCompliance * c.lagrangeMultiplier + gamma * relVelAlongGrad) / denom;
    c.lagrangeMultiplier += deltaLambda;

    float corrX = gradX * deltaLambda, corrY = gradY * deltaLambda, corrZ = gradZ * deltaLambda;

    // Only A moves. grad points A -> B, so pulling A toward the anchor is -corr * wA.
    if (!pinA)
    {
        bA->predictedX[c.nodeA] -= corrX * wA;
        bA->predictedY[c.nodeA] -= corrY * wA;
        bA->predictedZ[c.nodeA] -= corrZ * wA;
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
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (!outPositions) { Error_SetError(ErrorCode::InvalidArgument); return; }
    int count = (int)body->currentX.size();
    for (int i = 0; i < count; ++i) {
        outPositions[i] = { body->currentX[i], body->currentY[i], body->currentZ[i], 0.0f };
    }
}

EXPORT void SoftBody_GetRotations(SoftBodyInstance* body, Quaternion* outRotations)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (!outRotations) { Error_SetError(ErrorCode::InvalidArgument); return; }
    std::copy(body->currentRot.begin(), body->currentRot.end(), outRotations);
}

EXPORT void SoftBody_ClearNodes(SoftBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->currentX.clear(); body->currentY.clear(); body->currentZ.clear();
    body->previousX.clear(); body->previousY.clear(); body->previousZ.clear();
    body->predictedX.clear(); body->predictedY.clear(); body->predictedZ.clear();
    body->initialX.clear(); body->initialY.clear(); body->initialZ.clear();
    body->currentRot.clear();
    body->masses.clear();
    body->isPinned.clear();
    body->beams.clear();
    body->colorOffsets.clear();
    body->colorBeamIndices.clear();
    body->crossBodyBeams.clear();
    body->crossBodyBeamsDirty = false;
    body->worldConstraints.clear();
}

EXPORT void SoftBody_SetupNodes(SoftBodyInstance* body, Vector3* worldPositions, int count)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (!worldPositions || count <= 0) { Error_SetError(ErrorCode::InvalidArgument); return; }

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
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
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
    body->RebuildBeamColoring();
    body->SyncMultipliers();
}

EXPORT void SoftBody_SetBeamCompliance(SoftBodyInstance* body, int index, float compliance, float damping)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (index < 0 || index >= (int)body->beams.size()) { Error_SetError(ErrorCode::InvalidNodeIndex); return; }

    SoftBody_Wake(body);

    body->beams[index].compliance = compliance;
    body->beams[index].damping = damping;
}

EXPORT void SoftBody_ApplyWorldForceToNode(SoftBodyInstance* body, int nodeIndex, Vector3 worldForce, float dt)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (dt <= 0.0f) { Error_SetError(ErrorCode::InvalidDt); return; }
    if (nodeIndex < 0 || nodeIndex >= (int)body->masses.size()) { Error_SetError(ErrorCode::InvalidNodeIndex); return; }
    if (body->isPinned[nodeIndex]) return; // pinned nodes ignore forces by design, not an error

    float mass = body->masses[nodeIndex];
    if (mass <= 0.0f) return; // zero/negative mass node, treated as immovable, not an error

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

void SoftBodyInstance::ResetLagrangeMultipliers()
{
    // Internal constraints.
    std::fill(multipliers.begin(), multipliers.end(), 0.0f);

    // Cross-body constraints use the multiplier stored in BeamData; only the owner resets it.
    for (int index : CrossBodyBeams())
    {
        BeamData& beam = beams[index];
        if (beam.bodyA == this)
            beam.lagrangeMultiplier = 0.0f;
    }

    // World-space constraints store their multiplier in WorldSpaceConstraint.
    // The owning body is the only solver, so no double-reset risk.
    for (WorldSpaceConstraint& wc : worldConstraints)
        wc.lagrangeMultiplier = 0.0f;
}


EXPORT void SoftBody_CheckAndBreakConstraints(SoftBodyInstance* body, float dt)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (dt <= 0.0f) { Error_SetError(ErrorCode::InvalidDt); return; }

    constexpr float TIME_CONSTANT = 0.15f;   // ~150 ms window for "sustained"
    constexpr float BURST_MULT = 4.0f;   // tolerate short spikes up to 4x strength

    float alpha = 1.0f - std::exp(-dt / TIME_CONSTANT);
    float dtSq = dt * dt;

    for (int index : body->CrossBodyBeams()) {
        BeamData& beam = body->beams[index];
        if (!beam.isActive || beam.isBroken) continue;
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

    // World-space constraints use the same EMA-smoothed break logic.
    for (WorldSpaceConstraint& wc : body->worldConstraints) {
        if (!wc.isActive || wc.isBroken) continue;
        if (wc.strength <= 0.0f || !std::isfinite(wc.strength)) continue;

        float forceMag = AbsF(wc.lagrangeMultiplier) / dtSq;
        wc.forceEMA = (1.0f - alpha) * wc.forceEMA + alpha * forceMag;

        bool sustainedOverload = wc.forceEMA > wc.strength;
        bool catastrophicBurst = forceMag > wc.strength * BURST_MULT;

        if (sustainedOverload || catastrophicBurst) {
            wc.isBroken = 1;
            wc.isActive = 0;
        }
    }
}

EXPORT void SoftBody_SetBeamEdgeData(SoftBodyInstance* body, int beamIndex,
    float targetPerpDistance, float dirX, float dirY, float dirZ)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (beamIndex < 0 || beamIndex >= (int)body->beams.size()) { Error_SetError(ErrorCode::InvalidNodeIndex); return; }
    BeamData& b = body->beams[beamIndex];
    b.targetPerpDistance = targetPerpDistance;
    b.initialPerpDir = { dirX, dirY, dirZ };
}

EXPORT void SoftBody_SolveCrossBodyStep(SoftBodyInstance* body, float dt, int iterations)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (dt <= 0.0f) { Error_SetError(ErrorCode::InvalidDt); return; }
    if (iterations < 1) iterations = 1;

    for (int it = 0; it < iterations; ++it)
    {
        for (int index : body->CrossBodyBeams())
        {
            BeamData& beam = body->beams[index];
            if (!beam.isActive) continue;
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

// ---------------------------------------------------------------------------
// World-space distance constraints (anchor <-> soft-body node).
// Mirrors the sb-sb cross-body API; bodyB is replaced by a fixed world anchor.
// The anchor is treated as immovable (wB = 0). See SolveWorldSpaceConstraint
// above for the XPBD math.
// ---------------------------------------------------------------------------

EXPORT int SoftBody_AddWorldSpaceConstraint(
    SoftBodyInstance* body, int nodeA, Vector3 worldAnchor,
    float compliance, float damping, float restLength,
    float minLength, float maxLength, float strength)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return -1; }
    if (nodeA < 0 || nodeA >= (int)body->currentX.size())
    {
        Error_SetError(ErrorCode::InvalidNodeIndex); return -1;
    }
    if (restLength < 0.0f) restLength = 0.0f;

    SoftBody_Wake(body);

    WorldSpaceConstraint c = {};
    c.bodyA = body;
    c.nodeA = nodeA;
    c.worldAnchor = worldAnchor;
    c.compliance = compliance;
    c.damping = damping;
    c.restLength = restLength;
    c.minLength = minLength;
    c.maxLength = maxLength;
    c.strength = strength;
    c.lagrangeMultiplier = 0.0f;
    c.forceEMA = 0.0f;
    c.isActive = 1;
    c.isBroken = 0;

    body->worldConstraints.push_back(c);
    return (int)body->worldConstraints.size() - 1;
}

EXPORT void SoftBody_RemoveWorldSpaceConstraint(SoftBodyInstance* body, int index)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (index < 0 || index >= (int)body->worldConstraints.size())
    {
        Error_SetError(ErrorCode::InvalidNodeIndex); return;
    }

    SoftBody_Wake(body);
    body->worldConstraints.erase(body->worldConstraints.begin() + index);
}

EXPORT void SoftBody_RemoveAllWorldSpaceConstraints(SoftBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (body->worldConstraints.empty()) return;
    SoftBody_Wake(body);
    body->worldConstraints.clear();
}

EXPORT void SoftBody_SetWorldSpaceAnchor(SoftBodyInstance* body, int index, Vector3 worldAnchor)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (index < 0 || index >= (int)body->worldConstraints.size())
    {
        Error_SetError(ErrorCode::InvalidNodeIndex); return;
    }
    SoftBody_Wake(body);
    body->worldConstraints[index].worldAnchor = worldAnchor;
}

EXPORT void SoftBody_SetWorldSpaceCompliance(SoftBodyInstance* body, int index, float compliance, float damping)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (index < 0 || index >= (int)body->worldConstraints.size())
    {
        Error_SetError(ErrorCode::InvalidNodeIndex); return;
    }
    SoftBody_Wake(body);
    body->worldConstraints[index].compliance = compliance;
    body->worldConstraints[index].damping = damping;
}

EXPORT void SoftBody_SetWorldSpaceRestLength(SoftBodyInstance* body, int index, float restLength)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (index < 0 || index >= (int)body->worldConstraints.size())
    {
        Error_SetError(ErrorCode::InvalidNodeIndex); return;
    }
    if (restLength < 0.0f) restLength = 0.0f;
    SoftBody_Wake(body);
    body->worldConstraints[index].restLength = restLength;
}

EXPORT void SoftBody_SetWorldSpaceLimits(SoftBodyInstance* body, int index, float minLength, float maxLength, float strength)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (index < 0 || index >= (int)body->worldConstraints.size())
    {
        Error_SetError(ErrorCode::InvalidNodeIndex); return;
    }
    SoftBody_Wake(body);
    body->worldConstraints[index].minLength = minLength;
    body->worldConstraints[index].maxLength = maxLength;
    body->worldConstraints[index].strength = strength;
}

EXPORT int SoftBody_GetWorldSpaceConstraintCount(SoftBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return 0; }
    return (int)body->worldConstraints.size();
}

EXPORT int SoftBody_IsWorldSpaceConstraintBroken(SoftBodyInstance* body, int index)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return 0; }
    if (index < 0 || index >= (int)body->worldConstraints.size())
    {
        Error_SetError(ErrorCode::InvalidNodeIndex); return 0;
    }
    return body->worldConstraints[index].isBroken ? 1 : 0;
}

EXPORT void SoftBody_SolveWorldSpaceStep(SoftBodyInstance* body, float dt, int iterations)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (dt <= 0.0f) { Error_SetError(ErrorCode::InvalidDt); return; }
    if (iterations < 1) iterations = 1;
    if (body->worldConstraints.empty()) return;

    for (int it = 0; it < iterations; ++it)
    {
        for (WorldSpaceConstraint& c : body->worldConstraints)
        {
            if (!c.isActive) continue;
            SolveWorldSpaceConstraint(c, dt);
        }
    }
}

EXPORT void SoftBody_UpdateNodeRotations(SoftBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    int nodeCount = (int)body->currentX.size();
    if (nodeCount == 0) return;

    if (body->nbrOffsets.empty()) body->RebuildNeighborCache();

    ::UpdateNodeRotations(body->currentRot.data(),
        body->currentX.data(), body->currentY.data(), body->currentZ.data(),
        body->nbrData.data(), body->nbrOffsets.data(), body->restOffsetsCache.data(), nodeCount);
}

EXPORT void SoftBody_SetNodePinned(SoftBodyInstance* body, int index, int pinned)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (index < 0 || index >= (int)body->isPinned.size()) { Error_SetError(ErrorCode::InvalidNodeIndex); return; }

    body->isPinned[index] = (pinned != 0);

    // Zero out velocity so it instantly freezes in place
    if (pinned != 0) {
        body->previousX[index] = body->currentX[index];
        body->previousY[index] = body->currentY[index];
        body->previousZ[index] = body->currentZ[index];

        // Also clamp predicted positions to prevent substep drift
        body->predictedX[index] = body->currentX[index];
        body->predictedY[index] = body->currentY[index];
        body->predictedZ[index] = body->currentZ[index];
    }
}

EXPORT void SoftBody_SetPreviousPos(SoftBodyInstance* body, int index, Vector3 pos)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (index < 0 || index >= (int)body->previousX.size()) { Error_SetError(ErrorCode::InvalidNodeIndex); return; }
    body->previousX[index] = pos.x; body->previousY[index] = pos.y; body->previousZ[index] = pos.z;
}

EXPORT Vector3 SoftBody_GetPredictedPos(SoftBodyInstance* body, int index)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return { 0,0,0,0 }; }
    if (index < 0 || index >= (int)body->predictedX.size()) { Error_SetError(ErrorCode::InvalidNodeIndex); return { 0,0,0,0 }; }
    return { body->predictedX[index], body->predictedY[index], body->predictedZ[index], 0.0f };
}

EXPORT void SoftBody_SetBeamRestLength(SoftBodyInstance* body, int beamIndex, float length)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (beamIndex < 0 || beamIndex >= (int)body->beams.size()) { Error_SetError(ErrorCode::InvalidNodeIndex); return; }
    body->beams[beamIndex].restLength = length;
}

EXPORT int SoftBody_AddBeam(
    SoftBodyInstance* bodyA, int nodeA,
    SoftBodyInstance* bodyB, int nodeB,
    float compliance, float damping, float restLength,
    float minL, float maxL, float strength, float plasticityThreshold, float plasticityRate, float maxDeformation,
    int isEdgeSliding, int edgeA, int edgeB, int slidingNode)
{
    if (!bodyA) { Error_SetError(ErrorCode::NullHandle); return -1; }

    SoftBody_Wake(bodyA);
    if (bodyB && bodyB != bodyA) SoftBody_Wake(bodyB);

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

    // New beam changes the conflict graph — drop the stale coloring so the solve rebuilds it.
    if (!b.isCrossBody)
    {
        bodyA->colorOffsets.clear();
        bodyA->colorBeamIndices.clear();
    }
    else if (!bodyA->crossBodyBeamsDirty)
    {
        // Appending keeps every existing index valid, so extend the cache in place.
        bodyA->crossBodyBeams.push_back((int)bodyA->beams.size() - 1);
    }

    return (int)bodyA->beams.size() - 1;
}

EXPORT void SoftBody_AddBeamBatch(SoftBodyInstance* body, const NativeBeamInit* beams, int count)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (!beams || count <= 0) { Error_SetError(ErrorCode::InvalidArgument); return; }

    body->beams.reserve(body->beams.size() + (size_t)count);
    body->multipliers.reserve(body->multipliers.size() + (size_t)count);

    for (int i = 0; i < count; ++i)
    {
        const NativeBeamInit& b = beams[i];
        SoftBody_AddBeam(
            body, b.nodeA, body, b.nodeB,
            b.compliance, b.damping, b.restLength,
            b.minLength, b.maxLength, b.strength,
            b.plasticityThreshold, b.plasticityRate, b.maxDeformation,
            b.isEdgeSliding, b.edgeNodeA, b.edgeNodeB, b.slidingNode
        );
    }
}

EXPORT void SoftBody_UpdateBeamBulk(SoftBodyInstance* body, float compliance, float damping)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
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
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (index < 0 || index >= (int)body->predictedX.size()) { Error_SetError(ErrorCode::InvalidNodeIndex); return; }
    body->predictedX[index] = pos.x; body->predictedY[index] = pos.y; body->predictedZ[index] = pos.z;
}

EXPORT Vector3 SoftBody_GetCurrentPos(SoftBodyInstance* body, int index)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return { 0.0f, 0.0f, 0.0f, 0.0f }; }
    if (index < 0 || index >= (int)body->currentX.size()) { Error_SetError(ErrorCode::InvalidNodeIndex); return { 0.0f, 0.0f, 0.0f, 0.0f }; }
    return { body->currentX[index], body->currentY[index], body->currentZ[index], 0.0f };
}

EXPORT void SoftBody_SetCurrentPos(SoftBodyInstance* body, int index, Vector3 pos)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (index < 0 || index >= (int)body->currentX.size()) { Error_SetError(ErrorCode::InvalidNodeIndex); return; }
    body->currentX[index] = pos.x; body->currentY[index] = pos.y; body->currentZ[index] = pos.z;
}

EXPORT void SoftBody_SetNodeMass(SoftBodyInstance* body, int index, float mass)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (index < 0 || index >= (int)body->masses.size()) { Error_SetError(ErrorCode::InvalidNodeIndex); return; }
    body->masses[index] = mass;
}

static inline Vector3 PredictedPos(const SoftBodyInstance* body, int i)
{
    return V3(body->predictedX[i], body->predictedY[i], body->predictedZ[i]);
}

// The part of (pos - axisCenter) perpendicular to the axis: the node's lever arm.
static inline Vector3 MotorRadialArm(const Vector3& pos, const Vector3& axisCenter, const Vector3& axisDirection)
{
    const Vector3 diff = Sub(pos, axisCenter);
    return Sub(diff, Scale(axisDirection, Dot(diff, axisDirection)));
}

// Unit tangent of the node's rotation about the axis; false if it sits on the axis.
static inline bool MotorTangent(const Vector3& axisDirection, const Vector3& radial, Vector3& outTangent)
{
    const Vector3 tangent = Cross(axisDirection, radial);
    const float len = Length(tangent);
    if (len < 1e-5f) return false;

    outTangent = Scale(tangent, 1.0f / len);
    return true;
}

EXPORT float SoftBody_EvaluateMotor(
    SoftBodyInstance* body,
    Vector3 axisCenter,
    Vector3 axisDirection,
    float targetRate,
    float maxTorque,
    float propGain,
    float dt,
    int applyTorque)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return 0.0f; }
    if (body->predictedX.empty()) { Error_SetError(ErrorCode::InvalidArgument); return 0.0f; }
    if (dt <= 0.0f) { Error_SetError(ErrorCode::InvalidDt); return 0.0f; }

    int count = (int)body->predictedX.size();

    float totalOmega = 0.0f;
    int omegaCount = 0;

    for (int i = 0; i < count; i++)
    {
        if (body->isPinned[i]) continue;

        const Vector3 pos = PredictedPos(body, i);
        const Vector3 radial = MotorRadialArm(pos, axisCenter, axisDirection);

        const float dist = Length(radial);
        if (dist < 0.001f) continue;

        Vector3 tangent;
        if (!MotorTangent(axisDirection, radial, tangent)) continue;

        const Vector3 prev = V3(body->currentX[i], body->currentY[i], body->currentZ[i]);
        const float tangentialSpeed = Dot(Scale(Sub(pos, prev), 1.0f / dt), tangent);

        totalOmega += tangentialSpeed / dist;
        omegaCount++;
    }

    float currentAngularVelocity =
        (omegaCount > 0) ? (totalOmega / omegaCount) : 0.0f;

    // Spin rate is always worth reporting; torque is not — an off motor should coast.
    if (!applyTorque)
        return currentAngularVelocity;

    float inertia = 0.0f;
    float totalUnpinnedMass = 0.0f;

    for (int i = 0; i < count; i++)
    {
        if (body->isPinned[i])
            continue;

        const float r = Length(MotorRadialArm(PredictedPos(body, i), axisCenter, axisDirection));
        inertia += body->masses[i] * r * r;

        if (body->masses[i] > 0.0f)
            totalUnpinnedMass += body->masses[i];
    }

    float angularAccel = maxTorque / std::max(inertia, 1e-6f);

    float maxDeltaOmega = angularAccel * dt;

    static thread_local std::vector<Vector3> deltaVs;
    deltaVs.resize(count);
    std::fill(deltaVs.begin(), deltaVs.end(), V3Zero());

    Vector3 netDeltaMomentum = V3Zero();

    int applied = 0;

    float omegaError = targetRate - currentAngularVelocity;

    float desiredDeltaOmega = std::clamp(
        omegaError * propGain,
        -maxDeltaOmega,
        maxDeltaOmega);

    if (omegaError >= 0.0f)
        desiredDeltaOmega = std::min(desiredDeltaOmega, omegaError);
    else
        desiredDeltaOmega = std::max(desiredDeltaOmega, omegaError);

    for (int i = 0; i < count; i++)
    {
        if (body->isPinned[i] || body->masses[i] <= 0.0f)
            continue;

        const Vector3 radial = MotorRadialArm(PredictedPos(body, i), axisCenter, axisDirection);

        const float dist = Length(radial);
        if (dist < 0.001f)
            continue;

        Vector3 tangent;
        if (!MotorTangent(axisDirection, radial, tangent)) continue;

        deltaVs[i] = Scale(tangent, desiredDeltaOmega * dist);

        netDeltaMomentum =
            Add(netDeltaMomentum,
                Scale(deltaVs[i], body->masses[i]));

        applied++;
    }

    if (applied > 0 && totalUnpinnedMass > 0.0f)
    {
        Vector3 biasVelocity =
            Scale(netDeltaMomentum, 1.0f / totalUnpinnedMass);

        for (int i = 0; i < count; i++)
        {
            if (body->isPinned[i] || body->masses[i] <= 0.0f)
                continue;

            Vector3 adjust = Sub(deltaVs[i], biasVelocity);

            body->predictedX[i] += adjust.x * dt;
            body->predictedY[i] += adjust.y * dt;
            body->predictedZ[i] += adjust.z * dt;
        }
    }

    return currentAngularVelocity;
}

EXPORT void SoftBody_SetInternalPressure(SoftBodyInstance* body, float pressure)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }

    if (body->internalPressure != pressure)
        SoftBody_Wake(body);

    body->internalPressure = pressure;
}

EXPORT void SoftBody_SetFaces(SoftBodyInstance* body, const int* triangles, int triangleCount)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->faces.clear();
    if (triangleCount <= 0) return; // clearing faces is valid usage, not an error
    if (!triangles) { Error_SetError(ErrorCode::InvalidArgument); return; }
    body->faces.resize(triangleCount);
    for (int i = 0; i < triangleCount; ++i)
    {
        body->faces[i].nodeA = triangles[i * 3 + 0];
        body->faces[i].nodeB = triangles[i * 3 + 1];
        body->faces[i].nodeC = triangles[i * 3 + 2];
    }
}

EXPORT int SoftBody_GetFaceCount(SoftBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return 0; }
    return (int)body->faces.size();
}

// Motor

EXPORT void SoftBody_ConfigureMotor(SoftBodyInstance* body, SoftBodyInstance* baseBody,
    const int* axisIndices, int axisCount, float propGain)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }

    MotorState& m = body->motor;
    m.axisIndices.clear();

    const int nodeCount = (int)body->predictedX.size();
    if (axisIndices && axisCount > 0)
    {
        m.axisIndices.reserve((size_t)axisCount);
        for (int i = 0; i < axisCount; ++i)
        {
            const int idx = axisIndices[i];
            if (idx >= 0 && idx < nodeCount) m.axisIndices.push_back(idx);
            else Error_SetError(ErrorCode::InvalidNodeIndex);
        }
    }

    m.baseBody = baseBody;
    m.propGain = propGain;
    // Two nodes are the minimum needed to define a direction.
    m.configured = m.axisIndices.size() >= 2;
    m.axisDirectionInitialized = false;
    m.currentAngularVelocity = 0.0f;
}

EXPORT void SoftBody_ClearMotor(SoftBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->motor = MotorState{};
}

EXPORT void SoftBody_SetMotorDrive(SoftBodyInstance* body, float targetRate, float maxTorque, int enabled)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->motor.targetRate = targetRate;
    body->motor.maxTorque = maxTorque;
    body->motor.enabled = (enabled != 0);
}

EXPORT float SoftBody_GetMotorAngularVelocity(SoftBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return 0.0f; }
    return body->motor.currentAngularVelocity;
}

static Vector3 ComputeMotorAxisCenter(const SoftBodyInstance* body)
{
    const MotorState& m = body->motor;
    if (m.axisIndices.empty()) return V3Zero();

    Vector3 center = V3Zero();
    for (int idx : m.axisIndices)
        center = Add(center, V3(body->predictedX[idx], body->predictedY[idx], body->predictedZ[idx]));

    return Scale(center, 1.0f / (float)m.axisIndices.size());
}

// Rederives the axis direction and latches its sign, so the spin rate can't invert mid-drive.
static Vector3 UpdateMotorAxisDirection(SoftBodyInstance* body)
{
    MotorState& m = body->motor;
    if (m.axisIndices.size() < 2)
        return m.axisDirectionInitialized ? m.cachedAxisDirection : V3(0.0f, 1.0f, 0.0f);

    const int a = m.axisIndices[0];
    const int b = m.axisIndices[1];

    const Vector3 delta = Sub(
        V3(body->predictedX[b], body->predictedY[b], body->predictedZ[b]),
        V3(body->predictedX[a], body->predictedY[a], body->predictedZ[a]));

    const float len = Length(delta);
    if (len < 1e-5f)
        return m.axisDirectionInitialized ? m.cachedAxisDirection : V3(0.0f, 1.0f, 0.0f);

    Vector3 axis = Scale(delta, 1.0f / len);

    if (m.axisDirectionInitialized && Dot(axis, m.cachedAxisDirection) < 0.0f)
        axis = Neg(axis);

    m.axisDirectionInitialized = true;
    m.cachedAxisDirection = axis;
    return axis;
}

EXPORT Vector3 SoftBody_GetMotorAxisDirection(SoftBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return V3(0.0f, 1.0f, 0.0f); }
    if (body->predictedX.empty()) { Error_SetError(ErrorCode::InvalidArgument); return V3(0.0f, 1.0f, 0.0f); }
    return UpdateMotorAxisDirection(body);
}

EXPORT Vector3 SoftBody_GetMotorAxisCenter(SoftBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return V3Zero(); }
    if (body->predictedX.empty()) { Error_SetError(ErrorCode::InvalidArgument); return V3Zero(); }
    return ComputeMotorAxisCenter(body);
}

void SoftBody_StepMotor(SoftBodyInstance* body, float dt)
{
    if (!body || !body->motor.configured || body->predictedX.empty()) return;

    MotorState& m = body->motor;

    const Vector3 axisCenter = ComputeMotorAxisCenter(body);
    const Vector3 axisDirection = UpdateMotorAxisDirection(body);

    SoftBody_Wake(body);
    if (m.baseBody) SoftBody_Wake(m.baseBody);

    m.currentAngularVelocity = SoftBody_EvaluateMotor(
        body, axisCenter, axisDirection,
        m.targetRate, m.maxTorque, m.propGain, dt,
        m.enabled ? 1 : 0);
}

// Render pose

static Vector3 ComputeCenter(const SoftBodyInstance* body)
{
    const int count = (int)body->currentX.size();
    if (count == 0) return V3Zero();

    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    for (int i = 0; i < count; ++i)
    {
        sumX += body->currentX[i];
        sumY += body->currentY[i];
        sumZ += body->currentZ[i];
    }

    const float invN = 1.0f / (float)count;
    return V3(sumX * invN, sumY * invN, sumZ * invN);
}

static bool TryComputeReferenceRotation(const SoftBodyInstance* body, Quaternion& outRotation)
{
    const int count = (int)body->currentX.size();
    if (count < 4 || !body->renderBasisValid) return false;

    auto nodeAt = [&](int i) { return V3(body->currentX[i], body->currentY[i], body->currentZ[i]); };

    const Vector3 forward = Sub(nodeAt(body->refNodeForward), nodeAt(body->refNodeBack));
    const Vector3 rightHint = Sub(nodeAt(body->refNodeRight), nodeAt(body->refNodeLeft));
    const Vector3 up = Cross(forward, rightHint);

    if (LengthSq(forward) < 1e-8f || LengthSq(rightHint) < 1e-8f || LengthSq(up) < 1e-8f)
        return false;

    outRotation = QLookRotation(Normalize(forward), Normalize(up));
    return IsFinite(outRotation);
}

static Quaternion ComputeBasisRotation(SoftBodyInstance* body)
{
    Quaternion reference;
    if (!TryComputeReferenceRotation(body, reference))
        return body->lastValidRotation;

    return QMul(reference, body->referenceRotationOffset);
}

EXPORT void SoftBody_SetRenderBasis(SoftBodyInstance* body, int forwardNode, int backNode,
    int leftNode, int rightNode, Quaternion rotationOffset, Vector3 centerToPivotLocal)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }

    body->refNodeForward = forwardNode;
    body->refNodeBack = backNode;
    body->refNodeLeft = leftNode;
    body->refNodeRight = rightNode;
    body->referenceRotationOffset = rotationOffset;
    body->centerToPivotLocal = centerToPivotLocal;
    body->renderBasisValid = true;

    const Vector3 center = ComputeCenter(body);
    const Quaternion rotation = ComputeBasisRotation(body);

    body->prevCenter = body->currCenter = center;
    body->prevRotation = body->currRotation = rotation;
    body->lastValidRotation = rotation;
}

EXPORT void SoftBody_CaptureRenderPose(SoftBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (body->currentX.empty()) { Error_SetError(ErrorCode::InvalidArgument); return; }

    body->prevCenter = body->currCenter;
    body->prevRotation = body->currRotation;

    body->currCenter = ComputeCenter(body);
    body->currRotation = ComputeBasisRotation(body);

    if (IsFinite(body->currRotation)) body->lastValidRotation = body->currRotation;
}

EXPORT void SoftBody_GetRenderPose(SoftBodyInstance* body, float alpha,
    Vector3* outPosition, Quaternion* outRotation)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }

    alpha = ClampF(alpha, 0.0f, 1.0f);

    const Quaternion rotation = QSlerp(body->prevRotation, body->currRotation, alpha);
    const Vector3 center = Lerp(body->prevCenter, body->currCenter, alpha);
    const Vector3 position = Add(center, QRotate(rotation, body->centerToPivotLocal));

    if (outRotation) *outRotation = rotation;
    if (outPosition) *outPosition = position;
}

EXPORT Vector3 SoftBody_GetCenter(SoftBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return V3Zero(); }
    if (body->currentX.empty()) { Error_SetError(ErrorCode::InvalidArgument); return V3Zero(); }
    return ComputeCenter(body);
}

EXPORT Quaternion SoftBody_GetBasisRotation(SoftBodyInstance* body)
{
    return (body && !body->currentX.empty()) ? ComputeBasisRotation(body) : QIdentity();
}

EXPORT Vector3 SoftBody_GetInterpolatedNodePosition(SoftBodyInstance* body, int index, float alpha)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return V3Zero(); }
    if (index < 0 || index >= (int)body->currentX.size()) { Error_SetError(ErrorCode::InvalidNodeIndex); return V3Zero(); }

    alpha = ClampF(alpha, 0.0f, 1.0f);
    return Lerp(
        V3(body->previousX[index], body->previousY[index], body->previousZ[index]),
        V3(body->currentX[index], body->currentY[index], body->currentZ[index]),
        alpha);
}

// Small accessors that keep the managed side from having to mirror state.

EXPORT int SoftBody_GetNodeCount(SoftBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return 0; }
    return (int)body->currentX.size();
}

EXPORT int SoftBody_IsNodePinned(SoftBodyInstance* body, int index)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return 0; }
    if (index < 0 || index >= (int)body->isPinned.size()) { Error_SetError(ErrorCode::InvalidNodeIndex); return 0; }
    return body->isPinned[index] ? 1 : 0;
}

EXPORT int SoftBody_GetBeamCount(SoftBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return 0; }
    return (int)body->beams.size();
}

EXPORT int SoftBody_IsBeamBroken(SoftBodyInstance* body, int beamIndex)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return 0; }
    if (beamIndex < 0 || beamIndex >= (int)body->beams.size()) { Error_SetError(ErrorCode::InvalidNodeIndex); return 0; }
    return body->beams[beamIndex].isBroken ? 1 : 0;
}

EXPORT void SoftBody_SetBeamLimits(SoftBodyInstance* body, int beamIndex,
    float minLength, float maxLength, float strength)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    if (beamIndex < 0 || beamIndex >= (int)body->beams.size()) { Error_SetError(ErrorCode::InvalidNodeIndex); return; }

    BeamData& beam = body->beams[beamIndex];
    beam.minLength = minLength;
    beam.maxLength = maxLength;
    beam.strength = strength;
}

EXPORT void SoftBody_FinalizeStep(SoftBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    FinalizePositions(body->currentX.data(), body->currentY.data(), body->currentZ.data(),
        body->previousX.data(), body->previousY.data(), body->previousZ.data(),
        body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(),
        body->isPinned.data(), (int)body->currentX.size());
}

EXPORT void* SoftBody_GetPredictedX(SoftBodyInstance* b) { if (!b) { Error_SetError(ErrorCode::NullHandle); return nullptr; } return b->predictedX.data(); }
EXPORT void* SoftBody_GetPredictedY(SoftBodyInstance* b) { if (!b) { Error_SetError(ErrorCode::NullHandle); return nullptr; } return b->predictedY.data(); }
EXPORT void* SoftBody_GetPredictedZ(SoftBodyInstance* b) { if (!b) { Error_SetError(ErrorCode::NullHandle); return nullptr; } return b->predictedZ.data(); }

EXPORT void* SoftBody_GetCurrentX(SoftBodyInstance* b) { if (!b) { Error_SetError(ErrorCode::NullHandle); return nullptr; } return b->currentX.data(); }
EXPORT void* SoftBody_GetCurrentY(SoftBodyInstance* b) { if (!b) { Error_SetError(ErrorCode::NullHandle); return nullptr; } return b->currentY.data(); }
EXPORT void* SoftBody_GetCurrentZ(SoftBodyInstance* b) { if (!b) { Error_SetError(ErrorCode::NullHandle); return nullptr; } return b->currentZ.data(); }

EXPORT void* SoftBody_GetPreviousX(SoftBodyInstance* b) { if (!b) { Error_SetError(ErrorCode::NullHandle); return nullptr; } return b->previousX.data(); }
EXPORT void* SoftBody_GetPreviousY(SoftBodyInstance* b) { if (!b) { Error_SetError(ErrorCode::NullHandle); return nullptr; } return b->previousY.data(); }
EXPORT void* SoftBody_GetPreviousZ(SoftBodyInstance* b) { if (!b) { Error_SetError(ErrorCode::NullHandle); return nullptr; } return b->previousZ.data(); }

EXPORT void* SoftBody_GetMasses(SoftBodyInstance* b) { if (!b) { Error_SetError(ErrorCode::NullHandle); return nullptr; } return b->masses.data(); }
EXPORT void* SoftBody_GetPinned(SoftBodyInstance* b) { if (!b) { Error_SetError(ErrorCode::NullHandle); return nullptr; } return b->isPinned.data(); }

EXPORT void SoftBody_RemoveCrossBodyBeams(SoftBodyInstance* body, SoftBodyInstance* target)
{
    if (!body || !target) { Error_SetError(ErrorCode::NullHandle); return; }
    auto& beams = body->beams;
    beams.erase(std::remove_if(beams.begin(), beams.end(),
        [target](const BeamData& b) {
            return b.isCrossBody && (b.bodyA == target || b.bodyB == target);
        }), beams.end());

    // Erasing shifts every later index, so the cache has to be rebuilt.
    body->crossBodyBeamsDirty = true;
}

EXPORT void SoftBody_IntegrateBatch(SoftBodyInstance** bodies, int count, float dt, float gravity, int isFirstSubstep)
{
    if (!bodies || count <= 0) { Error_SetError(ErrorCode::InvalidArgument); return; }
    if (dt <= 0.0f) { Error_SetError(ErrorCode::InvalidDt); return; }

    SoftBody_UpdateSleepIslands(bodies, count, dt);

    auto t0 = std::chrono::high_resolution_clock::now();

    GetGlobalPool().parallel_for(count, [&](int i)
        {
            SoftBodyInstance* body = bodies[i];
            if (!body || body->currentX.empty()) return;

            int nodeCount = (int)body->currentX.size();
            int beamCount = (int)body->beams.size();

            const float* masses = body->masses.data();
            const uint8_t* pinned = body->isPinned.data();

            if ((int)body->invMasses.size() != nodeCount)
                body->invMasses.resize(nodeCount);
            for (int n = 0; n < nodeCount; ++n)
                body->invMasses[n] = pinned[n] ? 0.0f : 1.0f / masses[n];

            if (isFirstSubstep)
            {
                AtomicAddInt(&g_Stats.totalNodes, nodeCount);
                AtomicAddInt(&g_Stats.totalBeams, beamCount);
            }

            Integrate(body->currentX.data(), body->currentY.data(), body->currentZ.data(),
                body->previousX.data(), body->previousY.data(), body->previousZ.data(),
                body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(),
                pinned, nodeCount, dt, body->defaultDamping, gravity);

            if (body->internalPressure > 0.0f)
                ApplyPressure(body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(),
                    body->currentX.data(), body->currentY.data(), body->currentZ.data(),
                    masses, pinned, nodeCount, body->internalPressure, dt);

            if (body->motor.configured && body->motor.enabled)
                SoftBody_StepMotor(body, dt);

            // Reset Lagrange multipliers once per substep, not per iteration/round.
            body->ResetLagrangeMultipliers();
        }, 4);

    auto t1 = std::chrono::high_resolution_clock::now();
    float integrateMs = std::chrono::duration<float, std::milli>(t1 - t0).count();
    g_Stats.integrateTimeMs += integrateMs;
}

EXPORT void SoftBody_SolveConstraintsBatch(SoftBodyInstance** bodies, int count, float dt, int iterations)
{
    if (!bodies || count <= 0) { Error_SetError(ErrorCode::InvalidArgument); return; }
    if (dt <= 0.0f) { Error_SetError(ErrorCode::InvalidDt); return; }
    if (iterations <= 0) { Error_SetError(ErrorCode::InvalidSubstep); return; }

    auto t0 = std::chrono::high_resolution_clock::now();
    SolveInterleavedConstraints(bodies, count, dt, iterations);
    auto t1 = std::chrono::high_resolution_clock::now();
    float constraintMs = std::chrono::duration<float, std::milli>(t1 - t0).count();
    g_Stats.constraintTimeMs += constraintMs;
}

EXPORT void SoftBody_ApplyPlasticityBatch(SoftBodyInstance** bodies, int count, float dt)
{
    if (!bodies || count <= 0) { Error_SetError(ErrorCode::InvalidArgument); return; }

    GetGlobalPool().parallel_for(count, [&](int i)
        {
            SoftBodyInstance* body = bodies[i];
            if (!body || body->beams.empty() || body->isSleeping) return;

            ApplyPlasticity(body->predictedX.data(), body->predictedY.data(), body->predictedZ.data(),
                body->beams.data(), (int)body->beams.size(), dt);
        }, 4);
}

EXPORT void SoftBody_FinalizeBatch(SoftBodyInstance** bodies, int count, float dt)
{
    if (!bodies || count <= 0) { Error_SetError(ErrorCode::InvalidArgument); return; }

    {
        GetGlobalPool().parallel_for(count, [&](int i) {
            if (bodies[i])
                SoftBody_CheckAndBreakConstraints(bodies[i], dt);
            });
    }

    {
        GetGlobalPool().parallel_for(count, [&](int i) {
            if (!bodies[i] || bodies[i]->isSleeping)
                return;

            SoftBody_FinalizeStep(bodies[i]);
            SoftBody_UpdateNodeRotations(bodies[i]);
            });
    }
}

EXPORT Vector3 SoftBody_GetLinearVelocity(SoftBodyInstance* body, float dt)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return { 0.0f, 0.0f, 0.0f, 0.0f }; }
    if (body->currentX.empty()) { Error_SetError(ErrorCode::InvalidArgument); return { 0.0f, 0.0f, 0.0f, 0.0f }; }
    if (dt <= 0.0f) { Error_SetError(ErrorCode::InvalidDt); return { 0.0f, 0.0f, 0.0f, 0.0f }; }

    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    const int count = (int)body->currentX.size();

    for (int i = 0; i < count; ++i)
    {
        sumX += (body->currentX[i] - body->previousX[i]);
        sumY += (body->currentY[i] - body->previousY[i]);
        sumZ += (body->currentZ[i] - body->previousZ[i]);
    }

    const float inv = 1.0f / (count * dt);

    return {
        sumX * inv,
        sumY * inv,
        sumZ * inv,
        0.0f
    };
}

EXPORT void SoftBody_Wake(SoftBodyInstance* body) {
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->isSleeping = false; body->sleepTimer = 0.0f;
}
EXPORT void SoftBody_Sleep(SoftBodyInstance* body) {
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->isSleeping = true;
}
EXPORT int SoftBody_IsSleeping(SoftBodyInstance* body) {
    if (!body) { Error_SetError(ErrorCode::NullHandle); return 0; }
    return body->isSleeping ? 1 : 0;
}
EXPORT void SoftBody_SetSleepThreshold(SoftBodyInstance* body, float threshold) {
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->sleepThreshold = threshold;
}
EXPORT void SoftBody_SetSleepTime(SoftBodyInstance* body, float time) {
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->sleepTime = time;
}
EXPORT void SoftBody_SetCanSleep(SoftBodyInstance* body, int canSleep) {
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }
    body->canSleep = (canSleep != 0);
}

EXPORT void SoftBody_UpdateSleepIslands(SoftBodyInstance** bodies, int count, float dt)
{
    if (!bodies || count <= 0) { Error_SetError(ErrorCode::InvalidArgument); return; }
    if (dt <= 0.0f) { Error_SetError(ErrorCode::InvalidDt); return; }
    const float invDt = 1.0f / dt;

    // Map body pointer to index, then build the cross-body adjacency graph.
    static thread_local std::unordered_map<SoftBodyInstance*, int> bodyToIndex;
    bodyToIndex.clear();
    bodyToIndex.reserve(count);
    for (int i = 0; i < count; ++i) {
        if (bodies[i]) bodyToIndex[bodies[i]] = i;
    }

    static thread_local std::vector<std::vector<int>> adj;
    adj.resize(count);
    for (std::vector<int>& neighbors : adj)
        neighbors.clear();
    for (int i = 0; i < count; ++i) {
        SoftBodyInstance* body = bodies[i];
        if (!body) continue;

        for (int index : body->CrossBodyBeams()) {
            const BeamData& beam = body->beams[index];
            if (!beam.isActive || beam.isBroken) continue;
            if (beam.bodyA && beam.bodyB && beam.bodyA != beam.bodyB) {
                auto itA = bodyToIndex.find(beam.bodyA);
                auto itB = bodyToIndex.find(beam.bodyB);
                if (itA != bodyToIndex.end() && itB != bodyToIndex.end()) {
                    adj[itA->second].push_back(itB->second);
                    adj[itB->second].push_back(itA->second);
                }
            }
        }
    }

    // Group connected components into islands (BFS).
    static thread_local std::vector<uint8_t> visited;
    static thread_local std::vector<int> island;
    static thread_local std::vector<int> queue;
    visited.assign(count, 0);

    for (int i = 0; i < count; ++i) {
        if (visited[i] || !bodies[i]) continue;

        island.clear();
        queue.clear();
        queue.push_back(i);
        visited[i] = true;

        int head = 0;
        while (head < (int)queue.size()) {
            int curr = queue[head++];
            island.push_back(curr);
            for (int nbr : adj[curr]) {
                if (!visited[nbr]) {
                    visited[nbr] = true;
                    queue.push_back(nbr);
                }
            }
        }

        // If any body in the island moves or can't sleep, the whole island stays awake.
        bool islandMustWake = false;

        for (int idx : island) {
            SoftBodyInstance* body = bodies[idx];
            if (!body->canSleep) {
                islandMustWake = true;
                break;
            }

            int nodeCount = (int)body->currentX.size();
            float totalSpeed = 0.0f;
            int activeNodes = 0;

            for (int n = 0; n < nodeCount; ++n) {
                if (body->isPinned[n]) continue;
                float vx = (body->currentX[n] - body->previousX[n]) * invDt;
                float vy = (body->currentY[n] - body->previousY[n]) * invDt;
                float vz = (body->currentZ[n] - body->previousZ[n]) * invDt;
                totalSpeed += std::sqrt(vx * vx + vy * vy + vz * vz);
                ++activeNodes;
            }

            float avgSpeed = (activeNodes > 0) ? (totalSpeed / activeNodes) : 0.0f;
            if (avgSpeed >= body->sleepThreshold) {
                islandMustWake = true;
                break;
            }
        }

        // Apply one uniform sleep/wake state across the island.
        for (int idx : island) {
            SoftBodyInstance* body = bodies[idx];
            if (islandMustWake) {
                body->isSleeping = false;
                body->sleepTimer = 0.0f;
            }
            else {
                body->sleepTimer += dt;
                if (body->sleepTimer >= body->sleepTime) {
                    body->isSleeping = true;
                }
            }
        }
    }
}

EXPORT void Physics_SetWorkerThreads(int threads)
{
    if (threads <= 0)
    {
        Error_SetError(ErrorCode::InvalidArgument);
        return;
    }

    GetGlobalPool().resize(threads);
}