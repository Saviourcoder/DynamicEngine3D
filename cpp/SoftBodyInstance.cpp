/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */
#include "SoftBodyInstance.h"
#include <unordered_map>

extern "C" void ApplyPlasticity(const Vector3* predictedPos, BeamData* beams, int beamCount, float dt);

void SoftBodyInstance::SyncMultipliers()
{
    multipliers.assign(beams.size(), 0.0f);
}

void SoftBodyInstance::RebuildNeighborCache()
{
    int nodeCount = (int)currentPos.size();
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
    SoftBodyInstance* bA = reinterpret_cast<SoftBodyInstance*>(beam.bodyA);
    SoftBodyInstance* bB = reinterpret_cast<SoftBodyInstance*>(beam.bodyB);
    if (!bA || !bB) return;

    if (beam.nodeA < 0 || beam.nodeA >= (int)bA->predictedPos.size() ||
        beam.nodeB < 0 || beam.nodeB >= (int)bB->predictedPos.size())
        return;

    Vector3 pA = bA->predictedPos[beam.nodeA];
    Vector3 pB = bB->predictedPos[beam.nodeB];
    Vector3 delta = Sub(pB, pA);
    float currentLen = Length(delta);

    if (currentLen < PhysicsConstants::MIN_BEAM_LENGTH) return;

    Vector3 grad = Scale(delta, 1.0f / currentLen);
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

    // Calculate deltaLambda without the broken dimensional clamp
    float deltaLambda = -(C + effectiveCompliance * beam.lagrangeMultiplier) / (wSum + effectiveCompliance);

    beam.lagrangeMultiplier += deltaLambda;
    Vector3 correction = Scale(grad, deltaLambda);

    if (!pinA) {
        bA->predictedPos[beam.nodeA] = Sub(bA->predictedPos[beam.nodeA], Scale(correction, wA));
    }
    if (!pinB) {
        bB->predictedPos[beam.nodeB] = Add(bB->predictedPos[beam.nodeB], Scale(correction, wB));
    }
}

void SolveEdgeSliding(BeamData& beam, float dt)
{
    SoftBodyInstance* sBody = reinterpret_cast<SoftBodyInstance*>(beam.bodyA);
    SoftBodyInstance* eBody = reinterpret_cast<SoftBodyInstance*>(beam.bodyB);
    if (!sBody || !eBody) return;

    if (beam.slidingNode < 0 || beam.slidingNode >= (int)sBody->predictedPos.size() ||
        beam.edgeNodeA < 0 || beam.edgeNodeA >= (int)eBody->predictedPos.size() ||
        beam.edgeNodeB < 0 || beam.edgeNodeB >= (int)eBody->predictedPos.size())
        return;

    Vector3 nodePos = sBody->predictedPos[beam.slidingNode];
    Vector3 edgePosA = eBody->predictedPos[beam.edgeNodeA];
    Vector3 edgePosB = eBody->predictedPos[beam.edgeNodeB];

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

    // Calculate deltaLambda without the broken dimensional clamp
    float deltaLambda = -(constraint + effectiveCompliance * beam.lagrangeMultiplier) / (wSum + effectiveCompliance);

    beam.lagrangeMultiplier += deltaLambda;

    if (!pinNode) sBody->predictedPos[beam.slidingNode] = Add(sBody->predictedPos[beam.slidingNode], Scale(perpDir, (1.0f / massNode) * deltaLambda));
    if (!pinEdgeA) eBody->predictedPos[beam.edgeNodeA] = Add(eBody->predictedPos[beam.edgeNodeA], Scale(perpDir, -(1.0f / massEdgeA) * deltaLambda * weightA));
    if (!pinEdgeB) eBody->predictedPos[beam.edgeNodeB] = Add(eBody->predictedPos[beam.edgeNodeB], Scale(perpDir, -(1.0f / massEdgeB) * deltaLambda * weightB));
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
    std::copy(body->currentPos.begin(), body->currentPos.end(), outPositions);
}

EXPORT void SoftBody_GetRotations(SoftBodyInstance* body, Quaternion* outRotations)
{
    if (!body || !outRotations) return;
    std::copy(body->currentRot.begin(), body->currentRot.end(), outRotations);
}

EXPORT void SoftBody_ClearNodes(SoftBodyInstance* body)
{
    if (!body) return;
    body->currentPos.clear();
    body->previousPos.clear();
    body->predictedPos.clear();
    body->initialPos.clear();
    body->currentRot.clear();
    body->masses.clear();
    body->isPinned.clear();
    body->beams.clear();
}

EXPORT void SoftBody_SetupNodes(SoftBodyInstance* body, Vector3* worldPositions, int count)
{
    if (!body || !worldPositions || count <= 0) return;
    body->currentPos.assign(worldPositions, worldPositions + count);
    body->previousPos.assign(worldPositions, worldPositions + count);
    body->predictedPos.assign(worldPositions, worldPositions + count);
    body->initialPos.assign(worldPositions, worldPositions + count);
    body->currentRot.resize(count, QIdentity());
    body->masses.resize(count, PhysicsConstants::DEFAULT_NODE_MASS);
    body->isPinned.resize(count, false);
}

EXPORT void SoftBody_GenerateBeamsFromDistance(SoftBodyInstance* body, float connectionDist)
{
    if (!body) return;
    float minDist = PhysicsConstants::MIN_BEAM_LENGTH;
    int count = (int)body->currentPos.size();

    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            float dist = Length(Sub(body->currentPos[i], body->currentPos[j]));
            if (dist <= connectionDist && dist > minDist) {
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
    body->beams[index].damping    = damping;
}

EXPORT void SoftBody_ApplyWorldForceToNode(SoftBodyInstance* body, int nodeIndex, Vector3 worldForce, float dt)
{
    if (!body) return;
    if (nodeIndex < 0 || nodeIndex >= (int)body->masses.size() || body->isPinned[nodeIndex]) return;

    float mass = body->masses[nodeIndex];
    if (mass <= 0.0f) return;

    Vector3 accel = Scale(worldForce, 1.0f / mass);
    Vector3 currentVel = Scale(Sub(body->currentPos[nodeIndex], body->previousPos[nodeIndex]), 1.0f / dt);
    Vector3 newVel = Add(currentVel, Scale(accel, dt));

    body->previousPos[nodeIndex] = Sub(body->currentPos[nodeIndex], Scale(newVel, dt));
}

EXPORT void SoftBody_ResetLagrangeMultipliers(SoftBodyInstance* body)
{
    if (body) std::fill(body->multipliers.begin(), body->multipliers.end(), 0.0f);
}

EXPORT void SoftBody_CheckAndBreakConstraints(SoftBodyInstance* body, float dt)
{
    if (!body) return;
    float dtSq = dt * dt;
    for (auto& beam : body->beams) {
        if (!beam.isActive || !beam.isCrossBody || beam.isBroken) continue;

        float forceMag = AbsF(beam.lagrangeMultiplier) / dtSq;
        if (beam.strength > 0.0f && forceMag > beam.strength) {
            beam.isBroken = 1;
            beam.isActive = 0;
        }
    }
}

EXPORT void SoftBody_SolveCrossBodyStep(SoftBodyInstance* body, float dt)
{
    if (!body) return;

    for (auto& beam : body->beams)
    {
        if (!beam.isActive || !beam.isCrossBody) continue;
        if (beam.bodyA != reinterpret_cast<uint64_t>(body)) continue;
        if (beam.bodyB == 0) continue;

        if (beam.isEdgeSliding)
        {
            SolveEdgeSliding(beam, dt);
        }
        else
        {
            SoftBodyInstance* bA = reinterpret_cast<SoftBodyInstance*>(beam.bodyA);
            SoftBodyInstance* bB = reinterpret_cast<SoftBodyInstance*>(beam.bodyB);
            if (!bA || !bB) continue;

            Vector3 pA = bA->predictedPos[beam.nodeA];
            Vector3 pB = bB->predictedPos[beam.nodeB];
            Vector3 delta = Sub(pB, pA);
            float currentLen = Length(delta);
            if (currentLen < PhysicsConstants::MIN_BEAM_LENGTH) continue;

            Vector3 grad = Scale(delta, 1.0f / currentLen);
            float targetLen = beam.restLength;
            if (beam.minLength > 0.0f && currentLen < beam.minLength) targetLen = beam.minLength;
            else if (beam.maxLength > 0.0f && currentLen > beam.maxLength) targetLen = beam.maxLength;

            float C = currentLen - targetLen;
            float effectiveCompliance = beam.compliance / (dt * dt);

            bool pinA = bA->isPinned[beam.nodeA];
            bool pinB = bB->isPinned[beam.nodeB];
            float wA = pinA ? 0.0f : 1.0f / bA->masses[beam.nodeA];
            float wB = pinB ? 0.0f : 1.0f / bB->masses[beam.nodeB];
            float wSum = wA + wB;
            if (wSum <= 0.0f) continue;

            float deltaLambda = -(C + effectiveCompliance * beam.lagrangeMultiplier) / (wSum + effectiveCompliance);

            beam.lagrangeMultiplier += deltaLambda;

            Vector3 correction = Scale(grad, deltaLambda);

            if (!pinA)
                bA->predictedPos[beam.nodeA] = Sub(bA->predictedPos[beam.nodeA], Scale(correction, wA));

            if (!pinB)
                bB->predictedPos[beam.nodeB] = Add(bB->predictedPos[beam.nodeB], Scale(correction, wB));
        }
    }
}

EXPORT void SoftBody_UpdateNodeRotations(SoftBodyInstance* body)
{
    if (!body) return;
    int nodeCount = (int)body->currentPos.size();
    if (nodeCount == 0) return;

    if (body->nbrOffsets.empty()) body->RebuildNeighborCache();

#pragma omp parallel for schedule(dynamic, 4)
    for (int i = 0; i < nodeCount; ++i)
    {
        int start = body->nbrOffsets[i];
        int end = body->nbrOffsets[i + 1];
        if (start == end) continue;

        float mSelf = body->masses[i];
        Vector3 comR = Scale(body->initialPos[i], mSelf);
        Vector3 comC = Scale(body->currentPos[i], mSelf);
        float mSum = mSelf;

        for (int k = start; k < end; ++k)
        {
            int j = body->nbrData[k];
            float m = body->masses[j];
            comR = Add(comR, Scale(body->initialPos[j], m));
            comC = Add(comC, Scale(body->currentPos[j], m));
            mSum += m;
        }

        if (mSum < PhysicsConstants::MIN_TOTAL_MASS) continue;

        float invM = 1.0f / mSum;
        comR = Scale(comR, invM);
        comC = Scale(comC, invM);

        float A[3][3] = { 0 };

        auto accumulateCovariance = [&](int idx, float m)
            {
                Vector3 pR = Sub(body->initialPos[idx], comR);
                Vector3 pC = Sub(body->currentPos[idx], comC);
                A[0][0] += m * pC.x * pR.x; A[0][1] += m * pC.x * pR.y; A[0][2] += m * pC.x * pR.z;
                A[1][0] += m * pC.y * pR.x; A[1][1] += m * pC.y * pR.y; A[1][2] += m * pC.y * pR.z;
                A[2][0] += m * pC.z * pR.x; A[2][1] += m * pC.z * pR.y; A[2][2] += m * pC.z * pR.z;
            };

        accumulateCovariance(i, mSelf);
        for (int k = start; k < end; ++k)
        {
            int j = body->nbrData[k];
            accumulateCovariance(j, body->masses[j]);
        }

        Quaternion q = body->currentRot[i];
        for (int iter = 0; iter < PhysicsConstants::MAX_ROTATION_ITERATIONS; ++iter)
        {
            Vector3 r0, r1, r2;
            float tx = 2.0f * q.x, ty = 2.0f * q.y, tz = 2.0f * q.z;
            float twx = tx * q.w, twy = ty * q.w, twz = tz * q.w;
            float txx = tx * q.x, txy = ty * q.x, txz = tz * q.x;
            float tyy = ty * q.y, tyz = tz * q.y, tzz = tz * q.z;

            r0 = { 1.0f - (tyy + tzz), txy + twz, txz - twy };
            r1 = { txy - twz, 1.0f - (txx + tzz), tyz + twx };
            r2 = { txz + twy, tyz - twx, 1.0f - (txx + tyy) };

            Vector3 col0 = { A[0][0], A[1][0], A[2][0] };
            Vector3 col1 = { A[0][1], A[1][1], A[2][1] };
            Vector3 col2 = { A[0][2], A[1][2], A[2][2] };

            Vector3 tau = Add(Add(Cross(r0, col0), Cross(r1, col1)), Cross(r2, col2));
            float denom = Dot(r0, col0) + Dot(r1, col1) + Dot(r2, col2);

            float safeDenom = AbsF(denom) + PhysicsConstants::ROTATION_CONVERGENCE_EPSILON;
            Vector3 omega = Scale(tau, 1.0f / safeDenom);
            float w = Length(omega);

            if (w < PhysicsConstants::MIN_OMEGA_MAGNITUDE) break;

            Quaternion dq = QAxisAngle(Scale(omega, 1.0f / w), w);
            q = QNormalize(QMul(dq, q));
        }
        body->currentRot[i] = q;
    }
}

EXPORT void SoftBody_SetNodePinned(SoftBodyInstance* body, int index, bool pinned)
{
    if (!body) return;
    if (index >= 0 && index < (int)body->isPinned.size()) body->isPinned[index] = pinned;
}

EXPORT void SoftBody_SetPreviousPos(SoftBodyInstance* body, int index, Vector3 pos)
{
    if (!body) return;
    if (index >= 0 && index < (int)body->previousPos.size()) body->previousPos[index] = pos;
}

EXPORT Vector3 SoftBody_GetPredictedPos(SoftBodyInstance* body, int index)
{
    if (!body) return { 0,0,0,0 };
    if (index >= 0 && index < (int)body->predictedPos.size()) return body->predictedPos[index];
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
    b.isCrossBody = (bodyA == bodyB) ? 0 : 1; 
    b.isActive = 1;
    b.isBroken = 0;
    b.bodyA = reinterpret_cast<uint64_t>(bodyA);
    b.bodyB = reinterpret_cast<uint64_t>(bodyB);
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
    if (index >= 0 && index < (int)body->predictedPos.size())
        body->predictedPos[index] = pos;
}

EXPORT Vector3 SoftBody_GetCurrentPos(SoftBodyInstance* body, int index)
{
    if (!body) return { 0.0f, 0.0f, 0.0f, 0.0f };
    if (index >= 0 && index < (int)body->currentPos.size())
        return body->currentPos[index];
    return { 0.0f, 0.0f, 0.0f, 0.0f };
}

EXPORT void SoftBody_SetCurrentPos(SoftBodyInstance* body, int index, Vector3 pos)
{
    if (!body) return;
    if (index >= 0 && index < (int)body->currentPos.size())
        body->currentPos[index] = pos;
}

EXPORT void SoftBody_SetNodeMass(SoftBodyInstance* body, int index, float mass)
{
    if (!body) return;
    if (index >= 0 && index < (int)body->masses.size())
        body->masses[index] = mass;
}
EXPORT void SoftBody_SetInternalPressure(SoftBodyInstance* body, float pressure)
{
    if (body) body->internalPressure = pressure;
}

EXPORT void SoftBody_SetPredictedPosBulk(SoftBodyInstance* body, Vector3* positions, int count)
{
    if (!body || !positions || count <= 0) return;
    if (count == (int)body->predictedPos.size()) {
        std::copy(positions, positions + count, body->predictedPos.begin());
    }
}

EXPORT void SoftBody_SetCurrentPosBulk(SoftBodyInstance* body, Vector3* positions, int count)
{
    if (!body || !positions || count <= 0) return;
    if (count == (int)body->currentPos.size()) {
        std::copy(positions, positions + count, body->currentPos.begin());
    }
}

EXPORT void SoftBody_SetPreviousPosBulk(SoftBodyInstance* body, Vector3* positions, int count)
{
    if (!body || !positions || count <= 0) return;
    if (count == (int)body->previousPos.size()) {
        std::copy(positions, positions + count, body->previousPos.begin());
    }
}

EXPORT void SoftBody_StepPhysics(SoftBodyInstance* body, float dt, int constraintIters, float gravity)
{
    if (!body) return;
    if (body->currentPos.empty() || dt <= 0.0f) return;
    int nodeCount = (int)body->currentPos.size();
    int beamCount = (int)body->beams.size();
    float* masses = body->masses.data();
    const uint8_t* pinned = body->isPinned.data();

    Integrate(body->currentPos.data(), body->previousPos.data(), body->predictedPos.data(), pinned, nodeCount, dt, body->defaultDamping, gravity);
    if (body->internalPressure > 0.0f) 
        ApplyPressure(body->predictedPos.data(), body->currentPos.data(), masses, pinned, nodeCount, body->internalPressure, dt);
    if (beamCount > 0) 
        ApplyBeamDampingForces(body->predictedPos.data(), body->previousPos.data(), body->beams.data(), masses, pinned, beamCount, dt);
    std::fill(body->multipliers.begin(), body->multipliers.end(), 0.0f);
    for (auto& beam : body->beams)
    {
        if (beam.isCrossBody)
            beam.lagrangeMultiplier = 0.0f;
    }
    for (int iter = 0; iter < constraintIters; ++iter) 
        SolveConstraints(body->predictedPos.data(), body->beams.data(), body->multipliers.data(), masses, pinned, beamCount, dt);
    if (beamCount > 0)
    {
        // Assuming ApplyPlasticity signature is updated as shown below
        ApplyPlasticity(body->predictedPos.data(), body->beams.data(), beamCount, dt);
    }
}

EXPORT void SoftBody_FinalizeStep(SoftBodyInstance* body)
{
    if (!body) return;
    FinalizePositions(body->currentPos.data(), body->previousPos.data(), body->predictedPos.data(), body->isPinned.data(), (int)body->currentPos.size());
}

EXPORT void* SoftBody_GetPredicted(SoftBodyInstance* b)
{
    if (!b) return nullptr;
    return b->predictedPos.data();
}

EXPORT void* SoftBody_GetCurrent(SoftBodyInstance* b)
{
    if (!b) return nullptr;
    return b->currentPos.data();
}

EXPORT void* SoftBody_GetPrevious(SoftBodyInstance* b)
{
    if (!b) return nullptr;
    return b->previousPos.data();
}

EXPORT void* SoftBody_GetMasses(SoftBodyInstance* b)
{
    if (!b) return nullptr;
    return b->masses.data();
}

EXPORT void* SoftBody_GetPinned(SoftBodyInstance* b)
{
    if (!b) return nullptr;
    return b->isPinned.data();
}
EXPORT void SoftBody_RemoveCrossBodyBeams(SoftBodyInstance* body, SoftBodyInstance* target)
{
    if (!body || !target) return;
    auto& beams = body->beams;
    beams.erase(std::remove_if(beams.begin(), beams.end(),
        [target](const BeamData& b) {
            return b.isCrossBody && (b.bodyA == (uint64_t)target || b.bodyB == (uint64_t)target);
        }), beams.end());
}