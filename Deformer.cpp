/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝
   Mesh deformation: maps vertices to nodes, applies skinning each frame.
*/
#include "EngineTypes.h"
#include "MathUtils.h"
#include "Constants.h"

#include <algorithm>
#include <vector>
#include <cmath>
#include <cstdint>

#if defined(_WIN32)
#define EXPORT extern "C" __declspec(dllexport)
#else
#define EXPORT extern "C" __attribute__((visibility("default")))
#endif


//Internal skinning helpers 

namespace
{
    struct VertexInfluencePair
    {
        int   nodeIndex;
        float weight;
    };

    //Distance-based skinning
    void BuildDistanceSkinning(const Vector3* localVertices, int vertexCount,
        const Vector3* nodeLocalPos, int nodeCount,
        float influenceRadius, int maxInfluencesPerVertex,
        std::vector<int>& outIndices,
        std::vector<float>& outWeights,
        std::vector<Vector3>& outOffsets,
        std::vector<int>& outCSR)
    {
        outIndices.clear();
        outWeights.clear();
        outOffsets.clear();
        outCSR.assign(vertexCount + 1, 0);

        std::vector<VertexInfluencePair> tmp;
        tmp.reserve(maxInfluencesPerVertex);

        float r2 = influenceRadius * influenceRadius;

        for (int v = 0; v < vertexCount; ++v)
        {
            outCSR[v] = (int)outIndices.size();
            tmp.clear();

            const Vector3& vp = localVertices[v];
            for (int n = 0; n < nodeCount; ++n)
            {
                Vector3 d = Sub(vp, nodeLocalPos[n]);
                float d2 = LengthSq(d);
                if (d2 > r2) continue;

                float dist = std::sqrt(d2);
                float t = dist / influenceRadius;
                float t2 = t * t;
                float w = (1.0f - t2) * (1.0f - t2); 
                tmp.push_back({ n, w });
            }

            // Keep top-K by weight.
            if ((int)tmp.size() > maxInfluencesPerVertex)
            {
                std::partial_sort(tmp.begin(), tmp.begin() + maxInfluencesPerVertex, tmp.end(),
                    [](const VertexInfluencePair& a, const VertexInfluencePair& b) {
                        return a.weight > b.weight;
                    });
                tmp.resize(maxInfluencesPerVertex);
            }

            if (tmp.empty() && nodeCount > 0)
            {
                int   bestIdx = 0;
                float bestD2 = LengthSq(Sub(vp, nodeLocalPos[0]));
                for (int n = 1; n < nodeCount; ++n)
                {
                    float d2 = LengthSq(Sub(vp, nodeLocalPos[n]));
                    if (d2 < bestD2) { bestD2 = d2; bestIdx = n; }
                }
                tmp.push_back({ bestIdx, 1.0f });
            }

            // Normalise weights.
            float total = 0.0f;
            for (auto& p : tmp) total += p.weight;
            if (total > 1e-7f)
            {
                float inv = 1.0f / total;
                for (auto& p : tmp) p.weight *= inv;
            }

            for (auto& p : tmp)
            {
                outIndices.push_back(p.nodeIndex);
                outWeights.push_back(p.weight);
                outOffsets.push_back(Sub(vp, nodeLocalPos[p.nodeIndex]));
            }
        }

        outCSR[vertexCount] = (int)outIndices.size();
    }

    //Volume / nearest-neighbour skinning
    void BuildVolumeSkinning(const Vector3* localVertices, int vertexCount,
        const Vector3* nodeLocalPos, int nodeCount, int kNearest,
        std::vector<int>& outIndices,
        std::vector<float>& outWeights,
        std::vector<Vector3>& outOffsets,
        std::vector<int>& outCSR)
    {
        outIndices.clear();
        outWeights.clear();
        outOffsets.clear();
        outCSR.assign(vertexCount + 1, 0);

        int K = (kNearest < nodeCount) ? kNearest : nodeCount;
        std::vector<std::pair<float, int>> dist(nodeCount);
        std::vector<float> ws(K);

        for (int v = 0; v < vertexCount; ++v)
        {
            outCSR[v] = (int)outIndices.size();
            const Vector3& vp = localVertices[v];

            for (int n = 0; n < nodeCount; ++n)
                dist[n] = { LengthSq(Sub(vp, nodeLocalPos[n])), n };

            std::partial_sort(dist.begin(), dist.begin() + K, dist.end(),
                [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
                    return a.first < b.first;
                });

            float total = 0.0f;
            for (int k = 0; k < K; ++k)
            {
                float w = 1.0f / (1.0f + dist[k].first);
                ws[k] = w;
                total += w;
            }
            if (total < 1e-7f) total = 1.0f;
            float inv = 1.0f / total;

            for (int k = 0; k < K; ++k)
            {
                int ni = dist[k].second;
                outIndices.push_back(ni);
                outWeights.push_back(ws[k] * inv);

                // Keep the raw, unaltered offset vector
                Vector3 off = Sub(vp, nodeLocalPos[ni]);
                outOffsets.push_back(off);
            }
        }

        outCSR[vertexCount] = (int)outIndices.size();
    }

}


// DeformMesh

EXPORT void DeformMesh(Vector3* outVertices,
    const Vector3* originalVertices,
    const Vector3* nodeLocalPos,
    const Quaternion* nodeLocalRot,
    const Quaternion* nodeInitRot,
    const InfluenceData* influences,
    const int* influenceOffsets,
    int vertexCount,
    int nodeCount)
{
    if (vertexCount <= 0 || nodeCount <= 0) return;

    Quaternion bodyDeltaRot = QMul(nodeLocalRot[0], QInverse(nodeInitRot[0]));
    std::vector<Quaternion> deltaRots(nodeCount);
    for (int j = 0; j < nodeCount; ++j)
    {
        Quaternion delta = QMul(nodeLocalRot[j], QInverse(nodeInitRot[j]));

        float dot = delta.x * bodyDeltaRot.x + delta.y * bodyDeltaRot.y + delta.z * bodyDeltaRot.z + delta.w * bodyDeltaRot.w;
        if (dot < 0.0f)
        {
            delta = { -delta.x, -delta.y, -delta.z, -delta.w };
        }

        deltaRots[j] = delta;
    }

#pragma omp parallel for schedule(static)
    for (int i = 0; i < vertexCount; ++i)
    {
        int start = influenceOffsets[i];
        int end = influenceOffsets[i + 1];

        if (start == end) { outVertices[i] = originalVertices[i]; continue; }

        float vx = 0.0f, vy = 0.0f, vz = 0.0f, totalWeight = 0.0f;
        for (int j = start; j < end; ++j)
        {
            const InfluenceData& inf = influences[j];
            if (inf.nodeIndex < 0 || inf.nodeIndex >= nodeCount) continue;

            Vector3 rotatedOff = QRotate(deltaRots[inf.nodeIndex], inf.localOffset);
            const Vector3& targetPos = nodeLocalPos[inf.nodeIndex];

            vx += (targetPos.x + rotatedOff.x) * inf.weight;
            vy += (targetPos.y + rotatedOff.y) * inf.weight;
            vz += (targetPos.z + rotatedOff.z) * inf.weight;
            totalWeight += inf.weight;
        }

        if (totalWeight > 0.001f)
        {
            float invW = 1.0f / totalWeight;
            outVertices[i] = { vx * invW, vy * invW, vz * invW, 0.0f };
        }
        else outVertices[i] = originalVertices[i];
    }
}

// Backward-compat mapping EXPORTs

EXPORT int MapVerticesToNodes_Distance(const Vector3* localVertices, int vertexCount,
    const Vector3* nodeLocalPos, int nodeCount,
    float influenceRadius, int maxInfluencesPerVertex,
    int* outIndices,
    float* outWeights,
    Vector3* outLocalOffsets,
    int* outOffsetsCSR,
    int      outCapacity)
{
    if (vertexCount <= 0 || nodeCount <= 0) return 0;

    std::vector<int>     idx;
    std::vector<float>   wts;
    std::vector<Vector3> offs;
    std::vector<int>     csr;

    BuildDistanceSkinning(localVertices, vertexCount, nodeLocalPos, nodeCount,
        influenceRadius, maxInfluencesPerVertex, idx, wts, offs, csr);

    int total = (int)idx.size();

    if (outIndices && outWeights && outLocalOffsets && outOffsetsCSR)
    {
        if (outCapacity < total) total = outCapacity;
        for (int i = 0; i < total; ++i)
        {
            outIndices[i] = idx[i];
            outWeights[i] = wts[i];
            outLocalOffsets[i] = offs[i];
        }
        for (int v = 0; v <= vertexCount; ++v)
            outOffsetsCSR[v] = csr[v];
    }

    return (int)idx.size();
}

EXPORT int MapVerticesToNodes_Volume(const Vector3* localVertices, int vertexCount,
    const Vector3* nodeLocalPos, int nodeCount,
    int  kNearest,
    int* outIndices,
    float* outWeights,
    Vector3* outLocalOffsets,
    int* outOffsetsCSR,
    int      outCapacity)
{
    if (vertexCount <= 0 || nodeCount <= 0 || kNearest <= 0) return 0;

    std::vector<int>     idx;
    std::vector<float>   wts;
    std::vector<Vector3> offs;
    std::vector<int>     csr;

    BuildVolumeSkinning(localVertices, vertexCount, nodeLocalPos, nodeCount,
        kNearest, idx, wts, offs, csr);

    int total = (int)idx.size();

    if (outIndices && outWeights && outLocalOffsets && outOffsetsCSR)
    {
        if (outCapacity < total) total = outCapacity;
        for (int i = 0; i < total; ++i)
        {
            outIndices[i] = idx[i];
            outWeights[i] = wts[i];
            outLocalOffsets[i] = offs[i];
        }
        for (int v = 0; v <= vertexCount; ++v)
            outOffsetsCSR[v] = csr[v];
    }

    return (int)idx.size();
}

// DeformerState

struct DeformerState
{
    // Geometry
    std::vector<Vector3>      originalVerts;   // world-space
    std::vector<Quaternion>   nodeInitRot;     // initial rotations

    // Cache-Friendly CSR Data Structure
    std::vector<InfluenceData> influenceData;
    std::vector<int>           influenceOffsets; 

    // Quaternion sign-stabilisation buffers
    std::vector<Quaternion>   stabilizedRots;
    std::vector<Quaternion>   prevStabilizedRots;
    std::vector<Quaternion>   deltaRots;
    bool                      hasPrevRotations = false;

    // Config (mirrors public MeshDeformer fields)
    bool  useAdvancedSkinning = true;
    float influenceRadius = 1.5f;
    int   maxInfluences = 4;

    // Sizes
    int vertCount = 0;
    int nodeCount = 0;
};


// Deformer lifecycle EXPORTs

// Create
EXPORT DeformerState* Deformer_Create()
{
    return new DeformerState();
}

// Initialize
EXPORT void Deformer_Initialize(
    DeformerState* s,
    const Vector3* worldVerts, int vertCount,
    const Vector3* nodePositions,
    const Quaternion* nodeRotations, int nodeCount,
    int   maxInfluences,
    float influenceRadius,
    int   useAdvancedSkinning)
{
    if (!s || vertCount <= 0 || nodeCount <= 0) return;

    s->vertCount = vertCount;
    s->nodeCount = nodeCount;
    s->maxInfluences = maxInfluences;
    s->influenceRadius = influenceRadius;
    s->useAdvancedSkinning = (useAdvancedSkinning != 0);
    s->hasPrevRotations = false;

    s->originalVerts.assign(worldVerts, worldVerts + vertCount);
    s->nodeInitRot.assign(nodeRotations, nodeRotations + nodeCount);
    s->stabilizedRots.resize(nodeCount);
    s->prevStabilizedRots.resize(nodeCount);
    s->deltaRots.resize(nodeCount);

    // Build influence data using the appropriate skinning algorithm.
    std::vector<int>     idx;
    std::vector<float>   wts;
    std::vector<Vector3> offs;
    std::vector<int>     csr;

    if (s->useAdvancedSkinning)
        BuildVolumeSkinning(worldVerts, vertCount, nodePositions, nodeCount,
            maxInfluences, idx, wts, offs, csr);
    else
        BuildDistanceSkinning(worldVerts, vertCount, nodePositions, nodeCount,
            influenceRadius, maxInfluences, idx, wts, offs, csr);

    int total = (int)idx.size();
    s->influenceData.resize(total);
    for (int i = 0; i < total; ++i)
    {
        InfluenceData inf{};
        inf.nodeIndex = idx[i];
        inf.weight = wts[i];
        inf.localOffset = offs[i];
        s->influenceData[i] = inf;
    }
    s->influenceOffsets.assign(csr.begin(), csr.end());
}

// Deform
EXPORT void Deformer_Deform(
    DeformerState* s,
    const Vector3* currentPositions,
    const Quaternion* currentRotations,
    Vector3* outDeformedVerts,
    int nodeCount)
{
    if (!s || s->vertCount == 0 || !outDeformedVerts) return;

    // Use the passed nodeCount, clamped to what was initialized
    int nc = (nodeCount > 0 && nodeCount <= s->nodeCount) ? nodeCount : s->nodeCount;

    for (int i = 0; i < nc; ++i)
    {
        Quaternion q = currentRotations[i];

        // Maintain sign stabilization to prevent quaternion flipping, 
        // but remove the smoothFactor blending.
        if (s->hasPrevRotations)
        {
            const Quaternion& p = s->prevStabilizedRots[i];
            float dot = q.x * p.x + q.y * p.y + q.z * p.z + q.w * p.w;
            if (dot < 0.0f) q = { -q.x, -q.y, -q.z, -q.w };
        }

        s->stabilizedRots[i] = q;
        s->prevStabilizedRots[i] = q;
    }
    s->hasPrevRotations = true;

    // Build deltaRots into the pre-allocated buffer — no heap allocation
    Quaternion bodyDeltaRot = QMul(s->stabilizedRots[0], QInverse(s->nodeInitRot[0]));
    for (int j = 0; j < nc; ++j)
    {
        Quaternion delta = QMul(s->stabilizedRots[j], QInverse(s->nodeInitRot[j]));
        float dot = delta.x * bodyDeltaRot.x + delta.y * bodyDeltaRot.y
            + delta.z * bodyDeltaRot.z + delta.w * bodyDeltaRot.w;
        if (dot < 0.0f) delta = { -delta.x, -delta.y, -delta.z, -delta.w };
        s->deltaRots[j] = delta;
    }

    DeformMesh(
        outDeformedVerts,
        s->originalVerts.data(),
        currentPositions,
        s->stabilizedRots.data(),
        s->nodeInitRot.data(),
        s->influenceData.data(),
        s->influenceOffsets.data(),
        s->vertCount,
        nc                   
    );
}
// SetSkinningMethod
EXPORT void Deformer_SetSkinningMethod(DeformerState* s, int useAdvanced)
{
    if (s) s->useAdvancedSkinning = (useAdvanced != 0);
}

// Destroy
EXPORT void Deformer_Destroy(DeformerState* s)
{
    if (s) delete s;
}