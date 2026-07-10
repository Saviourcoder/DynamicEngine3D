/*
 DYNAMICENGINE3D
 AI-Assisted Soft-Body Physics for Unity3D
 By: Elitmers
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

    // DeformerState

    struct DeformerState
    {
        // Geometry
        std::vector<Vector3>      originalVerts;   // world-space
        std::vector<Quaternion>   nodeInitRot;     // initial rotations
        std::vector<Quaternion> nodeInitRotInv;
        std::vector<float> matBuf;

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

    //Distance-based skinning
    void BuildDistanceSkinning(const Vector3* localVertices, int vertexCount,
        const float* nodeLocalPosX, const float* nodeLocalPosY, const float* nodeLocalPosZ, int nodeCount,
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
                Vector3 nodePos = { nodeLocalPosX[n], nodeLocalPosY[n], nodeLocalPosZ[n], 0.0f };
                Vector3 d = Sub(vp, nodePos);
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
                Vector3 np0 = { nodeLocalPosX[0], nodeLocalPosY[0], nodeLocalPosZ[0], 0.0f };
                float bestD2 = LengthSq(Sub(vp, np0));
                for (int n = 1; n < nodeCount; ++n)
                {
                    Vector3 np = { nodeLocalPosX[n], nodeLocalPosY[n], nodeLocalPosZ[n], 0.0f };
                    float d2 = LengthSq(Sub(vp, np));
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
                Vector3 nodePos = { nodeLocalPosX[p.nodeIndex], nodeLocalPosY[p.nodeIndex], nodeLocalPosZ[p.nodeIndex], 0.0f };
                outIndices.push_back(p.nodeIndex);
                outWeights.push_back(p.weight);
                outOffsets.push_back(Sub(vp, nodePos));
            }
        }

        outCSR[vertexCount] = (int)outIndices.size();
    }

    //Volume / nearest-neighbour skinning
    void BuildVolumeSkinning(const Vector3* localVertices, int vertexCount,
        const float* nodeLocalPosX, const float* nodeLocalPosY, const float* nodeLocalPosZ, int nodeCount, int kNearest,
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
        if (K < 1) K = 1;
        std::vector<std::pair<float, int>> dist(nodeCount);
        std::vector<float> ws(K);

        for (int v = 0; v < vertexCount; ++v)
        {
            outCSR[v] = (int)outIndices.size();
            const Vector3& vp = localVertices[v];

            for (int n = 0; n < nodeCount; ++n)
            {
                Vector3 nodePos = { nodeLocalPosX[n], nodeLocalPosY[n], nodeLocalPosZ[n], 0.0f };
                dist[n] = { LengthSq(Sub(vp, nodePos)), n };
            }

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
                Vector3 nodePos = { nodeLocalPosX[ni], nodeLocalPosY[ni], nodeLocalPosZ[ni], 0.0f };
                Vector3 off = Sub(vp, nodePos);
                outOffsets.push_back(off);
            }
        }

        outCSR[vertexCount] = (int)outIndices.size();
    }

}


// DeformMesh
static void DeformMesh_Internal(
    Vector3* outVertices,
    const Vector3* originalVertices,
    const float* nodeLocalPosX, const float* nodeLocalPosY, const float* nodeLocalPosZ,
    const Quaternion* deltaRots,       // pre-computed, sign-stabilised
    const InfluenceData* influences,
    const int* influenceOffsets,
    int vertexCount,int nodeCount,
    float* matBuf)
{

    // Precompute rotation matrices - nodeCount iterations, not vertexCount*4
    for (int n = 0; n < nodeCount; ++n)
    {
        const Quaternion& q = deltaRots[n];
        float qx = q.x, qy = q.y, qz = q.z, qw = q.w;
        float x2 = qx + qx, y2 = qy + qy, z2 = qz + qz;
        float xx = qx * x2, xy = qx * y2, xz = qx * z2;
        float yy = qy * y2, yz = qy * z2, zz = qz * z2;
        float wx = qw * x2, wy = qw * y2, wz = qw * z2;
        float* m = matBuf + n * 9;
        m[0] = 1.0f - (yy + zz); m[1] = xy - wz;        m[2] = xz + wy;
        m[3] = xy + wz;        m[4] = 1.0f - (xx + zz); m[5] = yz - wx;
        m[6] = xz - wy;        m[7] = yz + wx;        m[8] = 1.0f - (xx + yy);
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

            const float* m = matBuf + inf.nodeIndex * 9;
            const Vector3& off = inf.localOffset;

            // matrix-vector instead of QRotate
            float rx = m[0] * off.x + m[1] * off.y + m[2] * off.z;
            float ry = m[3] * off.x + m[4] * off.y + m[5] * off.z;
            float rz = m[6] * off.x + m[7] * off.y + m[8] * off.z;

            vx += (nodeLocalPosX[inf.nodeIndex] + rx) * inf.weight;
            vy += (nodeLocalPosY[inf.nodeIndex] + ry) * inf.weight;
            vz += (nodeLocalPosZ[inf.nodeIndex] + rz) * inf.weight;
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
    const float* nodePosX, const float* nodePosY, const float* nodePosZ,
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
    s->nodeInitRotInv.resize(nodeCount);
    for (int i = 0; i < nodeCount; ++i)
        s->nodeInitRotInv[i] = QInverse(s->nodeInitRot[i]);
    s->stabilizedRots.resize(nodeCount);
    s->prevStabilizedRots.resize(nodeCount);
    s->deltaRots.resize(nodeCount);
    s->matBuf.resize(nodeCount * 9);

    // Build influence data using the appropriate skinning algorithm.
    std::vector<int>     idx;
    std::vector<float>   wts;
    std::vector<Vector3> offs;
    std::vector<int>     csr;

    if (s->useAdvancedSkinning)
        BuildVolumeSkinning(worldVerts, vertCount, nodePosX, nodePosY, nodePosZ, nodeCount,
            maxInfluences, idx, wts, offs, csr);
    else
        BuildDistanceSkinning(worldVerts, vertCount, nodePosX, nodePosY, nodePosZ, nodeCount,
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
    const float* cx, const float* cy, const float* cz,
    const Quaternion* currentRotations,
    Vector3* outDeformedVerts,
    int nodeCount)
{
    if (!s || s->vertCount == 0 || !outDeformedVerts) return;

    int nc = (nodeCount > 0 && nodeCount <= s->nodeCount) ? nodeCount : s->nodeCount;

    // Sign stabilisation pass (unchanged)
    for (int i = 0; i < nc; ++i)
    {
        Quaternion q = currentRotations[i];
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

    // Build deltaRots once into the pre-allocated buffer
    Quaternion bodyDeltaRot = QMul(s->stabilizedRots[0], s->nodeInitRotInv[0]);
    for (int j = 0; j < nc; ++j)
    {
        Quaternion delta = QMul(s->stabilizedRots[j], s->nodeInitRotInv[j]);
        float dot = delta.x * bodyDeltaRot.x + delta.y * bodyDeltaRot.y
            + delta.z * bodyDeltaRot.z + delta.w * bodyDeltaRot.w;
        if (dot < 0.0f) delta = { -delta.x, -delta.y, -delta.z, -delta.w };
        s->deltaRots[j] = delta;
    }

    // call internal helper
    DeformMesh_Internal(
        outDeformedVerts,
        s->originalVerts.data(),
        cx, cy, cz,
        s->deltaRots.data(),        // precomputed buffer, not recomputed inside
        s->influenceData.data(),
        s->influenceOffsets.data(),
        s->vertCount,
        nc, s->matBuf.data()
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