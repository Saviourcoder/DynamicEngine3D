/*
 DYNAMICENGINE3D
 AI-Assisted 3D Physics Engine
 By: Elitmers
*/

#include "EngineTypes.h"
#include "MathUtils.h"
#include "Constants.h"
#include "EngineStats.h"
#include "Error.h"

#include <algorithm>
#include <vector>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <utility>
#include "ThreadPool.h"


// LBS skinning: out = Σ w_i · (pos_i + R_i · offLocal_i) over the K nearest truss nodes.


namespace
{
    constexpr int   kInfluencesPerVertex = 4;   // fixed influence count per vertex
    constexpr float kWeightEpsilon = 1e-8f;     // d^2 denominator guard

    // DeformerState

    struct DeformerState
    {
        std::vector<Vector3> originalVerts;   // world-space, bind-time

        // LBS skinning data, SoA; per vertex: infCount influences in the K slots at [v*K, v*K+K).
        std::vector<int>     infCount;        // [V] actual influences (0..K)
        std::vector<int>     infNodes;        // [V*K] node indices, weight order
        std::vector<float>   infWeights;      // [V*K] normalized weights
        std::vector<Vector3> infOffsetsLocal; // [V*K] offset in each node's bind frame

        int vertCount = 0;
        int nodeCount = 0;
    };

    // Per-vertex candidate gathering: the K nearest non-excluded nodes, closest first
    void GatherCandidates(
        const Vector3& vp,
        const float* nodePosX,
        const float* nodePosY,
        const float* nodePosZ,
        const uint8_t* excludedNodesMask,
        int nodeCount,
        std::vector<std::pair<float, int>>& scratch,
        int poolSize)
    {
        scratch.clear();

        if (nodeCount <= 0)
            return;

        scratch.reserve(nodeCount);

        for (int n = 0; n < nodeCount; ++n)
        {
            // Excluded nodes are completely invisible to the deformer.
            if (excludedNodesMask && excludedNodesMask[n] != 0)
                continue;

            Vector3 np = {
                nodePosX[n],
                nodePosY[n],
                nodePosZ[n],
                0.0f
            };

            scratch.emplace_back(
                LengthSq(Sub(vp, np)),
                n
            );
        }

        int k = std::min(poolSize, static_cast<int>(scratch.size()));

        if (k <= 0)
            return;

        std::partial_sort(
            scratch.begin(),
            scratch.begin() + k,
            scratch.end(),
            [](const std::pair<float, int>& a,
                const std::pair<float, int>& b)
            {
                return a.first < b.first;
            });

        scratch.resize(k);
    }

    // Builds LBS data for every vertex in parallel; offLocal_i = R_bind_i⁻¹ · (vert − node_i).
    void BuildSkinning(
        const Vector3* worldVerts,
        int vertexCount,
        const float* nodePosX,
        const float* nodePosY,
        const float* nodePosZ,
        const Quaternion* nodeBindRotations,
        const uint8_t* excludedNodesMask,
        int nodeCount,
        DeformerState& s)
    {
        s.infCount.assign(vertexCount, 0);
        s.infNodes.assign((size_t)vertexCount * kInfluencesPerVertex, -1);
        s.infWeights.assign((size_t)vertexCount * kInfluencesPerVertex, 0.0f);
        s.infOffsetsLocal.assign((size_t)vertexCount * kInfluencesPerVertex, Vector3{ 0, 0, 0, 0 });

        constexpr int kMinBindVertices = 256;
        const int K = kInfluencesPerVertex;

        GetGlobalPool().parallel_for(vertexCount, [&](int v)
            {
                thread_local std::vector<std::pair<float, int>> candidates;
                const Vector3& vp = worldVerts[v];

                const int base = v * K;
                int* nodes = &s.infNodes[base];
                float* weights = &s.infWeights[base];
                Vector3* offsets = &s.infOffsetsLocal[base];
                int& count = s.infCount[v];

                if (nodeCount < 1)
                {
                    return; // no rig at all: vertex stays at originalVerts
                }

                GatherCandidates(
                    vp,
                    nodePosX,
                    nodePosY,
                    nodePosZ,
                    excludedNodesMask,
                    nodeCount,
                    candidates,
                    K);

                if (candidates.empty())
                {
                    // No non-excluded node available — keep the vertex at its bind position.
                    return;
                }

                const int n = (int)candidates.size();

                // Inverse-square-distance weights, normalized over the influences available.
                float wsum = 0.0f;
                for (int i = 0; i < n; ++i)
                {
                    weights[i] = 1.0f / (candidates[i].first + kWeightEpsilon);
                    wsum += weights[i];
                }
                const float invWsum = 1.0f / wsum;

                for (int i = 0; i < n; ++i)
                {
                    const int node = candidates[i].second;
                    nodes[i] = node;
                    weights[i] *= invWsum;

                    const Vector3 nodePos = { nodePosX[node], nodePosY[node], nodePosZ[node], 0.0f };
                    const Vector3 offset = Sub(vp, nodePos);

                    // Bind-frame offset; R_i · offLocal_i recovers the world offset under rotation.
                    const Quaternion bindRotInv = nodeBindRotations
                        ? QConj(nodeBindRotations[node])
                        : QIdentity();
                    offsets[i] = QRotate(bindRotInv, offset);
                }

                count = n; // candidates arrive sorted by distance = weight order
            }, kMinBindVertices);
    }


    // DeformMesh: out = Σ w_i · (pos_i + R_i · offLocal_i); rotations converted once up front.
    static void DeformMesh_Internal(
        Vector3* outVertices,
        const Vector3* originalVertices,
        const float* cx, const float* cy, const float* cz,
        const Quaternion* nodeRotations,
        const DeformerState& s,
        int vertexCount, int nodeCount)
    {
        constexpr int kMinDeformVertices = 1024;
        const int K = kInfluencesPerVertex;

        // Per-node rotation matrices, column-major (m[col*3 + row]), heap-allocated for the loop.
        std::vector<float> mats;
        const bool haveRotations = nodeRotations != nullptr;
        if (haveRotations)
        {
            mats.resize((size_t)nodeCount * 9);
            for (int n = 0; n < nodeCount; ++n)
            {
                const Quaternion& q = nodeRotations[n];
                float x2 = q.x + q.x, y2 = q.y + q.y, z2 = q.z + q.z;
                float xx = q.x * x2, xy = q.x * y2, xz = q.x * z2;
                float yy = q.y * y2, yz = q.y * z2, zz = q.z * z2;
                float wx = q.w * x2, wy = q.w * y2, wz = q.w * z2;

                float* m = &mats[(size_t)n * 9];
                // column 0
                m[0] = 1.0f - (yy + zz); m[1] = xy + wz;        m[2] = xz - wy;
                // column 1
                m[3] = xy - wz;         m[4] = 1.0f - (xx + zz); m[5] = yz + wx;
                // column 2
                m[6] = xz + wy;         m[7] = yz - wx;         m[8] = 1.0f - (xx + yy);
            }
        }

        const int* infCount = s.infCount.data();
        const int* infNodes = s.infNodes.data();
        const float* infWeights = s.infWeights.data();
        const Vector3* infOffsets = s.infOffsetsLocal.data();

        GetGlobalPool().parallel_for(vertexCount, [&](int i)
            {
                const int count = infCount[i];
                if (count <= 0)
                {
                    outVertices[i] = originalVertices[i];
                    return;
                }

                const int base = i * K;
                float ox = 0.0f, oy = 0.0f, oz = 0.0f;

                for (int k = 0; k < count; ++k)
                {
                    const int node = infNodes[base + k];
                    if (node < 0 || node >= nodeCount)
                    {
                        // Stale binding after a node-count change — skip, others carry the vertex.
                        continue;
                    }

                    const float w = infWeights[base + k];
                    const Vector3& off = infOffsets[base + k];

                    float px = cx[node], py = cy[node], pz = cz[node];

                    if (haveRotations)
                    {
                        const float* m = &mats[(size_t)node * 9];
                        px += m[0] * off.x + m[3] * off.y + m[6] * off.z;
                        py += m[1] * off.x + m[4] * off.y + m[7] * off.z;
                        pz += m[2] * off.x + m[5] * off.y + m[8] * off.z;
                    }
                    else
                    {
                        px += off.x; py += off.y; pz += off.z;
                    }

                    ox += w * px; oy += w * py; oz += w * pz;
                }

                outVertices[i] = { ox, oy, oz, 0.0f };
            }, kMinDeformVertices);
    }


    // Deformer lifecycle EXPORTs

    EXPORT DeformerState* Deformer_Create()
    {
        return new DeformerState();
    }

    EXPORT void Deformer_Initialize(
        DeformerState* s,
        const Vector3* worldVerts,
        int vertCount,
        const float* nodePosX,
        const float* nodePosY,
        const float* nodePosZ,
        const Quaternion* nodeBindRotations,
        int nodeCount,
        const uint8_t* excludedNodesMask)
    {
        if (!s) { Error_SetError(ErrorCode::NullHandle); return; }
        if (!worldVerts || vertCount <= 0 || !nodePosX || !nodePosY || !nodePosZ || nodeCount <= 0)
        {
            Error_SetError(ErrorCode::InvalidArgument);
            return;
        }

        s->vertCount = vertCount;
        s->nodeCount = nodeCount;

        s->originalVerts.assign(worldVerts, worldVerts + vertCount);

        BuildSkinning(
            worldVerts,
            vertCount,
            nodePosX,
            nodePosY,
            nodePosZ,
            nodeBindRotations,
            excludedNodesMask,
            nodeCount,
            *s);
    }

    // Deform
    EXPORT void Deformer_Deform(
        DeformerState* s,
        const float* cx, const float* cy, const float* cz,
        const Quaternion* nodeRotations,
        Vector3* outDeformedVerts,
        int nodeCount)
    {
        if (!s) { Error_SetError(ErrorCode::NullHandle); return; }
        if (s->vertCount == 0 || !outDeformedVerts) { Error_SetError(ErrorCode::InvalidArgument); return; }

        auto deformT0 = std::chrono::high_resolution_clock::now();

        // Always take the true minimum. A non-positive nodeCount means the caller currently
        // has zero live nodes (mesh rebuild, broken truss, etc.) and must NOT fall back to
        // the larger bind-time count, or we'll read past the end of cx/cy/cz/nodeRotations.
        int nc = (nodeCount > 0) ? std::min(nodeCount, s->nodeCount) : 0;

        DeformMesh_Internal(
            outDeformedVerts,
            s->originalVerts.data(),
            cx, cy, cz,
            nodeRotations,
            *s,
            s->vertCount,
            nc
        );

        auto deformT1 = std::chrono::high_resolution_clock::now();
        AtomicAddFloat(&g_Stats.deformTimeMs, std::chrono::duration<float, std::milli>(deformT1 - deformT0).count());
    }

    // Deform + world→local transform in one call (Unity SoftBody.LateUpdate).
    // outWorldVerts: NativeVector3[vertCount] (16-byte stride); outLocalVerts: 12-byte stride.
    // worldToLocal: Unity Matrix4x4 raw memory — column-major from m00, m[col*4 + row].
    EXPORT void Deformer_DeformToLocal(
        DeformerState* s,
        const float* cx, const float* cy, const float* cz,
        const Quaternion* nodeRotations,
        Vector3* outWorldVerts,
        float* outLocalVerts,
        int vertCount,
        int nodeCount,
        const float* worldToLocal)
    {
        if (!s) { Error_SetError(ErrorCode::NullHandle); return; }
        if (s->vertCount == 0 || !outLocalVerts || !worldToLocal) { Error_SetError(ErrorCode::InvalidArgument); return; }

        auto deformT0 = std::chrono::high_resolution_clock::now();

        // Same rule as Deformer_Deform: a non-positive count means "zero live elements this
        // frame," not "fall back to the old bind-time count." The old fallback let the loops
        // below read past the end of caller-supplied arrays whenever the mesh or truss shrank
        // (or was momentarily empty during a rebuild), which is the source of the crash.
        int vc = (vertCount > 0) ? std::min(vertCount, s->vertCount) : 0;
        int nc = (nodeCount > 0) ? std::min(nodeCount, s->nodeCount) : 0;

        std::vector<Vector3> scratch;
        Vector3* world = outWorldVerts;
        if (!world)
        {
            scratch.resize(vc);
            world = scratch.data();
        }

        DeformMesh_Internal(
            world,
            s->originalVerts.data(),
            cx, cy, cz,
            nodeRotations,
            *s,
            vc,
            nc
        );

        const float* m = worldToLocal;
        for (int i = 0; i < vc; ++i)
        {
            const float x = world[i].x, y = world[i].y, z = world[i].z;
            outLocalVerts[i * 3 + 0] = m[0] * x + m[4] * y + m[8] * z + m[12];
            outLocalVerts[i * 3 + 1] = m[1] * x + m[5] * y + m[9] * z + m[13];
            outLocalVerts[i * 3 + 2] = m[2] * x + m[6] * y + m[10] * z + m[14];
        }

        auto deformT1 = std::chrono::high_resolution_clock::now();
        AtomicAddFloat(&g_Stats.deformTimeMs, std::chrono::duration<float, std::milli>(deformT1 - deformT0).count());
    }

    EXPORT void Deformer_Destroy(DeformerState* s)
    {
        if (s) delete s;
    }

    EXPORT void Deformer_SetSkinningMethod(DeformerState* /*s*/, int /*useAdvanced*/)
    {
        // No-op on the LBS path; kept for C# P/Invoke ABI compatibility.
    }

    EXPORT void Deformer_GetBindDebugStats(
        DeformerState* s,
        int* vertCount,
        int* nodeCount,
        int* fallback3,
        int* pinned)
    {
        if (!s) { Error_SetError(ErrorCode::NullHandle); return; }
        if (!vertCount || !nodeCount || !fallback3 || !pinned) { Error_SetError(ErrorCode::InvalidArgument); return; }

        *vertCount = s->vertCount;
        *nodeCount = s->nodeCount;

        // fallback3: partial bind (1..K-1 influences). pinned: no bind at all.
        int fb3Count = 0;
        int pinnedCount = 0;

        for (int c : s->infCount)
        {
            if (c == 0)
            {
                pinnedCount++;
            }
            else if (c < kInfluencesPerVertex)
            {
                fb3Count++;
            }
        }

        *fallback3 = fb3Count;
        *pinned = pinnedCount;
    }

    EXPORT bool Deformer_GetVertexBindingDebug(
        DeformerState* s,
        int vIdx,
        int* c,
        int* vx,
        int* vy,
        int* vz,
        Vector3* local)
    {
        if (!s) { Error_SetError(ErrorCode::NullHandle); return false; }
        if (!c || !vx || !vy || !vz || !local) { Error_SetError(ErrorCode::InvalidArgument); return false; }
        if (vIdx < 0 || vIdx >= s->vertCount) { Error_SetError(ErrorCode::InvalidNodeIndex); return false; }

        // Influence nodes in weight order (-1 when absent); local = strongest influence's offset.
        *c = (s->infCount[vIdx] > 0) ? s->infNodes[vIdx * kInfluencesPerVertex + 0] : -1;
        *vx = (s->infCount[vIdx] > 1) ? s->infNodes[vIdx * kInfluencesPerVertex + 1] : -1;
        *vy = (s->infCount[vIdx] > 2) ? s->infNodes[vIdx * kInfluencesPerVertex + 2] : -1;
        *vz = (s->infCount[vIdx] > 3) ? s->infNodes[vIdx * kInfluencesPerVertex + 3] : -1;
        *local = (s->infCount[vIdx] > 0) ? s->infOffsetsLocal[vIdx * kInfluencesPerVertex] : Vector3{ 0, 0, 0, 0 };
        return true;
    }

    EXPORT bool Deformer_GetOriginalVertex(DeformerState* s, int vIdx, Vector3* orig)
    {
        if (!s) { Error_SetError(ErrorCode::NullHandle); return false; }
        if (!orig) { Error_SetError(ErrorCode::InvalidArgument); return false; }
        if (vIdx < 0 || vIdx >= s->vertCount) { Error_SetError(ErrorCode::InvalidNodeIndex); return false; }

        *orig = s->originalVerts[vIdx];
        return true;
    }
}