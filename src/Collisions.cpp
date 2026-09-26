/*
 DYNAMICENGINE3D
 AI-Assisted 3D Physics Engine
 By: Elitmers
*/

#include "EngineTypes.h"
#include "MathUtils.h"
#include "Constants.h"
#include "EngineStats.h"
#include "Collisions.h"
#include "SoftBodyInstance.h"
#include "RigidBodyInstance.h"
#include "Error.h"

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <limits>
#include <utility>
#include <atomic>

#include "ThreadPool.h"


// Contact event recording. See Collisions.h for the buffer's contract.
constexpr int MAX_CONTACT_EVENTS = 16384;

static ContactEvent g_ContactEvents[MAX_CONTACT_EVENTS];
static std::atomic<int> g_ContactEventCount{ 0 };

// Below this a contact isn't worth reporting — keeps the buffer free of settled contacts.
constexpr float CONTACT_REPORT_MIN_SPEED = 0.05f;

static inline void RecordContactEvent(int idA, int idB, const Vector3& point, const Vector3& normal, float normalSpeed, const Vector3& tangentVel)
{
    float tangentSpeed = Length(tangentVel);
    if (normalSpeed < CONTACT_REPORT_MIN_SPEED && tangentSpeed < CONTACT_REPORT_MIN_SPEED) return;

    int slot = g_ContactEventCount.fetch_add(1, std::memory_order_relaxed);
    if (slot >= MAX_CONTACT_EVENTS) { g_ContactEventCount.store(MAX_CONTACT_EVENTS, std::memory_order_relaxed); return; }

    ContactEvent& e = g_ContactEvents[slot];
    e.idA = idA; e.idB = idB;
    e.px = point.x; e.py = point.y; e.pz = point.z;
    e.nx = normal.x; e.ny = normal.y; e.nz = normal.z;
    e.normalSpeed = normalSpeed;
    e.tx = tangentVel.x; e.ty = tangentVel.y; e.tz = tangentVel.z;
}

constexpr int STATIC_CONTACT_ID = COLLISIONS_STATIC_CONTACT_ID;

// Internal static-collider data; definitions live below (after QueryStaticBVH).

enum class StaticType { Sphere, Box, Capsule, Mesh, Terrain };

struct NativeStaticCollider {
    int id;
    StaticType type;
    float frictionStatic, frictionSliding, restitution;
    int layer;

    float center[3];
    float size[3];
    float rotation[4];

    float radius;
    float height;
    float axis[3];

    std::vector<float> vertices;
    std::vector<int> indices;
    std::vector<Vector3> edgeNormals; // 3 per triangle (indices.size()), see BuildTriangleEdgeNormals
    std::vector<float> terrainHeights;
    int terrainWidth, terrainLength;
};

struct TriAABB {
    float minX, minY, minZ, maxX, maxY, maxZ;
};

struct BVHNode {
    float minX, minY, minZ, maxX, maxY, maxZ;
    int left;   // child node index, internal nodes only
    int right;  // child node index, internal nodes only
    int start;  // offset into g_StaticBVHIndices, leaf nodes only
    int count;  // primitive count, leaf nodes only (0 means internal node)
};

// 16-byte aligned Dynamic Face BVH node for SIMD AABB overlap checks
struct alignas(16) FaceBVHNode {
    float minX, minY, minZ, _pad0;
    float maxX, maxY, maxZ, _pad1;
    int left;   // child node index, or -1 if leaf
    int right;  // child node index, or -1 if leaf
    int start;  // offset into faceIndices array (if leaf)
    int count;  // primitive count (>0 if leaf, 0 if internal)
};

struct BodyFaceBVH {
    std::vector<FaceBVHNode> nodes;
    std::vector<int> faceIndices;
    const FaceData* faceDataPtr = nullptr;
    int faceCount = 0;
    bool built = false;
};

static std::unordered_map<int, BodyFaceBVH> g_BodyFaceBVHs;

std::unordered_map<int, NativeStaticCollider> g_StaticBodies;
bool g_StaticsDirty = true;

// Internal static collider data for fast collision queries
static std::vector<StaticColliderData> g_StaticColliderData;
static std::vector<TriAABB> g_StaticColliderBounds;
static std::vector<BVHNode> g_StaticBVHNodes;
static std::vector<int> g_StaticBVHIndices;
static constexpr float kMaxStaticPushoutSpeed = 4.0f; // meters/second

// The frame registry lives in Registry.cpp; bodies reach here via NativeSoftBodyData descriptors.

EXPORT void RemoveStaticBody(int id) {
    if (g_StaticBodies.erase(id)) {
        g_StaticsDirty = true;
    }
    else {
        Error_SetError(ErrorCode::UnknownHandle);
    }
}

EXPORT void AddStaticBox(int id, float* center, float* size, float* rot, float margin, float sf, float slf, float res, int layer) {
    if (!center || !size || !rot) { Error_SetError(ErrorCode::InvalidArgument); return; }
    NativeStaticCollider& sc = g_StaticBodies[id];
    sc = { id, StaticType::Box, sf, slf, res, layer };
    for (int i = 0; i < 3; ++i) { sc.center[i] = center[i]; sc.size[i] = size[i] + (margin * 2.0f); }
    for (int i = 0; i < 4; ++i) { sc.rotation[i] = rot[i]; }
    g_StaticsDirty = true;
}

EXPORT void AddStaticSphere(int id, float* center, float radius, float margin, float sf, float slf, float res, int layer) {
    if (!center) { Error_SetError(ErrorCode::InvalidArgument); return; }
    NativeStaticCollider& sc = g_StaticBodies[id];
    sc = { id, StaticType::Sphere, sf, slf, res, layer };
    for (int i = 0; i < 3; ++i) sc.center[i] = center[i];
    sc.radius = radius + margin;
    g_StaticsDirty = true;
}

EXPORT void AddStaticCapsule(int id, float* center, float radius, float height, float* axis, float margin, float sf, float slf, float res, int layer) {
    if (!center || !axis) { Error_SetError(ErrorCode::InvalidArgument); return; }
    NativeStaticCollider& sc = g_StaticBodies[id];
    sc = { id, StaticType::Capsule, sf, slf, res, layer };
    for (int i = 0; i < 3; ++i) { sc.center[i] = center[i]; sc.axis[i] = axis[i]; }
    sc.radius = radius + margin;
    sc.height = height + (margin * 2.0f);
    g_StaticsDirty = true;
}

// Per triangle edge (e: v_e -> v_(e+1)%3)

static std::vector<Vector3> BuildTriangleEdgeNormals(const float* verts, int vertCount, const int* indices, int indexCount)
{
    (void)vertCount;
    int triCount = indexCount / 3;
    std::vector<Vector3> edgeNormals(triCount * 3, V3(0.0f, 1.0f, 0.0f));
    if (triCount <= 0) return edgeNormals;

    // Snap positions to a small grid so triangles with tiny float drift share an edge key.
    constexpr float kWeldEps = 1e-4f;
    constexpr float kInvWeldEps = 1.0f / kWeldEps;
    auto quantize = [&](float v) -> int64_t { return (int64_t)std::lround((double)v * kInvWeldEps); };

    struct EdgeKey {
        int64_t ax, ay, az, bx, by, bz;
        bool operator==(const EdgeKey& o) const {
            return ax == o.ax && ay == o.ay && az == o.az && bx == o.bx && by == o.by && bz == o.bz;
        }
    };
    struct EdgeKeyHash {
        size_t operator()(const EdgeKey& k) const {
            size_t h = 1469598103934665603ull;
            auto mix = [&](int64_t v) { h ^= (size_t)v; h *= 1099511628211ull; };
            mix(k.ax); mix(k.ay); mix(k.az); mix(k.bx); mix(k.by); mix(k.bz);
            return h;
        }
    };

    auto vertPos = [&](int idx) -> Vector3 {
        return V3(verts[idx * 3], verts[idx * 3 + 1], verts[idx * 3 + 2]);
        };

    // Order the endpoints so the same physical edge hashes identically from either triangle.
    auto makeKey = [&](const Vector3& a, const Vector3& b) -> EdgeKey {
        int64_t ax = quantize(a.x), ay = quantize(a.y), az = quantize(a.z);
        int64_t bx = quantize(b.x), by = quantize(b.y), bz = quantize(b.z);
        bool swap = (ax != bx) ? (ax > bx) : ((ay != by) ? (ay > by) : (az > bz));
        if (swap) { std::swap(ax, bx); std::swap(ay, by); std::swap(az, bz); }
        return { ax, ay, az, bx, by, bz };
        };

    // Per-triangle face normal: fallback for a boundary edge, averaged for a shared one.
    std::vector<Vector3> faceNormal(triCount, V3(0.0f, 1.0f, 0.0f));
    for (int t = 0; t < triCount; ++t)
    {
        int i0 = indices[t * 3 + 0], i1 = indices[t * 3 + 1], i2 = indices[t * 3 + 2];
        Vector3 v0 = vertPos(i0), v1 = vertPos(i1), v2 = vertPos(i2);
        Vector3 n = Cross(Sub(v1, v0), Sub(v2, v0));
        float lenSq = LengthSq(n);
        faceNormal[t] = (lenSq > 1e-12f) ? Scale(n, 1.0f / std::sqrt(lenSq)) : V3(0.0f, 1.0f, 0.0f);
    }

    // Accumulate each triangle's normal onto every edge key it touches, plus the touch count.
    struct EdgeAccum { Vector3 sum; int count; };
    std::unordered_map<EdgeKey, EdgeAccum, EdgeKeyHash> edgeNormalSum;
    edgeNormalSum.reserve((size_t)triCount * 3);

    for (int t = 0; t < triCount; ++t)
    {
        int i0 = indices[t * 3 + 0], i1 = indices[t * 3 + 1], i2 = indices[t * 3 + 2];
        Vector3 tv[3] = { vertPos(i0), vertPos(i1), vertPos(i2) };
        for (int e = 0; e < 3; ++e)
        {
            EdgeKey k = makeKey(tv[e], tv[(e + 1) % 3]);
            auto it = edgeNormalSum.find(k);
            if (it == edgeNormalSum.end()) edgeNormalSum.emplace(k, EdgeAccum{ faceNormal[t], 1 });
            else { it->second.sum = Add(it->second.sum, faceNormal[t]); it->second.count++; }
        }
    }

    // Below this dot product the edge is a genuine hard corner; hard edges get a sentinel.
    constexpr float kSharpEdgeDotThreshold = 0.98f;

    for (int t = 0; t < triCount; ++t)
    {
        int i0 = indices[t * 3 + 0], i1 = indices[t * 3 + 1], i2 = indices[t * 3 + 2];
        Vector3 tv[3] = { vertPos(i0), vertPos(i1), vertPos(i2) };
        for (int e = 0; e < 3; ++e)
        {
            const EdgeAccum& acc = edgeNormalSum[makeKey(tv[e], tv[(e + 1) % 3])];

            if (acc.count >= 2)
            {
                // |n1+n2|^2 = 2 + 2*dot(n1,n2) for two unit vectors.
                float approxDot = (LengthSq(acc.sum) * 0.5f) - 1.0f;
                if (approxDot < kSharpEdgeDotThreshold)
                {
                    edgeNormals[t * 3 + e] = V3Zero(); // sentinel: sharp edge, use radial normal instead
                    continue;
                }
            }

            float lenSq = LengthSq(acc.sum);
            // Only vanishes for a zero-thickness fold — use this triangle's own normal.
            edgeNormals[t * 3 + e] = (lenSq > 1e-12f) ? Scale(acc.sum, 1.0f / std::sqrt(lenSq)) : faceNormal[t];
        }
    }

    return edgeNormals;
}

EXPORT void AddStaticMesh(int id, float* center, float* verts, int vertCount, int* indices, int indexCount, float margin, float sf, float slf, float res, int layer) {
    if (!center || !verts || vertCount <= 0 || !indices || indexCount <= 0) { Error_SetError(ErrorCode::InvalidArgument); return; }
    NativeStaticCollider& sc = g_StaticBodies[id];
    sc = { id, StaticType::Mesh, sf, slf, res, layer };
    for (int i = 0; i < 3; ++i) sc.center[i] = center[i];

    sc.vertices.assign(verts, verts + (vertCount * 3));
    sc.indices.assign(indices, indices + indexCount);
    sc.edgeNormals = BuildTriangleEdgeNormals(verts, vertCount, indices, indexCount);

    if (margin > 0.0f && vertCount > 0) {
        // Accumulate face normals per WELDED POSITION
        constexpr float kWeldEps = 1e-4f;
        constexpr float kInvWeldEps = 1.0f / kWeldEps;
        auto quantize = [&](float v) -> int64_t { return (int64_t)std::lround((double)v * kInvWeldEps); };
        struct PosKey {
            int64_t x, y, z;
            bool operator==(const PosKey& o) const { return x == o.x && y == o.y && z == o.z; }
        };
        struct PosKeyHash {
            size_t operator()(const PosKey& k) const {
                size_t h = 1469598103934665603ull;
                auto mix = [&](int64_t v) { h ^= (size_t)v; h *= 1099511628211ull; };
                mix(k.x); mix(k.y); mix(k.z);
                return h;
            }
        };
        auto posKeyOf = [&](int v) -> PosKey {
            return { quantize(sc.vertices[v * 3 + 0]), quantize(sc.vertices[v * 3 + 1]), quantize(sc.vertices[v * 3 + 2]) };
            };

        std::vector<Vector3> normals(vertCount, V3Zero());
        int triCount = indexCount / 3;
        for (int t = 0; t < triCount; ++t) {
            int i0 = sc.indices[t * 3 + 0], i1 = sc.indices[t * 3 + 1], i2 = sc.indices[t * 3 + 2];
            Vector3 v0 = V3(sc.vertices[i0 * 3], sc.vertices[i0 * 3 + 1], sc.vertices[i0 * 3 + 2]);
            Vector3 v1 = V3(sc.vertices[i1 * 3], sc.vertices[i1 * 3 + 1], sc.vertices[i1 * 3 + 2]);
            Vector3 v2 = V3(sc.vertices[i2 * 3], sc.vertices[i2 * 3 + 1], sc.vertices[i2 * 3 + 2]);
            Vector3 faceN = Cross(Sub(v1, v0), Sub(v2, v0)); // unnormalized, so bigger triangles weigh more
            normals[i0] = Add(normals[i0], faceN);
            normals[i1] = Add(normals[i1], faceN);
            normals[i2] = Add(normals[i2], faceN);
        }

        // Merge each vertex's sum onto its welded position, so all duplicates see every triangle.
        std::unordered_map<PosKey, Vector3, PosKeyHash> weldedNormalSum;
        weldedNormalSum.reserve((size_t)vertCount);
        for (int v = 0; v < vertCount; ++v) {
            PosKey k = posKeyOf(v);
            auto it = weldedNormalSum.find(k);
            if (it == weldedNormalSum.end()) weldedNormalSum.emplace(k, normals[v]);
            else it->second = Add(it->second, normals[v]);
        }

        for (int v = 0; v < vertCount; ++v) {
            const Vector3& sum = weldedNormalSum[posKeyOf(v)];
            float lenSq = LengthSq(sum);
            if (lenSq > 1e-12f) {
                Vector3 n = Scale(sum, 1.0f / std::sqrt(lenSq));
                sc.vertices[v * 3 + 0] += n.x * margin;
                sc.vertices[v * 3 + 1] += n.y * margin;
                sc.vertices[v * 3 + 2] += n.z * margin;
            }
        }
    }

    g_StaticsDirty = true;
}

// Builds one BVH node over indices[start, start+count) and recurses postorder (root last).
static constexpr int kBVHLeafSize = 4;

static int BuildStaticBVHNode(std::vector<int>& indices, int start, int count,
    const std::vector<TriAABB>& bounds, const std::vector<Vector3>& centroids,
    std::vector<BVHNode>& nodes)
{
    float minX = 1e30f, minY = 1e30f, minZ = 1e30f;
    float maxX = -1e30f, maxY = -1e30f, maxZ = -1e30f;
    for (int i = 0; i < count; ++i) {
        const TriAABB& b = bounds[indices[start + i]];
        minX = MinF(minX, b.minX); minY = MinF(minY, b.minY); minZ = MinF(minZ, b.minZ);
        maxX = MaxF(maxX, b.maxX); maxY = MaxF(maxY, b.maxY); maxZ = MaxF(maxZ, b.maxZ);
    }

    BVHNode node{};
    node.minX = minX; node.minY = minY; node.minZ = minZ;
    node.maxX = maxX; node.maxY = maxY; node.maxZ = maxZ;

    if (count <= kBVHLeafSize) {
        node.left = -1; node.right = -1;
        node.start = start; node.count = count;
        nodes.push_back(node);
        return (int)nodes.size() - 1;
    }

    // Object median split on the widest centroid axis: cheap, balanced, no SAH evaluation.
    float cx0 = 1e30f, cy0 = 1e30f, cz0 = 1e30f;
    float cx1 = -1e30f, cy1 = -1e30f, cz1 = -1e30f;
    for (int i = 0; i < count; ++i) {
        const Vector3& c = centroids[indices[start + i]];
        cx0 = MinF(cx0, c.x); cy0 = MinF(cy0, c.y); cz0 = MinF(cz0, c.z);
        cx1 = MaxF(cx1, c.x); cy1 = MaxF(cy1, c.y); cz1 = MaxF(cz1, c.z);
    }
    float ex = cx1 - cx0, ey = cy1 - cy0, ez = cz1 - cz0;
    int axis = (ex > ey) ? ((ex > ez) ? 0 : 2) : ((ey > ez) ? 1 : 2);

    auto axisVal = [&](int idx) {
        const Vector3& c = centroids[idx];
        return axis == 0 ? c.x : (axis == 1 ? c.y : c.z);
        };

    int mid = start + count / 2;
    std::nth_element(indices.begin() + start, indices.begin() + mid, indices.begin() + start + count,
        [&](int a, int b) { return axisVal(a) < axisVal(b); });

    int leftIdx = BuildStaticBVHNode(indices, start, mid - start, bounds, centroids, nodes);
    int rightIdx = BuildStaticBVHNode(indices, mid, start + count - mid, bounds, centroids, nodes);

    node.left = leftIdx; node.right = rightIdx;
    node.start = 0; node.count = 0;
    nodes.push_back(node);
    return (int)nodes.size() - 1;
}

static void BuildStaticBVH()
{
    g_StaticBVHNodes.clear();
    g_StaticBVHIndices.clear();

    const int totalColliders = (int)g_StaticColliderData.size();
    if (totalColliders <= 0) return;

    g_StaticBVHIndices.resize(totalColliders);
    std::vector<Vector3> centroids(totalColliders);
    for (int i = 0; i < totalColliders; ++i) {
        g_StaticBVHIndices[i] = i;
        const TriAABB& b = g_StaticColliderBounds[i];
        centroids[i] = V3((b.minX + b.maxX) * 0.5f, (b.minY + b.maxY) * 0.5f, (b.minZ + b.maxZ) * 0.5f);
    }

    g_StaticBVHNodes.reserve((size_t)totalColliders * 2);
    BuildStaticBVHNode(g_StaticBVHIndices, 0, totalColliders, g_StaticColliderBounds, centroids, g_StaticBVHNodes);
}

// Iterative stack-walk query: each overlapping leaf's primitive reaches fn exactly once.
template <class Fn>
static inline void QueryStaticBVH(const Vector3& qmin, const Vector3& qmax, Fn&& fn)
{
    if (g_StaticBVHNodes.empty()) return;

    int stack[64];
    int sp = 0;
    stack[sp++] = (int)g_StaticBVHNodes.size() - 1; // postorder build -> root is last

    while (sp > 0)
    {
        const BVHNode& node = g_StaticBVHNodes[stack[--sp]];

        if (qmax.x < node.minX || qmin.x > node.maxX ||
            qmax.y < node.minY || qmin.y > node.maxY ||
            qmax.z < node.minZ || qmin.z > node.maxZ)
            continue;

        if (node.count > 0)
        {
            for (int i = 0; i < node.count; ++i)
            {
                const int colliderIndex = g_StaticBVHIndices[node.start + i];
                const TriAABB& bounds = g_StaticColliderBounds[colliderIndex];
                if (qmax.x < bounds.minX || qmin.x > bounds.maxX ||
                    qmax.y < bounds.minY || qmin.y > bounds.maxY ||
                    qmax.z < bounds.minZ || qmin.z > bounds.maxZ)
                    continue;
                fn(colliderIndex);
            }
        }
        else if (sp < 62)
        {
            stack[sp++] = node.left;
            stack[sp++] = node.right;
        }
    }
}

// Fast vectorized 3D AABB overlap check using SSE intrinsics
static inline bool AABBOverlapSIMD(const __m128 qMin, const __m128 qMax, const float* bMinPtr, const float* bMaxPtr)
{
    const __m128 bMin = _mm_loadu_ps(bMinPtr);
    const __m128 bMax = _mm_loadu_ps(bMaxPtr);
    const __m128 noOverlap = _mm_or_ps(_mm_cmplt_ps(qMax, bMin), _mm_cmpgt_ps(qMin, bMax));
    return (_mm_movemask_ps(noOverlap) & 7) == 0;
}

static constexpr int kFaceBVHLeafSize = 2;

static int BuildFaceBVHNode(
    std::vector<int>& indices, int start, int count,
    const std::vector<TriAABB>& bounds, const std::vector<Vector3>& centroids,
    std::vector<FaceBVHNode>& nodes)
{
    float minX = 1e30f, minY = 1e30f, minZ = 1e30f;
    float maxX = -1e30f, maxY = -1e30f, maxZ = -1e30f;
    for (int i = 0; i < count; ++i) {
        const TriAABB& b = bounds[indices[start + i]];
        minX = MinF(minX, b.minX); minY = MinF(minY, b.minY); minZ = MinF(minZ, b.minZ);
        maxX = MaxF(maxX, b.maxX); maxY = MaxF(maxY, b.maxY); maxZ = MaxF(maxZ, b.maxZ);
    }

    FaceBVHNode node{};
    node.minX = minX; node.minY = minY; node.minZ = minZ; node._pad0 = 0.0f;
    node.maxX = maxX; node.maxY = maxY; node.maxZ = maxZ; node._pad1 = 0.0f;

    if (count <= kFaceBVHLeafSize) {
        node.left = -1; node.right = -1;
        node.start = start; node.count = count;
        nodes.push_back(node);
        return (int)nodes.size() - 1;
    }

    float cx0 = 1e30f, cy0 = 1e30f, cz0 = 1e30f;
    float cx1 = -1e30f, cy1 = -1e30f, cz1 = -1e30f;
    for (int i = 0; i < count; ++i) {
        const Vector3& c = centroids[indices[start + i]];
        cx0 = MinF(cx0, c.x); cy0 = MinF(cy0, c.y); cz0 = MinF(cz0, c.z);
        cx1 = MaxF(cx1, c.x); cy1 = MaxF(cy1, c.y); cz1 = MaxF(cz1, c.z);
    }
    float ex = cx1 - cx0, ey = cy1 - cy0, ez = cz1 - cz0;
    int axis = (ex > ey) ? ((ex > ez) ? 0 : 2) : ((ey > ez) ? 1 : 2);

    auto axisVal = [&](int idx) {
        const Vector3& c = centroids[idx];
        return axis == 0 ? c.x : (axis == 1 ? c.y : c.z);
        };

    int mid = start + count / 2;
    std::nth_element(indices.begin() + start, indices.begin() + mid, indices.begin() + start + count,
        [&](int a, int b) { return axisVal(a) < axisVal(b); });

    int leftIdx = BuildFaceBVHNode(indices, start, mid - start, bounds, centroids, nodes);
    int rightIdx = BuildFaceBVHNode(indices, mid, start + count - mid, bounds, centroids, nodes);

    node.left = leftIdx; node.right = rightIdx;
    node.start = 0; node.count = 0;
    nodes.push_back(node);
    return (int)nodes.size() - 1;
}

static void BuildBodyFaceBVH(BodyFaceBVH& bvh, const NativeSoftBodyData& body)
{
    bvh.nodes.clear();
    bvh.faceIndices.clear();
    bvh.faceCount = body.faceCount;
    bvh.faceDataPtr = body.faces;
    bvh.built = false;

    if (body.faceCount <= 0 || !body.faces || body.nodeCount <= 0) return;

    bvh.faceIndices.resize(body.faceCount);
    std::vector<TriAABB> bounds(body.faceCount);
    std::vector<Vector3> centroids(body.faceCount);

    const float triThickness = PhysicsConstants::DEFAULT_TRIANGLE_THICKNESS;
    const FaceData* faces = body.faces;
    const float* cx = body.currentX;
    const float* cy = body.currentY;
    const float* cz = body.currentZ;
    const float* px = body.predictedX;
    const float* py = body.predictedY;
    const float* pz = body.predictedZ;

    for (int i = 0; i < body.faceCount; ++i)
    {
        bvh.faceIndices[i] = i;
        const FaceData& f = faces[i];
        const int i0 = f.nodeA, i1 = f.nodeB, i2 = f.nodeC;

        float minX = MinF(MinF(cx[i0], px[i0]), MinF(MinF(cx[i1], px[i1]), MinF(cx[i2], px[i2])));
        float minY = MinF(MinF(cy[i0], py[i0]), MinF(MinF(cy[i1], py[i1]), MinF(cy[i2], py[i2])));
        float minZ = MinF(MinF(cz[i0], pz[i0]), MinF(MinF(cz[i1], pz[i1]), MinF(cz[i2], pz[i2])));
        float maxX = MaxF(MaxF(cx[i0], px[i0]), MaxF(MaxF(cx[i1], px[i1]), MaxF(cx[i2], px[i2])));
        float maxY = MaxF(MaxF(cy[i0], py[i0]), MaxF(MaxF(cy[i1], py[i1]), MaxF(cy[i2], py[i2])));
        float maxZ = MaxF(MaxF(cz[i0], pz[i0]), MaxF(MaxF(cz[i1], pz[i1]), MaxF(cz[i2], pz[i2])));

        bounds[i] = TriAABB{ minX - triThickness, minY - triThickness, minZ - triThickness,
                             maxX + triThickness, maxY + triThickness, maxZ + triThickness };
        centroids[i] = V3((minX + maxX) * 0.5f, (minY + maxY) * 0.5f, (minZ + maxZ) * 0.5f);
    }

    bvh.nodes.reserve((size_t)body.faceCount * 2);
    BuildFaceBVHNode(bvh.faceIndices, 0, body.faceCount, bounds, centroids, bvh.nodes);
    bvh.built = true;
}

static void RefitFaceBVH(BodyFaceBVH& bvh, const NativeSoftBodyData& body)
{
    if (!bvh.built || bvh.nodes.empty()) return;

    const FaceData* faces = body.faces;
    const float* cx = body.currentX;
    const float* cy = body.currentY;
    const float* cz = body.currentZ;
    const float* px = body.predictedX;
    const float* py = body.predictedY;
    const float* pz = body.predictedZ;
    const int* faceIndices = bvh.faceIndices.data();
    FaceBVHNode* nodes = bvh.nodes.data();
    const int nodeCount = (int)bvh.nodes.size();

    const float triThickness = PhysicsConstants::DEFAULT_TRIANGLE_THICKNESS;

    for (int i = 0; i < nodeCount; ++i)
    {
        FaceBVHNode& node = nodes[i];
        if (node.count > 0)
        {
            float minX = 1e30f, minY = 1e30f, minZ = 1e30f;
            float maxX = -1e30f, maxY = -1e30f, maxZ = -1e30f;

            for (int k = 0; k < node.count; ++k)
            {
                const int faceIdx = faceIndices[node.start + k];
                const FaceData& f = faces[faceIdx];
                const int i0 = f.nodeA, i1 = f.nodeB, i2 = f.nodeC;

                minX = MinF(minX, MinF(MinF(cx[i0], px[i0]), MinF(MinF(cx[i1], px[i1]), MinF(cx[i2], px[i2]))));
                minY = MinF(minY, MinF(MinF(cy[i0], py[i0]), MinF(MinF(cy[i1], py[i1]), MinF(cy[i2], py[i2]))));
                minZ = MinF(minZ, MinF(MinF(cz[i0], pz[i0]), MinF(MinF(cz[i1], pz[i1]), MinF(cz[i2], pz[i2]))));
                maxX = MaxF(maxX, MaxF(MaxF(cx[i0], px[i0]), MaxF(MaxF(cx[i1], px[i1]), MaxF(cx[i2], px[i2]))));
                maxY = MaxF(maxY, MaxF(MaxF(cy[i0], py[i0]), MaxF(MaxF(cy[i1], py[i1]), MaxF(cy[i2], py[i2]))));
                maxZ = MaxF(maxZ, MaxF(MaxF(cz[i0], pz[i0]), MaxF(MaxF(cz[i1], pz[i1]), MaxF(cz[i2], pz[i2]))));
            }

            node.minX = minX - triThickness;
            node.minY = minY - triThickness;
            node.minZ = minZ - triThickness;
            node.maxX = maxX + triThickness;
            node.maxY = maxY + triThickness;
            node.maxZ = maxZ + triThickness;
        }
        else
        {
            const FaceBVHNode& left = nodes[node.left];
            const FaceBVHNode& right = nodes[node.right];

            node.minX = MinF(left.minX, right.minX);
            node.minY = MinF(left.minY, right.minY);
            node.minZ = MinF(left.minZ, right.minZ);
            node.maxX = MaxF(left.maxX, right.maxX);
            node.maxY = MaxF(left.maxY, right.maxY);
            node.maxZ = MaxF(left.maxZ, right.maxZ);
        }
    }
}

EXPORT void AddStaticTerrain(int id, float* center, float* size, float* heights, int width, int length, float margin, float sf, float slf, float res, int layer) {
    if (!center || !size || !heights || width <= 0 || length <= 0) { Error_SetError(ErrorCode::InvalidArgument); return; }
    NativeStaticCollider& sc = g_StaticBodies[id];
    sc = { id, StaticType::Terrain, sf, slf, res, layer };
    for (int i = 0; i < 3; ++i) { sc.center[i] = center[i]; sc.size[i] = size[i]; }
    sc.terrainWidth = width;
    sc.terrainLength = length;
    sc.terrainHeights.assign(heights, heights + (width * length));
    g_StaticsDirty = true;
}

// Set when the static set is rebuilt; the rigid layer consumes it to wake frozen bodies.
static bool g_StaticsChangedWake = false;

EXPORT void ClearStaticColliders() {
    g_StaticBodies.clear();
    g_StaticColliderData.clear();
    g_StaticColliderBounds.clear();
    g_StaticBVHNodes.clear();
    g_StaticBVHIndices.clear();
    g_StaticsDirty = true;
    g_StaticsChangedWake = true;
}

EXPORT void ProcessDirtyStatics() {
    if (!g_StaticsDirty) return;
    g_StaticsDirty = false;
    g_StaticsChangedWake = true;

    // Convert NativeStaticCollider -> StaticColliderData
    g_StaticColliderData.clear();
    g_StaticColliderBounds.clear();

    size_t totalExpected = 0;
    for (const auto& kv : g_StaticBodies) {
        if (kv.second.type == StaticType::Mesh) {
            totalExpected += kv.second.indices.size() / 3;
        } else {
            totalExpected++;
        }
    }
    g_StaticColliderData.reserve(totalExpected);
    g_StaticColliderBounds.reserve(totalExpected);

    for (auto& kv : g_StaticBodies) {
        const NativeStaticCollider& src = kv.second;
        StaticColliderData dst = {};
        dst.center = Vector3{ src.center[0], src.center[1], src.center[2], 0.0f };
        dst.staticFriction = src.frictionStatic;
        dst.slidingFriction = src.frictionSliding;
        dst.restitution = src.restitution;
        dst.layer = src.layer;

        switch (src.type) {
        case StaticType::Sphere:
            dst.type = COL_SPHERE;
            dst.radius = src.radius;
            g_StaticColliderData.push_back(dst);
            break;
        case StaticType::Box:
            dst.type = COL_BOX;
            dst.size = Vector3{ src.size[0], src.size[1], src.size[2], 0.0f };
            dst.rotation = Quaternion{ src.rotation[0], src.rotation[1], src.rotation[2], src.rotation[3] };
            g_StaticColliderData.push_back(dst);
            break;
        case StaticType::Capsule:
            dst.type = COL_CAPSULE;
            dst.radius = src.radius;
            dst.height = src.height;
            dst.axis = Vector3{ src.axis[0], src.axis[1], src.axis[2], 0.0f };
            g_StaticColliderData.push_back(dst);
            break;
        case StaticType::Mesh:
            dst.type = COL_TRIANGLE;
            for (size_t i = 0; i < src.indices.size(); i += 3) {
                int i0 = src.indices[i], i1 = src.indices[i + 1], i2 = src.indices[i + 2];
                Vector3 v0 = Vector3{ src.vertices[i0 * 3], src.vertices[i0 * 3 + 1], src.vertices[i0 * 3 + 2], 0.0f };
                Vector3 v1 = Vector3{ src.vertices[i1 * 3], src.vertices[i1 * 3 + 1], src.vertices[i1 * 3 + 2], 0.0f };
                Vector3 v2 = Vector3{ src.vertices[i2 * 3], src.vertices[i2 * 3 + 1], src.vertices[i2 * 3 + 2], 0.0f };
                dst.v0 = v0; dst.v1 = v1; dst.v2 = v2;
                size_t triIdx = i / 3;
                Vector3 en0 = (triIdx * 3 + 0 < src.edgeNormals.size()) ? src.edgeNormals[triIdx * 3 + 0] : V3(0.0f, 1.0f, 0.0f);
                Vector3 en1 = (triIdx * 3 + 1 < src.edgeNormals.size()) ? src.edgeNormals[triIdx * 3 + 1] : V3(0.0f, 1.0f, 0.0f);
                Vector3 en2 = (triIdx * 3 + 2 < src.edgeNormals.size()) ? src.edgeNormals[triIdx * 3 + 2] : V3(0.0f, 1.0f, 0.0f);
                // Triangle statics piggyback their 3 precomputed edge normals on size/rotation/axis.
                dst.size = Vector3{ en0.x, en0.y, en0.z, 0.0f };
                dst.axis = Vector3{ en1.x, en1.y, en1.z, 0.0f };
                dst.rotation = Quaternion{ en2.x, en2.y, en2.z, 0.0f };
                g_StaticColliderData.push_back(dst);
            }
            break;
        case StaticType::Terrain:
            dst.type = COL_TERRAIN;
            dst.size = Vector3{ src.size[0], src.size[1], src.size[2], 0.0f };
            dst.terrainWidth = src.terrainWidth;
            dst.terrainLength = src.terrainLength;
            dst.terrainHeights = const_cast<float*>(src.terrainHeights.data()); // careful: we need to ensure lifetime
            g_StaticColliderData.push_back(dst);
            break;
        }
    }

    // --- Pass 1: AABBs ---
    int totalColliders = (int)g_StaticColliderData.size();
    g_StaticColliderBounds.resize(totalColliders);

    for (int i = 0; i < totalColliders; ++i) {
        StaticColliderData& c = g_StaticColliderData[i];
        Vector3 cMin, cMax;

        if (c.type == COL_TRIANGLE) {
            cMin = Vector3{ MinF(c.v0.x, MinF(c.v1.x, c.v2.x)), MinF(c.v0.y, MinF(c.v1.y, c.v2.y)), MinF(c.v0.z, MinF(c.v1.z, c.v2.z)), 0.0f };
            cMax = Vector3{ MaxF(c.v0.x, MaxF(c.v1.x, c.v2.x)), MaxF(c.v0.y, MaxF(c.v1.y, c.v2.y)), MaxF(c.v0.z, MaxF(c.v1.z, c.v2.z)), 0.0f };
        }
        else if (c.type == COL_SPHERE) {
            Vector3 r3 = { c.radius, c.radius, c.radius, 0.0f };
            cMin = Sub(c.center, r3);
            cMax = Add(c.center, r3);
        }
        else if (c.type == COL_BOX) {
            // Exact AABB of an oriented box: project each local half-axis onto every world axis.
            Vector3 halfSize = Scale(c.size, 0.5f);
            Vector3 ax = QRotate(c.rotation, V3(1.0f, 0.0f, 0.0f));
            Vector3 ay = QRotate(c.rotation, V3(0.0f, 1.0f, 0.0f));
            Vector3 az = QRotate(c.rotation, V3(0.0f, 0.0f, 1.0f));
            Vector3 extent = {
                AbsF(ax.x) * halfSize.x + AbsF(ay.x) * halfSize.y + AbsF(az.x) * halfSize.z,
                AbsF(ax.y) * halfSize.x + AbsF(ay.y) * halfSize.y + AbsF(az.y) * halfSize.z,
                AbsF(ax.z) * halfSize.x + AbsF(ay.z) * halfSize.y + AbsF(az.z) * halfSize.z,
                0.0f
            };
            cMin = Sub(c.center, extent);
            cMax = Add(c.center, extent);
        }
        else if (c.type == COL_CAPSULE) {
            // Cylinder half-length projected per axis, plus the radius.
            float cylHalfLen = MaxF(0.0f, c.height * 0.5f - c.radius);
            Vector3 extent = {
                AbsF(c.axis.x) * cylHalfLen + c.radius,
                AbsF(c.axis.y) * cylHalfLen + c.radius,
                AbsF(c.axis.z) * cylHalfLen + c.radius,
                0.0f
            };
            cMin = Sub(c.center, extent);
            cMax = Add(c.center, extent);
        }
        else {
            Vector3 half = Scale(c.size, 0.5f);
            cMin = Sub(c.center, half);
            cMax = Add(c.center, half);
        }

        g_StaticColliderBounds[i] = { cMin.x, cMin.y, cMin.z, cMax.x, cMax.y, cMax.z };
    }

    // --- Pass 2: build the BVH over every static collider, small or huge ---
    BuildStaticBVH();

    g_Stats.totalStaticColliders = totalColliders;
    g_Stats.staticOversizedColliders = 0; // no size-based fallback bucket anymore
    g_Stats.staticGridRebuilt = 1;
}

#define VEC3(ARR, IDX) Vector3({ARR##X[IDX], ARR##Y[IDX], ARR##Z[IDX], 0.0f})
struct NodeContact {
    int a;   // local node index in body A
    int b;   // local node index in body B
};

struct KeyVal {
    int key;
    int value;
    bool operator<(const KeyVal& o) const { return key < o.key; }
};

// Cell -> node-run lookup over the sorted flat hash.
struct CellTable
{
    std::vector<KeyVal> entries;   // sorted by key; value is the node index
    std::vector<int>    slotKey;
    std::vector<int>    slotStart; // -1 marks an empty slot
    std::vector<int>    slotCount;
    int                 mask = 0;

    bool Empty() const { return entries.empty(); }

    // entries must already be sorted by key.
    void BuildIndex()
    {
        const int n = (int)entries.size();
        if (n == 0) { mask = 0; return; }

        // Load factor ≤ 0.5, so linear probing terminates on the first empty slot.
        int cap = 16;
        while (cap < n * 2) cap <<= 1;
        mask = cap - 1;

        slotKey.assign(cap, 0);
        slotStart.assign(cap, -1);
        slotCount.assign(cap, 0);

        int i = 0;
        while (i < n)
        {
            const int key = entries[i].key;
            int j = i + 1;
            while (j < n && entries[j].key == key) ++j;

            int slot = (int)((uint32_t)key * 2654435761u) & mask;
            while (slotStart[slot] >= 0) slot = (slot + 1) & mask;
            slotKey[slot] = key;
            slotStart[slot] = i;
            slotCount[slot] = j - i;

            i = j;
        }
    }

    template <class Fn>
    void ForEachInCell(int key, Fn&& fn) const
    {
        if (mask == 0) return;
        int slot = (int)((uint32_t)key * 2654435761u) & mask;
        while (slotStart[slot] >= 0)
        {
            if (slotKey[slot] == key)
            {
                const int start = slotStart[slot];
                const int count = slotCount[slot];
                for (int k = 0; k < count; ++k) fn(entries[start + k].value);
                return;
            }
            slot = (slot + 1) & mask;
        }
    }
};

static inline int ClampI(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ===========================================================================
//  UniformNodeGrid - shared broadphase for node-node contacts
//
//  One grid for the whole scene, rebuilt every substep: nodes are binned at the midpoint of
//  their current -> predicted sweep by a counting sort (O(N), no hashing, sequential writes).
//  A query probes the 3x3x3 block around its own cell — sufficient because the cell size is
//  re-derived each frame to cover the widest pair reach, so cells two apart are always further
//  apart than any tested pair can span. A node lives in exactly one cell, so the walk visits
//  each candidate once (the old swept walk needed dedup). Cell coords clamp inward, which can
//  only add false positives, never miss a pair.
// ===========================================================================

struct GridEntry
{
    int body;   // index into this substep's bodies[] array
    int node;   // node index within that body
};

struct UniformNodeGrid
{
    float originX = 0.0f, originY = 0.0f, originZ = 0.0f;
    float cellSize = 1.0f;
    float invCellSize = 1.0f;
    int   dimX = 0, dimY = 0, dimZ = 0;
    int   totalCells = 0;

    std::vector<int>       cellStart;  // totalCells + 1 prefix sums into entries
    std::vector<GridEntry> entries;    // one record per binned node, grouped by cell

    bool Empty() const { return entries.empty(); }

    inline int CellIndex(int cx, int cy, int cz) const
    {
        return cx + dimX * (cy + dimY * cz);
    }

    inline int CellCoord(float v, float origin, int dim) const
    {
        const float t = (v - origin) * invCellSize;
        if (!(t > -1.0e8f)) return 0;        // NaN or far below the domain
        if (t > 1.0e8f)     return dim - 1;  // far above the domain
        const int c = FloorToInt(t);
        return c < 0 ? 0 : (c >= dim ? dim - 1 : c);
    }
};

static UniformNodeGrid g_NodeGrid;                     // rebuilt every substep
static std::vector<std::vector<float>> g_NodeHalfTraj; // per-node |pred - curr| * 0.5

// Build scratch; Collisions_ResolveAll is single-threaded, and it persists across substeps.
static std::vector<int> s_GridCellOfNode; // per binned node: cell id, -1 = skip
static std::vector<int> s_GridCellCursor; // scatter cursors

static void BuildUniformNodeGrid(
    NativeSoftBodyData* bodies, int bodyCount,
    const uint8_t* bodyInActivePair,   // bodies that can actually collide this substep
    float activeMaxBodyRadius,         // max radius over those bodies
    float bpEpsilon)                   // epsilon + BROADPHASE_MARGIN
{
    g_NodeGrid.entries.clear();
    g_NodeGrid.cellStart.clear();

    if (bodyCount <= 0) return;
    if ((int)g_NodeHalfTraj.size() < bodyCount) g_NodeHalfTraj.resize(bodyCount);

    const auto bodyActive = [&](int b)
        {
            return !bodyInActivePair || bodyInActivePair[b] != 0;
        };

    // ---- Pass 1: per-node half sweep, domain bounds, widest active sweep ----
    bool anyValid = false;
    float minX = 0.0f, minY = 0.0f, minZ = 0.0f;
    float maxX = 0.0f, maxY = 0.0f, maxZ = 0.0f;
    float maxActiveHalfSweep = 0.0f;
    int totalNodes = 0;

    for (int b = 0; b < bodyCount; ++b)
    {
        if (!bodyActive(b)) continue;

        NativeSoftBodyData& body = bodies[b];
        const int n = body.nodeCount;
        if (n <= 0) { g_NodeHalfTraj[b].clear(); continue; }

        totalNodes += n;
        std::vector<float>& ht = g_NodeHalfTraj[b];
        ht.resize((size_t)n);

        for (int i = 0; i < n; ++i)
        {
            // Must match the query midpoint bit-for-bit: cells come from the same value.
            const float mx = (body.currentX[i] + body.predictedX[i]) * 0.5f;
            const float my = (body.currentY[i] + body.predictedY[i]) * 0.5f;
            const float mz = (body.currentZ[i] + body.predictedZ[i]) * 0.5f;

            if (!std::isfinite(mx) || !std::isfinite(my) || !std::isfinite(mz))
            {
                ht[i] = 0.0f;  // never binned; nothing sane to collide with
                continue;
            }

            if (!anyValid)
            {
                minX = maxX = mx; minY = maxY = my; minZ = maxZ = mz;
                anyValid = true;
            }
            else
            {
                if (mx < minX) minX = mx; if (mx > maxX) maxX = mx;
                if (my < minY) minY = my; if (my > maxY) maxY = my;
                if (mz < minZ) minZ = mz; if (mz > maxZ) maxZ = mz;
            }

            const float dx = body.predictedX[i] - body.currentX[i];
            const float dy = body.predictedY[i] - body.currentY[i];
            const float dz = body.predictedZ[i] - body.currentZ[i];
            float hl = 0.5f * std::sqrt(dx * dx + dy * dy + dz * dz);
            if (!std::isfinite(hl)) hl = 0.0f;
            ht[i] = hl;

            if (hl > maxActiveHalfSweep) maxActiveHalfSweep = hl;
        }
    }

    if (!anyValid || totalNodes <= 0) return;

    // ---- Cell size: must cover the widest pair reach; the 1.0001 dodges boundary rounding.
    const float maxReach = 2.0f * activeMaxBodyRadius + bpEpsilon + 2.0f * maxActiveHalfSweep;
    g_NodeGrid.cellSize = MaxF(PhysicsConstants::SPATIAL_CELL_SIZE, maxReach * 1.0001f);
    g_NodeGrid.invCellSize = 1.0f / g_NodeGrid.cellSize;

    // Half a cell of padding so boundary nodes land in interior cells, not on the clamp.
    g_NodeGrid.originX = minX - 0.5f * g_NodeGrid.cellSize;
    g_NodeGrid.originY = minY - 0.5f * g_NodeGrid.cellSize;
    g_NodeGrid.originZ = minZ - 0.5f * g_NodeGrid.cellSize;

    constexpr int kMaxGridDim = 4096;
    auto dimFor = [&](float ext)
        {
            float e = ext * g_NodeGrid.invCellSize;
            if (!(e < 1.0e9f)) e = 1.0e9f;  // also catches NaN
            if (e < 0.0f) e = 0.0f;
            return ClampI((int)std::ceil(e) + 1, 1, kMaxGridDim);
        };
    g_NodeGrid.dimX = dimFor(maxX - g_NodeGrid.originX);
    g_NodeGrid.dimY = dimFor(maxY - g_NodeGrid.originY);
    g_NodeGrid.dimZ = dimFor(maxZ - g_NodeGrid.originZ);

    // Grid memory stays proportional to binned nodes — false positives only, never missed pairs.
    {
        int64_t cellBudget = (int64_t)totalNodes * 4;
        if (cellBudget < 64) cellBudget = 64;
        if (cellBudget > (int64_t)(1 << 22)) cellBudget = (int64_t)(1 << 22);

        while ((int64_t)g_NodeGrid.dimX * g_NodeGrid.dimY * g_NodeGrid.dimZ > cellBudget)
        {
            if (g_NodeGrid.dimX >= g_NodeGrid.dimY && g_NodeGrid.dimX >= g_NodeGrid.dimZ)
            {
                if (g_NodeGrid.dimX <= 1) break; g_NodeGrid.dimX = (g_NodeGrid.dimX + 1) / 2;
            }
            else if (g_NodeGrid.dimY >= g_NodeGrid.dimZ)
            {
                if (g_NodeGrid.dimY <= 1) break; g_NodeGrid.dimY = (g_NodeGrid.dimY + 1) / 2;
            }
            else
            {
                if (g_NodeGrid.dimZ <= 1) break; g_NodeGrid.dimZ = (g_NodeGrid.dimZ + 1) / 2;
            }
        }
    }

    g_NodeGrid.totalCells = g_NodeGrid.dimX * g_NodeGrid.dimY * g_NodeGrid.dimZ;
    if (g_NodeGrid.totalCells <= 0) return;

    // ---- Pass 2: histogram + record each node's cell ----
    g_NodeGrid.cellStart.assign((size_t)g_NodeGrid.totalCells + 1, 0);
    s_GridCellOfNode.resize((size_t)totalNodes);

    int* cellCount = g_NodeGrid.cellStart.data() + 1; // occupancy of cell c lives at [c + 1]
    int validNodes = 0;
    int idx = 0;

    for (int b = 0; b < bodyCount; ++b)
    {
        if (!bodyActive(b)) continue;
        const NativeSoftBodyData& body = bodies[b];

        for (int i = 0; i < body.nodeCount; ++i)
        {
            const float mx = (body.currentX[i] + body.predictedX[i]) * 0.5f;
            const float my = (body.currentY[i] + body.predictedY[i]) * 0.5f;
            const float mz = (body.currentZ[i] + body.predictedZ[i]) * 0.5f;

            int c = -1;
            if (std::isfinite(mx) && std::isfinite(my) && std::isfinite(mz))
            {
                c = g_NodeGrid.CellIndex(
                    g_NodeGrid.CellCoord(mx, g_NodeGrid.originX, g_NodeGrid.dimX),
                    g_NodeGrid.CellCoord(my, g_NodeGrid.originY, g_NodeGrid.dimY),
                    g_NodeGrid.CellCoord(mz, g_NodeGrid.originZ, g_NodeGrid.dimZ));
                cellCount[c]++;
                validNodes++;
            }
            s_GridCellOfNode[idx++] = c;
        }
    }

    if (validNodes <= 0) return;

    for (int c = 0; c < g_NodeGrid.totalCells; ++c)
        g_NodeGrid.cellStart[c + 1] += g_NodeGrid.cellStart[c];

    // ---- Pass 3: scatter entries into per-cell runs ----
    g_NodeGrid.entries.resize((size_t)g_NodeGrid.cellStart[g_NodeGrid.totalCells]);
    s_GridCellCursor.assign(g_NodeGrid.cellStart.begin(), g_NodeGrid.cellStart.end() - 1);

    idx = 0;
    for (int b = 0; b < bodyCount; ++b)
    {
        if (!bodyActive(b)) continue;
        const NativeSoftBodyData& body = bodies[b];

        for (int i = 0; i < body.nodeCount; ++i)
        {
            const int c = s_GridCellOfNode[idx++];
            if (c < 0) continue;
            g_NodeGrid.entries[s_GridCellCursor[c]] = GridEntry{ b, i };
            s_GridCellCursor[c]++;
        }
    }
}

// Forward declarations of internal helpers

static void ApplyXPBDFaceNode(int idxP, int idxA, int idxB, int idxC,
    const Vector3& normal, float penetration,
    float u, float v, float w,
    float* nodePredX, float* nodePredY, float* nodePredZ,
    const float* nodePrevX, const float* nodePrevY, const float* nodePrevZ,
    float* facePredX, float* facePredY, float* facePredZ,
    const float* facePrevX, const float* facePrevY, const float* facePrevZ,
    const float* nodeMasses, const float* faceMasses,
    const uint8_t* nodePinned, const uint8_t* facePinned,
    const Vector3& contactRelativeVelocity,
    float dt, float restitution, float staticFriction, float slidingFriction, bool applyRestitution);

static bool CheckSweptFaceNode(const Vector3& p0, const Vector3& p1, float radius,
    const Vector3& a0, const Vector3& a1,
    const Vector3& b0, const Vector3& b1,
    const Vector3& c0, const Vector3& c1,
    float& tOut, Vector3& nOut);

static bool GetBarycentric(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c,
    float& u, float& v, float& w);

// Shared geometry primitives, defined with the rigid layer at the end of this file.
static Vector3 ClosestPointOnSegment(const Vector3& p, const Vector3& a0, const Vector3& a1);
static Vector3 ClosestPointOnTriangle(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c);

static bool SweptSphereSphere(const Vector3& start, const Vector3& vel, float radiusA,
    const StaticColliderData& sphere,
    float& outT, Vector3& outPoint, Vector3& outNormal);
static bool SweptSphereBox(const Vector3& start, const Vector3& vel, float radius,
    const StaticColliderData& box,
    float& outT, Vector3& outPoint, Vector3& outNormal);
static bool SweptSphereCapsule(const Vector3& start, const Vector3& vel, float radius,
    const StaticColliderData& capsule,
    float& outT, Vector3& outPoint, Vector3& outNormal);
static bool SweptSphereTriangle(const Vector3& start, const Vector3& vel, float radius,
    const StaticColliderData& tri,
    float& outT, Vector3& outPoint, Vector3& outNormal);

static int  BuildNodeNodeContacts(
    const NativeSoftBodyData& bA, int bodyAIdx,
    const NativeSoftBodyData& bB, int bodyBIdx,
    const UniformNodeGrid& grid,
    const float* halfTrajA, const float* halfTrajB,
    float threshold,
    NodeContact* out, int cap);

static void ResolveNodeNodeContacts(
    float* predAx, float* predAy, float* predAz, const float* currAx, const float* currAy, const float* currAz, const float* massA, const uint8_t* pinA, float* predBx, float* predBy, float* predBz, const float* currBx, const float* currBy, const float* currBz, const float* massB, const uint8_t* pinB,
    const NodeContact* contacts, int contactCount,
    float dt, float invDt, float combinedR,
    float restitution, float staticFriction, float slidingFriction,
    int idA, int idB);

static bool IsCrossBodyExcluded(int idA, int nodeA, int idB, int nodeB);


struct FaceNodeBVHJobGroup {
    const FaceData* faces;
    float* facePredX; float* facePredY; float* facePredZ;                   // write target (corrections)
    const float* faceSnapX; const float* faceSnapY; const float* faceSnapZ; // read-only snapshot (geometry)
    const float* facePrevX; const float* facePrevY; const float* facePrevZ;
    const float* faceMasses;
    const uint8_t* faceIsPinned;
    int faceCount;

    float* nodePredX; float* nodePredY; float* nodePredZ;                   // write target (corrections)
    const float* nodeSnapX; const float* nodeSnapY; const float* nodeSnapZ; // read-only snapshot (geometry)
    const float* nodePrevX; const float* nodePrevY; const float* nodePrevZ;
    const float* nodeMasses;
    const uint8_t* nodeIsPinned;
    int nodeCount;
    int nodeStart; // offset of this group's nodes in the flattened job index

    const BodyFaceBVH* bvh;

    float radiusNode, thickness, epsilon, dt;
    float restitution, staticFriction, slidingFriction;
    bool applyRestitution;

    int bodyOwnerIdx;    // body the faces belong to
    int bodyOpposingIdx; // body the nodes belong to

    int ownerBodyId;
    int opposingBodyId;
};

static void ProcessNodeVsFaceBVH(const FaceNodeBVHJobGroup& grp, int n,
    int& localCandidates, int& localResolved);

//  Node-node contact resolution

// Returns true once contact geometry exists (incl. the separating case); callers gate events on it.
static inline bool ResolveOneNodeContact(
    Vector3& predI, const Vector3& currI, float wi,
    Vector3& predJ, const Vector3& currJ, float wj,
    float dt, float invDt, float combinedR,
    float restitution, float staticFriction, float slidingFriction,
    Vector3* outPoint = nullptr, Vector3* outNormal = nullptr,
    float* outNormalSpeed = nullptr, Vector3* outTangentVel = nullptr)
{
    float wSum = wi + wj;
    if (wSum <= 0.0f) return false;
    const float invWSum = 1.0f / wSum;

    const Vector3 velI = Sub(predI, currI);
    const Vector3 velJ = Sub(predJ, currJ);
    const Vector3 relVel = Sub(velI, velJ);
    const Vector3 relStart = Sub(currI, currJ);

    const float a = Dot(relVel, relVel);
    const float c = Dot(relStart, relStart) - combinedR * combinedR;

    float toi = 1.0f;
    const bool alreadyOverlapping = (c <= 0.0f);
    if (!alreadyOverlapping)
    {
        if (a < 1e-12f) return false;
        const float b = 2.0f * Dot(relStart, relVel);
        const float disc = b * b - 4.0f * a * c;
        if (disc < 0.0f) return false;
        const float t = (-b - std::sqrt(disc)) / (2.0f * a);
        if (t < 0.0f || t > 1.0f) return false;
        toi = t;
    }

    const Vector3 pi = Add(currI, Scale(velI, toi));
    const Vector3 pj = Add(currJ, Scale(velJ, toi));
    const Vector3 d = Sub(pi, pj);
    const float dist = Length(d);
    if (dist < PhysicsConstants::MIN_COLLISION_DISTANCE) return false;
    const Vector3 n = Scale(d, 1.0f / dist);

    // Geometrically real from here — report it whether or not the impulse branch fires.
    if (outPoint || outNormal || outNormalSpeed || outTangentVel)
    {
        const Vector3 relVelPerSec = Scale(relVel, invDt);
        const float closingSpeed = -Dot(relVelPerSec, n);
        const Vector3 tangentVel = Sub(relVelPerSec, Scale(n, Dot(relVelPerSec, n)));

        if (outPoint) *outPoint = Scale(Add(pi, pj), 0.5f);
        if (outNormal) *outNormal = n;
        if (outNormalSpeed) *outNormalSpeed = MaxF(closingSpeed, 0.0f);
        if (outTangentVel) *outTangentVel = tangentVel;
    }

    float penEnd = combinedR - Length(Sub(predI, predJ));
    if (penEnd < 0.0f) penEnd = 0.0f;
    const Vector3 corr = Scale(n, penEnd * invWSum);
    predI = Add(predI, Scale(corr, wi));
    predJ = Sub(predJ, Scale(corr, wj));

    const Vector3 vi = Scale(Sub(predI, currI), invDt);
    const Vector3 vj = Scale(Sub(predJ, currJ), invDt);
    const Vector3 relV = Sub(vi, vj);
    const float vn = Dot(relV, n);
    if (vn >= 0.0f) return true; // contact established, but already separating

    const float jn = -(1.0f + restitution) * vn * invWSum;
    const Vector3 impulse = Scale(n, jn);
    Vector3 viNew = Add(vi, Scale(impulse, wi));
    Vector3 vjNew = Sub(vj, Scale(impulse, wj));

    const Vector3 dv = Sub(viNew, vjNew);
    const Vector3 vtPair = Sub(dv, Scale(n, Dot(dv, n)));
    const float vtSpeed = Length(vtPair);
    if (vtSpeed > PhysicsConstants::MIN_TANGENT_SPEED)
    {
        const Vector3 tdir = Scale(vtPair, 1.0f / vtSpeed);
        const float jt = vtSpeed * invWSum;
        const float frictionMag = (jt <= staticFriction * jn)
            ? jt
            : MinF(slidingFriction * jn, jt);
        const Vector3 fImp = Scale(tdir, frictionMag);
        viNew = Sub(viNew, Scale(fImp, wi));
        vjNew = Add(vjNew, Scale(fImp, wj));
    }

    predI = Add(currI, Scale(viNew, dt));
    predJ = Add(currJ, Scale(vjNew, dt));
    return true;
}

static void ResolveNodeNodeContacts(
    float* predAx, float* predAy, float* predAz, const float* currAx, const float* currAy, const float* currAz, const float* massA, const uint8_t* pinA, float* predBx, float* predBy, float* predBz, const float* currBx, const float* currBy, const float* currBz, const float* massB, const uint8_t* pinB,
    const NodeContact* contacts, int contactCount,
    float dt, float invDt, float combinedR,
    float restitution, float staticFriction, float slidingFriction,
    int idA, int idB)
{
    if (contactCount <= 0 || dt <= 0.0f) return;

    for (int p = 0; p < contactCount; ++p)
    {
        const int i = contacts[p].a;
        const int j = contacts[p].b;
        const float wi = (pinA && pinA[i]) ? 0.0f : (massA[i] > 0.0f ? 1.0f / massA[i] : 0.0f);
        const float wj = (pinB && pinB[j]) ? 0.0f : (massB[j] > 0.0f ? 1.0f / massB[j] : 0.0f);

        Vector3 pI = { predAx[i], predAy[i], predAz[i], 0.0f };
        Vector3 pJ = { predBx[j], predBy[j], predBz[j], 0.0f };

        Vector3 contactPoint = V3Zero();
        Vector3 contactNormal = V3Zero();
        Vector3 tangentVel = V3Zero();
        float normalSpeed = 0.0f;

        const bool contacted = ResolveOneNodeContact(
            pI, { currAx[i], currAy[i], currAz[i], 0.0f }, wi,
            pJ, { currBx[j], currBy[j], currBz[j], 0.0f }, wj,
            dt, invDt, combinedR,
            restitution, staticFriction, slidingFriction,
            &contactPoint, &contactNormal, &normalSpeed, &tangentVel);

        // Record an event only for a real geometric contact — stale outputs would be phantom forces.
        if (contacted)
        {
            RecordContactEvent(
                idA, idB, contactPoint, contactNormal,
                normalSpeed, tangentVel);
        }

        predAx[i] = pI.x; predAy[i] = pI.y; predAz[i] = pI.z;
        predBx[j] = pJ.x; predBy[j] = pJ.y; predBz[j] = pJ.z;
    }
}

//  Radix sort for KeyVal

static void RadixSortKeyVal(std::vector<KeyVal>& arr)
{
    const int n = (int)arr.size();
    if (n <= 1) return;

    if (n <= 64) { std::sort(arr.begin(), arr.end()); return; }

    static thread_local std::vector<KeyVal> temp;
    temp.resize(n);

    for (int pass = 0; pass < 4; ++pass)
    {
        const int shift = pass * 8;
        const bool msbPass = (pass == 3);
        int counts[256] = {};

        for (int i = 0; i < n; ++i)
        {
            unsigned digit = ((unsigned)arr[i].key >> shift) & 0xFFu;
            if (msbPass) digit ^= 0x80u;
            counts[digit]++;
        }

        int total = 0;
        for (int i = 0; i < 256; ++i)
        {
            int c = counts[i];
            counts[i] = total;
            total += c;
        }

        for (int i = 0; i < n; ++i)
        {
            unsigned digit = ((unsigned)arr[i].key >> shift) & 0xFFu;
            if (msbPass) digit ^= 0x80u;
            temp[counts[digit]++] = arr[i];
        }

        arr.swap(temp);
    }
}

//  Broadphase: node-node contact generation via the uniform grid

static int BuildNodeNodeContacts(
    const NativeSoftBodyData& bA, int bodyAIdx,
    const NativeSoftBodyData& bB, int bodyBIdx,
    const UniformNodeGrid& grid,
    const float* halfTrajA, const float* halfTrajB,
    float threshold,
    NodeContact* out, int cap)
{
    if (cap <= 0 || !out) return 0;
    if (grid.Empty()) return 0;
    if (bA.nodeCount <= 0 || bB.nodeCount <= 0) return 0;
    if (!halfTrajA || !halfTrajB) return 0;

    const int dimX = grid.dimX, dimY = grid.dimY, dimZ = grid.dimZ;
    const int* cellStart = grid.cellStart.data();
    const GridEntry* entries = grid.entries.data();

    const float* currAx = bA.currentX; const float* currAy = bA.currentY; const float* currAz = bA.currentZ;
    const float* predAx = bA.predictedX; const float* predAy = bA.predictedY; const float* predAz = bA.predictedZ;
    const float* currBx = bB.currentX; const float* currBy = bB.currentY; const float* currBz = bB.currentZ;
    const float* predBx = bB.predictedX; const float* predBy = bB.predictedY; const float* predBz = bB.predictedZ;

    int written = 0;

    for (int i = 0; i < bA.nodeCount; ++i)
    {
        const float midAx = (currAx[i] + predAx[i]) * 0.5f;
        const float midAy = (currAy[i] + predAy[i]) * 0.5f;
        const float midAz = (currAz[i] + predAz[i]) * 0.5f;

        if (!std::isfinite(midAx) || !std::isfinite(midAy) || !std::isfinite(midAz)) continue;

        const int cx = grid.CellCoord(midAx, grid.originX, dimX);
        const int cy = grid.CellCoord(midAy, grid.originY, dimY);
        const int cz = grid.CellCoord(midAz, grid.originZ, dimZ);

        // cellSize ≥ widest pair reach, so every passing candidate sits in this 3x3x3 block.
        const float reachBase = threshold + halfTrajA[i];

        for (int dz = -1; dz <= 1; ++dz)
        {
            const int z = cz + dz;
            if (z < 0 || z >= dimZ) continue;

            for (int dy = -1; dy <= 1; ++dy)
            {
                const int y = cy + dy;
                if (y < 0 || y >= dimY) continue;
                const int slice = dimX * (y + dimY * z);

                for (int dx = -1; dx <= 1; ++dx)
                {
                    const int x = cx + dx;
                    if (x < 0 || x >= dimX) continue;

                    const int cell = slice + x;
                    for (int e = cellStart[cell]; e < cellStart[cell + 1]; ++e)
                    {
                        const GridEntry& en = entries[e];
                        if (en.body != bodyBIdx) continue;

                        const int j = en.node;
                        const float ex = midAx - (currBx[j] + predBx[j]) * 0.5f;
                        const float ey = midAy - (currBy[j] + predBy[j]) * 0.5f;
                        const float ez = midAz - (currBz[j] + predBz[j]) * 0.5f;

                        const float reach = reachBase + halfTrajB[j];
                        if (ex * ex + ey * ey + ez * ez < reach * reach)
                        {
                            if (IsCrossBodyExcluded(bA.id, i, bB.id, j)) continue;
                            out[written].a = i;
                            out[written].b = j;
                            if (++written >= cap) return written;
                        }
                    }
                }
            }
        }
    }
    return written;
}

static bool SweptSphereTerrain(const Vector3& start, const Vector3& vel, float radius,
    const StaticColliderData& terrain,
    float& outT, Vector3& outPoint, Vector3& outNormal)
{
    outT = std::numeric_limits<float>::max();
    outPoint = V3Zero();
    outNormal = V3(0.0f, 1.0f, 0.0f);

    if (!terrain.terrainHeights || terrain.terrainWidth < 2 || terrain.terrainLength < 2) return false;
    if (!std::isfinite(start.x) || !std::isfinite(start.y) || !std::isfinite(start.z) ||
        !std::isfinite(vel.x) || !std::isfinite(vel.y) || !std::isfinite(vel.z))
        return false;

    const int w = terrain.terrainWidth;
    const int len = terrain.terrainLength;
    Vector3 origin = Sub(terrain.center, Scale(terrain.size, 0.5f));

    // heightAt/normalAt clamp their sample coordinate into the valid heightmap range.

    auto insideBounds = [&](float localX, float localZ) -> bool
        {
            constexpr float kEdgeEpsilon = 1e-4f;
            return localX >= -kEdgeEpsilon && localX <= terrain.size.x + kEdgeEpsilon &&
                localZ >= -kEdgeEpsilon && localZ <= terrain.size.z + kEdgeEpsilon;
        };

    auto heightAt = [&](float localX, float localZ) -> float
        {
            float tX = ClampF(localX / terrain.size.x * (w - 1), 0.0f, (float)(w - 1));
            float tZ = ClampF(localZ / terrain.size.z * (len - 1), 0.0f, (float)(len - 1));

            int x0 = (int)tX; if (x0 >= w - 1) x0 = w - 2;
            int x1 = x0 + 1;
            int z0 = (int)tZ; if (z0 >= len - 1) z0 = len - 2;
            int z1 = z0 + 1;

            float fx = tX - x0;
            float fz = tZ - z0;

            float h00 = terrain.terrainHeights[z0 * w + x0];
            float h10 = terrain.terrainHeights[z0 * w + x1];
            float h01 = terrain.terrainHeights[z1 * w + x0];
            float h11 = terrain.terrainHeights[z1 * w + x1];

            float h0 = h00 * (1.0f - fx) + h10 * fx;
            float h1 = h01 * (1.0f - fx) + h11 * fx;
            return (h0 * (1.0f - fz) + h1 * fz) * terrain.size.y;
        };

    auto normalAt = [&](float localX, float localZ) -> Vector3
        {
            float tX = ClampF(localX / terrain.size.x * (w - 1), 0.0f, (float)(w - 1));
            float tZ = ClampF(localZ / terrain.size.z * (len - 1), 0.0f, (float)(len - 1));
            int x0 = (int)tX; if (x0 >= w - 1) x0 = w - 2;
            int x1 = x0 + 1;
            int z0 = (int)tZ; if (z0 >= len - 1) z0 = len - 2;
            int z1 = z0 + 1;

            float h00 = terrain.terrainHeights[z0 * w + x0];
            float h10 = terrain.terrainHeights[z0 * w + x1];
            float h01 = terrain.terrainHeights[z1 * w + x0];

            float dx = (h10 - h00) * terrain.size.y / (terrain.size.x / (w - 1));
            float dz = (h01 - h00) * terrain.size.y / (terrain.size.z / (len - 1));
            return Normalize(Vector3{ -dx, 1.0f, -dz, 0.0f });
        };

    float texelX = terrain.size.x / (float)(w - 1);
    float texelZ = terrain.size.z / (float)(len - 1);
    float texel = MaxF(1e-4f, MinF(texelX, texelZ));

    float pathLenXZ = std::sqrt(vel.x * vel.x + vel.z * vel.z);
    int steps = (int)std::ceil(MaxF(pathLenXZ, AbsF(vel.y)) / texel);
    steps = ClampI(steps, 1, 512);

    float prevT = 0.0f;
    float prevLocalX = start.x - origin.x;
    float prevLocalZ = start.z - origin.z;
    bool prevInside = insideBounds(prevLocalX, prevLocalZ);
    float prevClearance = prevInside
        ? (start.y - origin.y) - heightAt(prevLocalX, prevLocalZ) - radius
        : std::numeric_limits<float>::max();

    if (prevInside && prevClearance <= 0.0f)
    {
        outT = 0.0f;
        outNormal = normalAt(prevLocalX, prevLocalZ);
        outPoint = V3(start.x, origin.y + heightAt(prevLocalX, prevLocalZ), start.z);
        return true;
    }

    for (int s = 1; s <= steps; ++s)
    {
        float t = (float)s / (float)steps;
        Vector3 p = Add(start, Scale(vel, t));
        float localX = p.x - origin.x;
        float localZ = p.z - origin.z;
        bool inside = insideBounds(localX, localZ);
        float clearance = inside
            ? (p.y - origin.y) - heightAt(localX, localZ) - radius
            : std::numeric_limits<float>::max();

        if (inside && clearance <= 0.0f)
        {
            // Interpolate against the previous sample's clearance only if it was over the terrain too.
            float tHit;
            if (prevInside)
            {
                float denom = (prevClearance - clearance);
                float frac = denom > 1e-8f ? ClampF(prevClearance / denom, 0.0f, 1.0f) : 1.0f;
                tHit = prevT + (t - prevT) * frac;
            }
            else
            {
                tHit = t;
            }

            Vector3 hitPos = Add(start, Scale(vel, tHit));
            float hLocalX = hitPos.x - origin.x;
            float hLocalZ = hitPos.z - origin.z;

            outT = tHit;
            outNormal = normalAt(hLocalX, hLocalZ);
            outPoint = V3(hitPos.x, origin.y + heightAt(hLocalX, hLocalZ), hitPos.z);
            return true;
        }

        prevT = t;
        prevClearance = clearance;
        prevInside = inside;
    }

    return false;
}

static inline void TestSweptStaticCollider(
    const StaticColliderData& c, const Vector3& cur, const Vector3& vel, float radius,
    bool& found, float& earliestT, Vector3& hitPoint, Vector3& hitNormal, StaticColliderData& hitCol)
{
    float t = std::numeric_limits<float>::max();
    Vector3 p = V3Zero();
    Vector3 nrm = V3(0.0f, 1.0f, 0.0f);
    bool hit = false;

    if (c.type == COL_SPHERE)       hit = SweptSphereSphere(cur, vel, radius, c, t, p, nrm);
    else if (c.type == COL_BOX)     hit = SweptSphereBox(cur, vel, radius, c, t, p, nrm);
    else if (c.type == COL_CAPSULE) hit = SweptSphereCapsule(cur, vel, radius, c, t, p, nrm);
    else if (c.type == COL_TRIANGLE)hit = SweptSphereTriangle(cur, vel, radius, c, t, p, nrm);
    else if (c.type == COL_TERRAIN) hit = SweptSphereTerrain(cur, vel, radius, c, t, p, nrm);

    if (hit && t < earliestT) { earliestT = t; hitPoint = p; hitNormal = nrm; hitCol = c; found = true; }
}

static void ResolveStaticCollisions_Internal(
    float* predX, float* predY, float* predZ, float* currX, float* currY, float* currZ,
    const float* masses, const uint8_t* isPinned, int nodeCount,
    const StaticColliderData* colliders, int colliderCount,
    float radius, float epsilon,
    float defaultRestitution, float defaultStaticFriction,
    float defaultSlidingFriction, float dt, bool applyRestitution, int selfBodyId)
{
    if (nodeCount <= 0 || dt <= 0.0f) return;
    if (colliderCount <= 0) return;

    static std::vector<int> threadChecks;
    static std::vector<int> threadContacts;
    if (threadChecks.size() < (size_t)nodeCount) threadChecks.resize(nodeCount);
    if (threadContacts.size() < (size_t)nodeCount) threadContacts.resize(nodeCount);
    std::fill(threadChecks.begin(), threadChecks.begin() + nodeCount, 0);
    std::fill(threadContacts.begin(), threadContacts.begin() + nodeCount, 0);

    GetGlobalPool().parallel_for(nodeCount, [&](int i)
        {
            static thread_local std::vector<int> candidates;

            Vector3 cur = VEC3(curr, i);
            Vector3 pred = VEC3(pred, i);
            float mass = masses[i];

            if (isPinned && isPinned[i]) return;
            if (mass <= 0.0f) return;

            Vector3 vel = Sub(pred, cur);

            Vector3 sweptMin = { MinF(cur.x, pred.x) - radius,
                                 MinF(cur.y, pred.y) - radius,
                                 MinF(cur.z, pred.z) - radius, 0.0f };
            Vector3 sweptMax = { MaxF(cur.x, pred.x) + radius,
                                 MaxF(cur.y, pred.y) + radius,
                                 MaxF(cur.z, pred.z) + radius, 0.0f };

            // BVH query replaces the hash-grid walk; each collider is in one leaf, so no dedup.
            candidates.clear();
            QueryStaticBVH(sweptMin, sweptMax, [&](int idx) { candidates.push_back(idx); });

            bool found = false;
            float earliestT = std::numeric_limits<float>::max();
            Vector3 hitPoint = V3Zero();
            Vector3 hitNormal = V3(0.0f, 1.0f, 0.0f);
            StaticColliderData hitCol = {};

            int actualChecks = 0;
            int resolved = 0;

            for (int idx : candidates) {
                const StaticColliderData& c = colliders[idx];
                actualChecks++;
                TestSweptStaticCollider(c, cur, vel, radius, found, earliestT, hitPoint, hitNormal, hitCol);
                if (earliestT <= 0.0f) break;
            }

            threadChecks[i] = actualChecks;
            if (found) resolved = 1;
            threadContacts[i] = resolved;

            if (!found) return;

            Vector3 origPred = Add(cur, vel);
            float rawPen = MaxF(radius - Dot(Sub(origPred, hitPoint), hitNormal), 0.0f);
            Vector3 resolvedPos = Add(origPred, Scale(hitNormal, rawPen + epsilon));
            predX[i] = resolvedPos.x; predY[i] = resolvedPos.y; predZ[i] = resolvedPos.z;

            float preVn = Dot(Scale(vel, 1.0f / dt), hitNormal);

            {
                Vector3 velPerSec = Scale(vel, 1.0f / dt);
                Vector3 tangentVel = Sub(velPerSec, Scale(hitNormal, preVn));
                RecordContactEvent(selfBodyId, STATIC_CONTACT_ID, hitPoint, hitNormal, MaxF(-preVn, 0.0f), tangentVel);
            }

            if (preVn < 0.0f || rawPen > 0.0f)
            {
                float restA = (defaultRestitution + hitCol.restitution) * 0.5f;
                float sf = (defaultStaticFriction + hitCol.staticFriction) * 0.5f;
                float kf = (defaultSlidingFriction + hitCol.slidingFriction) * 0.5f;

                constexpr float BOUNCE_THRESHOLD = 0.2f;
                float appliedRest = (applyRestitution && -preVn > BOUNCE_THRESHOLD) ? restA : 0.0f;
                float bounceVn = (appliedRest > 0.0f) ? -preVn * appliedRest : 0.0f;

                float jn = (preVn < 0.0f) ? -(1.0f + appliedRest) * preVn : 0.0f;
                float positionalVn = rawPen / dt;
                float effectiveJn = jn + positionalVn;
                (void)effectiveJn;

                Vector3 postV = Scale(Sub(resolvedPos, cur), 1.0f / dt);
                float   postVn = Dot(postV, hitNormal);

                float clampedPostVn = ClampF(postVn, -kMaxStaticPushoutSpeed, kMaxStaticPushoutSpeed);
                float   targetVn = MaxF(clampedPostVn, bounceVn);
                Vector3 vNew = Add(postV, Scale(hitNormal, targetVn - postVn));

                Vector3 vt = Sub(vNew, Scale(hitNormal, Dot(vNew, hitNormal)));
                float   vtSpeed = Length(vt);

                if (vtSpeed > PhysicsConstants::MIN_TANGENT_SPEED)
                {
                    Vector3 tdir = Scale(vt, 1.0f / vtSpeed);
                    float jt = vtSpeed;
                    float fMag = (jt <= sf * jn) ? jt : MinF(kf * jn, jt);
                    vNew = Sub(vNew, Scale(tdir, fMag));
                }

                Vector3 finalPos = Add(cur, Scale(vNew, dt));

                float finalPen = radius - Dot(Sub(finalPos, hitPoint), hitNormal);
                if (finalPen > 0.0f)
                {
                    finalPos = Add(finalPos, Scale(hitNormal, finalPen + epsilon));
                }
                predX[i] = finalPos.x; predY[i] = finalPos.y; predZ[i] = finalPos.z;
            }
        }, 128);

    int totalChecks = 0, totalResolved = 0;
    for (int i = 0; i < nodeCount; ++i) {
        totalChecks += threadChecks[i];
        totalResolved += threadContacts[i];
    }
    AtomicAddInt(&g_Stats.staticCandidateChecks, totalChecks);
    AtomicAddInt(&g_Stats.staticContactsResolved, totalResolved);
}

//  Face-to-node collision

// Computes face-node collisions using the dynamic Face BVH
static void ProcessNodeVsFaceBVH(const FaceNodeBVHJobGroup& grp, int n,
    int& localCandidates, int& localResolved)
{
    const Vector3 nodeStart = Vector3{ grp.nodePrevX[n], grp.nodePrevY[n], grp.nodePrevZ[n], 0.0f };
    const Vector3 nodeEnd = Vector3{ grp.nodeSnapX[n], grp.nodeSnapY[n], grp.nodeSnapZ[n], 0.0f };

    const float reach = grp.radiusNode + grp.epsilon;
    const float qMinX = MinF(nodeStart.x, nodeEnd.x) - reach;
    const float qMinY = MinF(nodeStart.y, nodeEnd.y) - reach;
    const float qMinZ = MinF(nodeStart.z, nodeEnd.z) - reach;
    const float qMaxX = MaxF(nodeStart.x, nodeEnd.x) + reach;
    const float qMaxY = MaxF(nodeStart.y, nodeEnd.y) + reach;
    const float qMaxZ = MaxF(nodeStart.z, nodeEnd.z) + reach;

    const __m128 qMin = _mm_set_ps(0.0f, qMinZ, qMinY, qMinX);
    const __m128 qMax = _mm_set_ps(0.0f, qMaxZ, qMaxY, qMaxX);

    const BodyFaceBVH& bvh = *grp.bvh;
    if (bvh.nodes.empty()) return;

    int stack[64];
    int sp = 0;
    stack[sp++] = (int)bvh.nodes.size() - 1; // root is last node in postorder

    while (sp > 0)
    {
        const int nodeIdx = stack[--sp];
        const FaceBVHNode& bNode = bvh.nodes[nodeIdx];

        if (!AABBOverlapSIMD(qMin, qMax, &bNode.minX, &bNode.maxX))
            continue;

        if (bNode.count > 0)
        {
            for (int k = 0; k < bNode.count; ++k)
            {
                const int f = bvh.faceIndices[bNode.start + k];
                const FaceData& face = grp.faces[f];

                const Vector3 v0s = Vector3{ grp.facePrevX[face.nodeA], grp.facePrevY[face.nodeA], grp.facePrevZ[face.nodeA], 0.0f };
                const Vector3 v1s = Vector3{ grp.facePrevX[face.nodeB], grp.facePrevY[face.nodeB], grp.facePrevZ[face.nodeB], 0.0f };
                const Vector3 v2s = Vector3{ grp.facePrevX[face.nodeC], grp.facePrevY[face.nodeC], grp.facePrevZ[face.nodeC], 0.0f };
                const Vector3 v0e = Vector3{ grp.faceSnapX[face.nodeA], grp.faceSnapY[face.nodeA], grp.faceSnapZ[face.nodeA], 0.0f };
                const Vector3 v1e = Vector3{ grp.faceSnapX[face.nodeB], grp.faceSnapY[face.nodeB], grp.faceSnapZ[face.nodeB], 0.0f };
                const Vector3 v2e = Vector3{ grp.faceSnapX[face.nodeC], grp.faceSnapY[face.nodeC], grp.faceSnapZ[face.nodeC], 0.0f };

                const float thickness = grp.thickness;
                const float bMinX = MinF(MinF(v0s.x, v1s.x), MinF(v2s.x, MinF(MinF(v0e.x, v1e.x), v2e.x))) - thickness;
                const float bMinY = MinF(MinF(v0s.y, v1s.y), MinF(v2s.y, MinF(MinF(v0e.y, v1e.y), v2e.y))) - thickness;
                const float bMinZ = MinF(MinF(v0s.z, v1s.z), MinF(v2s.z, MinF(MinF(v0e.z, v1e.z), v2e.z))) - thickness;
                const float bMaxX = MaxF(MaxF(v0s.x, v1s.x), MaxF(v2s.x, MaxF(MaxF(v0e.x, v1e.x), v2e.x))) + thickness;
                const float bMaxY = MaxF(MaxF(v0s.y, v1s.y), MaxF(v2s.y, MaxF(MaxF(v0e.y, v1e.y), v2e.y))) + thickness;
                const float bMaxZ = MaxF(MaxF(v0s.z, v1s.z), MaxF(v2s.z, MaxF(MaxF(v0e.z, v1e.z), v2e.z))) + thickness;

                if (qMinX > bMaxX || qMaxX < bMinX ||
                    qMinY > bMaxY || qMaxY < bMinY ||
                    qMinZ > bMaxZ || qMaxZ < bMinZ) continue;

                localCandidates++;

                float t; Vector3 hitNormal;
                Vector3 pAtT, aAtT, bAtT, cAtT;

                if (CheckSweptFaceNode(nodeStart, nodeEnd, grp.radiusNode, v0s, v0e, v1s, v1e, v2s, v2e, t, hitNormal))
                {
                    pAtT = Lerp(nodeStart, nodeEnd, t);
                    aAtT = Lerp(v0s, v0e, t);
                    bAtT = Lerp(v1s, v1e, t);
                    cAtT = Lerp(v2s, v2e, t);
                }
                else
                {
                    Vector3 n1 = Normalize(Cross(Sub(v1e, v0e), Sub(v2e, v0e)));
                    if (LengthSq(n1) < 0.5f) continue;

                    float dEnd = Dot(Sub(nodeEnd, v0e), n1);
                    if (AbsF(dEnd) > thickness) continue;

                    t = 1.0f;
                    pAtT = nodeEnd; aAtT = v0e; bAtT = v1e; cAtT = v2e;
                    hitNormal = dEnd >= 0.0f ? n1 : Neg(n1);
                }

                float u, vBary, w;
                if (GetBarycentric(pAtT, aAtT, bAtT, cAtT, u, vBary, w))
                {
                    const Vector3 triPoint = Add(Add(Scale(v0e, u), Scale(v1e, vBary)), Scale(v2e, w));
                    const float dist = Dot(Sub(nodeEnd, triPoint), hitNormal);
                    float penetration = (grp.radiusNode + grp.epsilon) - dist;
                    if (penetration > 0.0f || t < 1.0f)
                    {
                        penetration = MaxF(penetration, grp.epsilon);
                        localResolved++;

                        // Snapshot-derived relative velocity — reading live positions here was a race.
                        const float invDtLocal = grp.dt > 0.0f ? 1.0f / grp.dt : 0.0f;

                        const Vector3 velNode = Scale(Sub(nodeEnd, nodeStart), invDtLocal);
                        const Vector3 velA = Scale(Sub(v0e, v0s), invDtLocal);
                        const Vector3 velB = Scale(Sub(v1e, v1s), invDtLocal);
                        const Vector3 velC = Scale(Sub(v2e, v2s), invDtLocal);

                        const Vector3 velTri = Add(
                            Add(Scale(velA, u), Scale(velB, vBary)),
                            Scale(velC, w));

                        const Vector3 contactRelativeVelocity = Sub(velNode, velTri);
                        const float vn = Dot(contactRelativeVelocity, hitNormal);
                        const Vector3 tangentVel = Sub(
                            contactRelativeVelocity, Scale(hitNormal, vn));

                        RecordContactEvent(
                            grp.ownerBodyId,
                            grp.opposingBodyId,
                            triPoint,
                            hitNormal,
                            MaxF(-vn, 0.0f),
                            tangentVel);

                        ApplyXPBDFaceNode(
                            n, face.nodeA, face.nodeB, face.nodeC,
                            hitNormal, penetration, u, vBary, w,
                            grp.nodePredX, grp.nodePredY, grp.nodePredZ,
                            grp.nodePrevX, grp.nodePrevY, grp.nodePrevZ,
                            grp.facePredX, grp.facePredY, grp.facePredZ,
                            grp.facePrevX, grp.facePrevY, grp.facePrevZ,
                            grp.nodeMasses, grp.faceMasses,
                            grp.nodeIsPinned, grp.faceIsPinned,
                            contactRelativeVelocity,
                            grp.dt, grp.restitution,
                            grp.staticFriction, grp.slidingFriction,
                            grp.applyRestitution);
                    }
                }
            }
        }
        else if (sp < 62)
        {
            stack[sp++] = bNode.left;
            stack[sp++] = bNode.right;
        }
    }
}

// Tight per-axis AABB overlap between two bodies' current/predicted node extents
static inline bool BodyBoundsOverlap(const Vector3& aMin, const Vector3& aMax,
    const Vector3& bMin, const Vector3& bMax, float margin)
{
    return !(aMin.x - margin > bMax.x || aMax.x + margin < bMin.x ||
        aMin.y - margin > bMax.y || aMax.y + margin < bMin.y ||
        aMin.z - margin > bMax.z || aMax.z + margin < bMin.z);
}

//  StepAllCollisions 

// Persistent ignore state, set once by C# when a designer toggles it — not rebuilt per substep.
static std::vector<uint64_t> g_IgnoredPairs;          // sorted, deduped, packed (idLo | idHi<<32)
static std::unordered_set<int> g_DisableAllCollisions; // body ids that ignore every soft body

static inline uint64_t PackPair(int idA, int idB)
{
    const uint64_t a = (uint32_t)idA, b = (uint32_t)idB;
    return (a < b) ? (a | (b << 32)) : (b | (a << 32));
}

static std::unordered_map<uint64_t, std::vector<std::pair<int, int>>> g_CrossBodyExclusions;

void Collisions_SetCrossBodyExclusions(const NodePairExclusion* pairs, int count)
{
    g_CrossBodyExclusions.clear();
    if (!pairs || count <= 0) return;

    for (int i = 0; i < count; ++i)
    {
        const NodePairExclusion& p = pairs[i];
        const uint64_t key = PackPair(p.idA, p.idB);
        // Store node indices in the same (low-id, high-id) order PackPair canonicalized to.
        if ((uint32_t)p.idA <= (uint32_t)p.idB)
            g_CrossBodyExclusions[key].emplace_back(p.nodeA, p.nodeB);
        else
            g_CrossBodyExclusions[key].emplace_back(p.nodeB, p.nodeA);
    }
}

static bool IsCrossBodyExcluded(int idA, int nodeA, int idB, int nodeB)
{
    if (g_CrossBodyExclusions.empty()) return false;
    auto it = g_CrossBodyExclusions.find(PackPair(idA, idB));
    if (it == g_CrossBodyExclusions.end()) return false;

    int lowNode = nodeA, highNode = nodeB;
    if ((uint32_t)idA > (uint32_t)idB) std::swap(lowNode, highNode);

    for (const auto& pr : it->second)
        if (pr.first == lowNode && pr.second == highNode) return true;
    return false;
}

// Bulk replace, for callers pushing a whole snapshot (e.g. loading an ignore list).
EXPORT void Collisions_SetIgnoredPairs(const NativeInt2* pairs, int count)
{
    g_IgnoredPairs.clear();
    if (count <= 0) return; // clearing the ignore list is valid usage, not an error
    if (!pairs) { Error_SetError(ErrorCode::InvalidArgument); return; }

    g_IgnoredPairs.reserve((size_t)count);
    for (int i = 0; i < count; ++i)
        g_IgnoredPairs.push_back(PackPair(pairs[i].x, pairs[i].y));

    std::sort(g_IgnoredPairs.begin(), g_IgnoredPairs.end());
    g_IgnoredPairs.erase(std::unique(g_IgnoredPairs.begin(), g_IgnoredPairs.end()), g_IgnoredPairs.end());
}

// Incremental single-pair toggle — what IgnoreBodyCollision / SetIgnoreCollision call directly.
EXPORT void Collisions_SetPairIgnored(int idA, int idB, bool ignore)
{
    const uint64_t key = PackPair(idA, idB);
    auto it = std::lower_bound(g_IgnoredPairs.begin(), g_IgnoredPairs.end(), key);

    if (ignore)
    {
        if (it == g_IgnoredPairs.end() || *it != key)
            g_IgnoredPairs.insert(it, key);
    }
    else
    {
        if (it != g_IgnoredPairs.end() && *it == key)
            g_IgnoredPairs.erase(it);
    }
}

// Drops every ignore-pair entry involving this id; call when a body is destroyed.
EXPORT void Collisions_ClearIgnoredPairsForId(int id)
{
    const uint64_t idMasked = (uint32_t)id;
    g_IgnoredPairs.erase(
        std::remove_if(g_IgnoredPairs.begin(), g_IgnoredPairs.end(),
            [&](uint64_t key) { return (key & 0xFFFFFFFFull) == idMasked || (key >> 32) == idMasked; }),
        g_IgnoredPairs.end());
    g_DisableAllCollisions.erase(id);
    g_BodyFaceBVHs.erase(id);
}

EXPORT void Collisions_InvalidateBodyFaceBVH(int bodyId)
{
    g_BodyFaceBVHs.erase(bodyId);
}

// "Ignores collision against every other soft body" — soft only, never rigid.
EXPORT void Collisions_SetDisableAllCollisions(int id, bool disable)
{
    if (disable) g_DisableAllCollisions.insert(id);
    else g_DisableAllCollisions.erase(id);
}

// Clears the rigid warm-start cache too; defined in the rigid layer below.
void Collisions_ClearRigidWarmCache();

EXPORT void Collisions_ResetPersistentCollisionState()
{
    g_IgnoredPairs.clear();
    g_DisableAllCollisions.clear();
    g_CrossBodyExclusions.clear();
    g_BodyFaceBVHs.clear();
    Collisions_ClearRigidWarmCache();
}

void Collisions_BeginFrame()
{
    g_ContactEventCount.store(0, std::memory_order_relaxed);
}

void Collisions_ResolveAll(
    NativeSoftBodyData* bodies, int bodyCount,
    float dt, float epsilon, float faceNodeCellSize, bool applyRestitution,
    bool shouldResolve)
{
    if (bodyCount <= 0 || dt <= 0.0f) return;
    if (!shouldResolve) return;

    // Rebuild static data if any static bodies were added/removed
    ProcessDirtyStatics();

    constexpr float MIN_SAFE_DT = 1.0f / 100000.0f;
    if (dt < MIN_SAFE_DT) dt = MIN_SAFE_DT;

    const bool doRestitution = applyRestitution;

    auto t0 = std::chrono::high_resolution_clock::now();
    const float invDt = 1.0f / dt;

    int totalFaces = 0;
    for (int i = 0; i < bodyCount; ++i) totalFaces += bodies[i].faceCount;
    g_Stats.totalFaces = totalFaces;

    auto tStaticStart = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < bodyCount; ++i) {
        NativeSoftBodyData& b = bodies[i];
        b.wakeRequested = 0;
        if (b.nodeCount == 0) continue;
        if (b.isSleeping) continue; // frozen bodies don't self-correct against static geometry

        //node-static
        ResolveStaticCollisions_Internal(
            b.predictedX, b.predictedY, b.predictedZ, b.currentX, b.currentY, b.currentZ, b.masses, b.isPinned, b.nodeCount,
            g_StaticColliderData.data(), (int)g_StaticColliderData.size(),
            b.radius + PhysicsConstants::SKIN_WIDTH, epsilon,
            b.restitution, b.staticFriction, b.slidingFriction, dt, doRestitution, b.id);
    }
    auto tStaticEnd = std::chrono::high_resolution_clock::now();
    g_Stats.staticCollisionTimeMs += std::chrono::duration<float, std::milli>(tStaticEnd - tStaticStart).count();

    // Recompute bounds and face BVHs AFTER the first static pass, so both describe the same state.
    for (int i = 0; i < bodyCount; ++i) {
        NativeSoftBodyData& b = bodies[i];
        if (b.nodeCount == 0) continue;

        float minX = 1e9f, minY = 1e9f, minZ = 1e9f;
        float maxX = -1e9f, maxY = -1e9f, maxZ = -1e9f;

        for (int n = 0; n < b.nodeCount; ++n) {
            float px = b.predictedX[n], py = b.predictedY[n], pz = b.predictedZ[n];
            if (px < minX) minX = px; if (px > maxX) maxX = px;
            if (py < minY) minY = py; if (py > maxY) maxY = py;
            if (pz < minZ) minZ = pz; if (pz > maxZ) maxZ = pz;

            float cx = b.currentX[n], cy = b.currentY[n], cz = b.currentZ[n];
            if (cx < minX) minX = cx; if (cx > maxX) maxX = cx;
            if (cy < minY) minY = cy; if (cy > maxY) maxY = cy;
            if (cz < minZ) minZ = cz; if (cz > maxZ) maxZ = cz;
        }
        b.boundsMin = Vector3{ minX, minY, minZ, 0.0f };
        b.boundsMax = Vector3{ maxX, maxY, maxZ, 0.0f };
    }

    // Refit or build Dynamic Face-BVH from those same post-static positions.
    for (int i = 0; i < bodyCount; ++i) {
        const NativeSoftBodyData& b = bodies[i];
        if (b.nodeCount == 0 || b.faceCount == 0 || !b.faces) continue;

        BodyFaceBVH& bvh = g_BodyFaceBVHs[b.id];
        if (!bvh.built || bvh.faceCount != b.faceCount || bvh.faceDataPtr != b.faces) {
            BuildBodyFaceBVH(bvh, b);
        }
        else {
            RefitFaceBVH(bvh, b);
        }
    }

    // ── Freeze predictedX/Y/Z per body right after static resolution ──
    // Face-node geometry reads use this frozen copy; ApplyXPBDFaceNode still writes the live
    // arrays, so neither of the two opposing job groups for a pair can build a torn triangle.
    static std::vector<std::vector<float>> snapX, snapY, snapZ;
    if ((int)snapX.size() < bodyCount) { snapX.resize(bodyCount); snapY.resize(bodyCount); snapZ.resize(bodyCount); }

    for (int i = 0; i < bodyCount; ++i)
    {
        const NativeSoftBodyData& b = bodies[i];
        if (b.nodeCount <= 0) continue;
        if ((int)snapX[i].size() != b.nodeCount)
        {
            snapX[i].resize(b.nodeCount);
            snapY[i].resize(b.nodeCount);
            snapZ[i].resize(b.nodeCount);
        }
        std::memcpy(snapX[i].data(), b.predictedX, sizeof(float) * b.nodeCount);
        std::memcpy(snapY[i].data(), b.predictedY, sizeof(float) * b.nodeCount);
        std::memcpy(snapZ[i].data(), b.predictedZ, sizeof(float) * b.nodeCount);
    }
    // ── end new block ──

    const std::vector<uint64_t>& ignoredSorted = g_IgnoredPairs;

    struct SapInterval { float minX, maxX; };
    static std::vector<SapInterval> sapItv;
    static std::vector<int>         sapOrder;
    static std::vector<int>         sapPairA;
    static std::vector<int>         sapPairB;

    if ((int)sapItv.size() < bodyCount) sapItv.resize(bodyCount);

    const float bpEpsilon = epsilon + PhysicsConstants::BROADPHASE_MARGIN;
    const float maxReachMargin = PhysicsConstants::SKIN_WIDTH * 2.0f + PhysicsConstants::DEFAULT_TRIANGLE_THICKNESS + bpEpsilon;

    int activeCount = 0;
    for (int i = 0; i < bodyCount; ++i)
    {
        const NativeSoftBodyData& b = bodies[i];
        if (b.nodeCount == 0) continue;
        sapItv[i].minX = b.boundsMin.x - b.radius - maxReachMargin;
        sapItv[i].maxX = b.boundsMax.x + b.radius + maxReachMargin;
        ++activeCount;
    }

    bool reuseOrder = ((int)sapOrder.size() == activeCount);
    if (reuseOrder)
        for (int idx : sapOrder)
            if (idx >= bodyCount || bodies[idx].nodeCount == 0) { reuseOrder = false; break; }

    if (!reuseOrder)
    {
        sapOrder.clear();
        for (int i = 0; i < bodyCount; ++i)
            if (bodies[i].nodeCount > 0) sapOrder.push_back(i);
        std::sort(sapOrder.begin(), sapOrder.end(),
            [&](int a, int b) { return sapItv[a].minX < sapItv[b].minX; });
    }
    else
    {
        for (int a = 1; a < (int)sapOrder.size(); ++a)
        {
            const int key = sapOrder[a];
            const float kmin = sapItv[key].minX;
            int b = a - 1;
            while (b >= 0 && sapItv[sapOrder[b]].minX > kmin) { sapOrder[b + 1] = sapOrder[b]; --b; }
            sapOrder[b + 1] = key;
        }
    }

    sapPairA.clear();
    sapPairB.clear();

    const int sapN = (int)sapOrder.size();
    for (int s = 0; s < sapN; ++s)
    {
        const int ia = sapOrder[s];
        const NativeSoftBodyData& bA = bodies[ia];
        const float aMaxX = sapItv[ia].maxX;

        for (int t = s + 1; t < sapN; ++t)
        {
            const int ib = sapOrder[t];
            if (sapItv[ib].minX > aMaxX) break;

            const NativeSoftBodyData& bB = bodies[ib];

            if (((1 << bB.layer) & bA.collisionLayerMask) == 0) continue;
            if (((1 << bA.layer) & bB.collisionLayerMask) == 0) continue;

            uint64_t key = PackPair(bA.id, bB.id);
            if (!ignoredSorted.empty() &&
                std::binary_search(ignoredSorted.begin(), ignoredSorted.end(), key)) continue;

            if (!g_DisableAllCollisions.empty())
            {
                const bool aDisablesB = g_DisableAllCollisions.count(bA.id) != 0;
                const bool bDisablesA = g_DisableAllCollisions.count(bB.id) != 0;
                if (aDisablesB || bDisablesA) continue;
            }

            const float exp = bA.radius + bB.radius + maxReachMargin;
            if (bA.boundsMin.y > bB.boundsMax.y + exp ||
                bA.boundsMax.y < bB.boundsMin.y - exp) continue;
            if (bA.boundsMin.z > bB.boundsMax.z + exp ||
                bA.boundsMax.z < bB.boundsMin.z - exp) continue;

            if (ia < ib) { sapPairA.push_back(ia); sapPairB.push_back(ib); }
            else { sapPairA.push_back(ib); sapPairB.push_back(ia); }
        }
    }

    const int pairCount = (int)sapPairA.size();
    g_Stats.sapPairCount += pairCount;

    struct PairContacts { int start; int count; };
    static std::vector<NodeContact> contactPool;
    static std::vector<PairContacts> pairContacts;

    pairContacts.assign(pairCount, { 0, 0 });

    const float nnBpEpsilon = epsilon + PhysicsConstants::BROADPHASE_MARGIN;
    const float fnReachExtra = PhysicsConstants::SKIN_WIDTH * 2.0f + PhysicsConstants::DEFAULT_TRIANGLE_THICKNESS + epsilon;

    static std::vector<uint8_t> pairCollidingMask;
    pairCollidingMask.assign(pairCount, 0);

    static std::vector<int> nnActivePair;
    nnActivePair.clear();
    for (int p = 0; p < pairCount; ++p)
    {
        const NativeSoftBodyData& bA = bodies[sapPairA[p]];
        const NativeSoftBodyData& bB = bodies[sapPairB[p]];

        // All bodies here are soft (rigids have their own layer), so every pair is node-node eligible.

        const float marginNN = bA.radius + bB.radius + nnBpEpsilon;
        if (BodyBoundsOverlap(bA.boundsMin, bA.boundsMax, bB.boundsMin, bB.boundsMax, marginNN))
        {
            pairCollidingMask[p] |= 1;
            nnActivePair.push_back(p);
        }

        const float marginFN = bA.radius + bB.radius + fnReachExtra;
        if (BodyBoundsOverlap(bA.boundsMin, bA.boundsMax, bB.boundsMin, bB.boundsMax, marginFN))
        {
            pairCollidingMask[p] |= 2;
        }
    }
    const int nnActiveCount = (int)nnActivePair.size();

    static std::vector<uint8_t> bodyInNNPair;
    bodyInNNPair.assign((size_t)bodyCount, 0);

    float activeMaxBodyRadius = 0.0f;
    for (int i = 0; i < nnActiveCount; ++i)
    {
        const int p = nnActivePair[i];
        bodyInNNPair[sapPairA[p]] = 1;
        bodyInNNPair[sapPairB[p]] = 1;
    }
    for (int b = 0; b < bodyCount; ++b)
        if (bodyInNNPair[b] && bodies[b].radius > activeMaxBodyRadius)
            activeMaxBodyRadius = bodies[b].radius;

    auto tNodeNodeStart = std::chrono::high_resolution_clock::now();

    BuildUniformNodeGrid(bodies, bodyCount, bodyInNNPair.data(), activeMaxBodyRadius, bpEpsilon);

    static std::vector<int> pairOffset;
    static std::vector<int> pairCap;
    pairOffset.assign(pairCount, 0);
    pairCap.assign(pairCount, 0);
    int totalCapacity = 0;
    for (int i = 0; i < nnActiveCount; ++i)
    {
        const int p = nnActivePair[i];
        const NativeSoftBodyData& bA = bodies[sapPairA[p]];
        const NativeSoftBodyData& bB = bodies[sapPairB[p]];
        int maxContacts = 16 * (bA.nodeCount + bB.nodeCount);
        if (maxContacts > PhysicsConstants::MAX_CONTACTS_PER_PAIR)
            maxContacts = PhysicsConstants::MAX_CONTACTS_PER_PAIR;
        if (maxContacts < 0) maxContacts = 0;
        pairCap[p] = maxContacts;
        pairOffset[p] = totalCapacity;
        totalCapacity += maxContacts;
    }
    contactPool.resize(totalCapacity);

    GetGlobalPool().parallel_for(nnActiveCount, [&](int i)
        {
            const int p = nnActivePair[i];
            if (pairCap[p] <= 0) { pairContacts[p] = { pairOffset[p], 0 }; return; }

            const NativeSoftBodyData& bA = bodies[sapPairA[p]];
            const NativeSoftBodyData& bB = bodies[sapPairB[p]];

            const int found = BuildNodeNodeContacts(
                bA, sapPairA[p],
                bB, sapPairB[p],
                g_NodeGrid,
                g_NodeHalfTraj[sapPairA[p]].data(),
                g_NodeHalfTraj[sapPairB[p]].data(),
                bA.radius + bB.radius + bpEpsilon,
                contactPool.data() + pairOffset[p], pairCap[p]);

            pairContacts[p] = { pairOffset[p], found };
        }, 4);
    auto tNodeNodeEnd = std::chrono::high_resolution_clock::now();
    g_Stats.nodeNodeCollisionTimeMs += std::chrono::duration<float, std::milli>(tNodeNodeEnd - tNodeNodeStart).count();

    for (int p = 0; p < pairCount; ++p)
        g_Stats.nodeNodeContactsResolved += pairContacts[p].count;

    (void)faceNodeCellSize; // Spatial hash grid for face-node replaced by dynamic Face BVH

    float nodeNodeResolveMs = 0.0f;
    bool needStaticRecheck = false;

    static std::vector<uint8_t> sleepPinnedScratch;
    int maxNodesForSleep = 0;
    for (int i = 0; i < bodyCount; ++i)
        if (bodies[i].nodeCount > maxNodesForSleep) maxNodesForSleep = bodies[i].nodeCount;
    if ((int)sleepPinnedScratch.size() < maxNodesForSleep)
        sleepPinnedScratch.assign(maxNodesForSleep, 1);

    // Dynamic Face-BVH face-to-node setup
    static std::vector<FaceNodeBVHJobGroup> faceNodeBVHGroups;
    static std::vector<int> faceNodeBVHGroupIndex;
    static std::vector<int> faceNodeWakeA;
    static std::vector<int> faceNodeWakeB;
    faceNodeBVHGroups.clear();
    faceNodeBVHGroupIndex.clear();
    faceNodeWakeA.clear();
    faceNodeWakeB.clear();

    for (int p = 0; p < pairCount; ++p)
    {
        if (pairCollidingMask[p] == 0) continue;

        NativeSoftBodyData& bA = bodies[sapPairA[p]];
        NativeSoftBodyData& bB = bodies[sapPairB[p]];

        if (bA.isSleeping && bB.isSleeping) continue;

        const uint8_t* pinA = bA.isSleeping ? sleepPinnedScratch.data() : bA.isPinned;
        const uint8_t* pinB = bB.isSleeping ? sleepPinnedScratch.data() : bB.isPinned;

        if ((pairCollidingMask[p] & 1) != 0)
        {
            const PairContacts& pc = pairContacts[p];
            if (pc.count > 0)
            {
                needStaticRecheck = true;
                auto rnStart = std::chrono::high_resolution_clock::now();
                const float combinedR = bA.radius + bB.radius + epsilon;
                ResolveNodeNodeContacts(
                    bA.predictedX, bA.predictedY, bA.predictedZ, bA.currentX, bA.currentY, bA.currentZ, bA.masses, pinA,
                    bB.predictedX, bB.predictedY, bB.predictedZ, bB.currentX, bB.currentY, bB.currentZ, bB.masses, pinB,
                    contactPool.data() + pc.start, pc.count,
                    dt, invDt, combinedR,
                    (bA.restitution + bB.restitution) * 0.5f,
                    (bA.staticFriction + bB.staticFriction) * 0.5f,
                    (bA.slidingFriction + bB.slidingFriction) * 0.5f,
                    bA.id, bB.id);
                auto rnEnd = std::chrono::high_resolution_clock::now();
                nodeNodeResolveMs += std::chrono::duration<float, std::milli>(rnEnd - rnStart).count();

                if (bA.isSleeping) bA.wakeRequested = 1;
                if (bB.isSleeping) bB.wakeRequested = 1;
            }
        }

        if ((pairCollidingMask[p] & 2) == 0) continue;

        // A-faces vs B-nodes (soft-soft only; rigids never appear in this list).
        if (bA.faceCount > 0 && bB.nodeCount > 0)
        {
            auto itA = g_BodyFaceBVHs.find(bA.id);
            if (itA != g_BodyFaceBVHs.end() && itA->second.built && !itA->second.nodes.empty())
            {
                FaceNodeBVHJobGroup grp{};
                grp.faces = bA.faces;
                grp.facePredX = bA.predictedX; grp.facePredY = bA.predictedY; grp.facePredZ = bA.predictedZ;
                // NEW: geometry reads use the frozen snapshot, not the live (concurrently-written) arrays
                grp.faceSnapX = snapX[sapPairA[p]].data(); grp.faceSnapY = snapY[sapPairA[p]].data(); grp.faceSnapZ = snapZ[sapPairA[p]].data();
                grp.facePrevX = bA.currentX; grp.facePrevY = bA.currentY; grp.facePrevZ = bA.currentZ;
                grp.faceMasses = bA.masses; grp.faceIsPinned = pinA; grp.faceCount = bA.faceCount;

                grp.nodePredX = bB.predictedX; grp.nodePredY = bB.predictedY; grp.nodePredZ = bB.predictedZ;
                // NEW
                grp.nodeSnapX = snapX[sapPairB[p]].data(); grp.nodeSnapY = snapY[sapPairB[p]].data(); grp.nodeSnapZ = snapZ[sapPairB[p]].data();
                grp.nodePrevX = bB.currentX; grp.nodePrevY = bB.currentY; grp.nodePrevZ = bB.currentZ;
                grp.nodeMasses = bB.masses; grp.nodeIsPinned = pinB; grp.nodeCount = bB.nodeCount;
                grp.nodeStart = (int)faceNodeBVHGroupIndex.size();

                grp.bvh = &itA->second;
                grp.radiusNode = bB.radius + PhysicsConstants::SKIN_WIDTH;
                grp.thickness = bB.radius + PhysicsConstants::SKIN_WIDTH + PhysicsConstants::DEFAULT_TRIANGLE_THICKNESS;
                grp.epsilon = epsilon; grp.dt = dt;
                grp.restitution = (bA.restitution + bB.restitution) * 0.5f;
                grp.staticFriction = (bA.staticFriction + bB.staticFriction) * 0.5f;
                grp.slidingFriction = (bA.slidingFriction + bB.slidingFriction) * 0.5f;
                grp.applyRestitution = doRestitution;
                grp.bodyOwnerIdx = sapPairA[p]; grp.bodyOpposingIdx = sapPairB[p];
                grp.ownerBodyId = bA.id; grp.opposingBodyId = bB.id;

                const int groupIdx = (int)faceNodeBVHGroups.size();
                faceNodeBVHGroupIndex.insert(faceNodeBVHGroupIndex.end(), grp.nodeCount, groupIdx);
                faceNodeBVHGroups.push_back(grp);
                faceNodeWakeA.push_back(sapPairA[p]);
                faceNodeWakeB.push_back(sapPairB[p]);
            }
        }

        // B-faces vs A-nodes (mirror of the block above).
        if (bB.faceCount > 0 && bA.nodeCount > 0)
        {
            auto itB = g_BodyFaceBVHs.find(bB.id);
            if (itB != g_BodyFaceBVHs.end() && itB->second.built && !itB->second.nodes.empty())
            {
                FaceNodeBVHJobGroup grp{};
                grp.faces = bB.faces;
                grp.facePredX = bB.predictedX; grp.facePredY = bB.predictedY; grp.facePredZ = bB.predictedZ;
                // NEW
                grp.faceSnapX = snapX[sapPairB[p]].data(); grp.faceSnapY = snapY[sapPairB[p]].data(); grp.faceSnapZ = snapZ[sapPairB[p]].data();
                grp.facePrevX = bB.currentX; grp.facePrevY = bB.currentY; grp.facePrevZ = bB.currentZ;
                grp.faceMasses = bB.masses; grp.faceIsPinned = pinB; grp.faceCount = bB.faceCount;

                grp.nodePredX = bA.predictedX; grp.nodePredY = bA.predictedY; grp.nodePredZ = bA.predictedZ;
                // NEW
                grp.nodeSnapX = snapX[sapPairA[p]].data(); grp.nodeSnapY = snapY[sapPairA[p]].data(); grp.nodeSnapZ = snapZ[sapPairA[p]].data();
                grp.nodePrevX = bA.currentX; grp.nodePrevY = bA.currentY; grp.nodePrevZ = bA.currentZ;
                grp.nodeMasses = bA.masses; grp.nodeIsPinned = pinA; grp.nodeCount = bA.nodeCount;
                grp.nodeStart = (int)faceNodeBVHGroupIndex.size();

                grp.bvh = &itB->second;
                grp.radiusNode = bA.radius + PhysicsConstants::SKIN_WIDTH;
                grp.thickness = bA.radius + PhysicsConstants::SKIN_WIDTH + PhysicsConstants::DEFAULT_TRIANGLE_THICKNESS;
                grp.epsilon = epsilon; grp.dt = dt;
                grp.restitution = (bA.restitution + bB.restitution) * 0.5f;
                grp.staticFriction = (bA.staticFriction + bB.staticFriction) * 0.5f;
                grp.slidingFriction = (bA.slidingFriction + bB.slidingFriction) * 0.5f;
                grp.applyRestitution = doRestitution;
                grp.bodyOwnerIdx = sapPairB[p]; grp.bodyOpposingIdx = sapPairA[p];
                grp.ownerBodyId = bB.id; grp.opposingBodyId = bA.id;

                const int groupIdx = (int)faceNodeBVHGroups.size();
                faceNodeBVHGroupIndex.insert(faceNodeBVHGroupIndex.end(), grp.nodeCount, groupIdx);
                faceNodeBVHGroups.push_back(grp);
                faceNodeWakeA.push_back(sapPairA[p]);
                faceNodeWakeB.push_back(sapPairB[p]);
            }
        }
    }

    auto tFaceNodeStart = std::chrono::high_resolution_clock::now();
    const int totalNodeJobs = (int)faceNodeBVHGroupIndex.size();
    const int groupCount = (int)faceNodeBVHGroups.size();
    static std::vector<std::atomic<int>> groupResolvedFlag;
    if ((int)groupResolvedFlag.size() < groupCount) groupResolvedFlag = std::vector<std::atomic<int>>(groupCount);
    for (int g = 0; g < groupCount; ++g) groupResolvedFlag[g].store(0, std::memory_order_relaxed);

    int totalCandidates = 0;
    int totalResolved = 0;

    if (totalNodeJobs > 0)
    {
        GetGlobalPool().parallel_for(totalNodeJobs, [&](int globalIdx)
            {
                const int g = faceNodeBVHGroupIndex[globalIdx];
                const FaceNodeBVHJobGroup& grp = faceNodeBVHGroups[g];
                const int n = globalIdx - grp.nodeStart;

                int localCandidates = 0, localResolved = 0;
                ProcessNodeVsFaceBVH(grp, n, localCandidates, localResolved);

                if (localCandidates > 0) AtomicAddInt(&totalCandidates, localCandidates);
                if (localResolved > 0)
                {
                    AtomicAddInt(&totalResolved, localResolved);
                    groupResolvedFlag[g].store(1, std::memory_order_relaxed);
                }
            }, 16);
    }

    for (int g = 0; g < groupCount; ++g)
    {
        if (groupResolvedFlag[g].load(std::memory_order_relaxed) == 0) continue;
        NativeSoftBodyData& owner = bodies[faceNodeWakeA[g]];
        NativeSoftBodyData& opposing = bodies[faceNodeWakeB[g]];
        if (owner.isSleeping) owner.wakeRequested = 1;
        if (opposing.isSleeping) opposing.wakeRequested = 1;
    }

    AtomicAddInt(&g_Stats.faceNodeCandidatesChecked, totalCandidates);
    AtomicAddInt(&g_Stats.faceNodeContactsResolved, totalResolved);
    if (totalResolved > 0) needStaticRecheck = true;

    // ── Second static pass ──
    // Pair pushouts can leave verts below static geometry, so re-resolve rather than start the
    // next substep buried. Velocities come from (pred − curr) / dt inside the resolve.
    if (needStaticRecheck)
    {
        auto tStatic2Start = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < bodyCount; ++i)
        {
            NativeSoftBodyData& b = bodies[i];
            // NOTE: wakeRequested is NOT reset here — the pair pass above just
            // set it, and the wake flags are drained by the caller after this.
            if (b.nodeCount == 0) continue;
            if (b.isSleeping) continue;

            ResolveStaticCollisions_Internal(
                b.predictedX, b.predictedY, b.predictedZ, b.currentX, b.currentY, b.currentZ, b.masses, b.isPinned, b.nodeCount,
                g_StaticColliderData.data(), (int)g_StaticColliderData.size(),
                b.radius + PhysicsConstants::SKIN_WIDTH, epsilon,
                b.restitution, b.staticFriction, b.slidingFriction, dt, doRestitution, b.id);
        }

        auto tStatic2End = std::chrono::high_resolution_clock::now();
        g_Stats.staticCollisionTimeMs += std::chrono::duration<float, std::milli>(tStatic2End - tStatic2Start).count();
    }

    auto tFaceNodeEnd = std::chrono::high_resolution_clock::now();
    const float faceNodeResolveMs = std::chrono::duration<float, std::milli>(tFaceNodeEnd - tFaceNodeStart).count();

    g_Stats.nodeNodeCollisionTimeMs += nodeNodeResolveMs;
    g_Stats.faceNodeCollisionTimeMs += faceNodeResolveMs;

    auto t1 = std::chrono::high_resolution_clock::now();
    g_Stats.collisionTimeMs += std::chrono::duration<float, std::milli>(t1 - t0).count();
}
EXPORT int Collisions_GetContactEventCount()
{
    int n = g_ContactEventCount.load(std::memory_order_relaxed);
    return n > MAX_CONTACT_EVENTS ? MAX_CONTACT_EVENTS : n;
}

EXPORT void Collisions_GetContactEvents(ContactEvent* outEvents, int maxCount)
{
    if (!outEvents || maxCount <= 0) { Error_SetError(ErrorCode::InvalidArgument); return; }
    int n = Collisions_GetContactEventCount();
    if (maxCount < n) n = maxCount;
    std::memcpy(outEvents, g_ContactEvents, sizeof(ContactEvent) * n);
}

EXPORT bool Collisions_GetContactEvent(int index, ContactEvent* outEvent)
{
    if (!outEvent) { Error_SetError(ErrorCode::InvalidArgument); return false; }
    int n = Collisions_GetContactEventCount();
    if (index < 0 || index >= n) { Error_SetError(ErrorCode::InvalidNodeIndex); return false; }

    *outEvent = g_ContactEvents[index];
    return true;
}


// Debug / introspection EXPORTs

EXPORT int Collisions_GetStaticBodyCount()
{
    return (int)g_StaticBodies.size();
}

// Copies up to maxIds static-body ids into outIds and returns how many were written.
// Lets the debugger iterate real ids instead of probing (probing latched UnknownHandle).
EXPORT int Collisions_GetStaticBodyIds(int* outIds, int maxIds)
{
    if (!outIds || maxIds <= 0) return 0;

    int n = 0;
    for (const auto& kv : g_StaticBodies)
    {
        if (n >= maxIds) break;
        outIds[n++] = kv.first;
    }
    return n;
}

EXPORT bool Collisions_GetStaticBodyDebugInfo(
    int id,
    int* outType, int* outLayer,
    float* outFrictionStatic, float* outFrictionSliding, float* outRestitution,
    float outCenter[3], float outSize[3],
    float* outRadius, float* outHeight,
    int* outVertexCount, int* outIndexCount)
{
    auto it = g_StaticBodies.find(id);
    if (it == g_StaticBodies.end()) return false;   // introspection miss: not an engine error

    const NativeStaticCollider& sc = it->second;
    if (outType)            *outType = (int)sc.type;
    if (outLayer)           *outLayer = sc.layer;
    if (outFrictionStatic)  *outFrictionStatic = sc.frictionStatic;
    if (outFrictionSliding) *outFrictionSliding = sc.frictionSliding;
    if (outRestitution)     *outRestitution = sc.restitution;
    if (outCenter)          for (int i = 0; i < 3; ++i) outCenter[i] = sc.center[i];
    if (outSize)            for (int i = 0; i < 3; ++i) outSize[i] = sc.size[i];
    if (outRadius)          *outRadius = sc.radius;
    if (outHeight)          *outHeight = sc.height;
    if (outVertexCount)     *outVertexCount = (int)(sc.vertices.size() / 3);
    if (outIndexCount)      *outIndexCount = (int)sc.indices.size();
    return true;
}

// Kept for ABI: "grid" is now a BVH, so outTableSize reports node count and oversized is always 0.
EXPORT void Collisions_GetStaticGridDebugInfo(
    int* outTableSize, int* outOversizedColliderCount, float* outCellSize, int* outGridDirty)
{
    if (outTableSize)              *outTableSize = (int)g_StaticBVHNodes.size();
    if (outOversizedColliderCount) *outOversizedColliderCount = 0;
    if (outCellSize)                *outCellSize = 0.0f;
    if (outGridDirty)               *outGridDirty = g_StaticsDirty ? 1 : 0;
}

// Internal helpers

static bool CheckSweptFaceNode(const Vector3& p0, const Vector3& p1, float radius,
    const Vector3& a0, const Vector3& a1,
    const Vector3& b0, const Vector3& b1,
    const Vector3& c0, const Vector3& c1,
    float& tOut, Vector3& nOut)
{
    Vector3 n0 = Normalize(Cross(Sub(b0, a0), Sub(c0, a0)));
    Vector3 n1 = Normalize(Cross(Sub(b1, a1), Sub(c1, a1)));

    if (LengthSq(n0) < 0.5f || LengthSq(n1) < 0.5f) return false;

    float d0 = Dot(Sub(p0, a0), n0);
    float d1 = Dot(Sub(p1, a1), n1);

    bool startInside = AbsF(d0) < radius;
    bool crossed = (d0 > 0.0f) != (d1 > 0.0f);

    if (!crossed && !startInside) return false;

    if (startInside)
    {
        float u, v, w;
        if (!GetBarycentric(p0, a0, b0, c0, u, v, w)) return false;
        tOut = 0.0f;
        nOut = d0 >= 0.0f ? n0 : Neg(n0);
        return true;
    }

    float ad0 = AbsF(d0), ad1 = AbsF(d1);
    float t = ClampF(ad0 / (ad0 + ad1), 0.0f, 1.0f);

    Vector3 pAtT = Lerp(p0, p1, t);
    Vector3 aAtT = Lerp(a0, a1, t);
    Vector3 bAtT = Lerp(b0, b1, t);
    Vector3 cAtT = Lerp(c0, c1, t);

    float u, v, w;
    if (!GetBarycentric(pAtT, aAtT, bAtT, cAtT, u, v, w)) return false;

    tOut = t;
    Vector3 nAtT = Normalize(Lerp(n0, n1, t));
    nOut = d0 > 0.0f ? nAtT : Neg(nAtT);
    return true;
}

static bool GetBarycentric(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c,
    float& u, float& v, float& w)
{
    Vector3 v0 = Sub(b, a);
    Vector3 v1 = Sub(c, a);
    Vector3 v2 = Sub(p, a);
    // 5 dot products as 2 SIMD pairs + 1 single instead of 5 separate _mm_dp_ps calls.
    Vector3 d0x = DotPair(v0, v0, v1); // d0x.x = d00, d0x.y = d01
    Vector3 d2x = DotPair(v2, v0, v1); // d2x.x = d20, d2x.y = d21
    float d00 = d0x.x, d01 = d0x.y, d11 = Dot(v1, v1);
    float d20 = d2x.x, d21 = d2x.y;
    float denom = d00 * d11 - d01 * d01;
    if (AbsF(denom) < 1e-9f) { u = v = w = 0.0f; return false; }
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0f - v - w;
    return (u >= -0.05f && v >= -0.05f && w >= -0.05f && (u + v + w) <= 1.05f);
}

static void ApplyXPBDFaceNode(int idxP, int idxA, int idxB, int idxC,
    const Vector3& normal, float penetration,
    float u, float v, float w,
    float* nodePredX, float* nodePredY, float* nodePredZ,
    const float* nodePrevX, const float* nodePrevY, const float* nodePrevZ,
    float* facePredX, float* facePredY, float* facePredZ,
    const float* facePrevX, const float* facePrevY, const float* facePrevZ,
    const float* nodeMasses, const float* faceMasses,
    const uint8_t* nodePinned, const uint8_t* facePinned, const Vector3& contactRelativeVelocity,
    float dt, float restitution, float staticFriction, float slidingFriction, bool applyRestitution)
{
    auto inverseMass = [](const float* masses,
        const uint8_t* pinned,
        int index) -> float
        {
            if (pinned && pinned[index]) return 0.0f;

            const float mass = masses[index];
            return std::isfinite(mass) && mass > 0.0f
                ? 1.0f / mass
                : 0.0f;
        };

    const float wP = inverseMass(nodeMasses, nodePinned, idxP);
    const float wA = inverseMass(faceMasses, facePinned, idxA);
    const float wB = inverseMass(faceMasses, facePinned, idxB);
    const float wC = inverseMass(faceMasses, facePinned, idxC);


    float wTri = (u * u * wA) + (v * v * wB) + (w * w * wC);
    float wSum = wP + wTri;
    if (wSum <= 1e-9f) return;

    auto applyDelta = [&](int idx, float weight, const Vector3& delta, float* pX, float* pY, float* pZ) {
        if (weight == 0.0f) return;
        AtomicAddFloat(&pX[idx], delta.x * weight);
        AtomicAddFloat(&pY[idx], delta.y * weight);
        AtomicAddFloat(&pZ[idx], delta.z * weight);
        };

    float dLambda = penetration / wSum;
    Vector3 corr = Scale(normal, dLambda);

    applyDelta(idxP, wP, corr, nodePredX, nodePredY, nodePredZ);
    applyDelta(idxA, -wA * u, corr, facePredX, facePredY, facePredZ);
    applyDelta(idxB, -wB * v, corr, facePredX, facePredY, facePredZ);
    applyDelta(idxC, -wC * w, corr, facePredX, facePredY, facePredZ);

    if (dt <= 0.0f) return;

    // RelVel is snapshot-derived; reading the live arrays here would race with the atomic adds.
    const Vector3 relVel = contactRelativeVelocity;
    float vn = Dot(relVel, normal);
    if (vn >= 0.0f) return;

    float jn = 0.0f;
    if (applyRestitution)
    {
        jn = -(1.0f + restitution) * vn / wSum;
        Vector3 impulse = Scale(normal, jn * dt);

        applyDelta(idxP, wP, impulse, nodePredX, nodePredY, nodePredZ);
        applyDelta(idxA, -wA * u, impulse, facePredX, facePredY, facePredZ);
        applyDelta(idxB, -wB * v, impulse, facePredX, facePredY, facePredZ);
        applyDelta(idxC, -wC * w, impulse, facePredX, facePredY, facePredZ);
    }
    else
    {
        jn = -vn / wSum;
    }

    Vector3 vt = Sub(relVel, Scale(normal, vn));
    float vtSpeed = Length(vt);
    if (vtSpeed > PhysicsConstants::MIN_TANGENT_SPEED)
    {
        Vector3 tdir = Scale(vt, 1.0f / vtSpeed);
        float jt = vtSpeed / wSum;
        float fMag = (jt <= staticFriction * jn) ? jt : MinF(slidingFriction * jn, jt);
        Vector3 fImpulse = Scale(tdir, fMag * dt);

        applyDelta(idxP, -wP, fImpulse, nodePredX, nodePredY, nodePredZ);
        applyDelta(idxA, wA * u, fImpulse, facePredX, facePredY, facePredZ);
        applyDelta(idxB, wB * v, fImpulse, facePredX, facePredY, facePredZ);
        applyDelta(idxC, wC * w, fImpulse, facePredX, facePredY, facePredZ);
    }
}

// Swept primitive tests

static bool SweptSphereSphere(const Vector3& start, const Vector3& vel, float radiusA,
    const StaticColliderData& sphere,
    float& outT, Vector3& outPoint, Vector3& outNormal)
{
    outT = std::numeric_limits<float>::max();
    outPoint = V3Zero();
    outNormal = V3(0.0f, 1.0f, 0.0f);

    float combined = radiusA + sphere.radius;
    Vector3 p = Sub(start, sphere.center);
    float a = Dot(vel, vel);
    float b = 2.0f * Dot(p, vel);
    float c = Dot(p, p) - combined * combined;

    if (c <= 0.0f) {
        outT = 0.0f;
        float pLen = Length(p);
        outNormal = pLen > 1e-7f ? Normalize(p) : V3(0.0f, 1.0f, 0.0f);
        outPoint = Add(sphere.center, Scale(outNormal, sphere.radius));
        return true;
    }

    if (a < 1e-12f) return false;
    float disc = b * b - 4.0f * a * c;
    if (disc < 0.0f) return false;

    float t = (-b - std::sqrt(disc)) / (2.0f * a);
    if (t < 0.0f || t > 1.0f) return false;

    outT = t;
    Vector3 hp = Add(start, Scale(vel, t));
    outNormal = Normalize(Sub(hp, sphere.center));
    outPoint = Add(sphere.center, Scale(outNormal, sphere.radius));
    return true;
}

static bool SweptSphereBox(const Vector3& start, const Vector3& vel, float radius,
    const StaticColliderData& box,
    float& outT, Vector3& outPoint, Vector3& outNormal)
{
    outT = std::numeric_limits<float>::max();
    outPoint = V3Zero();
    outNormal = V3(0.0f, 1.0f, 0.0f);

    Quaternion invRot = QInverse(box.rotation);
    Vector3 ls = QRotate(invRot, Sub(start, box.center));
    Vector3 lv = QRotate(invRot, vel);

    Vector3 half = { box.size.x * 0.5f + radius,
                     box.size.y * 0.5f + radius,
                     box.size.z * 0.5f + radius, 0.0f };
    Vector3 boxMin = Neg(half);
    Vector3 boxMax = half;

    float dx = half.x - AbsF(ls.x);
    float dy = half.y - AbsF(ls.y);
    float dz = half.z - AbsF(ls.z);

    if (dx > 0.0f && dy > 0.0f && dz > 0.0f) {
        // Actual box half-extents (without radius expansion)
        float halfX = box.size.x * 0.5f;
        float halfY = box.size.y * 0.5f;
        float halfZ = box.size.z * 0.5f;

        // Closest point on the *actual* box to the sphere center
        Vector3 localSurface = {
            ClampF(ls.x, -halfX, halfX),
            ClampF(ls.y, -halfY, halfY),
            ClampF(ls.z, -halfZ, halfZ),
            0.0f
        };

        // Vector from closest surface point -> sphere center
        Vector3 localNormal = Sub(ls, localSurface);
        float normalLenSq = LengthSq(localNormal);

        if (normalLenSq > 1e-12f) {
            // The radius-expanded AABB includes corners outside the actual rounded box.
            if (normalLenSq > radius * radius)
            {
                // No initial overlap reported; an exact rounded-box sweep would be needed.
            }
            else
            {
                outNormal = QRotate(
                    box.rotation,
                    Scale(localNormal, 1.0f / std::sqrt(normalLenSq)));

                outPoint = Add(
                    box.center, QRotate(box.rotation, localSurface));

                outT = 0.0f;
                return true;
            }
        }
        else {
            // Sphere center is deep inside; fallback to minimum axis.
            Vector3 ln = V3Zero();

            if (dx <= dy && dx <= dz) {
                ln.x = ls.x >= 0.0f ? 1.0f : -1.0f;
                localSurface.x = ln.x * halfX;
            }
            else if (dy <= dz) {
                ln.y = ls.y >= 0.0f ? 1.0f : -1.0f;
                localSurface.y = ln.y * halfY;
            }
            else {
                ln.z = ls.z >= 0.0f ? 1.0f : -1.0f;
                localSurface.z = ln.z * halfZ;
            }

            outNormal = QRotate(box.rotation, ln);
            outPoint = Add(box.center, QRotate(box.rotation, localSurface));

            outT = 0.0f;
            return true;
        }

        // Rejected corner candidates fall through to the sweep below, which is still a
        // conservative approximation of sphere-vs-rounded-box — it can report early edge contacts.
    }

    float tMin = -1e30f, tMax = 1.0f; // Initialize tMin to negative infinity
    int hitAxis = -1, hitSign = 0;
    const float p[3] = { ls.x, ls.y, ls.z };
    const float v[3] = { lv.x, lv.y, lv.z };
    const float lo[3] = { boxMin.x, boxMin.y, boxMin.z };
    const float hi[3] = { boxMax.x, boxMax.y, boxMax.z };

    for (int i = 0; i < 3; ++i)
    {
        if (AbsF(v[i]) < 1e-6f)
        {
            if (p[i] < lo[i] || p[i] > hi[i]) return false;
        }
        else
        {
            float t1 = (lo[i] - p[i]) / v[i];
            float t2 = (hi[i] - p[i]) / v[i];
            if (t1 > t2) std::swap(t1, t2);
            if (t1 > tMin) { tMin = t1; hitAxis = i; hitSign = v[i] > 0.0f ? -1 : 1; }
            if (t2 < tMax) tMax = t2;
            if (tMin > tMax + 1e-5f) return false;
        }
    }

    if (tMax < 0.0f || tMin > 1.0f || hitAxis == -1) return false;

    outT = MaxF(0.0f, tMin);
    Vector3 ln = V3Zero();
    if (hitAxis == 0) ln.x = (float)hitSign;
    else if (hitAxis == 1) ln.y = (float)hitSign;
    else if (hitAxis == 2) ln.z = (float)hitSign;
    outNormal = QRotate(box.rotation, ln);
    outPoint = Sub(Add(start, Scale(vel, outT)), Scale(outNormal, radius));
    return true;
}

// Slab clip of the segment start → start+vel (t in [0,1]); returns the inside range.

static bool ClipSegmentToAABB(const Vector3& start, const Vector3& vel,
    const Vector3& boxMin, const Vector3& boxMax,
    float& tEnter, float& tExit)
{
    tEnter = 0.0f;
    tExit = 1.0f;

    auto clipAxis = [&](float p, float v, float lo, float hi) -> bool
        {
            if (AbsF(v) < 1e-9f)
                return p >= lo && p <= hi;

            float t1 = (lo - p) / v;
            float t2 = (hi - p) / v;
            if (t1 > t2) std::swap(t1, t2);
            tEnter = MaxF(tEnter, t1);
            tExit = MinF(tExit, t2);
            return tEnter <= tExit;
        };

    if (!clipAxis(start.x, vel.x, boxMin.x, boxMax.x)) return false;
    if (!clipAxis(start.y, vel.y, boxMin.y, boxMax.y)) return false;
    if (!clipAxis(start.z, vel.z, boxMin.z, boxMax.z)) return false;
    return true;
}

static bool SweptSphereTriangle(
    const Vector3& start,
    const Vector3& vel,
    float radius,
    const StaticColliderData& tri,
    float& outT,
    Vector3& outPoint,
    Vector3& outNormal)
{
    outT = std::numeric_limits<float>::max();
    outPoint = V3Zero();
    outNormal = V3(0.0f, 1.0f, 0.0f);

    if (radius < 0.0f)
        return false;

    const float radiusSq = radius * radius;

    Vector3 e0 = Sub(tri.v1, tri.v0);
    Vector3 e1 = Sub(tri.v2, tri.v0);

    Vector3 normalRaw = Cross(e0, e1);
    float normalLenSq = LengthSq(normalRaw);

    if (normalLenSq < 1e-12f)
        return false;

    float normalLen = std::sqrt(normalLenSq);
    Vector3 n = Scale(normalRaw, 1.0f / normalLen);

    const float velLenSq = LengthSq(vel);

    // Initial overlap

    {
        Vector3 closest = ClosestPointOnTriangle(start, tri.v0, tri.v1, tri.v2);
        Vector3 diff = Sub(start, closest);
        float distSq = LengthSq(diff);

        if (distSq <= radiusSq)
        {
            float dist = std::sqrt(MaxF(distSq, 0.0f));

            outT = 0.0f;
            outPoint = closest;

            if (dist > 1e-7f)
            {
                outNormal = Scale(diff, 1.0f / dist);
            }
            else
            {
                float side = Dot(
                    Sub(start, tri.v0),
                    n
                );

                outNormal = side >= 0.0f ? n : Neg(n);
            }

            return true;
        }
    }

    if (velLenSq < 1e-16f)
        return false;

    bool found = false;
    float bestT = std::numeric_limits<float>::max();
    Vector3 bestPoint = V3Zero();
    Vector3 bestNormal = n;

    auto Consider =
        [&](float t, const Vector3& point, const Vector3& normal)
        {
            if (t < 0.0f || t > 1.0f)
                return;

            if (t >= bestT)
                return;

            bestT = t;
            bestPoint = point;
            bestNormal = normal;
            found = true;
        };

    // 1. TRIANGLE FACE CCD

    {
        float d0 = Dot(Sub(start, tri.v0), n);
        float dn = Dot(vel, n);

        if (AbsF(dn) > 1e-8f)
        {
            // Test both sides of the triangle.
            for (int side = -1; side <= 1; side += 2)
            {
                float target = radius * (float)side;

                float t = (target - d0) / dn;

                if (t < 0.0f || t > 1.0f)
                    continue;

                Vector3 center = Add(start, Scale(vel, t));

                // Project center onto the triangle plane.
                Vector3 projected = Sub(
                    center,
                    Scale(n, target)
                );

                // Barycentric inside-triangle test.
                Vector3 c0 = Cross(
                    e0,
                    Sub(projected, tri.v0)
                );

                Vector3 c1 = Cross(
                    Sub(tri.v2, tri.v1),
                    Sub(projected, tri.v1)
                );

                Vector3 e2 = Sub(tri.v0, tri.v2);

                Vector3 c2 = Cross(
                    e2,
                    Sub(projected, tri.v2)
                );

                float s0 = Dot(c0, n);
                float s1 = Dot(c1, n);
                float s2 = Dot(c2, n);

                const float eps = 1e-6f;

                if (s0 >= -eps &&
                    s1 >= -eps &&
                    s2 >= -eps)
                {
                    Vector3 normal =
                        side > 0 ? n : Neg(n);

                    Vector3 contactPoint =
                        Sub(center, Scale(normal, radius));

                    Consider(
                        t,
                        contactPoint,
                        normal
                    );
                }
            }
        }
    }

    // 2. EDGE CAPSULE CCD

    auto SweepSpherePoint =
        [&](const Vector3& point,
            float& hitT,
            Vector3& hitPoint,
            Vector3& hitNormal) -> bool
        {
            Vector3 m = Sub(start, point);

            float a = Dot(vel, vel);
            float b = 2.0f * Dot(m, vel);
            float c = Dot(m, m) - radiusSq;

            if (c <= 0.0f)
            {
                float dist = std::sqrt(
                    MaxF(Dot(m, m), 0.0f)
                );

                hitT = 0.0f;
                hitPoint = point;

                if (dist > 1e-7f)
                    hitNormal = Scale(m, 1.0f / dist);
                else
                    hitNormal = n;

                return true;
            }

            float discriminant = b * b - 4.0f * a * c;

            if (discriminant < 0.0f)
                return false;

            float sqrtD = std::sqrt(
                MaxF(discriminant, 0.0f)
            );

            float inv2a = 0.5f / a;

            float t0 = (-b - sqrtD) * inv2a;
            float t1 = (-b + sqrtD) * inv2a;

            float t = std::numeric_limits<float>::max();

            if (t0 >= 0.0f && t0 <= 1.0f)
                t = t0;
            else if (t1 >= 0.0f && t1 <= 1.0f)
                t = t1;
            else
                return false;

            Vector3 center = Add(
                start,
                Scale(vel, t)
            );

            Vector3 diff = Sub(center, point);
            float distSq = LengthSq(diff);

            if (distSq > 1e-12f)
            {
                float invDist =
                    1.0f / std::sqrt(distSq);

                hitNormal =
                    Scale(diff, invDist);
            }
            else
            {
                hitNormal = n;
            }

            hitT = t;
            hitPoint =
                Sub(center, Scale(hitNormal, radius));

            return true;
        };

    auto SweepSphereCapsule =
        [&](const Vector3& a,
            const Vector3& b) -> void
        {
            Vector3 ab = Sub(b, a);
            float abLenSq = LengthSq(ab);

            // Degenerate edge -> point.
            if (abLenSq < 1e-12f)
            {
                float t;
                Vector3 p;
                Vector3 nn;

                if (SweepSpherePoint(a, t, p, nn))
                    Consider(t, p, nn);

                return;
            }

            float abLen = std::sqrt(abLenSq);
            Vector3 axis = Scale(ab, 1.0f / abLen);

            Vector3 m = Sub(start, a);

            float md = Dot(m, axis);
            float nd = Dot(vel, axis);

            Vector3 mPerp =
                Sub(m, Scale(axis, md));

            Vector3 vPerp =
                Sub(vel, Scale(axis, nd));

            float A = Dot(vPerp, vPerp);
            float B = 2.0f * Dot(mPerp, vPerp);
            float C = Dot(mPerp, mPerp) - radiusSq;

            if (C <= 0.0f)
            {
                const float projection = ClampF(md, 0.0f, abLen);
                const Vector3 closest = Add(a, Scale(axis, projection));
                const Vector3 diff = Sub(start, closest);
                const float distSq = LengthSq(diff);

                // The infinite cylinder test is only a candidate; require overlap with the finite edge.
                if (distSq <= radiusSq)
                {
                    const Vector3 nn = distSq > 1e-12f
                        ? Scale(diff, 1.0f / std::sqrt(distSq))
                        : n;

                    Consider(0.0f, closest, nn);
                }
            }
            else if (A > 1e-16f)
            {
                float discriminant =
                    B * B - 4.0f * A * C;

                if (discriminant >= 0.0f)
                {
                    float sqrtD =
                        std::sqrt(
                            MaxF(discriminant, 0.0f)
                        );

                    float inv2A =
                        0.5f / A;

                    float roots[2] =
                    {
                        (-B - sqrtD) * inv2A,
                        (-B + sqrtD) * inv2A
                    };

                    for (int r = 0; r < 2; ++r)
                    {
                        float t = roots[r];

                        if (t < 0.0f || t > 1.0f)
                            continue;

                        Vector3 center =
                            Add(
                                start,
                                Scale(vel, t)
                            );

                        float axial =
                            Dot(
                                Sub(center, a),
                                axis
                            );

                        // The quadratic describes the infinite cylinder; accept only the finite edge.
                        if (axial < 0.0f ||
                            axial > abLen)
                        {
                            continue;
                        }

                        Vector3 closest =
                            Add(
                                a,
                                Scale(axis, axial)
                            );

                        Vector3 diff =
                            Sub(center, closest);

                        float diffSq =
                            LengthSq(diff);

                        Vector3 nn;

                        if (diffSq > 1e-12f)
                        {
                            nn = Scale(
                                diff,
                                1.0f /
                                std::sqrt(diffSq)
                            );
                        }
                        else
                        {
                            nn = n;
                        }

                        Vector3 contact =
                            Sub(
                                center,
                                Scale(nn, radius)
                            );

                        Consider(
                            t,
                            contact,
                            nn
                        );

                        // Earliest root is the only one we need.
                        break;
                    }
                }
            }

            // The finite capsule also contains two spherical caps.
            {
                float t;
                Vector3 p;
                Vector3 nn;

                if (SweepSpherePoint(a, t, p, nn))
                    Consider(t, p, nn);

                if (SweepSpherePoint(b, t, p, nn))
                    Consider(t, p, nn);
            }
        };

    SweepSphereCapsule(tri.v0, tri.v1);
    SweepSphereCapsule(tri.v1, tri.v2);
    SweepSphereCapsule(tri.v2, tri.v0);

    // Final result

    if (!found)
        return false;

    outT = bestT;
    outPoint = bestPoint;
    outNormal = bestNormal;

    // Ensure the normal is normalized.
    float normalSq = LengthSq(outNormal);

    if (normalSq > 1e-12f)
    {
        outNormal = Scale(
            outNormal,
            1.0f / std::sqrt(normalSq)
        );
    }
    else
    {
        outNormal = n;
    }

    return true;
}

static bool SweptSphereCapsule(const Vector3& start, const Vector3& vel, float radius,
    const StaticColliderData& capsule,
    float& outT, Vector3& outPoint, Vector3& outNormal)
{
    outT = std::numeric_limits<float>::max();
    outPoint = V3Zero();
    outNormal = V3(0.0f, 1.0f, 0.0f);

    float halfH = MaxF(0.0f, (capsule.height * 0.5f) - capsule.radius);
    Vector3 p1 = Add(capsule.center, Scale(capsule.axis, halfH));
    Vector3 p2 = Sub(capsule.center, Scale(capsule.axis, halfH));
    Vector3 segDir = Sub(p2, p1);
    float segLen = Length(segDir);

    if (segLen < 1e-6f)
    {
        StaticColliderData proxy = capsule;
        proxy.center = capsule.center;
        return SweptSphereSphere(start, vel, radius, proxy, outT, outPoint, outNormal);
    }
    segDir = Scale(segDir, 1.0f / segLen);

    float combinedR = radius + capsule.radius;

    Vector3 dStart = Sub(start, p1);
    float radialDistSq = LengthSq(dStart) - (Dot(dStart, segDir) * Dot(dStart, segDir));
    float projStart = Dot(dStart, segDir);
    if (radialDistSq < combinedR * combinedR && projStart >= 0.0f && projStart <= segLen) {
        outT = 0.0f;
        Vector3 closest = Add(p1, Scale(segDir, projStart));
        Vector3 pushDir = Sub(start, closest);
        float pushLen = Length(pushDir);
        outNormal = pushLen > 1e-7f ? Normalize(pushDir) : V3(0.0f, 1.0f, 0.0f);
        outPoint = Add(closest, Scale(outNormal, capsule.radius));
        return true;
    }

    Vector3 d = Sub(start, p1);
    float   a = LengthSq(vel) - (Dot(vel, segDir) * Dot(vel, segDir));
    float   b = 2.0f * (Dot(vel, d) - Dot(vel, segDir) * Dot(d, segDir));
    float   c = LengthSq(d) - (Dot(d, segDir) * Dot(d, segDir))
        - combinedR * combinedR;

    bool hitCylinder = false;
    float tCyl = std::numeric_limits<float>::max();

    if (AbsF(a) > 1e-12f)
    {
        float disc = b * b - 4.0f * a * c;
        if (disc >= 0.0f)
        {
            float t = (-b - std::sqrt(disc)) / (2.0f * a);
            if (t >= 0.0f && t <= 1.0f)
            {
                Vector3 hitPos = Add(start, Scale(vel, t));
                float   proj = Dot(Sub(hitPos, p1), segDir);
                if (proj >= 0.0f && proj <= segLen)
                {
                    tCyl = t;
                    hitCylinder = true;
                }
            }
        }
    }

    StaticColliderData cap1 = {}; cap1.center = p1; cap1.radius = capsule.radius;
    StaticColliderData cap2 = {}; cap2.center = p2; cap2.radius = capsule.radius;
    float t1, t2; Vector3 p1out, p2out, n1, n2;
    bool h1 = SweptSphereSphere(start, vel, radius, cap1, t1, p1out, n1);
    bool h2 = SweptSphereSphere(start, vel, radius, cap2, t2, p2out, n2);

    float bestT = std::numeric_limits<float>::max();
    if (hitCylinder && tCyl < bestT) bestT = tCyl;
    if (h1 && t1 < bestT)           bestT = t1;
    if (h2 && t2 < bestT)           bestT = t2;
    if (bestT > 1.0f)               return false;

    outT = bestT;
    if (hitCylinder && bestT == tCyl)
    {
        Vector3 hitPos = Add(start, Scale(vel, bestT));
        Vector3 closest = Add(p1, Scale(segDir, ClampF(Dot(Sub(hitPos, p1), segDir), 0.0f, segLen)));
        outNormal = Normalize(Sub(hitPos, closest));
        outPoint = Add(closest, Scale(outNormal, capsule.radius));
    }
    else if (h1 && bestT == t1) { outNormal = n1; outPoint = p1out; }
    else { outNormal = n2; outPoint = p2out; }
    return true;
}

EXPORT bool Collisions_Raycast(Vector3 origin, Vector3 direction, float maxDistance, int ignoreId,
    int* outHitId, Vector3* outPoint, Vector3* outNormal, int* outFaceIndex)
{
    if (outHitId) *outHitId = -1;
    if (outPoint) *outPoint = V3Zero();
    if (outNormal) *outNormal = V3Zero();
    if (outFaceIndex) *outFaceIndex = -1;

    float dirLen = Length(direction);
    if (maxDistance <= 0.0f || dirLen <= 1e-12f) { Error_SetError(ErrorCode::InvalidArgument); return false; }
    Vector3 dir = Scale(direction, 1.0f / dirLen);
    Vector3 vel = Scale(dir, maxDistance);

    bool found = false;
    float bestT = 1.0f; // parametric along vel, i.e. fraction of maxDistance
    int bestId = -1;
    int bestFace = -1;
    Vector3 bestPoint = V3Zero(), bestNormal = V3Zero();

    for (const auto& kv : g_StaticBodies)
    {
        const NativeStaticCollider& src = kv.second;
        if (src.id == ignoreId) continue;

        float t; Vector3 p, n;
        bool hit = false;
        int face = -1;

        switch (src.type)
        {
        case StaticType::Sphere:
        {
            StaticColliderData sc = {};
            sc.center = V3(src.center[0], src.center[1], src.center[2]);
            sc.radius = src.radius;
            hit = SweptSphereSphere(origin, vel, 0.0f, sc, t, p, n);
            break;
        }
        case StaticType::Box:
        {
            StaticColliderData sc = {};
            sc.center = V3(src.center[0], src.center[1], src.center[2]);
            sc.size = V3(src.size[0], src.size[1], src.size[2]);
            sc.rotation = Quaternion{ src.rotation[0], src.rotation[1], src.rotation[2], src.rotation[3] };
            hit = SweptSphereBox(origin, vel, 0.0f, sc, t, p, n);
            break;
        }
        case StaticType::Capsule:
        {
            StaticColliderData sc = {};
            sc.center = V3(src.center[0], src.center[1], src.center[2]);
            sc.radius = src.radius;
            sc.height = src.height;
            sc.axis = V3(src.axis[0], src.axis[1], src.axis[2]);
            hit = SweptSphereCapsule(origin, vel, 0.0f, sc, t, p, n);
            break;
        }
        case StaticType::Mesh:
        {
            // No broad-phase here
            int triCount = (int)src.indices.size() / 3;
            for (int tIdx = 0; tIdx < triCount; ++tIdx)
            {
                int i0 = src.indices[tIdx * 3 + 0], i1 = src.indices[tIdx * 3 + 1], i2 = src.indices[tIdx * 3 + 2];
                StaticColliderData sc = {};
                sc.v0 = V3(src.vertices[i0 * 3], src.vertices[i0 * 3 + 1], src.vertices[i0 * 3 + 2]);
                sc.v1 = V3(src.vertices[i1 * 3], src.vertices[i1 * 3 + 1], src.vertices[i1 * 3 + 2]);
                sc.v2 = V3(src.vertices[i2 * 3], src.vertices[i2 * 3 + 1], src.vertices[i2 * 3 + 2]);

                float triT; Vector3 triP, triN;
                if (SweptSphereTriangle(origin, vel, 0.0f, sc, triT, triP, triN) && triT < bestT)
                {
                    hit = true; t = triT; p = triP; n = triN; face = tIdx;
                    bestT = triT; // keep the running best across triangles directly
                }
            }
            break;
        }
        case StaticType::Terrain:
        {
            StaticColliderData sc = {};
            sc.center = V3(src.center[0], src.center[1], src.center[2]);
            sc.size = V3(src.size[0], src.size[1], src.size[2]);
            sc.terrainWidth = src.terrainWidth;
            sc.terrainLength = src.terrainLength;
            sc.terrainHeights = const_cast<float*>(src.terrainHeights.data());
            hit = SweptSphereTerrain(origin, vel, 0.0f, sc, t, p, n);
            break;
        }
        }

        if (!hit) continue;

        // Mesh already folded its own best-triangle comparison into bestT; commit only for the rest.
        if (src.type == StaticType::Mesh)
        {
            if (face == -1) continue; // no triangle beat the current best
            found = true;
            bestId = src.id; bestFace = face; bestPoint = p; bestNormal = n;
        }
        else if (t < bestT)
        {
            bestT = t;
            found = true;
            bestId = src.id; bestFace = -1; bestPoint = p; bestNormal = n;
        }
    }

    if (!found) return false;

    if (outHitId) *outHitId = bestId;
    if (outPoint) *outPoint = bestPoint;
    if (outNormal) *outNormal = bestNormal;
    if (outFaceIndex) *outFaceIndex = bestFace;
    return true;
}

EXPORT bool World_Raycast(
    Vector3 origin, Vector3 direction, float maxDistance,
    int ignoreStaticId, SoftBodyInstance* ignoreSoftBody,
    SoftBodyInstance** softBodies, int softBodyCount, float nodeRadius,
    int* outHitType, int* outStaticId, SoftBodyInstance** outSoftBody, int* outNodeIndex,
    Vector3* outPoint, Vector3* outNormal, int* outFaceIndex, float* outDistance)
{
    if (outHitType)   *outHitType = WORLD_HIT_NONE;
    if (outStaticId)  *outStaticId = -1;
    if (outSoftBody)  *outSoftBody = nullptr;
    if (outNodeIndex) *outNodeIndex = -1;
    if (outPoint)     *outPoint = V3Zero();
    if (outNormal)    *outNormal = V3Zero();
    if (outFaceIndex) *outFaceIndex = -1;
    if (outDistance)  *outDistance = 0.0f;

    if (maxDistance <= 0.0f)
    {
        Error_SetError(ErrorCode::InvalidArgument);
        return false;
    }

    const float dirLen = Length(direction);
    if (dirLen <= 1e-12f)
    {
        Error_SetError(ErrorCode::InvalidArgument);
        return false;
    }

    const Vector3 dir = Scale(direction, 1.0f / dirLen);

    // SweptSphere* uses velocity * t with t in [0,1], i.e. a ray of maxDistance units.
    const Vector3 vel = Scale(dir, maxDistance);

    bool found = false;

    // Parametric distance along the ray: 0 = origin, 1 = maxDistance.
    float bestT = 1.0f;

    int bestType = WORLD_HIT_NONE;
    int bestStaticId = -1;
    int bestFace = -1;
    int bestNodeIndex = -1;

    SoftBodyInstance* bestBody = nullptr;

    Vector3 bestPoint = V3Zero();
    Vector3 bestNormal = V3Zero();

    // PASS 1: STATIC WORLD

    for (const auto& kv : g_StaticBodies)
    {
        const NativeStaticCollider& src = kv.second;

        if (src.id == ignoreStaticId)
            continue;

        float t = 0.0f;
        Vector3 p = V3Zero();
        Vector3 n = V3Zero();

        bool hit = false;
        int face = -1;

        switch (src.type)
        {
        case StaticType::Sphere:
        {
            StaticColliderData sc = {};
            sc.center = V3(
                src.center[0],
                src.center[1],
                src.center[2]);

            sc.radius = src.radius;

            hit = SweptSphereSphere(
                origin,
                vel,
                0.0f,
                sc,
                t,
                p,
                n);

            break;
        }

        case StaticType::Box:
        {
            StaticColliderData sc = {};

            sc.center = V3(
                src.center[0],
                src.center[1],
                src.center[2]);

            sc.size = V3(
                src.size[0],
                src.size[1],
                src.size[2]);

            sc.rotation = Quaternion{
                src.rotation[0],
                src.rotation[1],
                src.rotation[2],
                src.rotation[3]
            };

            hit = SweptSphereBox(
                origin,
                vel,
                0.0f,
                sc,
                t,
                p,
                n);

            break;
        }

        case StaticType::Capsule:
        {
            StaticColliderData sc = {};

            sc.center = V3(
                src.center[0],
                src.center[1],
                src.center[2]);

            sc.radius = src.radius;
            sc.height = src.height;

            sc.axis = V3(
                src.axis[0],
                src.axis[1],
                src.axis[2]);

            hit = SweptSphereCapsule(
                origin,
                vel,
                0.0f,
                sc,
                t,
                p,
                n);

            break;
        }

        case StaticType::Mesh:
        {
            const int triCount =
                static_cast<int>(src.indices.size()) / 3;

            for (int tIdx = 0; tIdx < triCount; ++tIdx)
            {
                const int i0 = src.indices[tIdx * 3 + 0];
                const int i1 = src.indices[tIdx * 3 + 1];
                const int i2 = src.indices[tIdx * 3 + 2];

                StaticColliderData sc = {};

                sc.v0 = V3(
                    src.vertices[i0 * 3 + 0],
                    src.vertices[i0 * 3 + 1],
                    src.vertices[i0 * 3 + 2]);

                sc.v1 = V3(
                    src.vertices[i1 * 3 + 0],
                    src.vertices[i1 * 3 + 1],
                    src.vertices[i1 * 3 + 2]);

                sc.v2 = V3(
                    src.vertices[i2 * 3 + 0],
                    src.vertices[i2 * 3 + 1],
                    src.vertices[i2 * 3 + 2]);

                float triT;
                Vector3 triP;
                Vector3 triN;

                if (SweptSphereTriangle(
                    origin,
                    vel,
                    0.0f,
                    sc,
                    triT,
                    triP,
                    triN))
                {
                    if (triT < bestT)
                    {
                        bestT = triT;

                        found = true;
                        bestType = WORLD_HIT_STATIC;
                        bestStaticId = src.id;
                        bestFace = tIdx;
                        bestNodeIndex = -1;
                        bestBody = nullptr;

                        bestPoint = triP;
                        bestNormal = triN;
                    }
                }
            }

            // Mesh triangles were handled individually.
            continue;
        }

        case StaticType::Terrain:
        {
            StaticColliderData sc = {};

            sc.center = V3(
                src.center[0],
                src.center[1],
                src.center[2]);

            sc.size = V3(
                src.size[0],
                src.size[1],
                src.size[2]);

            sc.terrainWidth = src.terrainWidth;
            sc.terrainLength = src.terrainLength;

            sc.terrainHeights =
                const_cast<float*>(src.terrainHeights.data());

            hit = SweptSphereTerrain(
                origin,
                vel,
                0.0f,
                sc,
                t,
                p,
                n);

            break;
        }
        }

        if (!hit)
            continue;

        if (t < bestT)
        {
            bestT = t;

            found = true;
            bestType = WORLD_HIT_STATIC;
            bestStaticId = src.id;
            bestFace = face;
            bestNodeIndex = -1;
            bestBody = nullptr;

            bestPoint = p;
            bestNormal = n;
        }
    }

    // PASS 2: SOFT-BODY SURFACE FACES

    for (int bodyIdx = 0; bodyIdx < softBodyCount; ++bodyIdx)
    {
        SoftBodyInstance* body = softBodies[bodyIdx];

        if (!body ||
            body == ignoreSoftBody ||
            body->currentX.empty() ||
            body->faces.empty())
        {
            continue;
        }

        const int nodeCount =
            static_cast<int>(body->currentX.size());

        const int faceCount =
            static_cast<int>(body->faces.size());

        for (int faceIdx = 0; faceIdx < faceCount; ++faceIdx)
        {
            const FaceData& face = body->faces[faceIdx];

            const int i0 = face.nodeA;
            const int i1 = face.nodeB;
            const int i2 = face.nodeC;

            // Protect against malformed topology.
            if (i0 < 0 || i0 >= nodeCount ||
                i1 < 0 || i1 >= nodeCount ||
                i2 < 0 || i2 >= nodeCount)
            {
                continue;
            }

            StaticColliderData tri = {};

            tri.v0 = V3(
                body->currentX[i0],
                body->currentY[i0],
                body->currentZ[i0]);

            tri.v1 = V3(
                body->currentX[i1],
                body->currentY[i1],
                body->currentZ[i1]);

            tri.v2 = V3(
                body->currentX[i2],
                body->currentY[i2],
                body->currentZ[i2]);

            float triT;
            Vector3 triP;
            Vector3 triN;

            // radius = 0 because this is a true ray-vs-triangle test.
            if (!SweptSphereTriangle(
                origin,
                vel,
                0.0f,
                tri,
                triT,
                triP,
                triN))
            {
                continue;
            }

            // Ignore intersections outside the ray.
            if (triT < 0.0f || triT >= bestT)
                continue;

            bestT = triT;

            found = true;
            bestType = WORLD_HIT_SOFTBODY;

            bestStaticId = -1;
            bestBody = body;

            // This is a face hit, not a node hit.
            bestNodeIndex = -1;
            bestFace = faceIdx;

            bestPoint = triP;
            bestNormal = triN;
        }
    }

    // PASS 3: SOFT-BODY NODE FALLBACK

    if (nodeRadius > 0.0f)
    {
        const float r = nodeRadius;

        for (int bodyIdx = 0; bodyIdx < softBodyCount; ++bodyIdx)
        {
            SoftBodyInstance* body = softBodies[bodyIdx];

            if (!body ||
                body == ignoreSoftBody ||
                body->currentX.empty())
            {
                continue;
            }

            const int count =
                static_cast<int>(body->currentX.size());

            for (int nodeIdx = 0; nodeIdx < count; ++nodeIdx)
            {
                const Vector3 center = {
                    body->currentX[nodeIdx],
                    body->currentY[nodeIdx],
                    body->currentZ[nodeIdx],
                    0.0f
                };

                const Vector3 oc =
                    Sub(origin, center);

                const float b = Dot(oc, dir);
                const float c = Dot(oc, oc) - r * r;

                const float disc =
                    b * b - c;

                if (disc < 0.0f)
                    continue;

                const float sqrtDisc =
                    sqrtf(disc);

                float nodeT =
                    -b - sqrtDisc;

                if (nodeT < 0.0f)
                    nodeT = -b + sqrtDisc;

                if (nodeT < 0.0f)
                    continue;

                const float nodeTFrac =
                    nodeT / maxDistance;

                if (nodeTFrac >= bestT)
                    continue;

                bestT = nodeTFrac;

                found = true;
                bestType = WORLD_HIT_SOFTBODY;

                bestStaticId = -1;
                bestBody = body;

                bestNodeIndex = nodeIdx;
                bestFace = -1;

                bestPoint =
                    Add(origin, Scale(dir, nodeT));

                bestNormal =
                    Normalize(Sub(bestPoint, center));
            }
        }
    }

    // OUTPUT

    if (!found)
        return false;

    if (outHitType)
        *outHitType = bestType;

    if (outStaticId)
        *outStaticId = bestStaticId;

    if (outSoftBody)
        *outSoftBody = bestBody;

    if (outNodeIndex)
        *outNodeIndex = bestNodeIndex;

    if (outPoint)
        *outPoint = bestPoint;

    if (outNormal)
        *outNormal = bestNormal;

    if (outFaceIndex)
        *outFaceIndex = bestFace;

    if (outDistance)
        *outDistance = bestT * maxDistance;

    return true;
}


// ---- Rigid-body collision layer: rigids collide with their own shapes, never the soft node cloud ----

// Unified contact record; normal points A→B; b == nullptr means static; penetration is signed.
struct RigidContact
{
    RigidBodyInstance* a = nullptr;  // body A, never null
    RigidBodyInstance* b = nullptr;  // body B, nullptr = static
    Vector3 position;
    Vector3 normal;                  // points from A toward B
    float   penetration = 0.0f;      // > 0 overlap, < 0 speculative gap
    float   friction = 0.0f;
    float   restitution = 0.0f;
    float   impulseSum = 0.0f;

    // ---- solver state ----
    Vector3 rA, rB;                  // arms from each body's COM
    Vector3 tangent1, tangent2;      // fixed friction basis for the whole solve
    float   kNormal = 0.0f;          // effective mass along the normal
    float   kTangent1 = 0.0f, kTangent2 = 0.0f;
    float   approachVn = 0.0f;
    float   normalImpulse = 0.0f;
    float   pushImpulse = 0.0f;      // split-impulse positional part
    float   pushTan1 = 0.0f, pushTan2 = 0.0f; // positional friction, same cone as the velocity solve
    float   tanImpulse1 = 0.0f, tanImpulse2 = 0.0f;
};

static void ShapePoseAabb(const RigidCollisionShape& s, const Vector3& pos, const Quaternion& q,
    Vector3& outMin, Vector3& outMax)
{
    if (s.Empty())
    {
        outMin = outMax = pos;
        return;
    }

    const Vector3 ax = QRotate(q, V3(1.0f, 0.0f, 0.0f));
    const Vector3 ay = QRotate(q, V3(0.0f, 1.0f, 0.0f));
    const Vector3 az = QRotate(q, V3(0.0f, 0.0f, 1.0f));

    const Vector3 halfLocal = Scale(Sub(s.localAabbMax, s.localAabbMin), 0.5f);
    const Vector3 centerLocal = Scale(Add(s.localAabbMax, s.localAabbMin), 0.5f);
    const Vector3 worldCenter = Add(pos, QRotate(q, centerLocal));

    const float ex = AbsF(ax.x) * halfLocal.x + AbsF(ay.x) * halfLocal.y + AbsF(az.x) * halfLocal.z;
    const float ey = AbsF(ax.y) * halfLocal.x + AbsF(ay.y) * halfLocal.y + AbsF(az.y) * halfLocal.z;
    const float ez = AbsF(ax.z) * halfLocal.x + AbsF(ay.z) * halfLocal.y + AbsF(az.z) * halfLocal.z;

    outMin = Sub(worldCenter, V3(ex, ey, ez));
    outMax = Add(worldCenter, V3(ex, ey, ez));
}

void Collisions_RigidShapeWorldAabb(const RigidBodyInstance* body, Vector3& outMin, Vector3& outMax)
{
    ShapePoseAabb(body->shape, body->position, body->orientation, outMin, outMax);
}

// ApplyWorldInvInertia now lives in RigidBodyInstance.cpp; declared in RigidBodyInstance.h,
// which this file already includes.

static float AngularEffectiveMassTerm(const RigidBodyInstance* body, const Vector3& r, const Vector3& dir)
{
    if (body->isKinematic || body->isSleeping) return 0.0f;
    const Vector3 rxd = Cross(r, dir);
    const Vector3 iInvRxd = ApplyWorldInvInertia(body, rxd);
    return Dot(Cross(iInvRxd, r), dir);
}

static Vector3 PointVelocity(const RigidBodyInstance* body, const Vector3& r)
{
    return Add(body->linearVelocity, Cross(body->angularVelocity, r));
}

static Vector3 PointPseudoVelocity(const RigidBodyInstance* body, const Vector3& r)
{
    return Add(body->pseudoLinearVelocity, Cross(body->pseudoAngularVelocity, r));
}

constexpr float kContactSlop = 0.001f;
constexpr float kContactERP = 0.8f;
constexpr float kMaxPushVelocity = 3.0f;
constexpr float kRestitutionThreshold = 1.0f;

// Rigid-soft coupling (Collisions_RigidSoftContacts):
constexpr float kCouplingMassRatio = 1.0f;   // soft-side mobility clamp vs the rigid's (mass-ratio hack)
constexpr float kCouplingSoftERP = 0.4f;     // soft depenetration bias fraction per substep
constexpr float kCouplingSoftBiasCap = 2.0f; // m/s cap on that bias
constexpr int   kCouplingIterations = 4;     // sequential-impulse iterations (matches the rigid solver)

// Split-impulse pushout speed that recovers `pen` over one substep.
static float PushoutBias(float pen, float dt)
{
    return MinF((kContactERP / dt) * (pen - kContactSlop), kMaxPushVelocity);
}

// ---- Warm starting: impulses carried across substeps ----
constexpr float    kContactWarmMatchDist = 0.02f; // a manifold point may shift this far between substeps
constexpr uint32_t kContactWarmLifetime = 4;      // substeps an untouched pair survives in the cache
constexpr int      kContactWarmSlots = 16;        // a body may touch several statics under one key

struct WarmSlot
{
    Vector3 anchor;
    float   normalImpulse = 0.0f, tanImpulse1 = 0.0f, tanImpulse2 = 0.0f;
    float   pushImpulse = 0.0f, pushTan1 = 0.0f, pushTan2 = 0.0f;
};

struct WarmPair
{
    WarmSlot slot[kContactWarmSlots];
    int      count = 0;
    uint32_t stamp = 0;
};

static std::unordered_map<uint64_t, WarmPair> g_WarmCache;
static uint32_t g_WarmStamp = 0;

void Collisions_ClearRigidWarmCache()
{
    g_WarmCache.clear();
    g_WarmStamp = 0;
}

// Order-independent pair key; b == nullptr (static) keys on a alone, so anchors do the matching.
static uint64_t WarmKey(const RigidBodyInstance* a, const RigidBodyInstance* b)
{
    uintptr_t lo = (uintptr_t)a, hi = (uintptr_t)b;
    if (hi < lo) { const uintptr_t t = lo; lo = hi; hi = t; }
    return (uint64_t)lo * 0x9E3779B97F4A7C15ull ^ ((uint64_t)hi + 0x165667B19E3779F9ull);
}

static void ApplyContactImpulse(RigidContact& c, const Vector3& dir, float j);
static void ApplyPseudoImpulse(RigidContact& c, const Vector3& dir, float j);

// Seeds impulses from the previous substep; without this a stack restarts from zero every substep.
static void WarmStartContact(RigidContact& c)
{
    auto it = g_WarmCache.find(WarmKey(c.a, c.b));
    if (it == g_WarmCache.end() || it->second.count <= 0) return;

    const WarmPair& p = it->second;
    const WarmSlot* best = nullptr;
    float bestDistSq = kContactWarmMatchDist * kContactWarmMatchDist;
    for (int i = 0; i < p.count; ++i)
    {
        const float d2 = LengthSq(Sub(p.slot[i].anchor, c.position));
        if (d2 < bestDistSq) { bestDistSq = d2; best = &p.slot[i]; }
    }
    if (!best) return;

    c.normalImpulse = best->normalImpulse;
    c.tanImpulse1 = best->tanImpulse1;
    c.tanImpulse2 = best->tanImpulse2;
    c.pushImpulse = best->pushImpulse;
    c.pushTan1 = best->pushTan1;
    c.pushTan2 = best->pushTan2;

    // The iterations clamp these back down if the guess was too large, so the net stays exact.
    ApplyContactImpulse(c, c.normal, c.normalImpulse);
    ApplyContactImpulse(c, c.tangent1, c.tanImpulse1);
    ApplyContactImpulse(c, c.tangent2, c.tanImpulse2);
    ApplyPseudoImpulse(c, c.normal, c.pushImpulse);
    ApplyPseudoImpulse(c, c.tangent1, c.pushTan1);
    ApplyPseudoImpulse(c, c.tangent2, c.pushTan2);
}

// Stores this substep's converged velocity impulses for the next substep to start from.
static void WarmStoreContacts(std::vector<RigidContact>& contacts)
{
    for (RigidContact& c : contacts)
    {
        if (c.a->isSleeping && (!c.b || c.b->isSleeping)) continue;

        WarmPair& p = g_WarmCache[WarmKey(c.a, c.b)];
        if (p.stamp != g_WarmStamp) { p.stamp = g_WarmStamp; p.count = 0; }
        if (p.count >= kContactWarmSlots) continue;

        WarmSlot& s = p.slot[p.count++];
        s.anchor = c.position;
        s.normalImpulse = c.normalImpulse;
        s.tanImpulse1 = c.tanImpulse1;
        s.tanImpulse2 = c.tanImpulse2;
        s.pushImpulse = c.pushImpulse;
        s.pushTan1 = c.pushTan1;
        s.pushTan2 = c.pushTan2;
    }
}

// Drops pairs that have not been solved for a few substeps, so handles can never go stale.
static void WarmSweepCache()
{
    for (auto it = g_WarmCache.begin(); it != g_WarmCache.end(); )
    {
        if (g_WarmStamp - it->second.stamp > kContactWarmLifetime) it = g_WarmCache.erase(it);
        else ++it;
    }
}

static void PrepareContact(RigidContact& c)
{
    RigidBodyInstance* a = c.a;
    RigidBodyInstance* b = c.b;

    c.rA = Sub(c.position, a->position);
    c.rB = b ? Sub(c.position, b->position) : V3Zero();

    // Sleeping bodies are immovable: impulses must not accumulate on a frozen body.
    const float invMassA = (a->isKinematic || a->isSleeping) ? 0.0f : a->invMass;
    const float invMassB = (b && !b->isKinematic && !b->isSleeping) ? b->invMass : 0.0f;
    const float invMassSum = invMassA + invMassB;

    const Vector3 n = c.normal;
    // Any vector orthogonal to n works; pick the axis n is least aligned with for conditioning.
    if (AbsF(n.x) >= 0.57735f) c.tangent1 = Normalize(V3(n.y, -n.x, 0.0f));
    else                       c.tangent1 = Normalize(V3(0.0f, n.z, -n.y));
    c.tangent2 = Cross(n, c.tangent1);

    c.kNormal = invMassSum + AngularEffectiveMassTerm(a, c.rA, n)
        + (b ? AngularEffectiveMassTerm(b, c.rB, n) : 0.0f);
    c.kTangent1 = invMassSum + AngularEffectiveMassTerm(a, c.rA, c.tangent1)
        + (b ? AngularEffectiveMassTerm(b, c.rB, c.tangent1) : 0.0f);
    c.kTangent2 = invMassSum + AngularEffectiveMassTerm(a, c.rA, c.tangent2)
        + (b ? AngularEffectiveMassTerm(b, c.rB, c.tangent2) : 0.0f);

    // Captured BEFORE any impulse: restitution must come from the speed they met at.
    const Vector3 relVel = Sub(b ? PointVelocity(b, c.rB) : V3Zero(), PointVelocity(a, c.rA));
    c.approachVn = Dot(relVel, n);

    c.normalImpulse = 0.0f;
    c.pushImpulse = 0.0f;
    c.pushTan1 = 0.0f;
    c.pushTan2 = 0.0f;
    c.tanImpulse1 = 0.0f;
    c.tanImpulse2 = 0.0f;
    c.impulseSum = 0.0f;

    WarmStartContact(c);
}

static void ApplyContactImpulse(RigidContact& c, const Vector3& dir, float j)
{
    if (j == 0.0f) return;
    const Vector3 impulse = Scale(dir, j);
    RigidBodyInstance* a = c.a;
    RigidBodyInstance* b = c.b;

    if (!a->isKinematic && !a->isSleeping)
    {
        a->linearVelocity = Sub(a->linearVelocity, Scale(impulse, a->invMass));
        a->angularVelocity = Sub(a->angularVelocity, ApplyWorldInvInertia(a, Cross(c.rA, impulse)));
    }
    if (b && !b->isKinematic && !b->isSleeping)
    {
        b->linearVelocity = Add(b->linearVelocity, Scale(impulse, b->invMass));
        b->angularVelocity = Add(b->angularVelocity, ApplyWorldInvInertia(b, Cross(c.rB, impulse)));
    }
}

// Same, into the split-impulse positional velocities.
static void ApplyPseudoImpulse(RigidContact& c, const Vector3& dir, float j)
{
    if (j == 0.0f) return;
    const Vector3 impulse = Scale(dir, j);
    RigidBodyInstance* a = c.a;
    RigidBodyInstance* b = c.b;

    if (!a->isKinematic && !a->isSleeping)
    {
        a->pseudoLinearVelocity = Sub(a->pseudoLinearVelocity, Scale(impulse, a->invMass));
        a->pseudoAngularVelocity = Sub(a->pseudoAngularVelocity, ApplyWorldInvInertia(a, Cross(c.rA, impulse)));
    }
    if (b && !b->isKinematic && !b->isSleeping)
    {
        b->pseudoLinearVelocity = Add(b->pseudoLinearVelocity, Scale(impulse, b->invMass));
        b->pseudoAngularVelocity = Add(b->pseudoAngularVelocity, ApplyWorldInvInertia(b, Cross(c.rB, impulse)));
    }
}

// One velocity iteration: normal constraint, then the friction cone.
static void SolveContactVelocity(RigidContact& c)
{
    if (c.kNormal <= 1e-9f) return;

    RigidBodyInstance* a = c.a;
    RigidBodyInstance* b = c.b;

    // --- Normal ---
    // No penetration bias here: recovery belongs to the split-impulse solve, not to real velocity.
    {
        const Vector3 relVel = Sub(b ? PointVelocity(b, c.rB) : V3Zero(), PointVelocity(a, c.rA));
        const float vn = Dot(relVel, c.normal);

        const float delta = -vn / c.kNormal;
        const float oldSum = c.normalImpulse;
        c.normalImpulse = MaxF(oldSum + delta, 0.0f);
        ApplyContactImpulse(c, c.normal, c.normalImpulse - oldSum);
    }

    // --- Friction ---
    const float maxFriction = c.friction * c.normalImpulse;
    if (maxFriction <= 0.0f)
    {
        // No normal load left to fund friction
        ApplyContactImpulse(c, c.tangent1, -c.tanImpulse1);
        ApplyContactImpulse(c, c.tangent2, -c.tanImpulse2);
        c.tanImpulse1 = c.tanImpulse2 = 0.0f;
        return;
    }

    const Vector3 relVel = Sub(b ? PointVelocity(b, c.rB) : V3Zero(), PointVelocity(a, c.rA));

    float newT1 = c.tanImpulse1;
    float newT2 = c.tanImpulse2;
    if (c.kTangent1 > 1e-9f) newT1 += -Dot(relVel, c.tangent1) / c.kTangent1;
    if (c.kTangent2 > 1e-9f) newT2 += -Dot(relVel, c.tangent2) / c.kTangent2;

    // Clamp the 2D friction impulse to the cone as a vector
    const float tLenSq = newT1 * newT1 + newT2 * newT2;
    if (tLenSq > maxFriction * maxFriction)
    {
        const float scale = maxFriction / std::sqrt(tLenSq);
        newT1 *= scale;
        newT2 *= scale;
    }

    ApplyContactImpulse(c, c.tangent1, newT1 - c.tanImpulse1);
    ApplyContactImpulse(c, c.tangent2, newT2 - c.tanImpulse2);
    c.tanImpulse1 = newT1;
    c.tanImpulse2 = newT2;
}

// Restitution
static void ApplyRestitution(RigidContact& c)
{
    if (c.restitution <= 0.0f) return;
    if (c.approachVn > -kRestitutionThreshold) return;
    if (c.normalImpulse <= 0.0f) return;
    if (c.kNormal <= 1e-9f) return;

    RigidBodyInstance* a = c.a;
    RigidBodyInstance* b = c.b;

    const Vector3 relVel = Sub(b ? PointVelocity(b, c.rB) : V3Zero(), PointVelocity(a, c.rA));
    const float vn = Dot(relVel, c.normal);

    const float delta = -(vn + c.restitution * c.approachVn) / c.kNormal;
    const float oldSum = c.normalImpulse;
    c.normalImpulse = MaxF(oldSum + delta, 0.0f);
    ApplyContactImpulse(c, c.normal, c.normalImpulse - oldSum);
}

// One positional (split-impulse) iteration
static void SolveContactPosition(RigidContact& c, float dt)
{
    if (c.kNormal <= 1e-9f) return;
    if (c.penetration <= kContactSlop) return;

    RigidBodyInstance* a = c.a;
    RigidBodyInstance* b = c.b;

    const float bias = PushoutBias(c.penetration, dt);

    const Vector3 relPseudo = Sub(b ? PointPseudoVelocity(b, c.rB) : V3Zero(),
        PointPseudoVelocity(a, c.rA));
    const float vn = Dot(relPseudo, c.normal);

    const float delta = -(vn - bias) / c.kNormal;
    const float oldSum = c.pushImpulse;
    c.pushImpulse = MaxF(oldSum + delta, 0.0f);
    ApplyPseudoImpulse(c, c.normal, c.pushImpulse - oldSum);

    // Friction on the pushout: unclamped off-axis pushout is permanent uncorrected drift.
    const float maxPushFriction = c.friction * c.pushImpulse;
    if (maxPushFriction <= 0.0f)
    {
        ApplyPseudoImpulse(c, c.tangent1, -c.pushTan1);
        ApplyPseudoImpulse(c, c.tangent2, -c.pushTan2);
        c.pushTan1 = c.pushTan2 = 0.0f;
        return;
    }

    const Vector3 relPseudoT = Sub(b ? PointPseudoVelocity(b, c.rB) : V3Zero(),
        PointPseudoVelocity(a, c.rA));

    float newT1 = c.pushTan1;
    float newT2 = c.pushTan2;
    if (c.kTangent1 > 1e-9f) newT1 += -Dot(relPseudoT, c.tangent1) / c.kTangent1;
    if (c.kTangent2 > 1e-9f) newT2 += -Dot(relPseudoT, c.tangent2) / c.kTangent2;

    const float tLenSq = newT1 * newT1 + newT2 * newT2;
    if (tLenSq > maxPushFriction * maxPushFriction)
    {
        const float scale = maxPushFriction / std::sqrt(tLenSq);
        newT1 *= scale;
        newT2 *= scale;
    }

    ApplyPseudoImpulse(c, c.tangent1, newT1 - c.pushTan1);
    ApplyPseudoImpulse(c, c.tangent2, newT2 - c.pushTan2);
    c.pushTan1 = newT1;
    c.pushTan2 = newT2;
}

// Integrate the accumulated pseudo-velocities
static void ApplyPseudoVelocities(RigidBodyInstance** bodies, int count, float dt)
{
    for (int i = 0; i < count; ++i)
    {
        RigidBodyInstance* b = bodies[i];
        if (!b) continue;

        const Vector3 dv = b->pseudoLinearVelocity;
        const Vector3 dw = b->pseudoAngularVelocity;
        b->pseudoLinearVelocity = V3Zero();
        b->pseudoAngularVelocity = V3Zero();

        if (b->isKinematic || b->isSleeping) continue;
        if (LengthSq(dv) < 1e-16f && LengthSq(dw) < 1e-16f) continue;

        b->position = Add(b->position, Scale(dv, dt));

        const float wx = dw.x * 0.5f * dt, wy = dw.y * 0.5f * dt, wz = dw.z * 0.5f * dt;
        const Quaternion q = b->orientation;
        const Quaternion dq = {
            wx * q.w + wy * q.z - wz * q.y,
            wy * q.w + wz * q.x - wx * q.z,
            wz * q.w + wx * q.y - wy * q.x,
           -wx * q.x - wy * q.y - wz * q.z
        };
        b->orientation = QNormalize({ q.x + dq.x, q.y + dq.y, q.z + dq.z, q.w + dq.w });
    }
}

// How large a gap detection must still report as a contact for this pair.
static float SpeculativeMargin(const RigidBodyInstance* a, const RigidBodyInstance* b)
{
    float travel = Length(Sub(a->position, a->prevPosition));
    if (b) travel += Length(Sub(b->position, b->prevPosition));
    return travel;
}

struct OBB
{
    Vector3 center;
    Vector3 axis[3]; // world-space unit box axes
    Vector3 half;    // half-extents along axis[0..2]
};

static inline float VComp(const Vector3& v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }

static OBB MakeShapeOBB(const RigidBodyInstance* b, const Vector3& pos, const Quaternion& rot)
{
    OBB o;
    o.center = pos; // box local origin = COM for Box shapes
    o.axis[0] = QRotate(rot, V3(1.0f, 0.0f, 0.0f));
    o.axis[1] = QRotate(rot, V3(0.0f, 1.0f, 0.0f));
    o.axis[2] = QRotate(rot, V3(0.0f, 0.0f, 1.0f));
    o.half = b->shape.boxHalfExtents;
    return o;
}

static float ProjectRadius(const OBB& o, const Vector3& axis)
{
    return VComp(o.half, 0) * AbsF(Dot(o.axis[0], axis))
        + VComp(o.half, 1) * AbsF(Dot(o.axis[1], axis))
        + VComp(o.half, 2) * AbsF(Dot(o.axis[2], axis));
}

struct SATResult
{
    bool overlapping;
    float separation; // >=0 conservative lower-bound distance if separated; <0 (=-depth) if overlapping
    Vector3 axis;      // points from A to B
};

static SATResult TestOBBSAT(const OBB& A, const OBB& B)
{
    Vector3 axes[15];
    int n = 0;
    for (int i = 0; i < 3; ++i) axes[n++] = A.axis[i];
    for (int i = 0; i < 3; ++i) axes[n++] = B.axis[i];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            Vector3 c = Cross(A.axis[i], B.axis[j]);
            if (LengthSq(c) > 1e-8f) axes[n++] = Normalize(c);
        }

    const Vector3 d = Sub(B.center, A.center);

    bool anySeparating = false;
    float bestSep = -std::numeric_limits<float>::max();
    Vector3 bestSepAxis = V3(0.0f, 1.0f, 0.0f);

    float minPen = std::numeric_limits<float>::max();
    Vector3 minPenAxis = V3(0.0f, 1.0f, 0.0f);

    for (int i = 0; i < n; ++i)
    {
        const Vector3 axis = axes[i];
        const float dist = AbsF(Dot(d, axis));
        const float ra = ProjectRadius(A, axis);
        const float rb = ProjectRadius(B, axis);
        const float gap = dist - (ra + rb);

        if (gap > 0.0f)
        {
            anySeparating = true;
            // Orient the separating axis consistently: point from A to B.
            if (gap > bestSep)
            {
                bestSep = gap;
                bestSepAxis = Dot(d, axis) < 0.0f ? Neg(axis) : axis;
            }
        }
        else
        {
            const float pen = (ra + rb) - dist;
            // Bias face axes over edge-edge axes to ensure stable 4-point face contact manifolds
            const float effectivePen = (i >= 6) ? pen * 1.25f : pen;
            if (effectivePen < minPen)
            {
                minPen = effectivePen;
                minPenAxis = (Dot(d, axis) < 0.0f) ? Neg(axis) : axis;
            }
        }
    }

    SATResult r;
    if (anySeparating) { r.overlapping = false; r.separation = bestSep; r.axis = bestSepAxis; }
    else { r.overlapping = true; r.separation = -minPen; r.axis = minPenAxis; }
    return r;
}

// Conservative advancement: swept OBB vs OBB

struct RigidCCDHit
{
    bool hit;
    float toi;    // 0..1, fraction of dt
    Vector3 axis;  // contact axis A->B at the hit time
};

static Quaternion QNlerp(const Quaternion& a, const Quaternion& b, float t)
{
    float dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    Quaternion bb = b;
    if (dot < 0.0f) { bb.x = -b.x; bb.y = -b.y; bb.z = -b.z; bb.w = -b.w; }
    Quaternion r = {
        a.x + (bb.x - a.x) * t,
        a.y + (bb.y - a.y) * t,
        a.z + (bb.z - a.z) * t,
        a.w + (bb.w - a.w) * t
    };
    return QNormalize(r);
}

static bool SweptOBBvsOBB(
    const RigidBodyInstance* a, const RigidBodyInstance* b,
    const Vector3& posA0, const Quaternion& rotA0, const Vector3& posA1, const Quaternion& rotA1,
    const Vector3& posB0, const Quaternion& rotB0, const Vector3& posB1, const Quaternion& rotB1,
    float dt, RigidCCDHit& out)
{
    out = { false, 1.0f, V3(0.0f, 1.0f, 0.0f) };
    if (dt <= 0.0f) return false;

    OBB A0 = MakeShapeOBB(a, posA0, rotA0);
    OBB B0 = MakeShapeOBB(b, posB0, rotB0);
    SATResult s0 = TestOBBSAT(A0, B0);
    if (s0.overlapping) { out = { true, 0.0f, s0.axis }; return true; }

    const float rA = Length(a->shape.boxHalfExtents); // bounding-sphere radius about the COM
    const float rB = Length(b->shape.boxHalfExtents);

    // Angular closing-speed bound from interpolated orientations, not stored angularVelocity.
    auto SweepAngularSpeed = [](const Quaternion& q0, const Quaternion& q1) -> float
        {
            Quaternion dq = QMul(q1, QConj(q0));
            if (dq.w < 0.0f) { dq.x = -dq.x; dq.y = -dq.y; dq.z = -dq.z; dq.w = -dq.w; } // shortest arc
            const float sinHalf = std::sqrt(MaxF(0.0f, dq.x * dq.x + dq.y * dq.y + dq.z * dq.z));
            const float angle = 2.0f * std::atan2(sinHalf, AbsF(dq.w));
            return angle;
        };

    const float wA = MaxF(Length(a->angularVelocity), SweepAngularSpeed(rotA0, rotA1) / dt);
    const float wB = MaxF(Length(b->angularVelocity), SweepAngularSpeed(rotB0, rotB1) / dt);

    const Vector3 linVelA = Scale(Sub(posA1, posA0), 1.0f / dt);
    const Vector3 linVelB = Scale(Sub(posB1, posB0), 1.0f / dt);
    const float boundRate = Length(Sub(linVelA, linVelB)) + wA * rA + wB * rB;

    if (boundRate <= 1e-6f) return false; // not closing fast enough to matter this step

    constexpr float kTolerance = 1e-4f;
    constexpr int kMaxIters = 16;

    float t = 0.0f;
    for (int iter = 0; iter < kMaxIters; ++iter)
    {
        const float u = t / dt;
        OBB A = MakeShapeOBB(a, Lerp(posA0, posA1, u), QNlerp(rotA0, rotA1, u));
        OBB B = MakeShapeOBB(b, Lerp(posB0, posB1, u), QNlerp(rotB0, rotB1, u));

        SATResult s = TestOBBSAT(A, B);
        if (s.overlapping || s.separation <= kTolerance)
        {
            out = { true, u, s.axis };
            return true;
        }

        const float step = s.separation / boundRate;
        t += step;
        if (t >= dt) return false; // provably clear for the rest of this step
    }

    out = { true, 1.0f, V3(0.0f, 1.0f, 0.0f) };
    return true;
}

// Contact manifold: reference/incident face clipping

struct BoxFace { Vector3 v[4]; Vector3 normal; };

static BoxFace GetBoxFace(const OBB& box, const Vector3& worldAxis)
{
    int bestI = 0; float bestAbsDot = -1.0f; float sign = 1.0f;
    for (int i = 0; i < 3; ++i)
    {
        const float d = Dot(box.axis[i], worldAxis);
        if (AbsF(d) > bestAbsDot) { bestAbsDot = AbsF(d); bestI = i; sign = (d >= 0.0f) ? 1.0f : -1.0f; }
    }

    const int u = (bestI + 1) % 3;
    const int v = (bestI + 2) % 3;

    const Vector3 n = Scale(box.axis[bestI], sign);
    const Vector3 faceCenter = Add(box.center, Scale(n, VComp(box.half, bestI)));
    const Vector3 uAxis = Scale(box.axis[u], VComp(box.half, u));
    const Vector3 vAxis = Scale(box.axis[v], VComp(box.half, v));

    BoxFace f;
    f.normal = n;
    f.v[0] = Add(Add(faceCenter, uAxis), vAxis);
    f.v[1] = Sub(Add(faceCenter, vAxis), uAxis);
    f.v[2] = Sub(Sub(faceCenter, uAxis), vAxis);
    f.v[3] = Add(Sub(faceCenter, vAxis), uAxis);
    return f;
}

// Sutherland-Hodgman clip of a polygon against one half-space (keeps Dot(p,n) − d ≤ 0).
static int ClipPolyToPlane(const Vector3* inPts, int inCount, const Vector3& n, float d, Vector3* outPts)
{
    int outCount = 0;
    for (int i = 0; i < inCount; ++i)
    {
        const Vector3& cur = inPts[i];
        const Vector3& prev = inPts[(i + inCount - 1) % inCount];
        const float curDist = Dot(cur, n) - d;
        const float prevDist = Dot(prev, n) - d;

        if (curDist <= 0.0f)
        {
            if (prevDist > 0.0f)
            {
                const float t = prevDist / (prevDist - curDist);
                outPts[outCount++] = Lerp(prev, cur, t);
            }
            outPts[outCount++] = cur;
        }
        else if (prevDist <= 0.0f)
        {
            const float t = prevDist / (prevDist - curDist);
            outPts[outCount++] = Lerp(prev, cur, t);
        }
    }
    return outCount;
}

struct ContactPoint { Vector3 point; float penetration; };

// Polar order around the centroid: a canonical sequence, so Gauss-Seidel bias cannot alternate.
static int SortManifoldByAngle(ContactPoint* pts, int count, const Vector3& axis)
{
    if (count < 3) return count;

    Vector3 center = V3Zero();
    for (int i = 0; i < count; ++i) center = Add(center, pts[i].point);
    center = Scale(center, 1.0f / (float)count);

    // Same least-aligned-axis rule as the solver basis, so the frame is stable across substeps.
    Vector3 u;
    if (AbsF(axis.x) >= 0.57735f) u = Normalize(V3(axis.y, -axis.x, 0.0f));
    else                          u = Normalize(V3(0.0f, axis.z, -axis.y));
    const Vector3 v = Cross(axis, u);

    float angle[8];
    for (int i = 0; i < count; ++i)
    {
        const Vector3 d = Sub(pts[i].point, center);
        angle[i] = std::atan2(Dot(d, v), Dot(d, u));
    }

    for (int i = 1; i < count; ++i)
    {
        const ContactPoint p = pts[i];
        const float a = angle[i];
        int j = i - 1;
        while (j >= 0 && angle[j] > a) { pts[j + 1] = pts[j]; angle[j + 1] = angle[j]; --j; }
        pts[j + 1] = p;
        angle[j + 1] = a;
    }
    return count;
}

static int GenerateBoxBoxManifold(const OBB& A, const OBB& B, const Vector3& axis, ContactPoint* outPts, int maxPts,
    float speculativeMargin = 0.0f)
{
    float aAlign = 0.0f, bAlign = 0.0f;
    for (int i = 0; i < 3; ++i) { aAlign = MaxF(aAlign, AbsF(Dot(A.axis[i], axis))); bAlign = MaxF(bAlign, AbsF(Dot(B.axis[i], axis))); }

    const bool aIsRef = aAlign >= bAlign;
    const OBB& refBox = aIsRef ? A : B;
    const OBB& incBox = aIsRef ? B : A;
    const Vector3 refAxis = aIsRef ? axis : Neg(axis);

    BoxFace refFace = GetBoxFace(refBox, refAxis);
    BoxFace incFace = GetBoxFace(incBox, Neg(refFace.normal));

    Vector3 poly[8];
    int polyCount = 4;
    for (int i = 0; i < 4; ++i) poly[i] = incFace.v[i];

    Vector3 faceCenter = V3Zero();
    for (int i = 0; i < 4; ++i)
        faceCenter = Add(faceCenter, refFace.v[i]);

    faceCenter = Scale(faceCenter, 0.25f);

    for (int e = 0; e < 4 && polyCount > 0; ++e)
    {
        const Vector3 edgeDir = Normalize(Sub(refFace.v[(e + 1) % 4], refFace.v[e]));

        Vector3 sideNormal = Cross(refFace.normal, edgeDir);

        // Interior points must satisfy Dot(p, n) - d <= 0.
        if (Dot(sideNormal, Sub(faceCenter, refFace.v[e])) > 0.0f)
            sideNormal = Neg(sideNormal);

        const float d = Dot(sideNormal, refFace.v[e]);

        Vector3 clipped[8];
        polyCount = ClipPolyToPlane(poly, polyCount, sideNormal, d, clipped);

        for (int i = 0; i < polyCount; ++i)
            poly[i] = clipped[i];
    }

    const float refD = Dot(refFace.normal, refFace.v[0]);
    constexpr float kContactAcceptance = 0.005f; // 5 mm resting contact acceptance tolerance
    const float acceptance = MaxF(kContactAcceptance, speculativeMargin);

    ContactPoint validPts[8];
    int validCount = 0;
    for (int i = 0; i < polyCount && validCount < 8; ++i)
    {
        const float sep = Dot(refFace.normal, poly[i]) - refD;
        if (sep <= acceptance)
        {
            validPts[validCount].point = poly[i];
            validPts[validCount].penetration = -sep; // signed: negative is a gap
            ++validCount;
        }
    }

    if (validCount <= maxPts)
    {
        for (int i = 0; i < validCount; ++i)
            outPts[i] = validPts[i];
        return SortManifoldByAngle(outPts, validCount, axis);
    }

    // Manifold reduction: 4 extremal points spanning max contact-polygon area — deepest first.
    int idx0 = 0;
    float maxPen = validPts[0].penetration;
    for (int i = 1; i < validCount; ++i)
    {
        if (validPts[i].penetration > maxPen)
        {
            maxPen = validPts[i].penetration;
            idx0 = i;
        }
    }

    // 2. Point furthest from idx0
    int idx1 = 0;
    float maxDistSq = -1.0f;
    for (int i = 0; i < validCount; ++i)
    {
        if (i == idx0) continue;
        float d2 = LengthSq(Sub(validPts[i].point, validPts[idx0].point));
        if (d2 > maxDistSq)
        {
            maxDistSq = d2;
            idx1 = i;
        }
    }

    // 3. Point maximizing triangle area with idx0 and idx1
    int idx2 = 0;
    float maxTriAreaSq = -1.0f;
    Vector3 e01 = Sub(validPts[idx1].point, validPts[idx0].point);
    for (int i = 0; i < validCount; ++i)
    {
        if (i == idx0 || i == idx1) continue;
        float a2 = LengthSq(Cross(e01, Sub(validPts[i].point, validPts[idx0].point)));
        if (a2 > maxTriAreaSq)
        {
            maxTriAreaSq = a2;
            idx2 = i;
        }
    }

    // 4. Point maximizing quadrilateral area with idx0, idx1, idx2
    int idx3 = 0;
    float maxQuadArea = -1.0f;
    for (int i = 0; i < validCount; ++i)
    {
        if (i == idx0 || i == idx1 || i == idx2) continue;
        float a0 = Length(Cross(Sub(validPts[i].point, validPts[idx0].point), Sub(validPts[idx1].point, validPts[idx0].point)));
        float a1 = Length(Cross(Sub(validPts[i].point, validPts[idx1].point), Sub(validPts[idx2].point, validPts[idx1].point)));
        float a2 = Length(Cross(Sub(validPts[i].point, validPts[idx2].point), Sub(validPts[idx0].point, validPts[idx2].point)));
        float quadArea = a0 + a1 + a2;
        if (quadArea > maxQuadArea)
        {
            maxQuadArea = quadArea;
            idx3 = i;
        }
    }

    outPts[0] = validPts[idx0];
    outPts[1] = validPts[idx1];
    outPts[2] = validPts[idx2];
    outPts[3] = validPts[idx3];
    return SortManifoldByAngle(outPts, 4, axis);
}

// Waking a sleeping body on any contact at all made sleep unreachable and let stacks creep.
static void WakeIfDisturbed(RigidBodyInstance* sleeping, const RigidBodyInstance* other)
{
    if (!sleeping || !sleeping->isSleeping) return;
    if (other)
    {
        if (other->isSleeping) return;
        const float speed = Length(other->linearVelocity) + Length(other->angularVelocity) * 0.1f;
        if (speed < other->sleepThreshold) return;
    }
    RigidBody_Wake(sleeping);
}

// Top level: detect one rigid-rigid box pair for this step.
//
// Split into two PURE halves so detection can run on the thread pool:
//   BoxBoxSweptToi    - the conservative-advancement query, run against the bodies'
//                       UNMODIFIED prev->current poses. Reads only, writes nothing.
//   BoxBoxContactsCCD - SAT + manifold at caller-supplied poses. No body mutation.
// The old version rolled both bodies' poses back in place, which made every later
// pair test (and the pose the step commits) depend on pair processing order. The
// rollback now happens once per body in Collisions_RigidStep, at the body's
// earliest hit time, so results are identical however pairs are scheduled.
static bool BoxBoxSweptToi(const RigidBodyInstance* a, const RigidBodyInstance* b, float dt, float& outToi)
{
    RigidCCDHit hit;
    if (!SweptOBBvsOBB(a, b, a->prevPosition, a->prevOrientation, a->position, a->orientation,
        b->prevPosition, b->prevOrientation, b->position, b->orientation, dt, hit))
        return false; // provably no contact this step
    outToi = hit.toi;
    return true;
}

static int BoxBoxContactsCCD(RigidBodyInstance* a, RigidBodyInstance* b,
    const Vector3& posA, const Quaternion& rotA,
    const Vector3& posB, const Quaternion& rotB,
    RigidContact* out, int maxOut)
{
    OBB A = MakeShapeOBB(a, posA, rotA);
    OBB B = MakeShapeOBB(b, posB, rotB);
    SATResult s = TestOBBSAT(A, B);

    // Margin measured from the poses the manifold is built at, matching the old
    // post-rollback travel rather than the full-step travel.
    const float margin = Length(Sub(posA, a->prevPosition)) + Length(Sub(posB, b->prevPosition));
    if (!s.overlapping && s.separation > MaxF(1e-3f, margin)) return 0; // provably clear this step

    ContactPoint pts[4];
    const int count = GenerateBoxBoxManifold(A, B, s.axis, pts, 4, margin);

    const float restitution = 0.5f * (a->restitution + b->restitution);
    const float friction = 0.5f * (a->friction + b->friction);

    int written = 0;
    for (int i = 0; i < count && written < maxOut; ++i)
    {
        RigidContact& c = out[written++];
        c.a = a; c.b = b;
        c.position = pts[i].point;
        c.normal = s.axis;                  // A→B
        c.penetration = pts[i].penetration;
        c.friction = friction;
        c.restitution = restitution;
    }
    return written;
}

static Vector3 ClosestPointOnSegment(const Vector3& p, const Vector3& a0, const Vector3& a1)
{
    const Vector3 d = Sub(a1, a0);
    const float dd = Dot(d, d);
    if (dd < 1e-12f) return a0;
    float t = ClampF(Dot(Sub(p, a0), d) / dd, 0.0f, 1.0f);
    return Add(a0, Scale(d, t));
}

// World segment of a body's capsule shape (axis = local Y).
static void CapsuleSegment(const RigidBodyInstance* b, Vector3& outA, Vector3& outB)
{
    const Vector3 axis = QRotate(b->orientation, V3(0.0f, 1.0f, 0.0f));
    outA = Add(b->position, Scale(axis, b->shape.capsuleHeight));
    outB = Add(b->position, Scale(axis, -b->shape.capsuleHeight));
}

// Sphere vs Sphere → contact (A = a's sphere, B = b's sphere).
static bool SphereSphereContact(RigidBodyInstance* a, RigidBodyInstance* b, RigidContact& out)
{
    const float ra = a->shape.sphereRadius;
    const float rb = b->shape.sphereRadius;
    const Vector3 d = Sub(b->position, a->position);
    const float dist = Length(d);
    const float pen = ra + rb - dist;
    // Speculative: still report a near miss the pair could close this step.
    if (pen <= -SpeculativeMargin(a, b)) return false;
    out.a = a; out.b = b;
    out.position = Add(a->position, Scale(d, dist > 1e-9f ? ra / dist : 0.0f));
    out.normal = dist > 1e-9f ? Scale(d, 1.0f / dist) : V3(0.0f, 1.0f, 0.0f);
    out.penetration = pen;
    out.friction = 0.5f * (a->friction + b->friction);
    out.restitution = 0.5f * (a->restitution + b->restitution);
    return true;
}

// Sphere vs OBB; normal points A→B, from the sphere center toward the closest box point.
static bool SphereOBBContact(RigidBodyInstance* sphere, RigidBodyInstance* box, RigidContact& out)
{
    const OBB o = MakeShapeOBB(box, box->position, box->orientation);

    // Closest point on box to sphere center, in box-local space.
    const Vector3 rel = Sub(sphere->position, o.center);
    Vector3 local = V3Zero();
    float axisDist[3];
    for (int i = 0; i < 3; ++i)
    {
        axisDist[i] = Dot(rel, o.axis[i]);
        const float clamped = ClampF(axisDist[i], -VComp(o.half, i), VComp(o.half, i));
        local = Add(local, Scale(o.axis[i], clamped));
    }
    const Vector3 closest = Add(o.center, local);
    const Vector3 delta = Sub(sphere->position, closest);
    const float dist = Length(delta);
    const float pen = sphere->shape.sphereRadius - dist;
    // Speculative: still report a near miss the pair could close this step.
    if (pen <= -SpeculativeMargin(sphere, box)) return false;

    out.a = sphere; out.b = box;
    out.position = closest;
    out.normal = dist > 1e-9f ? Scale(delta, -1.0f / dist) : V3(0.0f, 1.0f, 0.0f);
    out.penetration = pen;
    out.friction = 0.5f * (sphere->friction + box->friction);
    out.restitution = 0.5f * (sphere->restitution + box->restitution);
    return true;
}

// Sphere vs Capsule → contact. Normal points A→B (sphere → capsule).
static bool SphereCapsuleContact(RigidBodyInstance* sphere, RigidBodyInstance* capsule, RigidContact& out)
{
    Vector3 capA, capB;
    CapsuleSegment(capsule, capA, capB);
    const Vector3 closest = ClosestPointOnSegment(sphere->position, capA, capB);
    const Vector3 delta = Sub(sphere->position, closest);
    const float dist = Length(delta);
    const float pen = sphere->shape.sphereRadius + capsule->shape.capsuleRadius - dist;
    // Speculative: still report a near miss the pair could close this step.
    if (pen <= -SpeculativeMargin(sphere, capsule)) return false;

    out.a = sphere; out.b = capsule;
    out.position = Add(closest, Scale(delta, dist > 1e-9f ? capsule->shape.capsuleRadius / dist : 0.0f));
    out.normal = dist > 1e-9f ? Scale(delta, -1.0f / dist) : V3(0.0f, 1.0f, 0.0f);
    out.penetration = pen;
    out.friction = 0.5f * (sphere->friction + capsule->friction);
    out.restitution = 0.5f * (sphere->restitution + capsule->restitution);
    return true;
}

// Capsule vs Capsule → contact (segment-segment closest points).
static void ClosestSegments(const Vector3& p1, const Vector3& q1, const Vector3& p2, const Vector3& q2,
    Vector3& outC1, Vector3& outC2)
{
    // Ericson, Real-Time Collision Detection §5.1.9 (clamped robust variant).
    const Vector3 d1 = Sub(q1, p1);
    const Vector3 d2 = Sub(q2, p2);
    const Vector3 r = Sub(p1, p2);
    const float a = Dot(d1, d1);
    const float e = Dot(d2, d2);
    const float f = Dot(d2, r);
    float s = 0.0f, t = 0.0f;

    if (a <= 1e-12f && e <= 1e-12f) { outC1 = p1; outC2 = p2; return; }
    if (a <= 1e-12f) t = ClampF(f / e, 0.0f, 1.0f);
    else
    {
        const float c = Dot(d1, r);
        if (e <= 1e-12f) s = ClampF(-c / a, 0.0f, 1.0f);
        else
        {
            const float b = Dot(d1, d2);
            const float denom = a * e - b * b;
            if (denom > 1e-12f) s = ClampF((b * f - c * e) / denom, 0.0f, 1.0f);
            else s = 0.0f;
            t = (b * s + f) / e;
            if (t < 0.0f) { t = 0.0f; s = ClampF(-c / a, 0.0f, 1.0f); }
            else if (t > 1.0f) { t = 1.0f; s = ClampF((b - c) / a, 0.0f, 1.0f); }
        }
    }
    outC1 = Add(p1, Scale(d1, s));
    outC2 = Add(p2, Scale(d2, t));
}

static bool CapsuleCapsuleContact(RigidBodyInstance* a, RigidBodyInstance* b, RigidContact& out)
{
    Vector3 a0, a1, b0, b1;
    CapsuleSegment(a, a0, a1);
    CapsuleSegment(b, b0, b1);
    Vector3 ca, cb;
    ClosestSegments(a0, a1, b0, b1, ca, cb);
    const Vector3 delta = Sub(cb, ca);
    const float dist = Length(delta);
    const float pen = a->shape.capsuleRadius + b->shape.capsuleRadius - dist;
    // Speculative: still report a near miss the pair could close this step.
    if (pen <= -SpeculativeMargin(a, b)) return false;

    out.a = a; out.b = b;
    out.position = dist > 1e-9f ? Add(ca, Scale(delta, 0.5f)) : ca;
    out.normal = dist > 1e-9f ? Scale(delta, 1.0f / dist) : V3(0.0f, 1.0f, 0.0f);
    out.penetration = pen;
    out.friction = 0.5f * (a->friction + b->friction);
    out.restitution = 0.5f * (a->restitution + b->restitution);
    return true;
}

// Capsule vs OBB → up to 2 contacts
static int CapsuleOBBContact(RigidBodyInstance* capsule, RigidBodyInstance* box, RigidContact* out, int maxOut)
{
    const OBB o = MakeShapeOBB(box, box->position, box->orientation);
    Vector3 capA, capB;
    CapsuleSegment(capsule, capA, capB);

    const float r = capsule->shape.capsuleRadius;
    // Speculative: still report a near miss the pair could close this step.
    const float spec = SpeculativeMargin(capsule, box);
    int written = 0;

    auto addContact = [&](const Vector3& closestInBox, const Vector3& segPoint) -> bool
        {
            if (written >= maxOut) return false;
            const Vector3 delta = Sub(segPoint, closestInBox);
            const float dist = Length(delta);
            const float pen = r - dist;
            if (pen <= -spec) return false;
            RigidContact& c = out[written++];
            c.a = capsule; c.b = box;
            c.position = closestInBox;
            c.normal = dist > 1e-9f ? Scale(delta, -1.0f / dist) : V3(0.0f, 1.0f, 0.0f);
            c.penetration = pen;
            c.friction = 0.5f * (capsule->friction + box->friction);
            c.restitution = 0.5f * (capsule->restitution + box->restitution);
            return true;
        };

    auto clampToBox = [&](const Vector3& p) -> Vector3
        {
            const Vector3 rel = Sub(p, o.center);
            Vector3 acc = o.center;
            for (int i = 0; i < 3; ++i)
            {
                const float clamped = ClampF(Dot(rel, o.axis[i]), -VComp(o.half, i), VComp(o.half, i));
                acc = Add(acc, Scale(o.axis[i], clamped));
            }
            return acc;
        };

    addContact(clampToBox(capA), capA);
    addContact(clampToBox(capB), capB);
    return written;
}


// Trimesh narrowphase (rigid-rigid)

// Ericson's ClosestPtPointTriangle; DotPair does each (ab,ac) pair in one SIMD shot.
static Vector3 ClosestPointOnTriangle(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c)
{
    const Vector3 ab = Sub(b, a);
    const Vector3 ac = Sub(c, a);
    const Vector3 ap = Sub(p, a);

    const Vector3 dAP = DotPair(ap, ab, ac);
    const float d1 = dAP.x;
    const float d2 = dAP.y;

    if (d1 <= 0.0f && d2 <= 0.0f)
        return a;

    const Vector3 bp = Sub(p, b);
    const Vector3 dBP = DotPair(bp, ab, ac);
    const float d3 = dBP.x;
    const float d4 = dBP.y;

    if (d3 >= 0.0f && d4 <= d3)
        return b;

    const float vc = d1 * d4 - d3 * d2;

    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
    {
        const float v = d1 / (d1 - d3);
        return Add(a, Scale(ab, v));
    }

    const Vector3 cp = Sub(p, c);
    const Vector3 dCP = DotPair(cp, ab, ac);
    const float d5 = dCP.x;
    const float d6 = dCP.y;

    if (d6 >= 0.0f && d5 <= d6)
        return c;

    const float vb = d5 * d2 - d1 * d6;

    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
    {
        const float w = d2 / (d2 - d6);
        return Add(a, Scale(ac, w));
    }

    const float va = d3 * d6 - d5 * d4;

    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
    {
        const Vector3 bc = Sub(c, b);
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));

        return Add(b, Scale(bc, w));
    }

    const float denom = 1.0f / (va + vb + vc);
    const float v = vb * denom;
    const float w = vc * denom;

    return Add(a, Add(Scale(ab, v), Scale(ac, w)));
}

// World-space triangle f of body's shape.
static void GetWorldTriangle(const RigidBodyInstance* body, int f, Vector3& a, Vector3& b, Vector3& c)
{
    const RigidCollisionShape& s = body->shape;
    const int i0 = s.faces[(size_t)f * 3 + 0];
    const int i1 = s.faces[(size_t)f * 3 + 1];
    const int i2 = s.faces[(size_t)f * 3 + 2];
    a = Add(body->position, QRotate(body->orientation, s.vertices[i0]));
    b = Add(body->position, QRotate(body->orientation, s.vertices[i1]));
    c = Add(body->position, QRotate(body->orientation, s.vertices[i2]));
}

// Closest-point geometry of p against a trimesh body's triangles.
static int TrimeshPointGeometry(const RigidBodyInstance* tri,
    const Vector3& p, const Vector3& refPoint, float pointRadius,
    Vector3& outQ, Vector3& outN, float& outPen, float speculativeMargin = 0.0f)
{
    const RigidCollisionShape& s = tri->shape;
    constexpr float kSlop = 0.005f;
    const float margin = pointRadius + kSlop + speculativeMargin;

    float bestDistSq = 1e30f;
    int bestF = -1;
    Vector3 bestQ = V3Zero();

    for (int f = 0; f < s.FaceCount(); ++f)
    {
        Vector3 a, b, c;
        GetWorldTriangle(tri, f, a, b, c);

        // Per-triangle AABB reject against the point sphere.
        if (MinF(MinF(a.x, b.x), c.x) > p.x + margin || MaxF(MaxF(a.x, b.x), c.x) < p.x - margin ||
            MinF(MinF(a.y, b.y), c.y) > p.y + margin || MaxF(MaxF(a.y, b.y), c.y) < p.y - margin ||
            MinF(MinF(a.z, b.z), c.z) > p.z + margin || MaxF(MaxF(a.z, b.z), c.z) < p.z - margin)
            continue;

        const Vector3 q = ClosestPointOnTriangle(p, a, b, c);
        const float dSq = LengthSq(Sub(p, q));
        if (dSq < bestDistSq)
        {
            bestDistSq = dSq;
            bestF = f;
            bestQ = q;
        }
    }
    if (bestF < 0) return 0;

    Vector3 a, b, c;
    GetWorldTriangle(tri, bestF, a, b, c);
    Vector3 n = Cross(Sub(b, a), Sub(c, a));
    const float nLen = Length(n);
    if (nLen < 1e-9f) return 0; // degenerate triangle
    n = Scale(n, 1.0f / nLen);

    if (Dot(Sub(refPoint, bestQ), n) < 0.0f)
        n = Neg(n);

    const float sDist = Dot(Sub(p, bestQ), n); // >0 on the approach side, <0 inside
    if (sDist >= margin) return 0;             // clear by more than slop + margin — no contact

    outQ = bestQ;
    outN = n;

    outPen = (speculativeMargin > 0.0f) ? (pointRadius - sDist) : MaxF(pointRadius - sDist, 0.0f);
    return 1;
}

// Rigid-contact wrapper
static int PointVsTrimeshContact(RigidBodyInstance* tri, RigidBodyInstance* pointOwner,
    const Vector3& p, float pointRadius, RigidContact& out)
{
    Vector3 q, n; float pen;
    const float spec = SpeculativeMargin(tri, pointOwner);
    if (!TrimeshPointGeometry(tri, p, pointOwner->position, pointRadius, q, n, pen, spec))
        return 0;

    out.a = tri;                    // triangle side
    out.b = pointOwner;             // point side
    out.position = q;
    out.normal = n;                 // A→B: surface toward the point side
    out.penetration = pen;
    out.friction = 0.5f * (tri->friction + pointOwner->friction);
    out.restitution = 0.5f * (tri->restitution + pointOwner->restitution);
    return 1;
}

// Trimesh vs Sphere
static int TrimeshSphereContacts(RigidBodyInstance* tri, RigidBodyInstance* sph, RigidContact* out, int maxOut)
{
    if (maxOut < 1) return 0;
    return PointVsTrimeshContact(tri, sph, sph->position,
        sph->shape.sphereRadius, out[0]);
}

// Trimesh vs Box
static int TrimeshBoxContacts(RigidBodyInstance* tri, RigidBodyInstance* box, RigidContact* out, int maxOut)
{
    const OBB o = MakeShapeOBB(box, box->position, box->orientation);
    int written = 0;

    // Box corners as points vs the trimesh.
    for (int ci = 0; ci < 8 && written < maxOut; ++ci)
    {
        Vector3 corner = o.center;
        for (int i = 0; i < 3; ++i)
        {
            const float sign = (ci & (1 << i)) ? 1.0f : -1.0f;
            corner = Add(corner, Scale(o.axis[i], sign * VComp(o.half, i)));
        }

        RigidContact c;
        if (PointVsTrimeshContact(tri, box, corner, 0.0f, c))
            out[written++] = c;
    }

    // Trimesh vertices vs the box, skipped when corners already produced a manifold.
    if (written < 2)
    {
        constexpr float kSlop = 0.005f;
        const float spec = SpeculativeMargin(tri, box);
        for (const Vector3& lv : tri->shape.vertices)
        {
            if (written >= maxOut) break;
            const Vector3 v = Add(tri->position, QRotate(tri->orientation, lv));
            const Vector3 rel = Sub(v, o.center);
            float axisDist[3];
            bool inside = true;
            Vector3 acc = o.center;
            for (int i = 0; i < 3; ++i)
            {
                axisDist[i] = Dot(rel, o.axis[i]);
                if (AbsF(axisDist[i]) >= VComp(o.half, i)) inside = false;
                const float clamped = ClampF(axisDist[i], -VComp(o.half, i), VComp(o.half, i));
                acc = Add(acc, Scale(o.axis[i], clamped));
            }

            float pen;
            Vector3 normal;
            Vector3 position;
            if (inside)
            {
                // Inside the box: push toward the nearest face (naive clamp gives a zero delta).
                float bestDepth = 1e30f; int bestAxis = 0; float bestSign = 1.0f;
                for (int i = 0; i < 3; ++i)
                {
                    const float d = VComp(o.half, i) - AbsF(axisDist[i]);
                    if (d < bestDepth)
                    {
                        bestDepth = d;
                        bestAxis = i;
                        bestSign = axisDist[i] >= 0.0f ? 1.0f : -1.0f;
                    }
                }
                pen = bestDepth;
                normal = Scale(o.axis[bestAxis], bestSign);
                position = v;
            }
            else
            {
                const Vector3 delta = Sub(v, acc);
                const float dist = Length(delta);

                if (dist > kSlop + spec) continue; // clear by more than slop + margin — no contact
                pen = (spec > 0.0f) ? -dist : MaxF(-dist, 0.0f);
                normal = dist > 1e-9f ? Scale(delta, 1.0f / dist) : V3(0.0f, 1.0f, 0.0f);
                position = acc;
            }

            RigidContact& c = out[written++];
            c.a = box; c.b = tri;
            c.position = position;
            c.normal = normal;             // A→B: box surface → trimesh vertex
            c.penetration = pen;
            c.friction = 0.5f * (box->friction + tri->friction);
            c.restitution = 0.5f * (box->restitution + tri->restitution);
        }
    }
    return written;
}

// Trimesh vs Capsule
static int TrimeshCapsuleContacts(RigidBodyInstance* tri, RigidBodyInstance* cap, RigidContact* out, int maxOut)
{
    Vector3 capA, capB;
    CapsuleSegment(cap, capA, capB);

    const float r = cap->shape.capsuleRadius;
    int written = 0;

    // Segment endpoints + midpoint as sphere samples.
    const Vector3 samples[3] = { capA, capB, Scale(Add(capA, capB), 0.5f) };
    for (int k = 0; k < 3 && written < maxOut; ++k)
    {
        RigidContact c;
        if (PointVsTrimeshContact(tri, cap, samples[k], r, c))
            out[written++] = c;
    }

    // Trimesh vertices vs the capsule segment.
    if (written < 2)
    {
        constexpr float kSlop = 0.005f;
        const float spec = SpeculativeMargin(tri, cap);
        for (const Vector3& lv : tri->shape.vertices)
        {
            if (written >= maxOut) break;
            const Vector3 v = Add(tri->position, QRotate(tri->orientation, lv));
            const Vector3 q = ClosestPointOnSegment(v, capA, capB);
            const Vector3 delta = Sub(v, q);
            const float dist = Length(delta);
            const float pen = r - dist;
            if (pen < -(kSlop + spec)) continue; // clear by more than slop + margin — no contact

            RigidContact& c = out[written++];
            c.a = cap; c.b = tri;
            c.position = q;
            c.normal = dist > 1e-9f ? Scale(delta, 1.0f / dist) : V3(0.0f, 1.0f, 0.0f);
            c.penetration = (spec > 0.0f) ? pen : MaxF(pen, 0.0f);
            c.friction = 0.5f * (cap->friction + tri->friction);
            c.restitution = 0.5f * (cap->restitution + tri->restitution);
        }
    }
    return written;
}

// Trimesh vs Trimesh
static int TrimeshTrimeshContacts(RigidBodyInstance* a, RigidBodyInstance* b, RigidContact* out, int maxOut)
{
    int written = 0;

    for (const Vector3& lv : a->shape.vertices)
    {
        if (written >= maxOut) break;
        const Vector3 v = Add(a->position, QRotate(a->orientation, lv));
        RigidContact c;
        // a's vertex vs b's triangles: triangle side = b, point side = a.
        if (PointVsTrimeshContact(b, a, v, 0.0f, c))
            out[written++] = c;
    }

    if (written < 2)
    {
        for (const Vector3& lv : b->shape.vertices)
        {
            if (written >= maxOut) break;
            const Vector3 v = Add(b->position, QRotate(b->orientation, lv));
            RigidContact c;
            if (PointVsTrimeshContact(a, b, v, 0.0f, c))
                out[written++] = c;
        }
    }
    return written;
}

// Shape-kind dispatch for rigid-rigid narrowphase
static int RigidRigidContact(RigidBodyInstance* a, RigidBodyInstance* b, RigidContact* out, int maxOut)
{
    const RigidShapeType ta = a->shape.type;
    const RigidShapeType tb = b->shape.type;

    if (ta == RigidShapeType::Sphere && tb == RigidShapeType::Sphere)
        return SphereSphereContact(a, b, out[0]) ? 1 : 0;
    if (ta == RigidShapeType::Sphere && tb == RigidShapeType::Box)
        return SphereOBBContact(a, b, out[0]) ? 1 : 0;
    if (ta == RigidShapeType::Box && tb == RigidShapeType::Sphere)
        return SphereOBBContact(b, a, out[0]) ? 1 : 0;
    if (ta == RigidShapeType::Sphere && tb == RigidShapeType::Capsule)
        return SphereCapsuleContact(a, b, out[0]) ? 1 : 0;
    if (ta == RigidShapeType::Capsule && tb == RigidShapeType::Sphere)
        return SphereCapsuleContact(b, a, out[0]) ? 1 : 0;
    if (ta == RigidShapeType::Capsule && tb == RigidShapeType::Capsule)
        return CapsuleCapsuleContact(a, b, out[0]) ? 1 : 0;
    if (ta == RigidShapeType::Capsule && tb == RigidShapeType::Box)
        return CapsuleOBBContact(a, b, out, maxOut);
    if (ta == RigidShapeType::Box && tb == RigidShapeType::Capsule)
        return CapsuleOBBContact(b, a, out, maxOut);

    // Trimesh pairs — the common case: every Unity mesh rigid is a Trimesh.
    if (ta == RigidShapeType::Trimesh && tb == RigidShapeType::Trimesh)
        return TrimeshTrimeshContacts(a, b, out, maxOut);
    if (ta == RigidShapeType::Trimesh && tb == RigidShapeType::Sphere)
        return TrimeshSphereContacts(a, b, out, maxOut);
    if (ta == RigidShapeType::Sphere && tb == RigidShapeType::Trimesh)
        return TrimeshSphereContacts(b, a, out, maxOut);
    if (ta == RigidShapeType::Trimesh && tb == RigidShapeType::Box)
        return TrimeshBoxContacts(a, b, out, maxOut);
    if (ta == RigidShapeType::Box && tb == RigidShapeType::Trimesh)
        return TrimeshBoxContacts(b, a, out, maxOut);
    if (ta == RigidShapeType::Trimesh && tb == RigidShapeType::Capsule)
        return TrimeshCapsuleContacts(a, b, out, maxOut);
    if (ta == RigidShapeType::Capsule && tb == RigidShapeType::Trimesh)
        return TrimeshCapsuleContacts(b, a, out, maxOut);

    return 0;
}

constexpr float kSweepMargin = 0.02f;

static inline Vector3 OrientStaticNormalOutward(const Vector3& n, const Vector3& hitPoint,
    const RigidBodyInstance* body)
{
    if (Dot(n, Sub(body->position, hitPoint)) < 0.0f)
        return Neg(n);
    return n;
}

// Deep-penetration recovery for point-vs-triangle-static: the sweep only sees a skin, so a
// point behind the face is invisible to it and sinks; this measures against the solid side.
static bool RecoverDeepTriangleContact(const StaticColliderData& c, const Vector3& point,
    const Vector3& ref, Vector3& outPoint, Vector3& outNormal, float& outDepth)
{
    if (c.type != COL_TRIANGLE) return false;

    Vector3 fn = Cross(Sub(c.v1, c.v0), Sub(c.v2, c.v0));
    const float len = Length(fn);
    if (len < 1e-12f) return false; // degenerate triangle
    fn = Scale(fn, 1.0f / len);

    const Vector3 q = ClosestPointOnTriangle(point, c.v0, c.v1, c.v2);
    if (Dot(fn, Sub(ref, q)) < 0.0f) fn = Neg(fn); // solid side faces away from ref

    // Ensure the point lies within the triangle's 2D footprint (in-plane deviation from triangle boundary)
    const Vector3 inPlane = Sub(point, Add(q, Scale(fn, Dot(Sub(point, q), fn))));
    if (LengthSq(inPlane) > 0.0025f) return false; // > 5cm outside the triangle 2D prism

    const float depth = -Dot(Sub(point, q), fn);
    if (depth <= 0.0f) return false; // in front of the face — the sweep owns that case

    outPoint = q;
    outNormal = fn;
    outDepth = depth;
    return true;
}

// Convenience flatten of TestSweptStaticCollider: writes the hit only when one was found.
static bool TestSweptStatic(const StaticColliderData& c, const Vector3& cur, const Vector3& vel, float radius,
    Vector3& outPoint, Vector3& outNormal)
{
    bool found = false;
    float earliestT = std::numeric_limits<float>::max();
    Vector3 hitPoint = V3Zero();
    Vector3 hitNormal = V3(0.0f, 1.0f, 0.0f);
    StaticColliderData hitCol = {};
    TestSweptStaticCollider(c, cur, vel, radius, found, earliestT, hitPoint, hitNormal, hitCol);
    if (found) { outPoint = hitPoint; outNormal = hitNormal; }
    return found;
}

// A static contact event parked by the parallel static pass, replayed in body order by the merge.
struct PendingStaticEvent
{
    Vector3 point;
    Vector3 normal;
};

// Shared by every branch of RigidStaticContacts: derives point velocity/tangent from a
// world contact point and records it. `outwardNormal` points away from the static surface.
static inline void RecordRigidStaticContact(RigidBodyInstance* body, const Vector3& point, const Vector3& outwardNormal)
{
    const Vector3 r = Sub(point, body->position);
    const Vector3 pointVel = Add(body->linearVelocity, Cross(body->angularVelocity, r));
    const float vn = Dot(pointVel, outwardNormal);
    const Vector3 tangentVel = Sub(pointVel, Scale(outwardNormal, vn));
    RecordContactEvent(body->instanceId, COLLISIONS_STATIC_CONTACT_ID,
        point, outwardNormal, MaxF(-vn, 0.0f), tangentVel);
}

// Detection is parallel, so it parks the event; the serial merge replays it in body order.
static inline void QueueStaticContactEvent(std::vector<PendingStaticEvent>& events, const Vector3& point, const Vector3& outwardNormal)
{
    events.push_back(PendingStaticEvent{ point, outwardNormal });
}

// Sweeps up to `pointCount` world-space points (trimesh vertices or box corners — supplied
// lazily via getWorldPoint(i) so we never transform more than we need) against one static
// collider, stopping once `maxContacts` contacts have been written. Shared by the Trimesh
// and Box branches of RigidStaticContacts, which previously duplicated this loop.
template <class PointFn>
static bool SweepPointsAgainstStatic(
    RigidBodyInstance* body, const StaticColliderData& c, int pointCount, PointFn&& getWorldPoint,
    const Vector3& travel, float specMargin, float friction, float restitution,
    int maxContacts, std::vector<RigidContact>& out, std::vector<PendingStaticEvent>& events)
{
    int written = 0;
    float deepest = 0.0f;
    Vector3 deepPos = V3Zero(), deepNrm = V3Zero();

    for (int i = 0; i < pointCount && written < maxContacts; ++i)
    {
        const Vector3 world = getWorldPoint(i);
        const Vector3 vVel = Add(travel, Cross(body->angularVelocity, Sub(world, body->position)));
        Vector3 p, n;
        float pen;
        if (TestSweptStatic(c, Sub(world, vVel), vVel, kSweepMargin, p, n))
        {
            n = OrientStaticNormalOutward(n, p, body);
            pen = -Dot(Sub(world, p), n);
        }
        else if (!RecoverDeepTriangleContact(c, world, body->position, p, n, pen))
            continue;
        const float margin = MaxF(0.005f, specMargin);
        if (pen <= -margin) continue;

        if (pen > deepest) { deepest = pen; deepPos = p; deepNrm = n; }

        RigidContact& vc = out.emplace_back();
        vc.a = body; vc.b = nullptr;
        vc.position = p;
        vc.normal = Neg(n);          // outward-static → A→B (rigid→static)
        vc.penetration = pen;
        vc.friction = friction;
        vc.restitution = restitution;
        ++written;
    }

    if (written > 0)
    {
        // One contact event for the collider (deepest point).
        QueueStaticContactEvent(events, deepPos, deepNrm);
    }
    return written > 0;
}

// Caller must run ProcessDirtyStatics() first and keep the static set frozen for the whole call.
static void RigidStaticContacts(RigidBodyInstance* body, float dt, std::vector<RigidContact>& out, std::vector<PendingStaticEvent>& events)
{
    const int colliderCount = (int)g_StaticColliderData.size();
    if (colliderCount <= 0) return;
    const StaticColliderData* colliders = g_StaticColliderData.data();

    const RigidCollisionShape& s = body->shape;
    if (s.Empty()) return;

    // Swept AABB of the whole shape this step (pose start → pose end).
    Vector3 curMin, curMax, endMin, endMax;
    Collisions_RigidShapeWorldAabb(body, curMin, curMax);
    ShapePoseAabb(body->shape, body->prevPosition, body->prevOrientation, endMin, endMax);
    const Vector3 qMin = Vector3{ MinF(curMin.x, endMin.x), MinF(curMin.y, endMin.y), MinF(curMin.z, endMin.z), 0.0f };
    const Vector3 qMax = Vector3{ MaxF(curMax.x, endMax.x), MaxF(curMax.y, endMax.y), MaxF(curMax.z, endMax.z), 0.0f };

    static thread_local std::vector<int> candidates;
    candidates.clear();
    QueryStaticBVH(qMin, qMax, [&](int idx) { candidates.push_back(idx); });

    // World-segment endpoints for capsule shapes.
    Vector3 capA, capB;
    if (s.type == RigidShapeType::Capsule) CapsuleSegment(body, capA, capB);

    const Vector3 travel = Sub(body->position, body->prevPosition);
    // Speculative margin for the primitive-vs-primitive branches with no swept test.
    const float specMargin = SpeculativeMargin(body, nullptr);

    for (int idx : candidates)
    {
        const StaticColliderData& c = colliders[idx];

        // Layer filtering matches the soft static path (no filter on static contacts).

        RigidContact contact;
        contact.a = body;
        contact.b = nullptr;
        contact.friction = 0.5f * (body->friction + c.slidingFriction);
        contact.restitution = 0.5f * (body->restitution + c.restitution);

        bool hit = false;

        if (s.type == RigidShapeType::Trimesh)
        {
            hit = SweepPointsAgainstStatic(body, c, (int)s.vertices.size(),
                [&](int i) { return Add(body->position, QRotate(body->orientation, s.vertices[i])); },
                travel, specMargin, contact.friction, contact.restitution, 8, out, events);
            continue; // trimesh fully handled — skip the single-contact path
        }

        // Evaluate overlap at the CURRENT pose; swept queries clamp tunneling for fast bodies.
        switch (s.type)
        {
        case RigidShapeType::Sphere:
        {
            if (c.type == COL_SPHERE)
            {
                const Vector3 d = Sub(c.center, body->position);
                const float dist = Length(d);
                contact.penetration = s.sphereRadius + c.radius - dist;
                if (contact.penetration > -specMargin) // speculative: gap within this step's travel
                {
                    hit = true;
                    contact.normal = dist > 1e-9f ? Scale(d, 1.0f / dist) : V3(0.0f, 1.0f, 0.0f);
                    contact.position = Add(body->position, Scale(d, dist > 1e-9f ? s.sphereRadius / dist : 0.0f));
                }
            }
            else if (c.type == COL_BOX)
            {
                // Point-OBB distance.
                const Vector3 halfSize = Scale(c.size, 0.5f);
                const Vector3 ax = QRotate(c.rotation, V3(1.0f, 0.0f, 0.0f));
                const Vector3 ay = QRotate(c.rotation, V3(0.0f, 1.0f, 0.0f));
                const Vector3 az = QRotate(c.rotation, V3(0.0f, 0.0f, 1.0f));
                const Vector3 rel = Sub(body->position, c.center);
                Vector3 closest = c.center;
                const float dx = Dot(rel, ax), dy = Dot(rel, ay), dz = Dot(rel, az);
                closest = Add(closest, Scale(ax, ClampF(dx, -halfSize.x, halfSize.x)));
                closest = Add(closest, Scale(ay, ClampF(dy, -halfSize.y, halfSize.y)));
                closest = Add(closest, Scale(az, ClampF(dz, -halfSize.z, halfSize.z)));
                const Vector3 delta = Sub(body->position, closest);
                const float dist = Length(delta);
                contact.penetration = s.sphereRadius - dist;
                if (contact.penetration > -specMargin) // speculative: gap within this step's travel
                {
                    hit = true;
                    // Solver convention: normal points A→B (rigid → static); delta points outward from the static.
                    contact.normal = dist > 1e-9f ? Scale(delta, -1.0f / dist) : V3(0.0f, -1.0f, 0.0f);
                    contact.position = closest;
                }
            }
            else if (c.type == COL_CAPSULE || c.type == COL_TRIANGLE || c.type == COL_TERRAIN)
            {
                // Swept-sphere fallback at t=1 (current position).
                Vector3 p, n;
                const Vector3 vel = travel; // displacement this step
                float depth;
                if (TestSweptStatic(c, body->prevPosition, vel, s.sphereRadius, p, n))
                {
                    // A center that started inside gets the raw inward normal — orient it outward.
                    n = OrientStaticNormalOutward(n, p, body);
                    hit = true;
                    // n is the outward static normal; solver wants A→B.
                    contact.normal = Neg(n);
                    contact.position = p;
                    const float d = Dot(Sub(body->position, p), n);
                    contact.penetration = MaxF(s.sphereRadius - d, 0.0f);
                }
                else if (RecoverDeepTriangleContact(c, body->position, body->prevPosition, p, n, depth))
                {
                    // Center behind a thin face, past the swept skin: recover full depth + radius.
                    hit = true;
                    contact.normal = Neg(n);
                    contact.position = p;
                    contact.penetration = s.sphereRadius + depth;
                }
            }
            break;
        }
        case RigidShapeType::Box:
        {
            if (c.type == COL_BOX)
            {
                OBB A = MakeShapeOBB(body, body->position, body->orientation);
                OBB B;
                B.center = c.center;
                B.axis[0] = QRotate(c.rotation, V3(1.0f, 0.0f, 0.0f));
                B.axis[1] = QRotate(c.rotation, V3(0.0f, 1.0f, 0.0f));
                B.axis[2] = QRotate(c.rotation, V3(0.0f, 0.0f, 1.0f));
                B.half = Scale(c.size, 0.5f);
                SATResult r = TestOBBSAT(A, B);
                // Speculative margin: this branch gets no swept test, so a fast box would tunnel.
                const float margin = SpeculativeMargin(body, nullptr);
                if (r.overlapping || r.separation <= margin)
                {
                    ContactPoint pts[4];
                    const int n = GenerateBoxBoxManifold(A, B, r.axis, pts, 4, margin);
                    if (n > 0)
                    {
                        for (int k = 0; k < n; ++k)
                        {
                            RigidContact& vc = out.emplace_back();
                            vc.a = body; vc.b = nullptr;
                            vc.position = pts[k].point;
                            vc.normal = r.axis;          // A→B (rigid → static)
                            vc.penetration = pts[k].penetration;
                            vc.friction = contact.friction;
                            vc.restitution = contact.restitution;
                        }
                        // One contact event for the collider (deepest point); speculative-only reports none.
                        int deep = 0;
                        for (int k = 1; k < n; ++k)
                            if (pts[k].penetration > pts[deep].penetration) deep = k;
                        if (pts[deep].penetration > 0.0f)
                            QueueStaticContactEvent(events, pts[deep].point, Neg(r.axis)); // static → body
                    }
                }
                continue; // fully handled — never fall through to the single-contact tail
            }
            else if (c.type == COL_SPHERE)
            {
                OBB A = MakeShapeOBB(body, body->position, body->orientation);
                const Vector3 rel = Sub(c.center, A.center);
                Vector3 closest = A.center;
                for (int i = 0; i < 3; ++i)
                {
                    const float clamped = ClampF(Dot(rel, A.axis[i]), -VComp(A.half, i), VComp(A.half, i));
                    closest = Add(closest, Scale(A.axis[i], clamped));
                }
                const Vector3 delta = Sub(c.center, closest);
                const float dist = Length(delta);
                contact.penetration = c.radius - dist;
                if (contact.penetration > -specMargin) // speculative: gap within this step's travel
                {
                    hit = true;
                    contact.normal = dist > 1e-9f ? Scale(delta, 1.0f / dist) : V3(0.0f, 1.0f, 0.0f);
                    contact.position = closest;
                }
            }
            else
            {
                OBB A = MakeShapeOBB(body, body->position, body->orientation);
                SweepPointsAgainstStatic(body, c, 8,
                    [&](int ci) {
                        Vector3 corner = A.center;
                        for (int i = 0; i < 3; ++i)
                            corner = Add(corner, Scale(A.axis[i], (ci & (1 << i)) ? VComp(A.half, i) : -VComp(A.half, i)));
                        return corner;
                    },
                    travel, specMargin, contact.friction, contact.restitution, 8, out, events);
                continue; // fully handled — never fall through to the single-contact tail
            }
            break;
        }
        case RigidShapeType::Capsule:
        {
            if (c.type == COL_SPHERE)
            {
                const Vector3 closest = ClosestPointOnSegment(c.center, capA, capB);
                const Vector3 delta = Sub(c.center, closest);
                const float dist = Length(delta);
                contact.penetration = c.radius + s.capsuleRadius - dist;
                if (contact.penetration > -specMargin) // speculative: gap within this step's travel
                {
                    hit = true;
                    contact.normal = dist > 1e-9f ? Scale(delta, 1.0f / dist) : V3(0.0f, 1.0f, 0.0f);
                    contact.position = closest;
                }
            }
            else if (c.type == COL_CAPSULE)
            {
                Vector3 bSegA = Add(c.center, Scale(c.axis, MaxF(0.0f, c.height * 0.5f - c.radius)));
                Vector3 bSegB = Add(c.center, Scale(c.axis, -MaxF(0.0f, c.height * 0.5f - c.radius)));
                Vector3 ca, cb;
                ClosestSegments(capA, capB, bSegA, bSegB, ca, cb);
                const Vector3 delta = Sub(cb, ca);
                const float dist = Length(delta);
                contact.penetration = s.capsuleRadius + c.radius - dist;
                if (contact.penetration > -specMargin) // speculative: gap within this step's travel
                {
                    hit = true;
                    contact.normal = dist > 1e-9f ? Scale(delta, 1.0f / dist) : V3(0.0f, 1.0f, 0.0f);
                    contact.position = dist > 1e-9f ? Add(ca, Scale(delta, 0.5f)) : ca;
                }
            }
            else if (c.type == COL_BOX)
            {
                const Vector3 halfSize = Scale(c.size, 0.5f);
                const Vector3 ax = QRotate(c.rotation, V3(1.0f, 0.0f, 0.0f));
                const Vector3 ay = QRotate(c.rotation, V3(0.0f, 1.0f, 0.0f));
                const Vector3 az = QRotate(c.rotation, V3(0.0f, 0.0f, 1.0f));
                auto clampToBox = [&](const Vector3& p) -> Vector3
                    {
                        const Vector3 rel = Sub(p, c.center);
                        Vector3 acc = c.center;
                        acc = Add(acc, Scale(ax, ClampF(Dot(rel, ax), -halfSize.x, halfSize.x)));
                        acc = Add(acc, Scale(ay, ClampF(Dot(rel, ay), -halfSize.y, halfSize.y)));
                        acc = Add(acc, Scale(az, ClampF(Dot(rel, az), -halfSize.z, halfSize.z)));
                        return acc;
                    };
                const Vector3 ca = clampToBox(capA);
                const Vector3 cb = clampToBox(capB);
                const Vector3 deltaA = Sub(capA, ca);
                const float distA = Length(deltaA);
                contact.penetration = s.capsuleRadius - distA;
                if (contact.penetration > -specMargin) // speculative: gap within this step's travel
                {
                    hit = true;
                    // deltaA points outward from the static; solver wants A→B.
                    contact.normal = distA > 1e-9f ? Scale(deltaA, -1.0f / distA) : V3(0.0f, -1.0f, 0.0f);
                    contact.position = ca;
                }
                else
                {
                    const Vector3 deltaB = Sub(capB, cb);
                    const float distB = Length(deltaB);
                    contact.penetration = s.capsuleRadius - distB;
                    if (contact.penetration > -specMargin) // speculative: gap within this step's travel
                    {
                        hit = true;
                        contact.normal = distB > 1e-9f ? Scale(deltaB, -1.0f / distB) : V3(0.0f, -1.0f, 0.0f);
                        contact.position = cb;
                    }
                }
            }
            else
            {
                // Triangle/terrain statics
                Vector3 p, n;
                if (TestSweptStatic(c, capA, Add(travel, Cross(body->angularVelocity, Sub(capA, body->position))), s.capsuleRadius, p, n))
                {
                    n = OrientStaticNormalOutward(n, p, body); // penetrating starts get inward raw normals
                    hit = true;
                    contact.normal = Neg(n);
                    contact.position = p;
                    contact.penetration = MaxF(s.capsuleRadius - Dot(Sub(capA, p), n), 0.0f);
                }
                else if (TestSweptStatic(c, capB, Add(travel, Cross(body->angularVelocity, Sub(capB, body->position))), s.capsuleRadius, p, n))
                {
                    n = OrientStaticNormalOutward(n, p, body);
                    hit = true;
                    contact.normal = Neg(n);
                    contact.position = p;
                    contact.penetration = MaxF(s.capsuleRadius - Dot(Sub(capB, p), n), 0.0f);
                }
            }
            break;
        }
        case RigidShapeType::Trimesh:
            break; // handled by the dedicated multi-contact path above
        }

        if (!hit) continue;

        if (contact.penetration > 0.0f)
            QueueStaticContactEvent(events, contact.position, Neg(contact.normal)); // A→B flipped

        out.push_back(contact);
    }
}

void Collisions_RigidStep(RigidBodyInstance** bodies, int count, float dt, int solverIterations)
{
    if (!bodies || count <= 0 || dt <= 0.0f) return;

    // Phase timing; each Lap() closes the segment the previous Lap() opened.
    float rigidDetectMs = 0.0f, rigidResolveMs = 0.0f, rigidSolveMs = 0.0f;
    auto tLap = std::chrono::high_resolution_clock::now();
    auto Lap = [&tLap]()
        {
            const auto now = std::chrono::high_resolution_clock::now();
            const float ms = std::chrono::duration<float, std::milli>(now - tLap).count();
            tLap = now;
            return ms;
        };

    // Broadphase
    static std::vector<Vector3> bMin, bMax;
    bMin.resize(count); bMax.resize(count);
    GetGlobalPool().parallel_for(count, [&](int i)
        {
            Collisions_RigidShapeWorldAabb(bodies[i], bMin[i], bMax[i]);
            // include the swept start pose
            Vector3 pMin, pMax;
            ShapePoseAabb(bodies[i]->shape, bodies[i]->prevPosition, bodies[i]->prevOrientation, pMin, pMax);
            bMin[i] = Vector3{ MinF(bMin[i].x, pMin.x), MinF(bMin[i].y, pMin.y), MinF(bMin[i].z, pMin.z), 0.0f };
            bMax[i] = Vector3{ MaxF(bMax[i].x, pMax.x), MaxF(bMax[i].y, pMax.y), MaxF(bMax[i].z, pMax.z), 0.0f };
            const float expand = Length(Sub(bodies[i]->position, bodies[i]->prevPosition));
            bMin[i] = Vector3{ bMin[i].x - expand, bMin[i].y - expand, bMin[i].z - expand, 0.0f };
            bMax[i] = Vector3{ bMax[i].x + expand, bMax[i].y + expand, bMax[i].z + expand, 0.0f };
        }, 16);

    static std::vector<RigidContact> contacts;
    contacts.clear();

    // Rigid-rigid narrowphase, parallel.
    //
    // Order-independence contract (replaces the old in-place CCD rollback, whose
    // committed poses and later pair tests depended on pair processing order):
    //   1. Candidate pairs: cheap flag/layer/AABB filters only. The sleep filters read
    //      pre-pass flags, so the candidate set is fixed before any wake happens (the
    //      old loop re-filtered after earlier pairs had already woken bodies).
    //   2. Swept-TOI pass: every box pair's CCD runs against unmodified poses and
    //      writes a per-pair hit time. Pure - safe on the pool.
    //   3. Rollback resolution: each body takes the MIN TOI over all sweeps that hit
    //      it. min() over a fixed input set is order-independent, so the committed
    //      pose is the same no matter how the pool schedules the pairs.
    //   4. Narrowphase: one pool task per pair, each writing only its own contact
    //      slot. Box pairs test at their own TOI pose, which is what the old code
    //      produced for the first pair that claimed a body.
    //   5. Serial merge: replays candidate order into the shared contact list (same
    //      contact order as the old serial loop) and performs the only shared-state
    //      writes on this path: wake, then the rollback commit.
    // The solver below stays serial (Gauss-Seidel); only detection is parallel.

    struct RigidPair { int a, b; };

    // Phase 1: sweep-and-prune candidate pairs. This is the same broadphase shape
    // used by the soft-body path: sorted X intervals, then Y/Z rejection. The order
    // is reused with insertion sort because rigid bodies usually move only a little.
    static std::vector<int> liveBodies;
    static std::vector<uint8_t> activeMask;
    static std::vector<int> sapOrder;
    liveBodies.clear();
    activeMask.assign((size_t)count, 0);
    for (int i = 0; i < count; ++i)
    {
        RigidBodyInstance* b = bodies[i];
        if (b && !b->shape.Empty())
        {
            liveBodies.push_back(i);
            activeMask[i] = 1;
        }
    }

    const int liveCount = (int)liveBodies.size();
    bool reuseOrder = ((int)sapOrder.size() == liveCount);
    if (reuseOrder)
    {
        for (int idx : sapOrder)
        {
            if (idx < 0 || idx >= count || !activeMask[idx])
            {
                reuseOrder = false;
                break;
            }
        }
    }

    auto SapBefore = [&](int lhs, int rhs)
        {
            if (bMin[lhs].x != bMin[rhs].x) return bMin[lhs].x < bMin[rhs].x;
            return lhs < rhs;
        };

    if (!reuseOrder)
    {
        sapOrder = liveBodies;
        std::sort(sapOrder.begin(), sapOrder.end(), SapBefore);
    }
    else
    {
        for (int i = 1; i < liveCount; ++i)
        {
            const int key = sapOrder[i];
            int j = i - 1;
            while (j >= 0 && SapBefore(key, sapOrder[j]))
            {
                sapOrder[j + 1] = sapOrder[j];
                --j;
            }
            sapOrder[j + 1] = key;
        }
    }

    static std::vector<RigidPair> rigidPairs;
    rigidPairs.clear();
    for (int s = 0; s < liveCount; ++s)
    {
        const int ia = sapOrder[s];
        RigidBodyInstance* a = bodies[ia];
        const float aMaxX = bMax[ia].x;

        for (int t = s + 1; t < liveCount; ++t)
        {
            const int ib = sapOrder[t];
            if (bMin[ib].x > aMaxX) break;

            RigidBodyInstance* b = bodies[ib];
            if (a->isKinematic && b->isKinematic) continue;
            if (a->isSleeping && b->isSleeping) continue;
            if (!(a->collisionLayerMask & (1 << b->layer))) continue;
            if (!(b->collisionLayerMask & (1 << a->layer))) continue;

            if (bMax[ia].y < bMin[ib].y || bMin[ia].y > bMax[ib].y ||
                bMax[ia].z < bMin[ib].z || bMin[ia].z > bMax[ib].z) continue;

            const int first = ia < ib ? ia : ib;
            const int second = ia < ib ? ib : ia;
            rigidPairs.push_back({ first, second });
        }
    }

    // Preserve the old lexicographic pair order for the serial solver and warm cache.
    std::sort(rigidPairs.begin(), rigidPairs.end(), [](const RigidPair& lhs, const RigidPair& rhs)
        {
            return lhs.a != rhs.a ? lhs.a < rhs.a : lhs.b < rhs.b;
        });

    const int pairCount = (int)rigidPairs.size();

    // Phase 2: swept TOI per box pair, against unmodified poses. Disjoint per-pair
    // writes only - no shared mutable state.
    struct PairHit { float toi = 1.0f; bool boxPair = false; bool hit = false; };
    static std::vector<PairHit> pairHits;
    pairHits.resize(pairCount);

    GetGlobalPool().parallel_for(pairCount, [&](int p)
        {
            const RigidPair& pr = rigidPairs[p];
            RigidBodyInstance* a = bodies[pr.a];
            RigidBodyInstance* b = bodies[pr.b];
            PairHit& h = pairHits[p];
            h.boxPair = (a->shape.type == RigidShapeType::Box && b->shape.type == RigidShapeType::Box);
            h.hit = false;
            if (!h.boxPair) return;
            float toi;
            if (BoxBoxSweptToi(a, b, dt, toi)) { h.hit = true; h.toi = toi; }
        }, 8);

    // Phase 3: per-body rollback target = the earliest TOI among all sweeps that hit it.
    static std::vector<float> rollbackT;
    rollbackT.assign((size_t)count, 1.0f);
    for (int p = 0; p < pairCount; ++p)
    {
        if (!pairHits[p].hit) continue;
        const float toi = pairHits[p].toi;
        // u == 0: already overlapping at the sweep start (no rollback);
        // u >= 1: contact at the step end (no rollback).
        if (toi <= 0.0f || toi >= 1.0f) continue;
        const RigidPair& pr = rigidPairs[p];
        if (toi < rollbackT[pr.a]) rollbackT[pr.a] = toi;
        if (toi < rollbackT[pr.b]) rollbackT[pr.b] = toi;
    }

    static std::vector<Vector3> rollbackPos;
    static std::vector<Quaternion> rollbackRot;
    rollbackPos.resize((size_t)count);
    rollbackRot.resize((size_t)count);
    for (int i = 0; i < count; ++i)
    {
        RigidBodyInstance* b = bodies[i];
        if (!b) continue;
        if (rollbackT[i] < 1.0f)
        {
            rollbackPos[i] = Lerp(b->prevPosition, b->position, rollbackT[i]);
            rollbackRot[i] = QNlerp(b->prevOrientation, b->orientation, rollbackT[i]);
        }
        else
        {
            rollbackPos[i] = b->position;
            rollbackRot[i] = b->orientation;
        }
    }

    // Phase 4: narrowphase. One task per pair; each writes only its own slot, so the
    // pool never contends. Slot capacity 8 matches the old per-pair contact cap.
    struct PairContacts { RigidContact c[8]; int count = 0; };
    static std::vector<PairContacts> pairContacts;
    pairContacts.resize(pairCount);

    GetGlobalPool().parallel_for(pairCount, [&](int p)
        {
            const RigidPair& pr = rigidPairs[p];
            RigidBodyInstance* a = bodies[pr.a];
            RigidBodyInstance* b = bodies[pr.b];
            PairContacts& slot = pairContacts[p];
            slot.count = 0;

            if (pairHits[p].boxPair)
            {
                // Manifold at this pair's own TOI pose; no body is touched here.
                const PairHit& h = pairHits[p];
                const bool rollback = h.hit && h.toi > 0.0f && h.toi < 1.0f;
                const Vector3 posA = rollback ? Lerp(a->prevPosition, a->position, h.toi) : a->position;
                const Quaternion rotA = rollback ? QNlerp(a->prevOrientation, a->orientation, h.toi) : a->orientation;
                const Vector3 posB = rollback ? Lerp(b->prevPosition, b->position, h.toi) : b->position;
                const Quaternion rotB = rollback ? QNlerp(b->prevOrientation, b->orientation, h.toi) : b->orientation;
                slot.count = BoxBoxContactsCCD(a, b, posA, rotA, posB, rotB, slot.c, 8);
            }
            else
            {
                slot.count = RigidRigidContact(a, b, slot.c, 8);
            }
        }, 8);

    // Phase 5: serial merge in candidate order (same contact order as the old serial
    // loop) plus the only shared-state writes on the rigid-rigid path.
    for (int p = 0; p < pairCount; ++p)
    {
        const RigidPair& pr = rigidPairs[p];
        const PairContacts& slot = pairContacts[p];
        for (int k = 0; k < slot.count; ++k)
            contacts.push_back(slot.c[k]);

        if (slot.count > 0)
        {
            RigidBodyInstance* a = bodies[pr.a];
            RigidBodyInstance* b = bodies[pr.b];
            if (a->isSleeping) WakeIfDisturbed(a, b);
            if (b->isSleeping) WakeIfDisturbed(b, a);
        }
    }

    // Commit the rollback: solver, static pass, and the C# side all see each body at
    // its earliest contact time, deterministically.
    for (int i = 0; i < count; ++i)
    {
        RigidBodyInstance* b = bodies[i];
        if (!b || rollbackT[i] >= 1.0f) continue;
        b->position = rollbackPos[i];
        b->orientation = rollbackRot[i];
    }

    // Rigid-static: frozen bodies are skipped, so a static contact never wakes anything.
    const bool staticsChanged = g_StaticsChangedWake;
    g_StaticsChangedWake = false;

    // Resolved serially up front: the wake writes body state, and only this pass decides who runs.
    static std::vector<uint8_t> staticActive;
    staticActive.assign((size_t)count, 0);
    bool anyStaticWork = false;
    for (int i = 0; i < count; ++i)
    {
        RigidBodyInstance* b = bodies[i];
        if (!b || b->shape.Empty()) continue;
        if (b->isSleeping)
        {
            if (!staticsChanged) continue;
            RigidBody_Wake(b);
        }
        staticActive[i] = 1;
        anyStaticWork = true;
    }

    // Rebuilt once, not per body, and still gated: it sets g_StaticsChangedWake for the next substep.
    if (anyStaticWork) ProcessDirtyStatics();

    static std::vector<std::vector<RigidContact>> staticSlots;
    static std::vector<std::vector<PendingStaticEvent>> staticEvents;
    staticSlots.resize((size_t)count);
    staticEvents.resize((size_t)count);

    GetGlobalPool().parallel_for(count, [&](int i)
        {
            staticSlots[i].clear();
            staticEvents[i].clear();
            if (!staticActive[i]) return;
            RigidStaticContacts(bodies[i], dt, staticSlots[i], staticEvents[i]);
        }, 8);

    // Ordered merge: replays the serial body order for the contacts and for the event buffer.
    for (int i = 0; i < count; ++i)
    {
        if (!staticActive[i]) continue;
        for (const RigidContact& c : staticSlots[i])
            contacts.push_back(c);
        for (const PendingStaticEvent& e : staticEvents[i])
            RecordRigidStaticContact(bodies[i], e.point, e.normal);
    }
    rigidDetectMs = Lap();

    for (RigidContact& c : contacts)
        PrepareContact(c);
    rigidResolveMs += Lap();

    for (int iter = 0; iter < solverIterations; ++iter)
        for (RigidContact& c : contacts)
            SolveContactVelocity(c);
    rigidSolveMs += Lap();

    // Bounce, once, for the contacts that actually fired.
    for (RigidContact& c : contacts)
        ApplyRestitution(c);
    rigidSolveMs += Lap();

    // Cache the converged velocity impulses for the next substep's warm start.
    ++g_WarmStamp;
    WarmStoreContacts(contacts);
    WarmSweepCache();
    rigidSolveMs += Lap();

    // Penetration recovery: solved into the pseudo-velocities, never into real velocity.
    for (int iter = 0; iter < solverIterations; ++iter)
        for (RigidContact& c : contacts)
            SolveContactPosition(c, dt);
    rigidResolveMs += Lap();

    ApplyPseudoVelocities(bodies, count, dt);
    rigidResolveMs += Lap();

    // Summed over substeps; Engine_ClearFrameStats zeroes these once per fixed step.
    g_Stats.rigidFaceDetectionTimeMs += rigidDetectMs;
    g_Stats.rigidResolutionTimeMs += rigidResolveMs;
    g_Stats.rigidSolveTimeMs += rigidSolveMs;

    for (RigidContact& c : contacts)
    {
        c.impulseSum = c.normalImpulse;
        RecordRigidContactDebug(c.a->instanceId, c.b ? c.b->instanceId : -1,
            c.position, c.normal, c.impulseSum, c.penetration);
    }
}

// Barycentric weights of q on triangle abc; always normalized, so callers get usable weights.
static void BarycentricOnTriangle(const Vector3& q, const Vector3& a, const Vector3& b, const Vector3& c,
    float& wA, float& wB, float& wC)
{
    const Vector3 ab = Sub(b, a), ac = Sub(c, a), aq = Sub(q, a);
    const float d00 = Dot(ab, ab), d01 = Dot(ab, ac), d11 = Dot(ac, ac);
    const float d20 = Dot(aq, ab), d21 = Dot(aq, ac);
    const float denom = d00 * d11 - d01 * d01;
    if (denom < 1e-12f) { wA = 1.0f; wB = 0.0f; wC = 0.0f; return; }
    const float v = (d11 * d20 - d01 * d21) / denom; // weight of b
    const float w = (d00 * d21 - d01 * d20) / denom; // weight of c
    wA = ClampF(1.0f - v - w, 0.0f, 1.0f);
    wB = ClampF(v, 0.0f, 1.0f);
    wC = ClampF(w, 0.0f, 1.0f);
    const float sum = wA + wB + wC;
    if (sum > 1e-9f) { wA /= sum; wB /= sum; wC /= sum; }
}

// ---- Rigid ↔ soft coupling: speculative contacts + sequential impulses ----

struct SoftCouplingContact
{
    int     node[3] = { 0, 0, 0 };  // soft nodes involved
    float   bary[3] = { 0.0f, 0.0f, 0.0f };
    int     softCount = 1;          // 1 = node contact, 3 = face contact
    Vector3 point{};                // world contact point on the rigid surface
    Vector3 normal{};               // rigid surface → soft material
    float   pen = 0.0f;             // > 0 overlap, < 0 speculative gap
    float   wSoft = 0.0f;           // real soft inverse mass along n (Σ bary²·w)
    float   wSoftEff = 0.0f;        // mobility-clamped soft inverse mass (solver)
    float   wRigid = 0.0f;          // rigid inverse mass along n (0 = kinematic)
    float   softScale = 1.0f;       // wSoftEff / wSoft — impulse scale on the soft side
    Vector3 r{};                    // rigid arm from COM
    float   kN = 0.0f;              // normal effective mass
    Vector3 tangent1{}, tangent2{}; // fixed friction basis
    float   kT1 = 0.0f, kT2 = 0.0f;
    float   vn0 = 0.0f;             // pre-solve closing speed (contact events)
    float   targetVn = 0.0f;
    float   lambdaN = 0.0f;         // accumulated normal impulse
    float   lambdaT1 = 0.0f, lambdaT2 = 0.0f; // accumulated friction impulses
    float   friction = 0.0f;
    float   restitution = 0.0f;
};

// World position at the PREVIOUS pose of the material point now at worldPoint — the CCD margin.
static Vector3 RigidPointAtPrevPose(const RigidBodyInstance* b, const Vector3& worldPoint)
{
    const Quaternion invCur = { -b->orientation.x, -b->orientation.y, -b->orientation.z, b->orientation.w };
    const Vector3 local = QRotate(invCur, Sub(worldPoint, b->position));
    return Add(b->prevPosition, QRotate(b->prevOrientation, local));
}

int Collisions_RigidSoftContacts(RigidBodyInstance* rigid, NativeSoftBodyData* soft, float dt)
{
    if (!rigid || !soft || dt <= 0.0f) return 0;
    if (rigid->shape.Empty()) return 0;
    if (soft->nodeCount <= 0) return 0;
    if (rigid->isSleeping && soft->isSleeping) return 0;
    if (!(rigid->collisionLayerMask & (1 << soft->layer))) return 0;
    if (!(soft->collisionLayerMask & (1 << rigid->layer))) return 0;

    // Broadphase: rigid shape AABB vs soft node AABB.
    Vector3 rMin, rMax;
    Collisions_RigidShapeWorldAabb(rigid, rMin, rMax);
    const Vector3 sMin = soft->boundsMin;
    const Vector3 sMax = soft->boundsMax;
    if (rMax.x < sMin.x || rMin.x > sMax.x ||
        rMax.y < sMin.y || rMin.y > sMax.y ||
        rMax.z < sMin.z || rMin.z > sMax.z) return 0;

    const float restitution = 0.5f * (rigid->restitution + soft->restitution);
    const float friction = 0.5f * (rigid->friction + soft->slidingFriction);
    const float skin = soft->radius;

    // ---- Phase 0: snapshot soft node velocities BEFORE anything is written.
    // Verlet history (curr − prev)/dt only — (pred − curr)/dt folds constraint residue into a 1/dt phantom bounce.
    static thread_local std::vector<Vector3> velSnap;
    static thread_local std::vector<Vector3> dvel;
    static thread_local std::vector<SoftCouplingContact> contacts;
    velSnap.resize((size_t)soft->nodeCount);
    dvel.assign((size_t)soft->nodeCount, V3Zero());
    contacts.clear();
    if (soft->previousX && soft->previousY && soft->previousZ)
    {
        for (int n = 0; n < soft->nodeCount; ++n)
        {
            velSnap[n] = Vector3{
                (soft->currentX[n] - soft->previousX[n]) / dt,
                (soft->currentY[n] - soft->previousY[n]) / dt,
                (soft->currentZ[n] - soft->previousZ[n]) / dt, 0.0f };
        }
    }
    else
    {
        for (int n = 0; n < soft->nodeCount; ++n)
        {
            velSnap[n] = Vector3{
                (soft->predictedX[n] - soft->currentX[n]) / dt,
                (soft->predictedY[n] - soft->currentY[n]) / dt,
                (soft->predictedZ[n] - soft->currentZ[n]) / dt, 0.0f };
        }
    }

    // Soft point velocity at a contact (snapshot + impulses applied so far).
    auto softPointVel = [&](const SoftCouplingContact& c)
        {
            Vector3 v = Scale(Add(velSnap[c.node[0]], dvel[c.node[0]]), c.bary[0]);
            for (int k = 1; k < c.softCount; ++k)
                v = Add(v, Scale(Add(velSnap[c.node[k]], dvel[c.node[k]]), c.bary[k]));
            return v;
        };

    // Apply impulse P: soft side along +P (mobility-clamped), rigid side along −P at full Δv/Δω.
    auto applyImpulse = [&](const SoftCouplingContact& c, const Vector3& P)
        {
            for (int k = 0; k < c.softCount; ++k)
            {
                const float m = soft->masses[c.node[k]];
                if (m <= 0.0f) continue;
                const Vector3 dv = Scale(P, (c.bary[k] * c.softScale) / m);
                dvel[c.node[k]] = Add(dvel[c.node[k]], dv);
            }
            if (!rigid->isKinematic)
            {
                rigid->linearVelocity = Sub(rigid->linearVelocity, Scale(P, rigid->invMass));
                rigid->angularVelocity = Sub(rigid->angularVelocity,
                    ApplyWorldInvInertia(rigid, Cross(c.r, P)));
            }
        };

    // Upper bound on rigid surface travel this substep (translation + rotation).
    float rigidTravelBound = Length(Sub(rigid->position, rigid->prevPosition));
    {
        const RigidCollisionShape& s = rigid->shape;
        for (int bits = 0; bits < 8; ++bits)
        {
            const Vector3 local = V3(
                (bits & 1) ? s.localAabbMax.x : s.localAabbMin.x,
                (bits & 2) ? s.localAabbMax.y : s.localAabbMin.y,
                (bits & 4) ? s.localAabbMax.z : s.localAabbMin.z);
            const Vector3 nowP = Add(rigid->position, QRotate(rigid->orientation, local));
            const Vector3 prvP = Add(rigid->prevPosition, QRotate(rigid->prevOrientation, local));
            rigidTravelBound = MaxF(rigidTravelBound, Length(Sub(nowP, prvP)));
        }
    }

    // ---- Phase 1a: soft NODES vs the rigid shape (speculative) ----
    for (int n = 0; n < soft->nodeCount; ++n)
    {
        if (soft->isPinned && soft->isPinned[n]) continue;
        const float nodeMass = soft->masses[n];
        if (nodeMass <= 0.0f) continue;

        const Vector3 pred = { soft->predictedX[n], soft->predictedY[n], soft->predictedZ[n], 0.0f };
        const Vector3 curr = { soft->currentX[n], soft->currentY[n], soft->currentZ[n], 0.0f };
        const float nodeTravel = Length(Sub(pred, curr));

        Vector3 normal = V3Zero();   // rigid surface → node
        Vector3 q = V3Zero();        // closest point on the rigid surface
        float pen = 0.0f;            // signed: > 0 overlap, < 0 gap
        bool hit = false;

        switch (rigid->shape.type)
        {
        case RigidShapeType::Sphere:
        {
            const Vector3 d = Sub(pred, rigid->position);
            const float dist = Length(d);
            pen = rigid->shape.sphereRadius + skin - dist;
            if (dist > 1e-9f)
            {
                normal = Scale(d, 1.0f / dist);
                q = Add(rigid->position, Scale(normal, rigid->shape.sphereRadius));
            }
            else
            {
                normal = V3(0.0f, 1.0f, 0.0f);
                q = rigid->position;
            }
            hit = true; // the CCD-margin gate below decides relevance
            break;
        }
        case RigidShapeType::Box:
        {
            const OBB o = MakeShapeOBB(rigid, rigid->position, rigid->orientation);
            const Vector3 rel = Sub(pred, o.center);
            float axisDist[3];
            Vector3 closest = o.center;
            bool inside = true;
            for (int i = 0; i < 3; ++i)
            {
                axisDist[i] = Dot(rel, o.axis[i]);
                if (AbsF(axisDist[i]) >= VComp(o.half, i)) inside = false;
                const float clamped = ClampF(axisDist[i], -VComp(o.half, i), VComp(o.half, i));
                closest = Add(closest, Scale(o.axis[i], clamped));
            }
            if (inside)
            {
                // Node center inside the box: escape through the nearest face (clamp returns dist 0).
                int m = 0;
                float minGap = VComp(o.half, 0) - AbsF(axisDist[0]);
                for (int i = 1; i < 3; ++i)
                {
                    const float gap = VComp(o.half, i) - AbsF(axisDist[i]);
                    if (gap < minGap) { minGap = gap; m = i; }
                }
                normal = Scale(o.axis[m], axisDist[m] >= 0.0f ? 1.0f : -1.0f);
                q = Add(pred, Scale(normal, minGap));
                pen = skin + minGap;
            }
            else
            {
                const Vector3 delta = Sub(pred, closest);
                const float dist = Length(delta);
                normal = dist > 1e-9f ? Scale(delta, 1.0f / dist) : V3(0.0f, 1.0f, 0.0f);
                q = closest;
                pen = skin - dist;
            }
            hit = true; // gated below
            break;
        }
        case RigidShapeType::Capsule:
        {
            Vector3 capA, capB;
            CapsuleSegment(rigid, capA, capB);
            const Vector3 closest = ClosestPointOnSegment(pred, capA, capB);
            const Vector3 delta = Sub(pred, closest);
            const float dist = Length(delta);
            normal = dist > 1e-9f ? Scale(delta, 1.0f / dist) : V3(0.0f, 1.0f, 0.0f);
            q = Add(closest, Scale(normal, rigid->shape.capsuleRadius));
            pen = rigid->shape.capsuleRadius + skin - dist;
            hit = true; // gated below
            break;
        }
        case RigidShapeType::Trimesh:
        {
            // Closest triangle, normal toward the node's previous position; margin covers travel.
            if (TrimeshPointGeometry(rigid, pred, curr, skin, q, normal, pen,
                nodeTravel + rigidTravelBound))
            {
                hit = true;
            }
            break;
        }
        }

        if (!hit) continue;

        // CCD margin: combined travel of both sides; a larger gap cannot touch before the next pass.
        const float rigidTravel = Length(Sub(q, RigidPointAtPrevPose(rigid, q)));
        if (pen <= -(rigidTravel + nodeTravel)) continue;

        SoftCouplingContact c;
        c.node[0] = n;
        c.bary[0] = 1.0f;
        c.softCount = 1;
        c.point = q;
        c.normal = normal;
        c.pen = pen;
        c.wSoft = 1.0f / nodeMass;
        c.r = Sub(q, rigid->position);
        c.wRigid = rigid->isKinematic ? 0.0f
            : rigid->invMass + AngularEffectiveMassTerm(rigid, c.r, normal);
        c.friction = friction;
        c.restitution = restitution;
        contacts.push_back(c);
    }

    // ---- Phase 1b: rigid surface POINTS vs soft faces (a rigid on a large face hits no node).
    if (soft->faces && soft->faceCount > 0)
    {
        static thread_local std::vector<Vector3> pts, prevPts;
        pts.clear(); prevPts.clear();

        float surfaceRadius = skin;
        switch (rigid->shape.type)
        {
        case RigidShapeType::Sphere:
            surfaceRadius += rigid->shape.sphereRadius;
            pts.push_back(rigid->position);
            prevPts.push_back(rigid->prevPosition);
            break;
        case RigidShapeType::Capsule:
        {
            surfaceRadius += rigid->shape.capsuleRadius;
            Vector3 a, b; CapsuleSegment(rigid, a, b);
            const Vector3 axisPrev = QRotate(rigid->prevOrientation, V3(0.0f, 1.0f, 0.0f));
            const Vector3 aPrev = Add(rigid->prevPosition, Scale(axisPrev, rigid->shape.capsuleHeight));
            const Vector3 bPrev = Add(rigid->prevPosition, Scale(axisPrev, -rigid->shape.capsuleHeight));
            pts.push_back(a); prevPts.push_back(aPrev);
            pts.push_back(b); prevPts.push_back(bPrev);
            pts.push_back(Scale(Add(a, b), 0.5f));
            prevPts.push_back(Scale(Add(aPrev, bPrev), 0.5f));
            break;
        }
        case RigidShapeType::Box:
        {
            const OBB cur = MakeShapeOBB(rigid, rigid->position, rigid->orientation);
            const OBB prv = MakeShapeOBB(rigid, rigid->prevPosition, rigid->prevOrientation);
            for (int bits = 0; bits < 8; ++bits)
            {
                Vector3 c = cur.center, p = prv.center;
                for (int i = 0; i < 3; ++i)
                {
                    const float s = (bits & (1 << i)) ? 1.0f : -1.0f;
                    c = Add(c, Scale(cur.axis[i], s * VComp(cur.half, i)));
                    p = Add(p, Scale(prv.axis[i], s * VComp(prv.half, i)));
                }
                pts.push_back(c); prevPts.push_back(p);
            }
            break;
        }
        case RigidShapeType::Trimesh:
            for (const Vector3& lv : rigid->shape.vertices)
            {
                pts.push_back(Add(rigid->position, QRotate(rigid->orientation, lv)));
                prevPts.push_back(Add(rigid->prevPosition, QRotate(rigid->prevOrientation, lv)));
            }
            break;
        }

        if (!pts.empty())
        {
            for (int f = 0; f < soft->faceCount; ++f)
            {
                const int idx[3] = { soft->faces[f].nodeA, soft->faces[f].nodeB, soft->faces[f].nodeC };
                if (idx[0] < 0 || idx[0] >= soft->nodeCount ||
                    idx[1] < 0 || idx[1] >= soft->nodeCount ||
                    idx[2] < 0 || idx[2] >= soft->nodeCount) continue;

                Vector3 pred[3], curr[3];
                float w[3];
                for (int k = 0; k < 3; ++k)
                {
                    pred[k] = { soft->predictedX[idx[k]], soft->predictedY[idx[k]], soft->predictedZ[idx[k]], 0.0f };
                    curr[k] = { soft->currentX[idx[k]],   soft->currentY[idx[k]],   soft->currentZ[idx[k]],   0.0f };
                    const float m = soft->masses[idx[k]];
                    w[k] = m > 0.0f ? 1.0f / m : 0.0f;
                }
                if (w[0] <= 0.0f && w[1] <= 0.0f && w[2] <= 0.0f) continue; // fully pinned

                Vector3 n = Cross(Sub(pred[1], pred[0]), Sub(pred[2], pred[0]));
                const float nLen = Length(n);
                if (nLen < 1e-9f) continue; // degenerate face
                n = Scale(n, 1.0f / nLen);

                // Face AABB reject vs the rigid shape AABB.
                Vector3 fMin = V3(1e30f, 1e30f, 1e30f), fMax = V3(-1e30f, -1e30f, -1e30f);
                for (int k = 0; k < 3; ++k)
                {
                    fMin = Vector3{ MinF(fMin.x, pred[k].x), MinF(fMin.y, pred[k].y), MinF(fMin.z, pred[k].z), 0.0f };
                    fMax = Vector3{ MaxF(fMax.x, pred[k].x), MaxF(fMax.y, pred[k].y), MaxF(fMax.z, pred[k].z), 0.0f };
                }
                if (fMax.x < rMin.x || fMin.x > rMax.x ||
                    fMax.y < rMin.y || fMin.y > rMax.y ||
                    fMax.z < rMin.z || fMin.z > rMax.z) continue;

                for (size_t pi = 0; pi < pts.size(); ++pi)
                {
                    const Vector3& v = pts[pi];
                    const Vector3& vPrev = prevPts[pi];

                    // Plane quick reject, but a point that CROSSED the plane stays a candidate.
                    const float planeDist = Dot(Sub(v, pred[0]), n);
                    const float planeDistPrev = Dot(Sub(vPrev, pred[0]), n);
                    if (AbsF(planeDist) > surfaceRadius && planeDist * planeDistPrev > 0.0f)
                        continue;

                    const Vector3 q = ClosestPointOnTriangle(v, pred[0], pred[1], pred[2]);

                    // Orient N toward the rigid body so coupling normal Neg(N) points from rigid toward soft face.
                    Vector3 toRigid = Sub(rigid->position, q);
                    if (LengthSq(toRigid) < 1e-6f) toRigid = Sub(rigid->prevPosition, q);
                    Vector3 N = (Dot(toRigid, n) >= 0.0f) ? n : Neg(n);

                    const float sDist = Dot(Sub(v, q), N);
                    if (sDist >= surfaceRadius) continue; // not touching this face

                    const float pen = surfaceRadius - sDist; // > surfaceRadius when crossed through

                    float bary[3];
                    BarycentricOnTriangle(q, pred[0], pred[1], pred[2], bary[0], bary[1], bary[2]);

                    float wSoft = 0.0f;
                    Vector3 softDisp = V3Zero();
                    for (int k = 0; k < 3; ++k)
                    {
                        wSoft += bary[k] * bary[k] * w[k];
                        softDisp = Add(softDisp, Scale(Sub(pred[k], curr[k]), bary[k]));
                    }
                    if (wSoft <= 1e-12f) continue;

                    // CCD margin: both sides' travel at this contact.
                    const float margin = Length(Sub(v, vPrev)) + Length(softDisp);
                    if (pen <= -margin) continue;

                    SoftCouplingContact c;
                    c.node[0] = idx[0]; c.node[1] = idx[1]; c.node[2] = idx[2];
                    c.bary[0] = bary[0]; c.bary[1] = bary[1]; c.bary[2] = bary[2];
                    c.softCount = 3;
                    c.point = v;
                    c.normal = Neg(N);
                    c.pen = pen;
                    c.wSoft = wSoft;
                    c.r = Sub(v, rigid->position);
                    c.wRigid = rigid->isKinematic ? 0.0f
                        : rigid->invMass + AngularEffectiveMassTerm(rigid, c.r, c.normal);
                    c.friction = friction;
                    c.restitution = restitution;
                    contacts.push_back(c);
                }
            }
        }
    }

    if (contacts.empty()) return 0;

    // ---- Phase 2: prepare (mobility clamp, effective masses, targets) ----
    for (SoftCouplingContact& c : contacts)
    {
        // Extreme mass ratios: clamp the soft side's mobility to kCouplingMassRatio × the rigid's.
        c.wSoftEff = rigid->isKinematic ? c.wSoft
            : MinF(c.wSoft, kCouplingMassRatio * c.wRigid);
        c.softScale = c.wSoft > 1e-12f ? c.wSoftEff / c.wSoft : 1.0f;

        c.kN = 1.0f / (c.wSoftEff + c.wRigid + 1e-12f);

        // Fixed friction basis ⊥ the normal.
        const Vector3 ref = AbsF(c.normal.y) < 0.9f ? V3(0.0f, 1.0f, 0.0f) : V3(1.0f, 0.0f, 0.0f);
        Vector3 t1 = Sub(ref, Scale(c.normal, Dot(ref, c.normal)));
        const float t1Len = Length(t1);
        c.tangent1 = t1Len > 1e-9f ? Scale(t1, 1.0f / t1Len) : V3(1.0f, 0.0f, 0.0f);
        c.tangent2 = Cross(c.normal, c.tangent1);

        const float wRigidT1 = rigid->isKinematic ? 0.0f
            : rigid->invMass + AngularEffectiveMassTerm(rigid, c.r, c.tangent1);
        const float wRigidT2 = rigid->isKinematic ? 0.0f
            : rigid->invMass + AngularEffectiveMassTerm(rigid, c.r, c.tangent2);
        c.kT1 = 1.0f / (c.wSoftEff + wRigidT1 + 1e-12f);
        c.kT2 = 1.0f / (c.wSoftEff + wRigidT2 + 1e-12f);

        // Pre-solve closing speed (soft approaching rigid is negative).
        const float vn0 = Dot(Sub(softPointVel(c), PointVelocity(rigid, c.r)), c.normal);
        c.vn0 = vn0;

        // Velocity-constraint target.
        float target;
        if (c.pen > kContactSlop)
            target = MinF(kCouplingSoftERP * (c.pen - kContactSlop) / dt, kCouplingSoftBiasCap);
        else
            target = -MaxF(-c.pen, 0.0f) / dt; // speculative approach budget
        // No restitution here: prolonged contact + Verlet residue would re-fire −e·vn0 and pop; the cloth's elasticity provides the rebound.
        c.targetVn = target;

        // Contact event (pre-solve velocities).
        {
            const Vector3 relVel = Sub(softPointVel(c), PointVelocity(rigid, c.r));
            const Vector3 tangentVel = Sub(relVel, Scale(c.normal, Dot(relVel, c.normal)));
            RecordContactEvent(soft->id, rigid->instanceId, c.point, c.normal,
                MaxF(-vn0, 0.0f), tangentVel);
        }
    }

    // ---- Phase 3: sequential impulses (accumulated, clamped) ----
    for (int iter = 0; iter < kCouplingIterations; ++iter)
    {
        for (SoftCouplingContact& c : contacts)
        {
            // Normal constraint: v_n ≥ targetVn.
            {
                const float vn = Dot(Sub(softPointVel(c), PointVelocity(rigid, c.r)), c.normal);
                float dLambda = (c.targetVn - vn) * c.kN;
                const float newLambda = MaxF(c.lambdaN + dLambda, 0.0f);
                dLambda = newLambda - c.lambdaN;
                if (dLambda > 1e-12f)
                {
                    applyImpulse(c, Scale(c.normal, dLambda));
                    c.lambdaN = newLambda;
                }
            }

            // Friction: Coulomb cone around the accumulated normal impulse.
            if (c.lambdaN <= 1e-12f || c.friction <= 0.0f) continue;
            const float maxT = c.friction * c.lambdaN;

            const float vt1 = Dot(Sub(softPointVel(c), PointVelocity(rigid, c.r)), c.tangent1);
            float dT1 = -vt1 * c.kT1;
            const float newT1 = ClampF(c.lambdaT1 + dT1, -maxT, maxT);
            dT1 = newT1 - c.lambdaT1;
            if (AbsF(dT1) > 1e-12f)
            {
                applyImpulse(c, Scale(c.tangent1, dT1));
                c.lambdaT1 = newT1;
            }

            const float vt2 = Dot(Sub(softPointVel(c), PointVelocity(rigid, c.r)), c.tangent2);
            float dT2 = -vt2 * c.kT2;
            const float newT2 = ClampF(c.lambdaT2 + dT2, -maxT, maxT);
            dT2 = newT2 - c.lambdaT2;
            if (AbsF(dT2) > 1e-12f)
            {
                applyImpulse(c, Scale(c.tangent2, dT2));
                c.lambdaT2 = newT2;
            }
        }
    }

    // ---- Phase 4: writeback ----
    // Soft: fold the impulse velocity deltas into the predicted positions — genuine velocity.
    for (int n = 0; n < soft->nodeCount; ++n)
    {
        const Vector3& dv = dvel[n];
        if (dv.x == 0.0f && dv.y == 0.0f && dv.z == 0.0f) continue;
        soft->predictedX[n] += dv.x * dt;
        soft->predictedY[n] += dv.y * dt;
        soft->predictedZ[n] += dv.z * dt;
    }

    // Rigid: split-impulse recovery; a manifold of N contacts gives one pushout's worth, not N.
    if (!rigid->isKinematic && !rigid->isSleeping)
    {
        Vector3 pushVel = V3Zero();
        for (const SoftCouplingContact& c : contacts)
        {
            if (c.pen <= kContactSlop) continue;
            const Vector3 escape = Neg(c.normal);
            const float bias = PushoutBias(c.pen, dt);
            const float have = MaxF(Dot(pushVel, escape), 0.0f);
            if (bias > have)
                pushVel = Add(pushVel, Scale(escape, bias - have));
        }
        if (pushVel.x != 0.0f || pushVel.y != 0.0f || pushVel.z != 0.0f)
            rigid->position = Add(rigid->position, Scale(pushVel, dt));
    }

    // A resting soft body is no reason to keep its rigid neighbour awake.
    if (rigid->isSleeping && !soft->isSleeping) RigidBody_Wake(rigid);
    if (soft->isSleeping) soft->wakeRequested = 1;

    return (int)contacts.size();
}

EXPORT void RigidBody_SetCollisionShape2(RigidBodyInstance* body, int type,
    const Vector3* vertices, int vertCount,
    const int* faceIndices, int faceCount, float skinRadius)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return; }

    RigidCollisionShape& s = body->shape;
    s.type = (RigidShapeType)type;

    if (vertices && vertCount > 0)
        s.vertices.assign(vertices, vertices + vertCount);
    else
        s.vertices.clear();

    if (faceIndices && faceCount > 0)
        s.faces.assign(faceIndices, faceIndices + (size_t)faceCount * 3);
    else
        s.faces.clear();

    s.skinRadius = skinRadius;

    // Vertex AABB, computed once and reused below by both the Trimesh->Box reclassification
    // check and the Box recentering step (previously each recomputed it independently).
    Vector3 mn = V3(1e30f, 1e30f, 1e30f), mx = V3(-1e30f, -1e30f, -1e30f);
    const bool haveAabb = (s.type == RigidShapeType::Trimesh || s.type == RigidShapeType::Box) && !s.vertices.empty();
    if (haveAabb)
    {
        for (const Vector3& p : s.vertices)
        {
            mn = Vector3{ MinF(mn.x, p.x), MinF(mn.y, p.y), MinF(mn.z, p.z), 0.0f };
            mx = Vector3{ MaxF(mx.x, p.x), MaxF(mx.y, p.y), MaxF(mx.z, p.z), 0.0f };
        }
    }

    if (s.type == RigidShapeType::Trimesh && haveAabb)
    {
        const Vector3 half = Scale(Sub(mx, mn), 0.5f);
        const Vector3 center = Scale(Add(mx, mn), 0.5f);

        if (half.x > 1e-4f && half.y > 1e-4f && half.z > 1e-4f)
        {
            auto cornerPresent = [&](float sx, float sy, float sz)
                {
                    const Vector3 corner = Add(center,
                        V3(sx * half.x, sy * half.y, sz * half.z));
                    for (const Vector3& p : s.vertices)
                        if (LengthSq(Sub(p, corner)) < 1e-6f) // 1 mm
                            return true;
                    return false;
                };

            bool allCorners = true;
            for (int bits = 0; bits < 8 && allCorners; ++bits)
                allCorners = cornerPresent((bits & 1) ? 1.0f : -1.0f,
                    (bits & 2) ? 1.0f : -1.0f, (bits & 4) ? 1.0f : -1.0f);

            if (allCorners)
                s.type = RigidShapeType::Box;
        }
    }

    if (s.type == RigidShapeType::Box && haveAabb)
    {
        s.boxHalfExtents = Scale(Sub(mx, mn), 0.5f);
        // Recentre on the vertex AABB center so MakeShapeOBB's COM-origin assumption holds.
        const Vector3 center = Scale(Add(mx, mn), 0.5f);
        for (Vector3& p : s.vertices)
            p = Sub(p, center);
    }

    s.RecomputeLocalAabb();
}

EXPORT int RigidBody_GetShapeVertexCount(const RigidBodyInstance* body)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return 0; }
    return (int)body->shape.vertices.size();
}

EXPORT int RigidBody_GetShapeVertices(const RigidBodyInstance* body, Vector3* outVerts, int maxCount)
{
    if (!body) { Error_SetError(ErrorCode::NullHandle); return 0; }
    if (!outVerts || maxCount <= 0) { Error_SetError(ErrorCode::InvalidArgument); return 0; }
    const int count = (int)body->shape.vertices.size();
    const int n = count < maxCount ? count : maxCount;
    for (int i = 0; i < n; ++i)
        outVerts[i] = Add(body->position, QRotate(body->orientation, body->shape.vertices[i]));
    return n;
}