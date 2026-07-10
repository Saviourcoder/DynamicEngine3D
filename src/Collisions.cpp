/*
 DYNAMICENGINE3D                                          
 AI-Assisted Soft-Body Physics for Unity3D
 By: Elitmers                                             
*/

#include "EngineTypes.h"
#include "MathUtils.h"
#include "Constants.h"

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <limits>

#if defined(_WIN32)
#define EXPORT extern "C" __declspec(dllexport)
#else
#define EXPORT extern "C" __attribute__((visibility("default")))
#endif

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
    extern EngineStats g_Stats;
}

extern "C" EngineStats g_Stats; // Reference the global from SoftBodyInstance.cpp

struct HashEntry {
    int hash;
    int value;
};

// Node-node contact with LOCAL indices into each body's own buffers.
// (#3) Lets the solver index bA/bB directly --- no copy-into-combined-array staging.

struct SoAVec3 {
    float* x; float* y; float* z; int i;
    operator Vector3() const { return { x[i], y[i], z[i], 0.0f }; }
    SoAVec3& operator=(const Vector3& v) { x[i]=v.x; y[i]=v.y; z[i]=v.z; return *this; }
};
struct ConstSoAVec3 {
    const float* x; const float* y; const float* z; int i;
    operator Vector3() const { return { x[i], y[i], z[i], 0.0f }; }
};
struct SoAArray {
    float* x; float* y; float* z;
    SoAVec3 operator[](int i) const { return {x,y,z,i}; }
};
struct ConstSoAArray {
    const float* x; const float* y; const float* z;
    ConstSoAVec3 operator[](int i) const { return {x,y,z,i}; }
};

#define VEC3(ARR, IDX) Vector3({ARR##X[IDX], ARR##Y[IDX], ARR##Z[IDX], 0.0f})
struct NodeContact {
    int a;   // local node index in body A
    int b;   // local node index in body B
};

// Flat {cell-key -> node} pair used for the face->node broadphase.
// (#4) Replaces unordered_multimap with a contiguous, sortable, cache-friendly array.
struct KeyVal {
    int key;
    int value;
    bool operator<(const KeyVal& o) const { return key < o.key; }
};

// Forward declarations of internal helpers

static void ApplyXPBDFaceNode(int idxP, int idxA, int idxB, int idxC,
    const Vector3& normal, float penetration,
    float u, float v, float w,
    float* nodePredX, float* nodePredY, float* nodePredZ, float* facePredX, float* facePredY, float* facePredZ,
    const float* nodeMasses, const float* faceMasses,
    const uint8_t* nodePinned, const uint8_t* facePinned);

static bool CheckSweptFaceNode(const Vector3& p0, const Vector3& p1, float radius,
    const Vector3& a0, const Vector3& a1,
    const Vector3& b0, const Vector3& b1,
    const Vector3& c0, const Vector3& c1,
    float& tOut, Vector3& nOut);

static bool GetBarycentric(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c,
    float& u, float& v, float& w);

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

// New internal hot-path helpers (definitions further below)
static int  BuildNodeContactsSplit(const float* pAX, const float* pAY, const float* pAZ, int countA, const float* pBX, const float* pBY, const float* pBZ, int countB,
    float radiusA, float radiusB, float epsilon,
    NodeContact* out, int cap);
static void ResolveNodeContactsSplit(
    float* predAx, float* predAy, float* predAz, const float* currAx, const float* currAy, const float* currAz, const float* massA, const uint8_t* pinA, float* predBx, float* predBy, float* predBz, const float* currBx, const float* currBy, const float* currBz, const float* massB, const uint8_t* pinB,
    const NodeContact* contacts, int contactCount,
    float dt, float invDt, float combinedR,
    float restitution, float staticFriction, float slidingFriction);
static void BuildNodeSweptHash(const float* prevX, const float* prevY, const float* prevZ, const float* predX, const float* predY, const float* predZ, int count,
    float invCell, std::vector<KeyVal>& flatHash);
static void ResolveFaceToNodeImpl(
    float* nodePredX, float* nodePredY, float* nodePredZ, const float* nodePrevX, const float* nodePrevY, const float* nodePrevZ,
    const float* nodeMasses, const uint8_t* nodeIsPinned, int nodeCount,
    float* facePredX, float* facePredY, float* facePredZ, const float* facePrevX, const float* facePrevY, const float* facePrevZ,
    const float* faceMasses, const uint8_t* faceIsPinned,
    const FaceData* faces, int faceCount,
    float radiusNode, float thickness, float epsilon, float dt,
    float invCell, const std::vector<KeyVal>& flatHash);

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//  Node-node contact resolution
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//
// (#7) Per-contact math is computed exactly once. Intermediates that do not
//      change after the positional push-out are cached in locals; the only
//      values recomputed after the correction are the post-correction
//      velocities (they must reflect the moved positions).

// Shared kernel: resolves ONE contact given direct references to both nodes.
static inline void ResolveOneNodeContact(
    Vector3& predI, const Vector3& currI, float wi,
    Vector3& predJ, const Vector3& currJ, float wj,
    float dt, float invDt, float combinedR,
    float restitution, float staticFriction, float slidingFriction)
{
    float wSum = wi + wj;
    if (wSum <= 0.0f) return;
    const float invWSum = 1.0f / wSum;

    // Cache start state and step velocities once.
    const Vector3 velI = Sub(predI, currI);
    const Vector3 velJ = Sub(predJ, currJ);
    const Vector3 relVel = Sub(velI, velJ);
    const Vector3 relStart = Sub(currI, currJ);

    const float a = Dot(relVel, relVel);
    const float c = Dot(relStart, relStart) - combinedR * combinedR;

    float toi = 1.0f;                       // default: end-of-step positions
    const bool alreadyOverlapping = (c <= 0.0f);
    if (!alreadyOverlapping)
    {
        if (a < 1e-12f) return;             // no relative motion, no overlap
        const float b = 2.0f * Dot(relStart, relVel);
        const float disc = b * b - 4.0f * a * c;
        if (disc < 0.0f) return;            // paths don't intersect
        const float t = (-b - std::sqrt(disc)) / (2.0f * a);
        if (t < 0.0f || t > 1.0f) return;   // collision outside this step
        toi = t;
    }

    // Contact point / normal at TOI (reuse cached velocities).
    const Vector3 pi = Add(currI, Scale(velI, toi));
    const Vector3 pj = Add(currJ, Scale(velJ, toi));
    const Vector3 d = Sub(pi, pj);
    const float dist = Length(d);
    if (dist < PhysicsConstants::MIN_COLLISION_DISTANCE) return;
    const Vector3 n = Scale(d, 1.0f / dist);

    // Push out using end-of-step penetration (computed once).
    float penEnd = combinedR - Length(Sub(predI, predJ));
    if (penEnd < 0.0f) penEnd = 0.0f;
    const Vector3 corr = Scale(n, penEnd * invWSum);
    predI = Add(predI, Scale(corr, wi));
    predJ = Sub(predJ, Scale(corr, wj));

    // Velocity response from the corrected positions.
    const Vector3 vi = Scale(Sub(predI, currI), invDt);
    const Vector3 vj = Scale(Sub(predJ, currJ), invDt);
    const Vector3 relV = Sub(vi, vj);
    const float vn = Dot(relV, n);
    if (vn >= 0.0f) return;                 // separating --- nothing to do

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
}

// (#3) Split-buffer resolver: operates directly on body A / body B arrays.
// No combined-array staging, no per-iteration memcpy.
static void ResolveNodeContactsSplit(
    float* predAx, float* predAy, float* predAz, const float* currAx, const float* currAy, const float* currAz, const float* massA, const uint8_t* pinA, float* predBx, float* predBy, float* predBz, const float* currBx, const float* currBy, const float* currBz, const float* massB, const uint8_t* pinB,
    const NodeContact* contacts, int contactCount,
    float dt, float invDt, float combinedR,
    float restitution, float staticFriction, float slidingFriction)
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

        ResolveOneNodeContact(
            pI, { currAx[i], currAy[i], currAz[i], 0.0f }, wi,
            pJ, { currBx[j], currBy[j], currBz[j], 0.0f }, wj,
            dt, invDt, combinedR, restitution, staticFriction, slidingFriction);

        predAx[i] = pI.x; predAy[i] = pI.y; predAz[i] = pI.z;
        predBx[j] = pJ.x; predBy[j] = pJ.y; predBz[j] = pJ.z;
    }
}

// EXPORT kept for ABI / external callers. Combined-array form (indices into one
// shared buffer). Now delegates per-contact math to the shared, cached kernel.
EXPORT void ResolveNodePairs(float* predX, float* predY, float* predZ, float* currX, float* currY, float* currZ,
    const float* masses, const uint8_t* isPinned,
    const CollisionPair* pairs, int pairCount,
    float dt, float epsilon,
    float radiusA, float radiusB,
    float restitution, float staticFriction, float slidingFriction)
{
    if (pairCount <= 0 || dt <= 0.0f) return;

    const float combinedR = radiusA + radiusB + epsilon;
    const float invDt = 1.0f / dt;

    for (int p = 0; p < pairCount; ++p)
    {
        const int i = pairs[p].nodeA;
        const int j = pairs[p].nodeB;

        const float wi = (isPinned && isPinned[i]) ? 0.0f : (masses[i] > 0.0f ? 1.0f / masses[i] : 0.0f);
        const float wj = (isPinned && isPinned[j]) ? 0.0f : (masses[j] > 0.0f ? 1.0f / masses[j] : 0.0f);

        Vector3 pI = { predX[i], predY[i], predZ[i], 0.0f };
        Vector3 pJ = { predX[j], predY[j], predZ[j], 0.0f };

        ResolveOneNodeContact(
            pI, { currX[i], currY[i], currZ[i], 0.0f }, wi,
            pJ, { currX[j], currY[j], currZ[j], 0.0f }, wj,
            dt, invDt, combinedR, restitution, staticFriction, slidingFriction);

        predX[i] = pI.x; predY[i] = pI.y; predZ[i] = pI.z;
        predX[j] = pJ.x; predY[j] = pJ.y; predZ[j] = pJ.z;
    }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//  Broadphase: node-node pair generation
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//
// (#8) A hard MAX_CONTACTS_PER_PAIR cap bounds the worst case so a dense pile
//      can never blow up the contact buffer or the solver cost.

// (#3) Split form: emits LOCAL {i, j} indices straight into a NodeContact[].
static int BuildNodeContactsSplit(const float* pAX, const float* pAY, const float* pAZ, int countA, const float* pBX, const float* pBY, const float* pBZ, int countB,
    float radiusA, float radiusB, float epsilon,
    NodeContact* out, int cap)
{
    if (countA <= 0 || countB <= 0 || cap <= 0 || !out) return 0;

    const float threshold = radiusA + radiusB + epsilon;
    const float thresholdSq = threshold * threshold;
    const float cell = MaxF(PhysicsConstants::SPATIAL_CELL_SIZE, threshold);
    const float invCell = 1.0f / cell;
    const int range = 1;

    const int tableSize = countB * 2;
    static thread_local std::vector<HashEntry> hashTable;   // grow-only scratch
    hashTable.assign(tableSize, { -1, -1 });

    for (int i = 0; i < countB; ++i)
    {
        const int gx = FloorToInt(pBX[i] * invCell);
        const int gy = FloorToInt(pBY[i] * invCell);
        const int gz = FloorToInt(pBZ[i] * invCell);
        const int hash = SpatialHash(gx, gy, gz);

        int slot = (hash & 0x7FFFFFFF) % tableSize;
        while (hashTable[slot].value != -1)
            slot = (slot + 1) % tableSize;
        hashTable[slot] = { hash, i };
    }

    int written = 0;
    for (int i = 0; i < countA; ++i)
    {
        const Vector3& pa = VEC3(pA, i);
        const int gx0 = FloorToInt(pa.x * invCell);
        const int gy0 = FloorToInt(pa.y * invCell);
        const int gz0 = FloorToInt(pa.z * invCell);

        for (int dx = -range; dx <= range; ++dx)
            for (int dy = -range; dy <= range; ++dy)
                for (int dz = -range; dz <= range; ++dz)
                {
                    const int hash = SpatialHash(gx0 + dx, gy0 + dy, gz0 + dz);
                    int slot = (hash & 0x7FFFFFFF) % tableSize;
                    while (hashTable[slot].value != -1)
                    {
                        if (hashTable[slot].hash == hash)
                        {
                            const int j = hashTable[slot].value;
                            if (LengthSq(Sub(pa, VEC3(pB, j))) < thresholdSq)
                            {
                                out[written].a = i;
                                out[written].b = j;
                                if (++written >= cap) return written;   // (#8) hard cap
                            }
                        }
                        slot = (slot + 1) % tableSize;
                    }
                }
    }
    return written;
}

// EXPORT kept for ABI. Original combined-index contract (nodeB = countA + j).
EXPORT int BuildNodeNodePairs(const float* pAX, const float* pAY, const float* pAZ, int countA, const float* pBX, const float* pBY, const float* pBZ, int countB,
    float radiusA, float radiusB, float epsilon,
    CollisionPair* outPairs, int outCapacity)
{
    if (countA <= 0 || countB <= 0) return 0;

    const float threshold = radiusA + radiusB + epsilon;
    const float thresholdSq = threshold * threshold;
    const float cell = MaxF(PhysicsConstants::SPATIAL_CELL_SIZE, threshold);
    const float invCell = 1.0f / cell;
    const int range = 1;

    const int tableSize = countB * 2;
    static thread_local std::vector<HashEntry> hashTable;
    hashTable.assign(tableSize, { -1, -1 });

    for (int i = 0; i < countB; ++i)
    {
        const int gx = FloorToInt(pBX[i] * invCell);
        const int gy = FloorToInt(pBY[i] * invCell);
        const int gz = FloorToInt(pBZ[i] * invCell);
        const int hash = SpatialHash(gx, gy, gz);

        int slot = (hash & 0x7FFFFFFF) % tableSize;
        while (hashTable[slot].value != -1)
            slot = (slot + 1) % tableSize;
        hashTable[slot] = { hash, i };
    }

    int written = 0;
    for (int i = 0; i < countA; ++i)
    {
        const Vector3& pa = VEC3(pA, i);
        const int gx0 = FloorToInt(pa.x * invCell);
        const int gy0 = FloorToInt(pa.y * invCell);
        const int gz0 = FloorToInt(pa.z * invCell);

        for (int dx = -range; dx <= range; ++dx)
            for (int dy = -range; dy <= range; ++dy)
                for (int dz = -range; dz <= range; ++dz)
                {
                    const int hash = SpatialHash(gx0 + dx, gy0 + dy, gz0 + dz);
                    int slot = (hash & 0x7FFFFFFF) % tableSize;
                    while (hashTable[slot].value != -1)
                    {
                        if (hashTable[slot].hash == hash)
                        {
                            const int j = hashTable[slot].value;
                            if (LengthSq(Sub(pa, VEC3(pB, j))) < thresholdSq)
                            {
                                if (written < outCapacity && outPairs)
                                {
                                    outPairs[written].nodeA = i;
                                    outPairs[written].nodeB = countA + j;
                                    ++written;
                                }
                            }
                        }
                        slot = (slot + 1) % tableSize;
                    }
                }
    }
    return written;
}
EXPORT void ResolveStaticCollisions(float* predX, float* predY, float* predZ,
    const float* currX, const float* currY, const float* currZ,
    const float* masses,
    const uint8_t* isPinned,
    int nodeCount,
    const StaticColliderData* colliders,
    int colliderCount,
    float radius, float epsilon,
    float restitution, float staticFriction, float slidingFriction,
    float dt)
{
    if (nodeCount <= 0 || colliderCount <= 0 || dt <= 0.0f) return;

    float invDt = 1.0f / dt;
    float r = radius + PhysicsConstants::SKIN_WIDTH;

    for (int n = 0; n < nodeCount; ++n)
    {
        if (isPinned && isPinned[n]) continue;
        if (masses[n] <= 0.0f) continue;
        for (int c = 0; c < colliderCount; ++c)
        {
            const StaticColliderData& col = colliders[c];
            Vector3 closestPoint;
            Vector3 normal = V3(0.0f, 1.0f, 0.0f);
            float   dist;

            if (col.type == COL_SPHERE)
            {
                Vector3 d = Sub(VEC3(pred, n), col.center);
                float L = Length(d);
                if (L < 1e-7f) continue;
                normal = Scale(d, 1.0f / L);
                dist = L - col.radius;
                closestPoint = Add(col.center, Scale(normal, col.radius));
            }
            else if (col.type == COL_BOX)
            {
                Quaternion invRot = QInverse(col.rotation);
                Vector3 lp = QRotate(invRot, Sub(VEC3(pred, n), col.center));
                Vector3 half = Scale(col.size, 0.5f);

                Vector3 d = Sub(lp, { ClampF(lp.x, -half.x, half.x),
                                       ClampF(lp.y, -half.y, half.y),
                                       ClampF(lp.z, -half.z, half.z), 0.0f });
                float L = Length(d);

                if (L < 1e-7f)
                {
                    // Node is inside --- push out via minimum-penetration axis
                    float dx = half.x - AbsF(lp.x);
                    float dy = half.y - AbsF(lp.y);
                    float dz = half.z - AbsF(lp.z);
                    Vector3 ln = V3Zero();
                    if (dx <= dy && dx <= dz)      ln.x = lp.x > 0.0f ? 1.0f : -1.0f;
                    else if (dy <= dx && dy <= dz) ln.y = lp.y > 0.0f ? 1.0f : -1.0f;
                    else                           ln.z = lp.z > 0.0f ? 1.0f : -1.0f;
                    normal = QRotate(col.rotation, ln);
                    float minPen = MinF(dx, MinF(dy, dz));
                    dist = -minPen; // negative = inside
                    closestPoint = Sub(VEC3(pred, n), Scale(normal, minPen));
                }
                else
                {
                    Vector3 ln = Scale(d, 1.0f / L);
                    normal = QRotate(col.rotation, ln);
                    closestPoint = Add(col.center, QRotate(col.rotation,
                        { ClampF(lp.x,-half.x,half.x), ClampF(lp.y,-half.y,half.y), ClampF(lp.z,-half.z,half.z), 0.0f }));
                    dist = L;
                }
            }
            else if (col.type == COL_TRIANGLE)
            {
                float u, v, w;
                Vector3 cp = ClosestPointTriangle(VEC3(pred, n), col.v0, col.v1, col.v2, u, v, w);
                Vector3 d = Sub(VEC3(pred, n), cp);
                float L = Length(d);
                if (L < 1e-7f) continue;
                normal = Scale(d, 1.0f / L);
                closestPoint = cp;
                dist = L;
            }
            else if (col.type == COL_CAPSULE)
            {
                float halfH = MaxF(0.0f, (col.height * 0.5f) - col.radius);
                Vector3 axis = Normalize(col.axis);
                Vector3 p1 = Add(col.center, Scale(axis, halfH));
                Vector3 p2 = Sub(col.center, Scale(axis, halfH));
                Vector3 seg = Sub(p2, p1);
                float segLen = Length(seg);
                if (segLen < 1e-7f) continue;
                Vector3 segDir = Scale(seg, 1.0f / segLen);
                float t = ClampF(Dot(Sub(VEC3(pred, n), p1), segDir), 0.0f, segLen);
                Vector3 cp = Add(p1, Scale(segDir, t));
                Vector3 d = Sub(VEC3(pred, n), cp);
                float L = Length(d);
                if (L < 1e-7f) continue;
                normal = Scale(d, 1.0f / L);
                closestPoint = Add(cp, Scale(normal, col.radius));
                dist = L - col.radius;
            }
            else continue;

            if (dist >= r) continue;

            float penetration = r - dist;
            predX[n] = Add(VEC3(pred, n), Scale(normal, penetration + epsilon)).x; predY[n] = Add(VEC3(pred, n), Scale(normal, penetration + epsilon)).y; predZ[n] = Add(VEC3(pred, n), Scale(normal, penetration + epsilon)).z;

            Vector3 v = Scale(Sub(VEC3(pred, n), VEC3(curr, n)), invDt);
            float vn = Dot(v, normal);
            if (vn < 0.0f)
            {
                float restA = (restitution + col.restitution) * 0.5f;
                float sf = (staticFriction + col.staticFriction) * 0.5f;
                float kf = (slidingFriction + col.slidingFriction) * 0.5f;

                constexpr float BOUNCE_THRESHOLD = 0.2f;
                float appliedRest = (-vn > BOUNCE_THRESHOLD) ? restA : 0.0f;

                float jn = -(1.0f + appliedRest) * vn;
                float positionalVel = (penetration + epsilon) * invDt;
                float effectiveJn = jn + positionalVel;

                Vector3 vNew = Add(v, Scale(normal, jn));

                Vector3 vt = Sub(vNew, Scale(normal, Dot(vNew, normal)));
                float   vtSpeed = Length(vt);
                if (vtSpeed > PhysicsConstants::MIN_TANGENT_SPEED)
                {
                    Vector3 tdir = Scale(vt, 1.0f / vtSpeed);
                    float jt = vtSpeed;
                    float fMag = (jt <= sf * effectiveJn) ? jt : MinF(kf * effectiveJn, jt);
                    vNew = Sub(vNew, Scale(tdir, fMag));
                }
                predX[n] = Add(VEC3(curr, n), Scale(vNew, dt)).x; predY[n] = Add(VEC3(curr, n), Scale(vNew, dt)).y; predZ[n] = Add(VEC3(curr, n), Scale(vNew, dt)).z;
            }
        }
    }
}

static void ResolveStaticCollisionsCCD_Internal(
    float* predX, float* predY, float* predZ, float* currX, float* currY, float* currZ,
    const float* masses, const uint8_t* isPinned, int nodeCount,
    const StaticColliderData* colliders, int colliderCount,
    const HashEntry* hashTable, int tableSize,
    float radius, float epsilon, float gridCellSize,
    float defaultRestitution, float defaultStaticFriction,
    float defaultSlidingFriction, float dt)
{
    if (nodeCount <= 0 || colliderCount <= 0 || dt <= 0.0f || tableSize <= 0) return;

    // grow-only: candidate count per node is tiny, so std::find is faster
    // than hashing and avoids bucket-list heap allocations entirely.
    static thread_local std::vector<int> candidates;

    for (int i = 0; i < nodeCount; ++i)
    {
        Vector3 cur = VEC3(curr, i);
        Vector3 pred = VEC3(pred, i);
        float mass = masses[i];
        if (isPinned && isPinned[i]) continue;
        if (mass <= 0.0f) continue;

        Vector3 vel = Sub(pred, cur);

        Vector3 sweptMin = { MinF(cur.x, pred.x) - radius,
                             MinF(cur.y, pred.y) - radius,
                             MinF(cur.z, pred.z) - radius, 0.0f };
        Vector3 sweptMax = { MaxF(cur.x, pred.x) + radius,
                             MaxF(cur.y, pred.y) + radius,
                             MaxF(cur.z, pred.z) + radius, 0.0f };

        int x0 = FloorToInt(sweptMin.x / gridCellSize);
        int y0 = FloorToInt(sweptMin.y / gridCellSize);
        int z0 = FloorToInt(sweptMin.z / gridCellSize);
        int x1 = FloorToInt(sweptMax.x / gridCellSize);
        int y1 = FloorToInt(sweptMax.y / gridCellSize);
        int z1 = FloorToInt(sweptMax.z / gridCellSize);

        candidates.clear();
        for (int cx = x0; cx <= x1; ++cx)
        {
            for (int cy = y0; cy <= y1; ++cy)
            {
                for (int cz = z0; cz <= z1; ++cz)
                {
                    int hash = SpatialHash(cx, cy, cz);
                    int slot = (hash & 0x7FFFFFFF) % tableSize;
                    while (hashTable[slot].value != -1)
                    {
                        if (hashTable[slot].hash == hash)
                        {
                            int val = hashTable[slot].value;
                            // Sequential scan for duplicate prevention;
                            // candidate count per node is tiny (~2-8).
                            if (std::find(candidates.begin(), candidates.end(), val) == candidates.end())
                            {
                                candidates.push_back(val);
                            }
                        }
                        slot = (slot + 1) % tableSize;
                    }
                }
            }
        }

        bool found = false;
        float earliestT = std::numeric_limits<float>::max();
        Vector3 hitPoint = V3Zero();
        Vector3 hitNormal = V3(0.0f, 1.0f, 0.0f);
        StaticColliderData hitCol = {};

        for (int idx : candidates)
        {
            const StaticColliderData& c = colliders[idx];
            float t = std::numeric_limits<float>::max();
            Vector3 p = V3Zero();
            Vector3 nrm = V3(0.0f, 1.0f, 0.0f);
            bool hit = false;

            if (c.type == COL_SPHERE)       hit = SweptSphereSphere(cur, vel, radius, c, t, p, nrm);
            else if (c.type == COL_BOX)     hit = SweptSphereBox(cur, vel, radius, c, t, p, nrm);
            else if (c.type == COL_CAPSULE) hit = SweptSphereCapsule(cur, vel, radius, c, t, p, nrm);
            else if (c.type == COL_TRIANGLE)hit = SweptSphereTriangle(cur, vel, radius, c, t, p, nrm);

            if (hit && t < earliestT) { earliestT = t; hitPoint = p; hitNormal = nrm; hitCol = c; found = true; }
        }

        if (!found) continue;

        Vector3 origPred = Add(cur, vel);
        float rawPen = MaxF(radius - Dot(Sub(origPred, hitPoint), hitNormal), 0.0f);
        Vector3 resolvedPos = Add(origPred, Scale(hitNormal, rawPen + epsilon));
        predX[i] = resolvedPos.x; predY[i] = resolvedPos.y; predZ[i] = resolvedPos.z;

        float preVn = Dot(Scale(vel, 1.0f / dt), hitNormal);

        if (preVn < 0.0f || rawPen > 0.0f)
        {
            float restA = (defaultRestitution + hitCol.restitution) * 0.5f;
            float sf = (defaultStaticFriction + hitCol.staticFriction) * 0.5f;
            float kf = (defaultSlidingFriction + hitCol.slidingFriction) * 0.5f;

            constexpr float BOUNCE_THRESHOLD = 0.2f;
            float appliedRest = (-preVn > BOUNCE_THRESHOLD) ? restA : 0.0f;
            float bounceVn = (appliedRest > 0.0f) ? -preVn * appliedRest : 0.0f;

            float jn = (preVn < 0.0f) ? -(1.0f + appliedRest) * preVn : 0.0f;
            float positionalVn = rawPen / dt;
            float effectiveJn = jn + positionalVn;

            Vector3 postV = Scale(Sub(resolvedPos, cur), 1.0f / dt);
            float   postVn = Dot(postV, hitNormal);
            float   targetVn = MaxF(postVn, bounceVn);
            Vector3 vNew = Add(postV, Scale(hitNormal, targetVn - postVn));

            Vector3 vt = Sub(vNew, Scale(hitNormal, Dot(vNew, hitNormal)));
            float   vtSpeed = Length(vt);
            if (vtSpeed > PhysicsConstants::MIN_TANGENT_SPEED)
            {
                Vector3 tdir = Scale(vt, 1.0f / vtSpeed);
                float jt = vtSpeed;
                float fMag = (jt <= sf * effectiveJn) ? jt : MinF(kf * effectiveJn, jt);
                vNew = Sub(vNew, Scale(tdir, fMag));
            }
            predX[i] = Add(cur, Scale(vNew, dt)).x; predY[i] = Add(cur, Scale(vNew, dt)).y; predZ[i] = Add(cur, Scale(vNew, dt)).z;
        }
    }
}

EXPORT void ResolveStaticCollisionsCCD(
    float* predX, float* predY, float* predZ, float* currX, float* currY, float* currZ,
    const float* masses, const uint8_t* isPinned, int nodeCount,
    const StaticColliderData* colliders, int colliderCount,
    const int* gridKeys, const int* gridVals, int gridEntryCount,
    float radius, float epsilon, float gridCellSize,
    float defaultRestitution, float defaultStaticFriction, float defaultSlidingFriction, float dt)
{
    if (gridEntryCount <= 0) return;

    // grow-only: retains heap capacity across calls
    int tableSize = gridEntryCount * 2;
    static thread_local std::vector<HashEntry> hashTable;
    hashTable.assign(tableSize, { -1, -1 });

    for (int i = 0; i < gridEntryCount; ++i)
    {
        int hash = gridKeys[i];
        int slot = (hash & 0x7FFFFFFF) % tableSize;
        while (hashTable[slot].value != -1)
        {
            slot = (slot + 1) % tableSize;
        }
        hashTable[slot] = { hash, gridVals[i] };
    }

    ResolveStaticCollisionsCCD_Internal(
        predX, predY, predZ, currX, currY, currZ, masses, isPinned, nodeCount, colliders, colliderCount,
        hashTable.data(), tableSize, radius, epsilon, gridCellSize,
        defaultRestitution, defaultStaticFriction, defaultSlidingFriction, dt);
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//  Face-to-node collision
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//
// (#4) The per-frame unordered_multimap is gone --- node cells are stored in a
//      sorted, contiguous std::vector<KeyVal> and queried with lower_bound.
// (#5/#6) The node hash is now BUILT ONCE per body per substep (see
//      StepAllCollisions) and reused for every face query and every solver
//      iteration, instead of being rebuilt for every call.

// Build the swept-AABB cell hash for a set of nodes. Sorted for binary search.
static void BuildNodeSweptHash(const float* prevX, const float* prevY, const float* prevZ, const float* predX, const float* predY, const float* predZ, int count,
    float invCell, std::vector<KeyVal>& flatHash)
{
    flatHash.clear();
    for (int n = 0; n < count; ++n)
    {
        const Vector3& s = VEC3(prev, n);
        const Vector3& e = VEC3(pred, n);

        const int x0 = FloorToInt(MinF(s.x, e.x) * invCell);
        const int y0 = FloorToInt(MinF(s.y, e.y) * invCell);
        const int z0 = FloorToInt(MinF(s.z, e.z) * invCell);
        const int x1 = FloorToInt(MaxF(s.x, e.x) * invCell);
        const int y1 = FloorToInt(MaxF(s.y, e.y) * invCell);
        const int z1 = FloorToInt(MaxF(s.z, e.z) * invCell);

        for (int cx = x0; cx <= x1; ++cx)
            for (int cy = y0; cy <= y1; ++cy)
                for (int cz = z0; cz <= z1; ++cz)
                    flatHash.push_back({ (int)SpatialHash(cx, cy, cz), n });
    }
    std::sort(flatHash.begin(), flatHash.end());
}

// Resolve faces against nodes using a PRE-BUILT sorted node hash.
static void ResolveFaceToNodeImpl(
    float* nodePredX, float* nodePredY, float* nodePredZ, const float* nodePrevX, const float* nodePrevY, const float* nodePrevZ,
    const float* nodeMasses, const uint8_t* nodeIsPinned, int nodeCount,
    float* facePredX, float* facePredY, float* facePredZ, const float* facePrevX, const float* facePrevY, const float* facePrevZ,
    const float* faceMasses, const uint8_t* faceIsPinned,
    const FaceData* faces, int faceCount,
    float radiusNode, float thickness, float epsilon, float dt,
    float invCell, const std::vector<KeyVal>& flatHash)
{
    if (nodeCount <= 0 || faceCount <= 0 || dt <= 0.0f || flatHash.empty()) return;

    static thread_local std::vector<int> candidates;
    static thread_local std::vector<uint8_t> visited;     // grow-only
    if ((int)visited.size() < nodeCount) visited.resize(nodeCount);
    std::memset(visited.data(), 0, nodeCount * sizeof(uint8_t));

    for (int f = 0; f < faceCount; ++f)
    {
        const FaceData& face = faces[f];
        const Vector3 v0s = VEC3(facePrev, face.nodeA), v1s = VEC3(facePrev, face.nodeB), v2s = VEC3(facePrev, face.nodeC);
        const Vector3 v0e = VEC3(facePred, face.nodeA), v1e = VEC3(facePred, face.nodeB), v2e = VEC3(facePred, face.nodeC);

        const float bMinX = MinF(MinF(v0s.x, v1s.x), MinF(v2s.x, MinF(MinF(v0e.x, v1e.x), v2e.x))) - thickness;
        const float bMinY = MinF(MinF(v0s.y, v1s.y), MinF(v2s.y, MinF(MinF(v0e.y, v1e.y), v2e.y))) - thickness;
        const float bMinZ = MinF(MinF(v0s.z, v1s.z), MinF(v2s.z, MinF(MinF(v0e.z, v1e.z), v2e.z))) - thickness;
        const float bMaxX = MaxF(MaxF(v0s.x, v1s.x), MaxF(v2s.x, MaxF(MaxF(v0e.x, v1e.x), v2e.x))) + thickness;
        const float bMaxY = MaxF(MaxF(v0s.y, v1s.y), MaxF(v2s.y, MaxF(MaxF(v0e.y, v1e.y), v2e.y))) + thickness;
        const float bMaxZ = MaxF(MaxF(v0s.z, v1s.z), MaxF(v2s.z, MaxF(MaxF(v0e.z, v1e.z), v2e.z))) + thickness;

        candidates.clear();
        const int x0 = FloorToInt(bMinX * invCell), y0 = FloorToInt(bMinY * invCell), z0 = FloorToInt(bMinZ * invCell);
        const int x1 = FloorToInt(bMaxX * invCell), y1 = FloorToInt(bMaxY * invCell), z1 = FloorToInt(bMaxZ * invCell);

        for (int x = x0; x <= x1; ++x)
            for (int y = y0; y <= y1; ++y)
                for (int z = z0; z <= z1; ++z)
                {
                    const int key = (int)SpatialHash(x, y, z);
                    KeyVal probe = { key, 0 };
                    auto it = std::lower_bound(flatHash.begin(), flatHash.end(), probe);
                    for (; it != flatHash.end() && it->key == key; ++it)
                    {
                        const int n = it->value;
                        if (!visited[n]) { visited[n] = 1; candidates.push_back(n); }
                    }
                }

        for (int idx : candidates) visited[idx] = 0;   // reset for next face

        for (int idx : candidates)
        {
            const int n = idx;
            const Vector3 nodeStart = VEC3(nodePrev, n);
            const Vector3 nodeEnd = VEC3(nodePred, n);

            const float nsX = MinF(nodeStart.x, nodeEnd.x), nsY = MinF(nodeStart.y, nodeEnd.y), nsZ = MinF(nodeStart.z, nodeEnd.z);
            const float neX = MaxF(nodeStart.x, nodeEnd.x), neY = MaxF(nodeStart.y, nodeEnd.y), neZ = MaxF(nodeStart.z, nodeEnd.z);
            if (nsX > bMaxX || neX < bMinX || nsY > bMaxY || neY < bMinY || nsZ > bMaxZ || neZ < bMinZ) continue;

            float t; Vector3 hitNormal;
            if (CheckSweptFaceNode(nodeStart, nodeEnd, radiusNode,
                v0s, v0e, v1s, v1e, v2s, v2e, t, hitNormal))
            {
                const Vector3 pAtT = Lerp(nodeStart, nodeEnd, t);
                const Vector3 aAtT = Lerp(v0s, v0e, t);
                const Vector3 bAtT = Lerp(v1s, v1e, t);
                const Vector3 cAtT = Lerp(v2s, v2e, t);

                float u, vBary, w;
                if (GetBarycentric(pAtT, aAtT, bAtT, cAtT, u, vBary, w))
                {
                    const Vector3 triPoint = Add(Add(Scale(v0e, u), Scale(v1e, vBary)), Scale(v2e, w));
                    const float dist = Dot(Sub(nodeEnd, triPoint), hitNormal);
                    float penetration = (radiusNode + epsilon) - dist;
                    if (penetration > 0.0f || t < 1.0f)
                    {
                        penetration = MaxF(penetration, epsilon);
                        ApplyXPBDFaceNode(n, face.nodeA, face.nodeB, face.nodeC,
                            hitNormal, penetration, u, vBary, w,
                            nodePredX, nodePredY, nodePredZ, facePredX, facePredY, facePredZ,
                            nodeMasses, faceMasses,
                            nodeIsPinned, faceIsPinned);
                    }
                }
            }
        }
    }
}

// EXPORT kept for ABI. Builds the node hash inline (single call) then resolves.
EXPORT void ResolveFaceToNode(float* nodePredX, float* nodePredY, float* nodePredZ, const float* nodePrevX, const float* nodePrevY, const float* nodePrevZ,
    const float* nodeMasses, const uint8_t* nodeIsPinned, int nodeCount,
    float* facePredX, float* facePredY, float* facePredZ, const float* facePrevX, const float* facePrevY, const float* facePrevZ,
    const float* faceMasses, const uint8_t* faceIsPinned, int faceNodeCount,
    const FaceData* faces, int faceCount,
    float radiusNode, float thickness, float epsilon, float dt,
    float faceNodeCellSize)
{
    (void)faceNodeCount;
    if (nodeCount <= 0 || faceCount <= 0 || dt <= 0.0f) return;

    const float invCell = 1.0f / faceNodeCellSize;
    static thread_local std::vector<KeyVal> flatHash;
    BuildNodeSweptHash(nodePrevX, nodePrevY, nodePrevZ, nodePredX, nodePredY, nodePredZ, nodeCount, invCell, flatHash);

    ResolveFaceToNodeImpl(
        nodePredX, nodePredY, nodePredZ, nodePrevX, nodePrevY, nodePrevZ, nodeMasses, nodeIsPinned, nodeCount,
        facePredX, facePredY, facePredZ, facePrevX, facePrevY, facePrevZ, faceMasses, faceIsPinned,
        faces, faceCount, radiusNode, thickness, epsilon, dt, invCell, flatHash);
}
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//  StepAllCollisions --- top-level per-substep driver
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//
// Pipeline (per substep):
//   1. Build static grid hash once.
//   2. Static CCD once (it's a projection; repeating it just redoes work).
//   3. Build ignored-pair lookup as a sorted flat array            (#2).
//   4. SAP broadphase: persistent order + adaptive insertion sort  (#1).
//   5. Build node-node contact sets ONCE per pair (split indices)  (#3,#8,#9).
//   6. Build per-body node hash ONCE, lazily, reused every iter    (#4,#5,#6).
//   7. Solver iterations: resolve contacts in place, no staging    (#3,#7,#9).

EXPORT void StepAllCollisions(
    NativeSoftBodyData* bodies, int bodyCount,
    const StaticColliderData* colliders, int colliderCount,
    const int* gridKeys, const int* gridVals, int gridEntryCount,
    const NativeInt2* ignoredPairs, int ignoredCount,
    float dt, float epsilon, float staticGridCellSize, float faceNodeCellSize,
    int iterations, int frameId)
{
    (void)frameId;
    if (bodyCount <= 0 || dt <= 0.0f) return;
    auto t0 = std::chrono::high_resolution_clock::now();

    const float invDt = 1.0f / dt;

    // Accumulate faces (stats)
    int totalFaces = 0;
    for (int i = 0; i < bodyCount; ++i) totalFaces += bodies[i].faceCount;
    g_Stats.totalFaces = totalFaces;

    if (iterations < 1) iterations = 1;

    // ------ (1) Static grid hash --- built ONCE ------
    const int staticTableSize = gridEntryCount * 2 + 1;
    static thread_local std::vector<HashEntry> localStaticGridTable;     // grow-only scratch
    localStaticGridTable.assign(staticTableSize, { -1, -1 });
    for (int i = 0; i < gridEntryCount; ++i)
    {
        const int hash = gridKeys[i];
        int slot = (hash & 0x7FFFFFFF) % staticTableSize;
        while (localStaticGridTable[slot].value != -1)
            slot = (slot + 1) % staticTableSize;
        localStaticGridTable[slot] = { hash, gridVals[i] };
    }

    // ------ (2) Static CCD --- once per substep ------
    for (int i = 0; i < bodyCount; ++i)
    {
        NativeSoftBodyData& b = bodies[i];
        if (b.nodeCount == 0) continue;
        ResolveStaticCollisionsCCD_Internal(
            b.predictedX, b.predictedY, b.predictedZ, b.currentX, b.currentY, b.currentZ, b.masses, b.isPinned, b.nodeCount,
            colliders, colliderCount,
            localStaticGridTable.data(), staticTableSize,
            b.radius + PhysicsConstants::SKIN_WIDTH, epsilon, staticGridCellSize,
            b.restitution, b.staticFriction, b.slidingFriction, dt);
    }

    // ------ (3) Ignored-pair lookup as a SORTED FLAT ARRAY (replaces unordered_set) ------
    //    Binary search is branch-predictable and cache-friendly, vs. the hash
    //    probing + bucket chasing that caused the latency spikes.
    static thread_local std::vector<uint64_t> ignoredSorted;
    ignoredSorted.clear();
    if (ignoredCount > 0)
    {
        ignoredSorted.reserve((size_t)ignoredCount);
        for (int i = 0; i < ignoredCount; ++i)
        {
            uint64_t a = (uint64_t)ignoredPairs[i].x;
            uint64_t b = (uint64_t)ignoredPairs[i].y;
            ignoredSorted.push_back((a < b) ? (a | (b << 32)) : (b | (a << 32)));
        }
        std::sort(ignoredSorted.begin(), ignoredSorted.end());
    }

    // ------ (4) Sweep-and-Prune broadphase ------
    //    Persistent order across frames + adaptive insertion sort. Body minX
    //    order is almost stable frame-to-frame, so insertion sort runs in ~O(N)
    //    instead of the O(N log N) full std::sort that ran every step.
    struct SapInterval { float minX, maxX; };
    static thread_local std::vector<SapInterval> sapItv;     // indexed by body
    static thread_local std::vector<int>         sapOrder;   // PERSISTENT sorted active bodies
    static thread_local std::vector<int>         sapPairA;
    static thread_local std::vector<int>         sapPairB;

    if ((int)sapItv.size() < bodyCount) sapItv.resize(bodyCount);

    const float halfEps = epsilon * 0.5f;
    int activeCount = 0;
    for (int i = 0; i < bodyCount; ++i)
    {
        const NativeSoftBodyData& b = bodies[i];
        if (b.nodeCount == 0) continue;
        sapItv[i].minX = b.boundsMin.x - b.radius - halfEps;
        sapItv[i].maxX = b.boundsMax.x + b.radius + halfEps;
        ++activeCount;
    }

    // Is the persistent order still exactly the active set? (cheap O(N) check)
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
        // Adaptive insertion sort --- O(N) on near-sorted input.
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
            if (sapItv[ib].minX > aMaxX) break;     // O(K) axis early-out

            const NativeSoftBodyData& bB = bodies[ib];

            // Layer mask
            if (((1 << bB.layer) & bA.collisionLayerMask) == 0) continue;
            if (((1 << bA.layer) & bB.collisionLayerMask) == 0) continue;

            // Ignored-pair lookup (#2): binary search into the sorted flat array
            {
                uint64_t idA = (uint64_t)bA.id, idB = (uint64_t)bB.id;
                uint64_t key = (idA < idB) ? (idA | (idB << 32)) : (idB | (idA << 32));
                if (!ignoredSorted.empty() &&
                    std::binary_search(ignoredSorted.begin(), ignoredSorted.end(), key)) continue;
            }

            // Y/Z AABB overlap (X already confirmed by the SAP interval)
            const float exp = bA.radius + bB.radius + epsilon;
            if (bA.boundsMin.y > bB.boundsMax.y + exp ||
                bA.boundsMax.y < bB.boundsMin.y - exp) continue;
            if (bA.boundsMin.z > bB.boundsMax.z + exp ||
                bA.boundsMax.z < bB.boundsMin.z - exp) continue;

            if (ia < ib) { sapPairA.push_back(ia); sapPairB.push_back(ib); }
            else { sapPairA.push_back(ib); sapPairB.push_back(ia); }
        }
    }

    const int pairCount = (int)sapPairA.size();

    // ------ (5) Build node-node contacts ONCE per pair (#9) ------
    //    Contacts are generated from the start-of-substep predicted positions
    //    (plus a small broadphase margin so contacts persist across iterations)
    //    and reused for every solver iteration, replacing the per-iteration
    //    BuildNodeNodePairs rebuild + the per-pair memcpy staging (#3).
    struct PairContacts { int start; int count; };
    static thread_local std::vector<NodeContact> contactPool;
    static thread_local std::vector<PairContacts> pairContacts;
    static thread_local std::vector<NodeContact> contactScratch;

    contactPool.clear();
    pairContacts.clear();

    const float bpEpsilon = epsilon + PhysicsConstants::BROADPHASE_MARGIN;
    for (int p = 0; p < pairCount; ++p)
    {
        const NativeSoftBodyData& bA = bodies[sapPairA[p]];
        const NativeSoftBodyData& bB = bodies[sapPairB[p]];

        int maxContacts = 16 * (bA.nodeCount + bB.nodeCount);     // surface-ish, not all-pairs
        if (maxContacts > PhysicsConstants::MAX_CONTACTS_PER_PAIR) // (#8) hard cap
            maxContacts = PhysicsConstants::MAX_CONTACTS_PER_PAIR;
        if (maxContacts <= 0) { pairContacts.push_back({ (int)contactPool.size(), 0 }); continue; }

        if ((int)contactScratch.size() < maxContacts) contactScratch.resize(maxContacts);

        const int found = BuildNodeContactsSplit(
            bA.predictedX, bA.predictedY, bA.predictedZ, bA.nodeCount, bB.predictedX, bB.predictedY, bB.predictedZ, bB.nodeCount,
            bA.radius, bB.radius, bpEpsilon, contactScratch.data(), maxContacts);

        pairContacts.push_back({ (int)contactPool.size(), found });
        if (found > 0)
            contactPool.insert(contactPool.end(), contactScratch.begin(), contactScratch.begin() + found);
    }

    // ------ (6) Per-body node hash for face->node, built ONCE and lazily (#4,#5,#6) ------
    const float invFaceCell = 1.0f / faceNodeCellSize;
    static thread_local std::vector<std::vector<KeyVal>> nodeHashCache;
    static thread_local std::vector<uint8_t> nodeHashBuilt;
    if ((int)nodeHashCache.size() < bodyCount) nodeHashCache.resize(bodyCount);
    nodeHashBuilt.assign(bodyCount, 0);

    auto ensureNodeHash = [&](int b) -> const std::vector<KeyVal>&{
        if (!nodeHashBuilt[b])
        {
            BuildNodeSweptHash(
                bodies[b].currentX, bodies[b].currentY, bodies[b].currentZ,
                bodies[b].predictedX, bodies[b].predictedY, bodies[b].predictedZ,
                bodies[b].nodeCount, invFaceCell, nodeHashCache[b]);
            nodeHashBuilt[b] = 1;
        }
        return nodeHashCache[b];
        };

    // ------ (7) Solver iterations over the cached contact sets ------
    for (int iter = 0; iter < iterations; ++iter)
    {
        for (int p = 0; p < pairCount; ++p)
        {
            NativeSoftBodyData& bA = bodies[sapPairA[p]];
            NativeSoftBodyData& bB = bodies[sapPairB[p]];
            const PairContacts& pc = pairContacts[p];

            // Node-node: resolve directly on each body's buffers (no staging, #3)
            if (pc.count > 0)
            {
                const float combinedR = bA.radius + bB.radius + epsilon;
                ResolveNodeContactsSplit(
                    bA.predictedX, bA.predictedY, bA.predictedZ, bA.currentX, bA.currentY, bA.currentZ, bA.masses, bA.isPinned,
                    bB.predictedX, bB.predictedY, bB.predictedZ, bB.currentX, bB.currentY, bB.currentZ, bB.masses, bB.isPinned,
                    contactPool.data() + pc.start, pc.count,
                    dt, invDt, combinedR,
                    (bA.restitution + bB.restitution) * 0.5f,
                    (bA.staticFriction + bB.staticFriction) * 0.5f,
                    (bA.slidingFriction + bB.slidingFriction) * 0.5f);
            }

            // Face(A) vs Node(B) --- node side hash for bB
            if (bA.faceCount > 0)
            {
                const std::vector<KeyVal>& h = ensureNodeHash(sapPairB[p]);
                ResolveFaceToNodeImpl(
                    bB.predictedX, bB.predictedY, bB.predictedZ, bB.currentX, bB.currentY, bB.currentZ, bB.masses, bB.isPinned, bB.nodeCount,
                    bA.predictedX, bA.predictedY, bA.predictedZ, bA.currentX, bA.currentY, bA.currentZ, bA.masses, bA.isPinned,
                    bA.faces, bA.faceCount,
                    bB.radius + PhysicsConstants::SKIN_WIDTH,
                    bB.radius + PhysicsConstants::SKIN_WIDTH + PhysicsConstants::DEFAULT_TRIANGLE_THICKNESS,
                    epsilon, dt, invFaceCell, h);
            }

            // Face(B) vs Node(A) --- node side hash for bA
            if (bB.faceCount > 0)
            {
                const std::vector<KeyVal>& h = ensureNodeHash(sapPairA[p]);
                ResolveFaceToNodeImpl(
                    bA.predictedX, bA.predictedY, bA.predictedZ, bA.currentX, bA.currentY, bA.currentZ, bA.masses, bA.isPinned, bA.nodeCount,
                    bB.predictedX, bB.predictedY, bB.predictedZ, bB.currentX, bB.currentY, bB.currentZ, bB.masses, bB.isPinned,
                    bB.faces, bB.faceCount,
                    bA.radius + PhysicsConstants::SKIN_WIDTH,
                    bA.radius + PhysicsConstants::SKIN_WIDTH + PhysicsConstants::DEFAULT_TRIANGLE_THICKNESS,
                    epsilon, dt, invFaceCell, h);
            }
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    g_Stats.collisionTimeMs += std::chrono::duration<float, std::milli>(t1 - t0).count();
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
        // Already penetrating --- verify over triangle before accepting
        float u, v, w;
        if (!GetBarycentric(p0, a0, b0, c0, u, v, w)) return false;
        tOut = 0.0f;
        nOut = d0 >= 0.0f ? n0 : Neg(n0);
        return true;
    }

    // Interpolate crossing time
    float ad0 = AbsF(d0), ad1 = AbsF(d1);
    float t = ClampF(ad0 / (ad0 + ad1), 0.0f, 1.0f);

    // Check whether node is actually over the triangle at the crossing time
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
    float d00 = Dot(v0, v0), d01 = Dot(v0, v1), d11 = Dot(v1, v1);
    float d20 = Dot(v2, v0), d21 = Dot(v2, v1);
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
    float* nodePredX, float* nodePredY, float* nodePredZ, float* facePredX, float* facePredY, float* facePredZ,
    const float* nodeMasses, const float* faceMasses,
    const uint8_t* nodePinned, const uint8_t* facePinned)
{
    float wP = nodePinned[idxP] ? 0.0f : 1.0f / nodeMasses[idxP];
    float wA = facePinned[idxA] ? 0.0f : 1.0f / faceMasses[idxA];
    float wB = facePinned[idxB] ? 0.0f : 1.0f / faceMasses[idxB];
    float wC = facePinned[idxC] ? 0.0f : 1.0f / faceMasses[idxC];

    float wTri = (u * u * wA) + (v * v * wB) + (w * w * wC);
    float wSum = wP + wTri;
    if (wSum <= 1e-9f) return;

    float dLambda = penetration / wSum;
    Vector3 corr = Scale(normal, dLambda);

    if (!nodePinned[idxP])
    {
        nodePredX[idxP] = Add(VEC3(nodePred, idxP), Scale(corr, wP)).x; 
        nodePredY[idxP] = Add(VEC3(nodePred, idxP), Scale(corr, wP)).y; 
        nodePredZ[idxP] = Add(VEC3(nodePred, idxP), Scale(corr, wP)).z;
    }
    if (!facePinned[idxA])
    {
        facePredX[idxA] = Sub(VEC3(facePred, idxA), Scale(corr, wA * u)).x; 
        facePredY[idxA] = Sub(VEC3(facePred, idxA), Scale(corr, wA * u)).y; 
        facePredZ[idxA] = Sub(VEC3(facePred, idxA), Scale(corr, wA * u)).z;
    }
    if (!facePinned[idxB])
    {
        facePredX[idxB] = Sub(VEC3(facePred, idxB), Scale(corr, wB * v)).x; 
        facePredY[idxB] = Sub(VEC3(facePred, idxB), Scale(corr, wB * v)).y; 
        facePredZ[idxB] = Sub(VEC3(facePred, idxB), Scale(corr, wB * v)).z;
    }
    if (!facePinned[idxC])
    {
        facePredX[idxC] = Sub(VEC3(facePred, idxC), Scale(corr, wC * w)).x; 
        facePredY[idxC] = Sub(VEC3(facePred, idxC), Scale(corr, wC * w)).y; 
        facePredZ[idxC] = Sub(VEC3(facePred, idxC), Scale(corr, wC * w)).z;
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
        Vector3 ln = V3Zero();
        if (dx <= dy && dx <= dz) ln.x = ls.x >= 0.0f ? 1.0f : -1.0f;
        else if (dy <= dz)             ln.y = ls.y >= 0.0f ? 1.0f : -1.0f;
        else                           ln.z = ls.z >= 0.0f ? 1.0f : -1.0f;
        outT = 0.0f;
        outNormal = QRotate(box.rotation, ln);
        outPoint = Sub(start, Scale(outNormal, radius));
        return true;
    }

    float tMin = 0.0f, tMax = 1.0f;
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
            if (tMin > tMax) return false;
        }
    }

    if (tMin < 0.0f || tMin > 1.0f) return false;

    outT = tMin;
    Vector3 ln = V3Zero();
    if (hitAxis == 0) ln.x = (float)hitSign;
    else if (hitAxis == 1) ln.y = (float)hitSign;
    else if (hitAxis == 2) ln.z = (float)hitSign;
    outNormal = QRotate(box.rotation, ln);
    outPoint = Sub(Add(start, Scale(vel, outT)), Scale(outNormal, radius));
    return true;
}

static bool SweptSphereTriangle(const Vector3& start, const Vector3& vel, float radius,
    const StaticColliderData& tri,
    float& outT, Vector3& outPoint, Vector3& outNormal)
{
    outT = std::numeric_limits<float>::max();
    outPoint = V3Zero();
    outNormal = V3(0.0f, 1.0f, 0.0f);

    Vector3 e1 = Sub(tri.v1, tri.v0);
    Vector3 e2 = Sub(tri.v2, tri.v0);
    Vector3 triNormal = Normalize(Cross(e1, e2));
    float distToPlane = Dot(Sub(start, tri.v0), triNormal);
    bool isFront = distToPlane > 0.0f;
    Vector3 n = isFront ? triNormal : Neg(triNormal);
    float d = isFront ? distToPlane : -distToPlane;

    float nv = Dot(vel, n);
    if (d < radius) {
        // Node already inside or past the triangle plane
        if (nv < 0.0f || d < 0.0f) {
            outT = 0.0f;
            outNormal = n;
            outPoint = Sub(start, Scale(n, d - radius));
            return true;
        }
    }
    if (nv >= 0.0f) return false;

    float t = (d - radius) / -nv;
    if (t < 0.0f || t > 1.0f) return false;

    Vector3 p = Sub(Add(start, Scale(vel, t)), Scale(n, radius));
    Vector3 v0p = Sub(p, tri.v0);

    float dot00 = Dot(e1, e1);
    float dot01 = Dot(e1, e2);
    float dot02 = Dot(e1, v0p);
    float dot11 = Dot(e2, e2);
    float dot12 = Dot(e2, v0p);

    float denom = dot00 * dot11 - dot01 * dot01;
    if (AbsF(denom) < 1e-9f) return false;
    float invDenom = 1.0f / denom;
    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    float vv = (dot00 * dot12 - dot01 * dot02) * invDenom;

    if (u >= -0.01f && vv >= -0.01f && (u + vv) <= 1.01f)
    {
        outT = t;
        outNormal = n;
        outPoint = p;
        return true;
    }
    return false;
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

    // Degenerate capsule --- treat as sphere
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

    // Solve swept sphere vs. infinite cylinder, then clamp to segment
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
                // Check if hit point is within the segment ends
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

    // Test both hemisphere end-caps
    StaticColliderData cap1 = {}; cap1.center = p1; cap1.radius = capsule.radius;
    StaticColliderData cap2 = {}; cap2.center = p2; cap2.radius = capsule.radius;
    float t1, t2; Vector3 p1out, p2out, n1, n2;
    bool h1 = SweptSphereSphere(start, vel, radius, cap1, t1, p1out, n1);
    bool h2 = SweptSphereSphere(start, vel, radius, cap2, t2, p2out, n2);

    // Pick earliest
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


