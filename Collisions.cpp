/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝
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

#if defined(_WIN32)
#define EXPORT extern "C" __declspec(dllexport)
#else
#define EXPORT extern "C" __attribute__((visibility("default")))
#endif

struct HashEntry {
    int hash;
    int value;
};

// Forward declarations of internal helpers

static void ApplyXPBDFaceNode(int idxP, int idxA, int idxB, int idxC,
    const Vector3& normal, float penetration,
    float u, float v, float w,
    Vector3* nodePred, Vector3* facePred,
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

// ResolveNodePairs

EXPORT void ResolveNodePairs(Vector3* predictedPos, Vector3* currentPos,
    const float* masses, const uint8_t* isPinned,
    const CollisionPair* pairs, int pairCount,
    float dt, float epsilon,
    float radiusA, float radiusB,
    float restitution, float staticFriction, float slidingFriction)
{
    if (pairCount <= 0 || dt <= 0.0f) return;

    float combinedR = radiusA + radiusB + epsilon;
    float invDt = 1.0f / dt;

    for (int p = 0; p < pairCount; ++p)
    {
        int i = pairs[p].nodeA;
        int j = pairs[p].nodeB;

        float wi = (isPinned && isPinned[i]) ? 0.0f : (masses[i] > 0.0f ? 1.0f / masses[i] : 0.0f);
        float wj = (isPinned && isPinned[j]) ? 0.0f : (masses[j] > 0.0f ? 1.0f / masses[j] : 0.0f);
        float wSum = wi + wj;
        if (wSum <= 0.0f) continue;

        // --- CCD: relative sweep ---
        Vector3 velI = Sub(predictedPos[i], currentPos[i]);
        Vector3 velJ = Sub(predictedPos[j], currentPos[j]);
        Vector3 relVel = Sub(velI, velJ);
        Vector3 relStart = Sub(currentPos[i], currentPos[j]);

        float a = Dot(relVel, relVel);
        float b = 2.0f * Dot(relStart, relVel);
        float c = Dot(relStart, relStart) - combinedR * combinedR;

        float toi = 1.0f; // default: use end-of-step positions
        bool alreadyOverlapping = (c <= 0.0f);

        if (!alreadyOverlapping)
        {
            if (a < 1e-12f) continue; // no relative motion, no overlap
            float disc = b * b - 4.0f * a * c;
            if (disc < 0.0f) continue; // paths don't intersect
            float t = (-b - std::sqrt(disc)) / (2.0f * a);
            if (t < 0.0f || t > 1.0f) continue; // collision outside this step
            toi = t;
        }

        // Interpolate positions at TOI
        Vector3 pi = Add(currentPos[i], Scale(Sub(predictedPos[i], currentPos[i]), toi));
        Vector3 pj = Add(currentPos[j], Scale(Sub(predictedPos[j], currentPos[j]), toi));

        Vector3 d = Sub(pi, pj);
        float   dist = Length(d);
        float   minDist = PhysicsConstants::MIN_COLLISION_DISTANCE;
        if (dist < minDist) continue;

        Vector3 n = Scale(d, 1.0f / dist);
        float   penetration = combinedR - dist;
        if (penetration < 0.0f) penetration = 0.0f;

        // Push out from TOI contact point
        float penetrationAtEnd = combinedR - Length(Sub(predictedPos[i], predictedPos[j]));
        if (penetrationAtEnd < 0.0f) penetrationAtEnd = 0.0f;
        Vector3 corr = Scale(n, penetrationAtEnd / wSum);
        predictedPos[i] = Add(predictedPos[i], Scale(corr, wi));
        predictedPos[j] = Sub(predictedPos[j], Scale(corr, wj));;

        // Velocity response
        Vector3 vi = Scale(Sub(predictedPos[i], currentPos[i]), invDt);
        Vector3 vj = Scale(Sub(predictedPos[j], currentPos[j]), invDt);
        Vector3 relV = Sub(vi, vj);
        float   vn = Dot(relV, n);

        if (vn < 0.0f)
        {
            float jn = -(1.0f + restitution) * vn / wSum;
            Vector3 impulse = Scale(n, jn);

            Vector3 viNew = Add(vi, Scale(impulse, wi));
            Vector3 vjNew = Sub(vj, Scale(impulse, wj));

            Vector3 vtPair = Sub(Sub(viNew, vjNew), Scale(n, Dot(Sub(viNew, vjNew), n)));
            float   vtSpeed = Length(vtPair);
            if (vtSpeed > PhysicsConstants::MIN_TANGENT_SPEED)
            {
                Vector3 tdir = Scale(vtPair, 1.0f / vtSpeed);
                float   jt = vtSpeed / wSum;
                float   frictionMag = (jt <= staticFriction * jn)
                    ? jt
                    : MinF(slidingFriction * jn, jt);
                Vector3 fImp = Scale(tdir, frictionMag);
                viNew = Sub(viNew, Scale(fImp, wi));
                vjNew = Add(vjNew, Scale(fImp, wj));
            }

            predictedPos[i] = Add(currentPos[i], Scale(viNew, dt));
            predictedPos[j] = Add(currentPos[j], Scale(vjNew, dt));
        }
    }
}
// BuildNodeNodePairs — spatial-hash broadphase 

EXPORT int BuildNodeNodePairs(const Vector3* posA, int countA,
    const Vector3* posB, int countB,
    float radiusA, float radiusB, float epsilon,
    CollisionPair* outPairs, int outCapacity)
{
    if (countA <= 0 || countB <= 0) return 0;

    float threshold = radiusA + radiusB + epsilon;
    float thresholdSq = threshold * threshold;
    float cell = PhysicsConstants::SPATIAL_CELL_SIZE;
    int range = (int)std::ceil(threshold / cell);

    // Contiguous open-addressed hash map for B-side
    int tableSize = countB * 2;
    std::vector<HashEntry> hashTable(tableSize, { -1, -1 });

    for (int i = 0; i < countB; ++i)
    {
        int gx = FloorToInt(posB[i].x / cell);
        int gy = FloorToInt(posB[i].y / cell);
        int gz = FloorToInt(posB[i].z / cell);
        int hash = SpatialHash(gx, gy, gz);

        int slot = (hash & 0x7FFFFFFF) % tableSize;
        while (hashTable[slot].value != -1)
        {
            slot = (slot + 1) % tableSize;
        }
        hashTable[slot] = { hash, i };
    }

    int written = 0;
    int total = 0;

    for (int i = 0; i < countA; ++i)
    {
        const Vector3& pa = posA[i];
        int gx0 = FloorToInt(pa.x / cell);
        int gy0 = FloorToInt(pa.y / cell);
        int gz0 = FloorToInt(pa.z / cell);

        for (int dx = -range; dx <= range; ++dx)
        {
            for (int dy = -range; dy <= range; ++dy)
            {
                for (int dz = -range; dz <= range; ++dz)
                {
                    int hash = SpatialHash(gx0 + dx, gy0 + dy, gz0 + dz);
                    int slot = (hash & 0x7FFFFFFF) % tableSize;

                    while (hashTable[slot].value != -1)
                    {
                        if (hashTable[slot].hash == hash)
                        {
                            int j = hashTable[slot].value;
                            Vector3 d = Sub(pa, posB[j]);
                            if (LengthSq(d) < thresholdSq)
                            {
                                if (written < outCapacity && outPairs)
                                {
                                    outPairs[written].nodeA = i;
                                    outPairs[written].nodeB = countA + j;
                                    ++written;
                                }
                                ++total;
                            }
                        }
                        slot = (slot + 1) % tableSize;
                    }
                }
            }
        }
    }
    return total;
}
// ResolveStaticCollisions — non-CCD positional projection vs. an already-baked list of static colliders

EXPORT void ResolveStaticCollisions(Vector3* predictedPos,
    const Vector3* currentPos,
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
                Vector3 d = Sub(predictedPos[n], col.center);
                float L = Length(d);
                if (L < 1e-7f) continue;
                normal = Scale(d, 1.0f / L);
                dist = L - col.radius;
                closestPoint = Add(col.center, Scale(normal, col.radius));
            }
            else if (col.type == COL_BOX)
            {
                Quaternion invRot = QInverse(col.rotation);
                Vector3 lp = QRotate(invRot, Sub(predictedPos[n], col.center));
                Vector3 half = Scale(col.size, 0.5f);

                Vector3 d = Sub(lp, { ClampF(lp.x, -half.x, half.x),
                                       ClampF(lp.y, -half.y, half.y),
                                       ClampF(lp.z, -half.z, half.z), 0.0f });
                float L = Length(d);

                if (L < 1e-7f)
                {
                    // Node is inside — push out via minimum-penetration axis
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
                    closestPoint = Sub(predictedPos[n], Scale(normal, minPen));
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
                Vector3 cp = ClosestPointTriangle(predictedPos[n], col.v0, col.v1, col.v2, u, v, w);
                Vector3 d = Sub(predictedPos[n], cp);
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
                float t = ClampF(Dot(Sub(predictedPos[n], p1), segDir), 0.0f, segLen);
                Vector3 cp = Add(p1, Scale(segDir, t));
                Vector3 d = Sub(predictedPos[n], cp);
                float L = Length(d);
                if (L < 1e-7f) continue;
                normal = Scale(d, 1.0f / L);
                closestPoint = Add(cp, Scale(normal, col.radius));
                dist = L - col.radius;
            }
            else continue;

            if (dist >= r) continue;

            float penetration = r - dist;
            predictedPos[n] = Add(predictedPos[n], Scale(normal, penetration + epsilon));

            Vector3 v = Scale(Sub(predictedPos[n], currentPos[n]), invDt);
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
                predictedPos[n] = Add(currentPos[n], Scale(vNew, dt));
            }
        }
    }
}

static void ResolveStaticCollisionsCCD_Internal(
    Vector3* predictedPos, Vector3* currentPos,
    const float* masses, const uint8_t* isPinned, int nodeCount,
    const StaticColliderData* colliders, int colliderCount,
    const HashEntry* hashTable, int tableSize,
    float radius, float epsilon, float gridCellSize,
    float defaultRestitution, float defaultStaticFriction,
    float defaultSlidingFriction, float dt)
{
    if (nodeCount <= 0 || colliderCount <= 0 || dt <= 0.0f || tableSize <= 0) return;

    std::unordered_set<int> candidates;
    candidates.reserve(64);

    for (int i = 0; i < nodeCount; ++i)
    {
        Vector3 cur = currentPos[i];
        Vector3 pred = predictedPos[i];
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
                            candidates.insert(hashTable[slot].value);
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
        predictedPos[i] = resolvedPos;

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
            predictedPos[i] = Add(cur, Scale(vNew, dt));
        }
    }
}

EXPORT void ResolveStaticCollisionsCCD(
    Vector3* predictedPos, Vector3* currentPos,
    const float* masses, const uint8_t* isPinned, int nodeCount,
    const StaticColliderData* colliders, int colliderCount,
    const int* gridKeys, const int* gridVals, int gridEntryCount,
    float radius, float epsilon, float gridCellSize,
    float defaultRestitution, float defaultStaticFriction, float defaultSlidingFriction, float dt)
{
    if (gridEntryCount <= 0) return;

    int tableSize = gridEntryCount * 2;
    std::vector<HashEntry> hashTable(tableSize, { -1, -1 });

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
        predictedPos, currentPos, masses, isPinned, nodeCount, colliders, colliderCount,
        hashTable.data(), tableSize, radius, epsilon, gridCellSize,
        defaultRestitution, defaultStaticFriction, defaultSlidingFriction, dt);
}


EXPORT void ResolveFaceToNode(Vector3* nodePred, const Vector3* nodePrev,
    const float* nodeMasses, const uint8_t* nodeIsPinned, int nodeCount,
    Vector3* facePred, const Vector3* facePrev,
    const float* faceMasses, const uint8_t* faceIsPinned, int faceNodeCount,
    const FaceData* faces, int faceCount,
    float radiusNode, float thickness, float epsilon, float dt,
    float faceNodeCellSize)
{
    if (nodeCount <= 0 || faceCount <= 0 || dt <= 0.0f) return;

    // Build node spatial hash containing both prev and pred positions.
    std::unordered_multimap<int, int> hash;
    hash.reserve((size_t)nodeCount * 4);
    float invCell = 1.0f / faceNodeCellSize;
    for (int n = 0; n < nodeCount; ++n)
    {
        const Vector3& s = nodePrev[n];
        const Vector3& e = nodePred[n];

        int x0 = FloorToInt(MinF(s.x, e.x) * invCell);
        int y0 = FloorToInt(MinF(s.y, e.y) * invCell);
        int z0 = FloorToInt(MinF(s.z, e.z) * invCell);
        int x1 = FloorToInt(MaxF(s.x, e.x) * invCell);
        int y1 = FloorToInt(MaxF(s.y, e.y) * invCell);
        int z1 = FloorToInt(MaxF(s.z, e.z) * invCell);

        for (int cx = x0; cx <= x1; ++cx)
            for (int cy = y0; cy <= y1; ++cy)
                for (int cz = z0; cz <= z1; ++cz)
                    hash.emplace(SpatialHash(cx, cy, cz), n);
    }

    std::vector<int> candidates;
    candidates.reserve(64);
    std::vector<uint8_t> visited(nodeCount, 0);

    for (int f = 0; f < faceCount; ++f)
    {
        const FaceData& face = faces[f];
        Vector3 v0s = facePrev[face.nodeA], v1s = facePrev[face.nodeB], v2s = facePrev[face.nodeC];
        Vector3 v0e = facePred[face.nodeA], v1e = facePred[face.nodeB], v2e = facePred[face.nodeC];

        float bMinX = MinF(MinF(v0s.x, v1s.x), MinF(v2s.x, MinF(MinF(v0e.x, v1e.x), v2e.x))) - thickness;
        float bMinY = MinF(MinF(v0s.y, v1s.y), MinF(v2s.y, MinF(MinF(v0e.y, v1e.y), v2e.y))) - thickness;
        float bMinZ = MinF(MinF(v0s.z, v1s.z), MinF(v2s.z, MinF(MinF(v0e.z, v1e.z), v2e.z))) - thickness;
        float bMaxX = MaxF(MaxF(v0s.x, v1s.x), MaxF(v2s.x, MaxF(MaxF(v0e.x, v1e.x), v2e.x))) + thickness;
        float bMaxY = MaxF(MaxF(v0s.y, v1s.y), MaxF(v2s.y, MaxF(MaxF(v0e.y, v1e.y), v2e.y))) + thickness;
        float bMaxZ = MaxF(MaxF(v0s.z, v1s.z), MaxF(v2s.z, MaxF(MaxF(v0e.z, v1e.z), v2e.z))) + thickness;

        candidates.clear();
        int x0 = FloorToInt(bMinX * invCell), y0 = FloorToInt(bMinY * invCell), z0 = FloorToInt(bMinZ * invCell);
        int x1 = FloorToInt(bMaxX * invCell), y1 = FloorToInt(bMaxY * invCell), z1 = FloorToInt(bMaxZ * invCell);

        for (int x = x0; x <= x1; ++x)
            for (int y = y0; y <= y1; ++y)
                for (int z = z0; z <= z1; ++z)
                {
                    int key = SpatialHash(x, y, z);
                    auto rng = hash.equal_range(key);
                    for (auto it = rng.first; it != rng.second; ++it)
                    {
                        int n = it->second;
                        if (!visited[n]) { visited[n] = 1; candidates.push_back(n); }
                    }
                }

        for (int idx : candidates) visited[idx] = 0;

        for (int idx : candidates)
        {
            int n = idx;
            Vector3 nodeStart = nodePrev[n];
            Vector3 nodeEnd = nodePred[n];

            float nsX = MinF(nodeStart.x, nodeEnd.x), nsY = MinF(nodeStart.y, nodeEnd.y), nsZ = MinF(nodeStart.z, nodeEnd.z);
            float neX = MaxF(nodeStart.x, nodeEnd.x), neY = MaxF(nodeStart.y, nodeEnd.y), neZ = MaxF(nodeStart.z, nodeEnd.z);
            if (nsX > bMaxX || neX < bMinX || nsY > bMaxY || neY < bMinY || nsZ > bMaxZ || neZ < bMinZ) continue;

            float t; Vector3 hitNormal;
            if (CheckSweptFaceNode(nodeStart, nodeEnd, radiusNode,
                v0s, v0e, v1s, v1e, v2s, v2e, t, hitNormal))
            {
                Vector3 pAtT = Lerp(nodeStart, nodeEnd, t);
                Vector3 aAtT = Lerp(v0s, v0e, t);
                Vector3 bAtT = Lerp(v1s, v1e, t);
                Vector3 cAtT = Lerp(v2s, v2e, t);

                float u, vBary, w;
                if (GetBarycentric(pAtT, aAtT, bAtT, cAtT, u, vBary, w))
                {
                    Vector3 triPoint = Add(Add(Scale(v0e, u), Scale(v1e, vBary)), Scale(v2e, w));
                    float dist = Dot(Sub(nodeEnd, triPoint), hitNormal);
                    float penetration = (radiusNode + epsilon) - dist;
                    if (penetration > 0.0f || t < 1.0f)
                    {
                        penetration = MaxF(penetration, epsilon);
                        ApplyXPBDFaceNode(n, face.nodeA, face.nodeB, face.nodeC,
                            hitNormal, penetration, u, vBary, w,
                            nodePred, facePred,
                            nodeMasses, faceMasses,
                            nodeIsPinned, faceIsPinned);
                    }
                }
            }
        }
    }
}



EXPORT void StepAllCollisions(
    NativeSoftBodyData* bodies, int bodyCount,
    const StaticColliderData* colliders, int colliderCount,
    const int* gridKeys, const int* gridVals, int gridEntryCount,
    const NativeInt2* ignoredPairs, int ignoredCount,
    float dt, float epsilon, float staticGridCellSize, float faceNodeCellSize,
    int frameId)
{
    if (bodyCount <= 0 || dt <= 0.0f) return;

    int staticTableSize = gridEntryCount * 2 + 1;
    std::vector<HashEntry> localStaticGridTable(staticTableSize, { -1, -1 });

    for (int i = 0; i < gridEntryCount; ++i)
    {
        int hash = gridKeys[i];
        int slot = (hash & 0x7FFFFFFF) % staticTableSize;
        while (localStaticGridTable[slot].value != -1)
        {
            slot = (slot + 1) % staticTableSize;
        }
        localStaticGridTable[slot] = { hash, gridVals[i] };
    }

    std::unordered_set<uint64_t> localIgnored;
    localIgnored.reserve((size_t)ignoredCount);
    for (int i = 0; i < ignoredCount; ++i)
    {
        uint64_t a = ignoredPairs[i].x, b = ignoredPairs[i].y;
        localIgnored.insert((a < b) ? (a | (b << 32)) : (b | (a << 32)));
    }

    // Static CCD pass
    for (int i = 0; i < bodyCount; ++i)
    {
        NativeSoftBodyData& b = bodies[i];
        if (b.nodeCount == 0) continue;

        ResolveStaticCollisionsCCD_Internal(
            b.predicted, b.current, b.masses, b.isPinned, b.nodeCount,
            colliders, colliderCount,
            localStaticGridTable.data(), staticTableSize,
            b.radius + PhysicsConstants::SKIN_WIDTH, epsilon, staticGridCellSize,
            b.restitution, b.staticFriction, b.slidingFriction, dt);
    }

    std::vector<CollisionPair> pairs;
    std::vector<Vector3> cPred, cCurr;
    std::vector<float>   cMass;
    std::vector<uint8_t> cPinned;
    pairs.reserve(4096);
    cPred.reserve(1024);
    cCurr.reserve(1024);
    cMass.reserve(1024);
    cPinned.reserve(1024);

    for (int i = 0; i < bodyCount; ++i)
    {
        NativeSoftBodyData& bA = bodies[i];
        if (bA.nodeCount == 0) continue;

        for (int j = i + 1; j < bodyCount; ++j)
        {
            NativeSoftBodyData& bB = bodies[j];
            if (bB.nodeCount == 0) continue;

            if (((1 << bB.layer) & bA.collisionLayerMask) == 0) continue;
            if (((1 << bA.layer) & bB.collisionLayerMask) == 0) continue;

            uint64_t idA = bA.id, idB = bB.id;
            if (localIgnored.count((idA < idB) ? (idA | (idB << 32)) : (idB | (idA << 32)))) continue;

            float exp = bA.radius + bB.radius + epsilon;
            if (bA.boundsMin.x > bB.boundsMax.x + exp || bA.boundsMax.x < bB.boundsMin.x - exp ||
                bA.boundsMin.y > bB.boundsMax.y + exp || bA.boundsMax.y < bB.boundsMin.y - exp ||
                bA.boundsMin.z > bB.boundsMax.z + exp || bA.boundsMax.z < bB.boundsMin.z - exp) continue;

            int maxPairs = bA.nodeCount * bB.nodeCount;
            pairs.resize(maxPairs);

            int found = BuildNodeNodePairs(
                bA.predicted, bA.nodeCount, bB.predicted, bB.nodeCount,
                bA.radius, bB.radius, epsilon, pairs.data(), maxPairs);

            if (found > 0)
            {
                int total = bA.nodeCount + bB.nodeCount;
                if ((int)cPred.size() < total) { cPred.resize(total); cCurr.resize(total); cMass.resize(total); }
                if ((int)cPinned.size() < total) { cPinned.resize(total); }

                memcpy(cPred.data(), bA.predicted, bA.nodeCount * sizeof(Vector3));
                memcpy(cPred.data() + bA.nodeCount, bB.predicted, bB.nodeCount * sizeof(Vector3));
                memcpy(cCurr.data(), bA.current, bA.nodeCount * sizeof(Vector3));
                memcpy(cCurr.data() + bA.nodeCount, bB.current, bB.nodeCount * sizeof(Vector3));
                memcpy(cMass.data(), bA.masses, bA.nodeCount * sizeof(float));
                memcpy(cMass.data() + bA.nodeCount, bB.masses, bB.nodeCount * sizeof(float));

                for (int k = 0; k < bA.nodeCount; ++k) cPinned[k] = bA.isPinned[k];
                for (int k = 0; k < bB.nodeCount; ++k) cPinned[bA.nodeCount + k] = bB.isPinned[k];

                ResolveNodePairs(cPred.data(), cCurr.data(), cMass.data(), cPinned.data(),
                    pairs.data(), found, dt, epsilon,
                    bA.radius, bB.radius,
                    (bA.restitution + bB.restitution) * 0.5f,
                    (bA.staticFriction + bB.staticFriction) * 0.5f,
                    (bA.slidingFriction + bB.slidingFriction) * 0.5f);

                memcpy(bA.predicted, cPred.data(), bA.nodeCount * sizeof(Vector3));
                memcpy(bB.predicted, cPred.data() + bA.nodeCount, bB.nodeCount * sizeof(Vector3));
            }

            // FIX: bA.current (not bA.previous) — both face passes now use the same timestep
            if (bA.faceCount > 0)
            {
                ResolveFaceToNode(
                    bB.predicted, bB.current, bB.masses, bB.isPinned, bB.nodeCount,
                    bA.predicted, bA.current, bA.masses, bA.isPinned, bA.nodeCount,
                    bA.faces, bA.faceCount,
                    bB.radius + PhysicsConstants::SKIN_WIDTH,
                    bB.radius + PhysicsConstants::SKIN_WIDTH + PhysicsConstants::DEFAULT_TRIANGLE_THICKNESS,
                    epsilon, dt, faceNodeCellSize);
            }
            if (bB.faceCount > 0)
            {
                ResolveFaceToNode(
                    bA.predicted, bA.current, bA.masses, bA.isPinned, bA.nodeCount,
                    bB.predicted, bB.current, bB.masses, bB.isPinned, bB.nodeCount,
                    bB.faces, bB.faceCount,
                    bA.radius + PhysicsConstants::SKIN_WIDTH,
                    bA.radius + PhysicsConstants::SKIN_WIDTH + PhysicsConstants::DEFAULT_TRIANGLE_THICKNESS,
                    epsilon, dt, faceNodeCellSize);
            }
        }
    }
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
        // Already penetrating — verify over triangle before accepting
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
    Vector3* nodePred, Vector3* facePred,
    const float* nodeMasses, const float* faceMasses,
    const uint8_t * nodePinned, const uint8_t* facePinned)
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

    if (!nodePinned[idxP]) nodePred[idxP] = Add(nodePred[idxP], Scale(corr, wP));
    if (!facePinned[idxA]) facePred[idxA] = Sub(facePred[idxA], Scale(corr, wA * u));
    if (!facePinned[idxB]) facePred[idxB] = Sub(facePred[idxB], Scale(corr, wB * v));
    if (!facePinned[idxC]) facePred[idxC] = Sub(facePred[idxC], Scale(corr, wC * w));
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
    if      (dx <= dy && dx <= dz) ln.x = ls.x >= 0.0f ? 1.0f : -1.0f;
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

    // Degenerate capsule — treat as sphere
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