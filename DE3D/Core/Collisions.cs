/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D                                          ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using System;
using System.Collections.Generic;

namespace DynamicEngine
{
    public struct ContactCache
    {
        public float3 normal;
        public float penetration;
        public int frameLastSeen;
    }

    public struct CombinedMatterProps
    {
        public float StaticFriction;
        public float SlidingFriction;
    }

    public enum ColliderType : byte
    {
        Sphere,
        Box,
        Capsule,
        Triangle // Replaces Mesh/Terrain for Burst context
    }

    public struct StaticColliderData
    {
        public ColliderType Type;
        public float3 Center;
        public float3 Size;
        public float3 Axis;
        public float Radius;
        public float Height;
        public float3 V0, V1, V2;
        public quaternion Rotation;
        public float StaticFriction;
        public float SlidingFriction;
        public float Restitution;
        public int Layer;
    }

    public struct FaceData
    {
        public int nodeA;
        public int nodeB;
        public int nodeC;
    }

    public class CollisionHandler : IDisposable
    {
        #region Fields

        private NativeHashMap<int, ContactCache> _contactCache;
        private int _frameCount = 0;

        private NativeHashSet<int2> _ignoredBodyPairs;
        private NativeHashSet<int> _hardExcludedBodyIds;
        private LayerMask _collisionLayerMask = ~0;

        private const float SPATIAL_CELL_SIZE = 0.15f;
        private const float FACE_NODE_CELL_SIZE = 0.3f;
        private const float STATIC_GRID_CELL_SIZE = 2.0f;

        private NativeList<int> _cacheKeysToRemove;

        public NativeList<float3> collisionPoints;
        public bool enableDebugLogging = false;
        public float triangleThickness = 0.02f;

        private const float SKIN_WIDTH = 0.005f;

        // Unmanaged Static Collider Cache
        private NativeList<StaticColliderData> _staticColliders;
        private NativeParallelMultiHashMap<int, int> _staticGrid;
        private float _lastStaticColliderRefreshTime;
        private const float STATIC_COLLIDER_REFRESH_INTERVAL = 1.0f;

        private bool _isDisposed;

        #endregion

        #region Properties
        public LayerMask CollisionLayerMask
        {
            get => _collisionLayerMask;
            set => _collisionLayerMask = value;
        }
        #endregion

        #region Constructor & Disposal
        public CollisionHandler()
        {
            _contactCache = new NativeHashMap<int, ContactCache>(1024, Allocator.Persistent);
            _ignoredBodyPairs = new NativeHashSet<int2>(256, Allocator.Persistent);
            _hardExcludedBodyIds = new NativeHashSet<int>(128, Allocator.Persistent);
            _cacheKeysToRemove = new NativeList<int>(256, Allocator.Persistent);
            collisionPoints = new NativeList<float3>(1024, Allocator.Persistent);

            _staticColliders = new NativeList<StaticColliderData>(256, Allocator.Persistent);
            _staticGrid = new NativeParallelMultiHashMap<int, int>(1024, Allocator.Persistent);
        }

        public void Dispose()
        {
            if (_isDisposed) return;
            _contactCache.Dispose();
            _ignoredBodyPairs.Dispose();
            _hardExcludedBodyIds.Dispose();
            _cacheKeysToRemove.Dispose();
            collisionPoints.Dispose();
            _staticColliders.Dispose();
            _staticGrid.Dispose();
            _isDisposed = true;
        }
        #endregion

        #region Main Resolve Method

        public void ResolveCollisions(NodeManager nodeManager, Transform owner, Matter matterAsset, List<float> nodeMasses, float dt, float epsilon, bool visualizeForces)
        {
            if (!ValidateCollisionParameters(nodeManager, owner, nodeMasses)) return;

            _frameCount++;
            if (_frameCount % 60 == 0) CleanCache();

            collisionPoints.Clear();

            ResolveCrossBodyCollisions(nodeManager, owner, matterAsset, nodeMasses, dt, epsilon, visualizeForces);
            ResolveStaticCollisions(nodeManager, owner, matterAsset, nodeMasses, dt, epsilon, visualizeForces);
        }

        private void CleanCache()
        {
            _cacheKeysToRemove.Clear();
            using (var keys = _contactCache.GetKeyArray(Allocator.Temp))
            {
                foreach (int key in keys)
                {
                    if (_contactCache.TryGetValue(key, out ContactCache cache) && (_frameCount - cache.frameLastSeen > 5))
                    {
                        _cacheKeysToRemove.Add(key);
                    }
                }
            }

            foreach (int key in _cacheKeysToRemove)
            {
                _contactCache.Remove(key);
            }
        }

        #endregion

        #region Broad Phase Cross-Body

        private void ResolveCrossBodyCollisions(NodeManager nodeManagerA, Transform ownerA, Matter matterAssetA, List<float> massesA, float dt, float epsilon, bool visualizeForces)
        {
            Bounds boundsA = CalculateBodyBounds(nodeManagerA);
            float radiusA = nodeManagerA.GetNodeRadius();
            int ownerAId = ownerA.GetInstanceID();

            foreach (var bodyB in SoftBody.AllSoftBodies)
            {
                if (bodyB == null || bodyB.transform == ownerA || !bodyB.enabled) continue;

                int ownerBId = bodyB.transform.GetInstanceID();

                if (_hardExcludedBodyIds.Contains(ownerBId)) continue;

                int2 pair = ownerAId < ownerBId ? new int2(ownerAId, ownerBId) : new int2(ownerBId, ownerAId);
                if (_ignoredBodyPairs.Contains(pair)) continue;

                if (((1 << bodyB.gameObject.layer) & _collisionLayerMask) == 0) continue;

                NodeManager nodeManagerB = bodyB.solver.nodeManager;
                if (nodeManagerB == null) continue;

                Bounds boundsB = CalculateBodyBounds(nodeManagerB);
                boundsB.Expand(radiusA * 2f + epsilon);

                if (!boundsA.Intersects(boundsB)) continue;

                ResolveNodeToNodeOptimized(nodeManagerA, nodeManagerB, massesA, bodyB.solver.nodeMasses, matterAssetA, bodyB.matter, dt, epsilon, visualizeForces);
                ResolveFaceToNode(nodeManagerA, massesA, bodyB, nodeManagerB, bodyB.solver.nodeMasses, matterAssetA, bodyB.matter, dt, epsilon, visualizeForces);

                SoftBody bodyA = ownerA.GetComponent<SoftBody>();
                if (bodyA != null && bodyA.truss != null)
                {
                    ResolveFaceToNode(nodeManagerB, bodyB.solver.nodeMasses, bodyA, nodeManagerA, massesA, bodyB.matter, matterAssetA, dt, epsilon, visualizeForces);
                }
            }
        }

        private Bounds CalculateBodyBounds(NodeManager nm)
        {
            int count = nm.PredictedPositions.Count;
            if (count == 0) return new Bounds();

            Vector3 min = nm.PredictedPositions[0];
            Vector3 max = nm.PredictedPositions[0];

            for (int i = 1; i < count; i++)
            {
                Vector3 p = nm.PredictedPositions[i];
                if (p.x < min.x) min.x = p.x; else if (p.x > max.x) max.x = p.x;
                if (p.y < min.y) min.y = p.y; else if (p.y > max.y) max.y = p.y;
                if (p.z < min.z) min.z = p.z; else if (p.z > max.z) max.z = p.z;
            }

            Bounds b = new Bounds();
            b.SetMinMax(min, max);
            return b;
        }

        #endregion

        #region Burst Jobs 

        [BurstCompile(CompileSynchronously = true, OptimizeFor = OptimizeFor.Performance)]
        private struct NodeToNodeCollisionJob : IJob
        {
            public NativeArray<float3> predictedPosA;
            public NativeArray<float3> predictedPosB;
            public NativeArray<float3> currentPosA;
            public NativeArray<float3> currentPosB;
            [ReadOnly] public NativeArray<float> massesA;
            [ReadOnly] public NativeArray<float> massesB;
            public NativeQueue<float3>.ParallelWriter collisionPointsOut;

            public float nodeRadiusA;
            public float nodeRadiusB;
            public float epsilon;
            public float dt;
            public float restitution;
            public float staticFriction;
            public float kineticFriction;
            public float spatialCellSize;

            public void Execute()
            {
                float combinedRadius = nodeRadiusA + nodeRadiusB;
                float threshold = combinedRadius + epsilon;
                float thresholdSq = threshold * threshold;

                var hashMap = new NativeParallelMultiHashMap<int, int>(predictedPosB.Length, Allocator.Temp);
                for (int i = 0; i < predictedPosB.Length; i++)
                {
                    int3 gridPos = new int3(math.floor(predictedPosB[i] / spatialCellSize));
                    int hash = (gridPos.x * 73856093) ^ (gridPos.y * 19349663) ^ (gridPos.z * 83492791);
                    hashMap.Add(hash, i);
                }

                for (int i = 0; i < predictedPosA.Length; i++)
                {
                    float3 posA = predictedPosA[i];
                    int3 gridPos0 = new int3(math.floor(posA / spatialCellSize));
                    int range = (int)math.ceil(threshold / spatialCellSize);

                    for (int dx = -range; dx <= range; dx++)
                    {
                        for (int dy = -range; dy <= range; dy++)
                        {
                            for (int dz = -range; dz <= range; dz++)
                            {
                                int3 gridPos = gridPos0 + new int3(dx, dy, dz);
                                int hash = (gridPos.x * 73856093) ^ (gridPos.y * 19349663) ^ (gridPos.z * 83492791);

                                if (hashMap.TryGetFirstValue(hash, out int j, out NativeParallelMultiHashMapIterator<int> it))
                                {
                                    do
                                    {
                                        float3 posB = predictedPosB[j];
                                        float3 delta = posA - posB;
                                        float distSq = math.lengthsq(delta);

                                        if (distSq < thresholdSq && distSq > 1e-9f)
                                        {
                                            float dist = math.sqrt(distSq);
                                            float penetrationDepth = threshold - dist;
                                            float3 normal = delta / dist;

                                            float massA = massesA[i];
                                            float massB = massesB[j];
                                            float totalMass = massA + massB;
                                            float ratioA = massB / totalMass;
                                            float ratioB = massA / totalMass;

                                            // FIX: Calculate Pre-Velocities BEFORE modifying predicted positions
                                            float3 preVelA = (posA - currentPosA[i]) / dt;
                                            float3 preVelB = (predictedPosB[j] - currentPosB[j]) / dt;
                                            float3 preRelVel = preVelA - preVelB;
                                            float preNormalVel = math.dot(preRelVel, normal);

                                            // Apply positional projection
                                            predictedPosA[i] += normal * (penetrationDepth * ratioA + epsilon * 0.5f);
                                            predictedPosB[j] -= normal * (penetrationDepth * ratioB + epsilon * 0.5f);

                                            // FIX: Calculate Post-Velocities for accurate Tangent Friction
                                            float3 postVelA = (predictedPosA[i] - currentPosA[i]) / dt;
                                            float3 postVelB = (predictedPosB[j] - currentPosB[j]) / dt;
                                            float3 postRelVel = postVelA - postVelB;

                                            // FIX: Only trigger physics logic if nodes are moving towards each other OR penetrating
                                            if (preNormalVel < 0 || penetrationDepth > 0)
                                            {
                                                float invMassA = 1f / massA;
                                                float invMassB = 1f / massB;
                                                float invMassSum = invMassA + invMassB;
                                                float impactMultiplier = 1f + (math.abs(massA - massB) / math.max(massA, massB)) * 0.5f;

                                                float jn = preNormalVel < 0 ? -(1f + restitution) * preNormalVel / invMassSum * impactMultiplier : 0f;
                                                float positionalImpulseMag = (penetrationDepth / dt) / invMassSum;
                                                float effectiveJn = jn + positionalImpulseMag;

                                                float currentNormalVel = math.dot(postRelVel, normal);
                                                float3 tangentVel = postRelVel - normal * currentNormalVel;
                                                float3 frictionImpulse = float3.zero;

                                                if (math.length(tangentVel) > 0.001f)
                                                {
                                                    float stopImpulseMagnitude = math.length(tangentVel) / invMassSum;
                                                    float maxStaticMagnitude = staticFriction * effectiveJn;

                                                    float frictionMagnitude;
                                                    if (stopImpulseMagnitude <= maxStaticMagnitude)
                                                        frictionMagnitude = stopImpulseMagnitude;
                                                    else
                                                        frictionMagnitude = math.min(kineticFriction * effectiveJn, stopImpulseMagnitude);

                                                    frictionImpulse = -math.normalize(tangentVel) * frictionMagnitude;
                                                }

                                                float3 normalImpulse = preNormalVel < 0 ? normal * jn : float3.zero;
                                                float3 totalImpulse = normalImpulse + frictionImpulse;

                                                currentPosA[i] -= (totalImpulse * invMassA) * dt;
                                                currentPosB[j] += (totalImpulse * invMassB) * dt;
                                            }

                                            collisionPointsOut.Enqueue(posA - normal * nodeRadiusA);
                                            posA = predictedPosA[i]; // Update loop's local posA reference
                                        }
                                    } while (hashMap.TryGetNextValue(out j, ref it));
                                }
                            }
                        }
                    }
                }
                hashMap.Dispose();
            }
        }
        #endregion

        #region Narrow Phase Node-to-Node

        private void ResolveNodeToNodeOptimized(NodeManager nmA, NodeManager nmB, List<float> massesA, List<float> massesB,
                                        Matter matterA, Matter matterB, float dt, float epsilon, bool visualizeForces)
        {
            int countA = nmA.PredictedPositions.Count;
            int countB = nmB.PredictedPositions.Count;
            if (countA == 0 || countB == 0) return;

            NativeArray<float3> predA = new NativeArray<float3>(countA, Allocator.TempJob);
            NativeArray<float3> predB = new NativeArray<float3>(countB, Allocator.TempJob);
            NativeArray<float3> currA = new NativeArray<float3>(countA, Allocator.TempJob);
            NativeArray<float3> currB = new NativeArray<float3>(countB, Allocator.TempJob);
            NativeArray<float> massA = new NativeArray<float>(countA, Allocator.TempJob);
            NativeArray<float> massB = new NativeArray<float>(countB, Allocator.TempJob);
            NativeQueue<float3> colQueue = new NativeQueue<float3>(Allocator.TempJob);

            for (int i = 0; i < countA; i++)
            {
                predA[i] = nmA.PredictedPositions[i];
                currA[i] = nmA.GetPosition(i);
                massA[i] = massesA[i];
            }
            for (int i = 0; i < countB; i++)
            {
                predB[i] = nmB.PredictedPositions[i];
                currB[i] = nmB.GetPosition(i);
                massB[i] = massesB[i];
            }

            float restA = matterA != null ? matterA.Restitution : 0.5f;
            float restB = matterB != null ? matterB.Restitution : 0.5f;
            float sfA = matterA != null ? matterA.StaticFriction : 0.5f;
            float kfA = matterA != null ? matterA.SlidingFriction : 0.4f;
            float sfB = matterB != null ? matterB.StaticFriction : 0.5f;
            float kfB = matterB != null ? matterB.SlidingFriction : 0.4f;

            var job = new NodeToNodeCollisionJob
            {
                predictedPosA = predA,
                predictedPosB = predB,
                currentPosA = currA,
                currentPosB = currB,
                massesA = massA,
                massesB = massB,
                collisionPointsOut = colQueue.AsParallelWriter(),
                nodeRadiusA = nmA.GetNodeRadius(),
                nodeRadiusB = nmB.GetNodeRadius(),
                epsilon = epsilon,
                dt = dt,
                restitution = (restA + restB) * 0.5f,
                staticFriction = (sfA + sfB) * 0.5f,
                kineticFriction = (kfA + kfB) * 0.5f,
                spatialCellSize = SPATIAL_CELL_SIZE
            };

            job.Schedule().Complete();

            for (int i = 0; i < countA; i++)
            {
                nmA.PredictedPositions[i] = predA[i];
                nmA.SetCurrentPosition(i, currA[i]);
            }
            for (int i = 0; i < countB; i++)
            {
                nmB.PredictedPositions[i] = predB[i];
                nmB.SetCurrentPosition(i, currB[i]);
            }

            while (colQueue.TryDequeue(out float3 pt))
            {
                collisionPoints.Add(pt);
            }

            predA.Dispose();
            predB.Dispose();
            currA.Dispose();
            currB.Dispose();
            massA.Dispose();
            massB.Dispose();
            colQueue.Dispose();
        }

        #endregion

        #region Face-to-Node Collision (Jobified CCD)

        [BurstCompile(CompileSynchronously = true, OptimizeFor = OptimizeFor.Performance)]
        private struct FaceToNodeCollisionJob : IJob
        {
            public NativeArray<float3> nodePredPositions;
            [ReadOnly] public NativeArray<float3> nodePrevPositions;
            [ReadOnly] public NativeArray<float> nodeMasses;
            [ReadOnly] public NativeArray<bool> nodeIsPinned;

            public NativeArray<float3> facePredPositions;
            [ReadOnly] public NativeArray<float3> facePrevPositions;
            [ReadOnly] public NativeArray<float> faceMasses;
            [ReadOnly] public NativeArray<bool> faceIsPinned;
            [ReadOnly] public NativeArray<FaceData> faces;

            public NativeQueue<float3>.ParallelWriter collisionPointsOut;

            public float radiusNode;
            public float thickness;
            public float epsilon;
            public float dt;
            public float faceNodeCellSize;

            public void Execute()
            {
                int nodeCount = nodePredPositions.Length;
                var fnHashMap = new NativeParallelMultiHashMap<int, int>(nodeCount * 2, Allocator.Temp);

                for (int n = 0; n < nodeCount; n++)
                {
                    int3 gridPosS = new int3(math.floor(nodePrevPositions[n] / faceNodeCellSize));
                    int hashS = (gridPosS.x * 73856093) ^ (gridPosS.y * 19349663) ^ (gridPosS.z * 83492791);
                    fnHashMap.Add(hashS, n);

                    int3 gridPosE = new int3(math.floor(nodePredPositions[n] / faceNodeCellSize));
                    int hashE = (gridPosE.x * 73856093) ^ (gridPosE.y * 19349663) ^ (gridPosE.z * 83492791);
                    fnHashMap.Add(hashE, n);
                }

                NativeList<int> faceNodeCandidates = new NativeList<int>(64, Allocator.Temp);
                NativeArray<bool> nodeVisitedFlags = new NativeArray<bool>(nodeCount, Allocator.Temp);

                for (int f = 0; f < faces.Length; f++)
                {
                    FaceData face = faces[f];
                    float3 v0s = facePrevPositions[face.nodeA];
                    float3 v1s = facePrevPositions[face.nodeB];
                    float3 v2s = facePrevPositions[face.nodeC];
                    float3 v0e = facePredPositions[face.nodeA];
                    float3 v1e = facePredPositions[face.nodeB];
                    float3 v2e = facePredPositions[face.nodeC];

                    float bMinX = math.min(math.min(v0s.x, v1s.x), math.min(v2s.x, math.min(math.min(v0e.x, v1e.x), v2e.x))) - thickness;
                    float bMinY = math.min(math.min(v0s.y, v1s.y), math.min(v2s.y, math.min(math.min(v0e.y, v1e.y), v2e.y))) - thickness;
                    float bMinZ = math.min(math.min(v0s.z, v1s.z), math.min(v2s.z, math.min(math.min(v0e.z, v1e.z), v2e.z))) - thickness;
                    float bMaxX = math.max(math.max(v0s.x, v1s.x), math.max(v2s.x, math.max(math.max(v0e.x, v1e.x), v2e.x))) + thickness;
                    float bMaxY = math.max(math.max(v0s.y, v1s.y), math.max(v2s.y, math.max(math.max(v0e.y, v1e.y), v2e.y))) + thickness;
                    float bMaxZ = math.max(math.max(v0s.z, v1s.z), math.max(v2s.z, math.max(math.max(v0e.z, v1e.z), v2e.z))) + thickness;

                    faceNodeCandidates.Clear();
                    int x0 = (int)math.floor(bMinX / faceNodeCellSize);
                    int y0 = (int)math.floor(bMinY / faceNodeCellSize);
                    int z0 = (int)math.floor(bMinZ / faceNodeCellSize);
                    int x1 = (int)math.floor(bMaxX / faceNodeCellSize);
                    int y1 = (int)math.floor(bMaxY / faceNodeCellSize);
                    int z1 = (int)math.floor(bMaxZ / faceNodeCellSize);

                    for (int x = x0; x <= x1; x++)
                        for (int y = y0; y <= y1; y++)
                            for (int z = z0; z <= z1; z++)
                            {
                                int key = (x * 73856093) ^ (y * 19349663) ^ (z * 83492791);
                                if (fnHashMap.TryGetFirstValue(key, out int n, out NativeParallelMultiHashMapIterator<int> it))
                                {
                                    do
                                    {
                                        if (!nodeVisitedFlags[n])
                                        {
                                            nodeVisitedFlags[n] = true;
                                            faceNodeCandidates.Add(n);
                                        }
                                    } while (fnHashMap.TryGetNextValue(out n, ref it));
                                }
                            }

                    for (int c = 0; c < faceNodeCandidates.Length; c++)
                    {
                        int n = faceNodeCandidates[c];
                        nodeVisitedFlags[n] = false;

                        float3 nodeStart = nodePrevPositions[n];
                        float3 nodeEnd = nodePredPositions[n];

                        float nsX = math.min(nodeStart.x, nodeEnd.x);
                        float nsY = math.min(nodeStart.y, nodeEnd.y);
                        float nsZ = math.min(nodeStart.z, nodeEnd.z);
                        float neX = math.max(nodeStart.x, nodeEnd.x);
                        float neY = math.max(nodeStart.y, nodeEnd.y);
                        float neZ = math.max(nodeStart.z, nodeEnd.z);

                        if (nsX > bMaxX || neX < bMinX || nsY > bMaxY || neY < bMinY || nsZ > bMaxZ || neZ < bMinZ) continue;

                        if (CheckSweptFaceNode(nodeStart, nodeEnd, radiusNode, v0s, v0e, v1s, v1e, v2s, v2e, out float t, out float3 hitNormal))
                        {
                            float3 pAtT = math.lerp(nodeStart, nodeEnd, t);
                            float3 aAtT = math.lerp(v0s, v0e, t);
                            float3 bAtT = math.lerp(v1s, v1e, t);
                            float3 cAtT = math.lerp(v2s, v2e, t);

                            if (GetBarycentric(pAtT, aAtT, bAtT, cAtT, out float u, out float v, out float w))
                            {
                                float3 triPoint = u * v0e + v * v1e + w * v2e;
                                float dist = math.dot(nodeEnd - triPoint, hitNormal);
                                float penetration = (radiusNode + epsilon) - dist;

                                if (penetration > 0 || t < 1.0f)
                                {
                                    penetration = math.max(penetration, epsilon);
                                    ApplyXPBDFaceNode(n, face.nodeA, face.nodeB, face.nodeC, hitNormal, penetration, u, v, w);
                                    collisionPointsOut.Enqueue(pAtT);
                                }
                            }
                        }
                    }
                }
                fnHashMap.Dispose();
                faceNodeCandidates.Dispose();
                nodeVisitedFlags.Dispose();
            }

            private bool CheckSweptFaceNode(float3 p0, float3 p1, float radius, float3 a0, float3 a1, float3 b0, float3 b1, float3 c0, float3 c1, out float t, out float3 normal)
            {
                t = 1.0f;
                normal = math.normalize(math.cross(b0 - a0, c0 - a0));

                float3 tri0 = (a0 + b0 + c0) / 3f;
                float3 relMove = (p1 - p0) - ((a1 + b1 + c1) / 3f - tri0);

                float d0 = math.dot(p0 - tri0, normal);
                float d1 = math.dot(p0 + relMove - tri0, normal);

                if (math.sign(d0) != math.sign(d1))
                {
                    t = math.clamp(math.abs(d0) / (math.abs(d0) + math.abs(d1)), 0f, 1f);
                    if (d0 < 0) normal = -normal;
                    return true;
                }

                if (math.abs(d0) < radius) { t = 0; return true; }
                return false;
            }

            private bool GetBarycentric(float3 p, float3 a, float3 b, float3 c, out float u, out float v, out float w)
            {
                float3 v0 = b - a, v1 = c - a, v2 = p - a;
                float d00 = math.dot(v0, v0), d01 = math.dot(v0, v1), d11 = math.dot(v1, v1);
                float d20 = math.dot(v2, v0), d21 = math.dot(v2, v1);
                float denom = d00 * d11 - d01 * d01;
                if (math.abs(denom) < 1e-9f) { u = v = w = 0; return false; }
                v = (d11 * d20 - d01 * d21) / denom;
                w = (d00 * d21 - d01 * d20) / denom;
                u = 1.0f - v - w;
                return (u >= -0.05f && v >= -0.05f && w >= -0.05f && (u + v + w) <= 1.05f);
            }

            private void ApplyXPBDFaceNode(int idxP, int idxA, int idxB, int idxC, float3 normal, float penetration, float u, float v, float w)
            {
                float wP = nodeIsPinned[idxP] ? 0f : 1f / nodeMasses[idxP];
                float wA = faceIsPinned[idxA] ? 0f : 1f / faceMasses[idxA];
                float wB = faceIsPinned[idxB] ? 0f : 1f / faceMasses[idxB];
                float wC = faceIsPinned[idxC] ? 0f : 1f / faceMasses[idxC];

                float wTri = (u * u * wA) + (v * v * wB) + (w * w * wC);
                float wSum = wP + wTri;
                if (wSum <= 1e-9f) return;

                float dLambda = penetration / wSum;
                float3 correction = dLambda * normal;

                if (!nodeIsPinned[idxP]) nodePredPositions[idxP] += wP * correction;
                if (!faceIsPinned[idxA]) facePredPositions[idxA] -= wA * u * correction;
                if (!faceIsPinned[idxB]) facePredPositions[idxB] -= wB * v * correction;
                if (!faceIsPinned[idxC]) facePredPositions[idxC] -= wC * w * correction;
            }
        }

        private void ResolveFaceToNode(NodeManager nmNode, List<float> massesNode, SoftBody bodyFace,
                                       NodeManager nmFace, List<float> massesFace, Matter matterNode,
                                       Matter matterFace, float dt, float epsilon, bool visualizeForces)
        {
            Truss truss = bodyFace.truss;
            if (truss == null) return;

            List<Face> facesList = truss.GetTrussFaces();
            if (facesList == null || facesList.Count == 0) return;

            int nodeCount = nmNode.PredictedPositions.Count;
            if (nodeCount == 0) return;

            NativeArray<float3> nPred = new NativeArray<float3>(nodeCount, Allocator.TempJob);
            NativeArray<float3> nPrev = new NativeArray<float3>(nodeCount, Allocator.TempJob);
            NativeArray<float> nMass = new NativeArray<float>(nodeCount, Allocator.TempJob);
            NativeArray<bool> nPinned = new NativeArray<bool>(nodeCount, Allocator.TempJob);

            for (int i = 0; i < nodeCount; i++)
            {
                nPred[i] = nmNode.PredictedPositions[i];
                nPrev[i] = nmNode.PreviousPositions[i];
                nMass[i] = massesNode[i];
                nPinned[i] = nmNode.IsPinned[i];
            }

            int faceNodeCount = nmFace.PredictedPositions.Count;
            NativeArray<float3> fPred = new NativeArray<float3>(faceNodeCount, Allocator.TempJob);
            NativeArray<float3> fPrev = new NativeArray<float3>(faceNodeCount, Allocator.TempJob);
            NativeArray<float> fMass = new NativeArray<float>(faceNodeCount, Allocator.TempJob);
            NativeArray<bool> fPinned = new NativeArray<bool>(faceNodeCount, Allocator.TempJob);

            for (int i = 0; i < faceNodeCount; i++)
            {
                fPred[i] = nmFace.PredictedPositions[i];
                fPrev[i] = nmFace.PreviousPositions[i];
                fMass[i] = massesFace[i];
                fPinned[i] = nmFace.IsPinned[i];
            }

            NativeArray<FaceData> fData = new NativeArray<FaceData>(facesList.Count, Allocator.TempJob);
            for (int i = 0; i < facesList.Count; i++)
            {
                fData[i] = new FaceData { nodeA = facesList[i].nodeA, nodeB = facesList[i].nodeB, nodeC = facesList[i].nodeC };
            }

            NativeQueue<float3> colQueue = new NativeQueue<float3>(Allocator.TempJob);

            var job = new FaceToNodeCollisionJob
            {
                nodePredPositions = nPred,
                nodePrevPositions = nPrev,
                nodeMasses = nMass,
                nodeIsPinned = nPinned,
                facePredPositions = fPred,
                facePrevPositions = fPrev,
                faceMasses = fMass,
                faceIsPinned = fPinned,
                faces = fData,
                collisionPointsOut = colQueue.AsParallelWriter(),
                radiusNode = nmNode.GetNodeRadius() + SKIN_WIDTH,
                thickness = nmNode.GetNodeRadius() + SKIN_WIDTH + triangleThickness,
                epsilon = epsilon,
                dt = dt,
                faceNodeCellSize = FACE_NODE_CELL_SIZE
            };

            job.Schedule().Complete();

            for (int i = 0; i < nodeCount; i++) nmNode.PredictedPositions[i] = nPred[i];
            for (int i = 0; i < faceNodeCount; i++) nmFace.PredictedPositions[i] = fPred[i];

            while (colQueue.TryDequeue(out float3 pt)) collisionPoints.Add(pt);

            nPred.Dispose(); nPrev.Dispose(); nMass.Dispose(); nPinned.Dispose();
            fPred.Dispose(); fPrev.Dispose(); fMass.Dispose(); fPinned.Dispose();
            fData.Dispose(); colQueue.Dispose();
        }

        #endregion

        #region Custom Static Collision Detection with CCD (Burst)

        [BurstCompile(CompileSynchronously = true, OptimizeFor = OptimizeFor.Performance)]
        private struct StaticCollisionJob : IJobParallelFor
        {
            public NativeArray<float3> currentPositions;
            public NativeArray<float3> predictedPositions;
            [ReadOnly] public NativeArray<float> nodeMasses;
            [ReadOnly] public NativeArray<StaticColliderData> staticColliders;
            [ReadOnly] public NativeParallelMultiHashMap<int, int> staticGrid;

            public float radius;
            public float dt;
            public float epsilon;
            public float gridCellSize;
            public float defaultRestitution;
            public float defaultStaticFriction;
            public float defaultSlidingFriction;

            public void Execute(int i)
            {
                float3 currentPos = currentPositions[i];
                float3 predictedPos = predictedPositions[i];
                float mass = nodeMasses[i];

                if (mass <= 0f) return; // Ignore pinned/infinite mass

                float3 sweepVelocity = predictedPos - currentPos;
                float3 expand = new float3(radius, radius, radius);
                float3 sweptMin = math.min(currentPos, predictedPos) - expand;
                float3 sweptMax = math.max(currentPos, predictedPos) + expand;

                int x0 = (int)math.floor(sweptMin.x / gridCellSize);
                int y0 = (int)math.floor(sweptMin.y / gridCellSize);
                int z0 = (int)math.floor(sweptMin.z / gridCellSize);
                int x1 = (int)math.floor(sweptMax.x / gridCellSize);
                int y1 = (int)math.floor(sweptMax.y / gridCellSize);
                int z1 = (int)math.floor(sweptMax.z / gridCellSize);

                bool foundHit = false;
                float earliestTime = float.MaxValue;
                float3 hitPoint = float3.zero;
                float3 hitNormal = float3.zero;
                StaticColliderData hitCollider = default;

                NativeHashSet<int> candidates = new NativeHashSet<int>(16, Allocator.Temp);

                for (int cx = x0; cx <= x1; cx++)
                    for (int cy = y0; cy <= y1; cy++)
                        for (int cz = z0; cz <= z1; cz++)
                        {
                            int hash = (cx * 73856093) ^ (cy * 19349663) ^ (cz * 83492791);
                            if (staticGrid.TryGetFirstValue(hash, out int idx, out NativeParallelMultiHashMapIterator<int> it))
                            {
                                do
                                {
                                    candidates.Add(idx);
                                } while (staticGrid.TryGetNextValue(out idx, ref it));
                            }
                        }

                using (var candidateArray = candidates.ToNativeArray(Allocator.Temp))
                {
                    for (int c = 0; c < candidateArray.Length; c++)
                    {
                        StaticColliderData col = staticColliders[candidateArray[c]];
                        float t = float.MaxValue;
                        float3 point = float3.zero;
                        float3 normal = new float3(0, 1, 0);

                        if (col.Type == ColliderType.Sphere) SweptSphereSphere(currentPos, sweepVelocity, radius, col, out t, out point, out normal);
                        else if (col.Type == ColliderType.Box) SweptSphereBox(currentPos, sweepVelocity, radius, col, out t, out point, out normal);
                        else if (col.Type == ColliderType.Capsule) SweptSphereCapsule(currentPos, sweepVelocity, radius, col, out t, out point, out normal);
                        else if (col.Type == ColliderType.Triangle) SweptSphereTriangle(currentPos, sweepVelocity, radius, col, out t, out point, out normal);


                        if (t >= 0f && t <= 1f && t < earliestTime)
                        {
                            earliestTime = t;
                            hitPoint = point;
                            hitNormal = normal;
                            hitCollider = col;
                            foundHit = true;
                        }
                    }
                }
                candidates.Dispose();

                if (foundHit)
                {
                    float hitTime = math.clamp(earliestTime, 0f, 1f);
                    float3 collisionPos = math.lerp(currentPos, predictedPos, hitTime);
                    float penetration = math.max(radius - math.dot(predictedPos - hitPoint, hitNormal), 0f);

                    ApplyCollisionResolution(i, mass, hitCollider, currentPos, collisionPos, hitPoint, hitNormal, penetration);
                }
            }

            // Internal Swept Tests matching mathematical models
            private bool SweptSphereSphere(float3 startPos, float3 velocity, float radiusA, StaticColliderData sphereB, out float hitTime, out float3 hitPoint, out float3 hitNormal)
            {
                hitTime = float.MaxValue; hitPoint = float3.zero; hitNormal = new float3(0, 1, 0);
                float combinedRadius = radiusA + sphereB.Radius;
                float3 p = startPos - sphereB.Center;
                float a = math.dot(velocity, velocity);
                float b = 2f * math.dot(p, velocity);
                float c = math.dot(p, p) - combinedRadius * combinedRadius;

                if (a < 1e-12f) return false;
                float discriminant = b * b - 4f * a * c;
                if (discriminant < 0) return false;

                float t = (-b - math.sqrt(discriminant)) / (2f * a);
                if (t < 0f || t > 1f) return false;

                hitTime = t;
                hitNormal = math.normalize((startPos + velocity * t) - sphereB.Center);
                hitPoint = sphereB.Center + hitNormal * sphereB.Radius;
                return true;
            }

            private bool SweptSphereBox(float3 startPos, float3 velocity, float radius, StaticColliderData box, out float hitTime, out float3 hitPoint, out float3 hitNormal)
            {
                hitTime = float.MaxValue; hitPoint = float3.zero; hitNormal = new float3(0, 1, 0);

                // Transform node into the box's local space
                quaternion invRot = math.inverse(box.Rotation);
                float3 localStart = math.mul(invRot, startPos - box.Center);
                float3 localVel = math.mul(invRot, velocity);

                float3 boxMin = -box.Size * 0.5f - new float3(radius);
                float3 boxMax = box.Size * 0.5f + new float3(radius);

                float tMin = 0f, tMax = 1f;
                int hitAxis = -1, hitSign = 0;

                for (int i = 0; i < 3; i++)
                {
                    float p = localStart[i], v = localVel[i], min = boxMin[i], max = boxMax[i];
                    if (math.abs(v) < 1e-6f)
                    {
                        if (p < min || p > max) return false;
                    }
                    else
                    {
                        float t1 = (min - p) / v, t2 = (max - p) / v;
                        if (t1 > t2) { float temp = t1; t1 = t2; t2 = temp; }
                        if (t1 > tMin) { tMin = t1; hitAxis = i; hitSign = v > 0 ? -1 : 1; }
                        if (t2 < tMax) tMax = t2;
                        if (tMin > tMax) return false;
                    }
                }

                if (tMin < 0f || tMin > 1f) return false;

                hitTime = tMin;
                float3 localNormal = float3.zero;
                if (hitAxis >= 0) localNormal[hitAxis] = hitSign;

                // Transform normal back to world space
                hitNormal = math.mul(box.Rotation, localNormal);
                hitPoint = startPos + velocity * hitTime - hitNormal * radius;
                return true;
            }

            private bool SweptSphereTriangle(float3 startPos, float3 velocity, float radius, StaticColliderData tri, out float hitTime, out float3 hitPoint, out float3 hitNormal)
            {
                hitTime = float.MaxValue; hitPoint = float3.zero; hitNormal = new float3(0, 1, 0);

                float3 edge1 = tri.V1 - tri.V0;
                float3 edge2 = tri.V2 - tri.V0;
                float3 triNormal = math.normalize(math.cross(edge1, edge2));

                float distToPlane = math.dot(startPos - tri.V0, triNormal);
                bool isFront = distToPlane > 0;
                float3 n = isFront ? triNormal : -triNormal;
                float d = isFront ? distToPlane : -distToPlane;

                float nv = math.dot(velocity, n);
                if (nv >= 0f) return false;

                float t = (d - radius) / -nv;
                if (t < 0f || t > 1f) return false;

                float3 p = startPos + velocity * t - n * radius;
                float3 v0p = p - tri.V0;

                float dot00 = math.dot(edge1, edge1);
                float dot01 = math.dot(edge1, edge2);
                float dot02 = math.dot(edge1, v0p);
                float dot11 = math.dot(edge2, edge2);
                float dot12 = math.dot(edge2, v0p);

                float invDenom = 1f / (dot00 * dot11 - dot01 * dot01);
                float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
                float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

                if (u >= -0.01f && v >= -0.01f && (u + v) <= 1.01f)
                {
                    hitTime = t;
                    hitNormal = n;
                    hitPoint = p;
                    return true;
                }

                return false;
            }

            private bool SweptSphereCapsule(float3 startPos, float3 velocity, float radius, StaticColliderData capsule, out float hitTime, out float3 hitPoint, out float3 hitNormal)
            {
                hitTime = float.MaxValue; hitPoint = float3.zero; hitNormal = new float3(0, 1, 0);
                float halfHeight = math.max(0, (capsule.Height * 0.5f) - capsule.Radius);
                float3 p1 = capsule.Center + capsule.Axis * halfHeight;
                float3 p2 = capsule.Center - capsule.Axis * halfHeight;

                float3 segmentDir = p2 - p1;
                float segmentLength = math.length(segmentDir);
                if (segmentLength < 0.0001f) return false;
                segmentDir /= segmentLength;

                float3 w = startPos - p1;
                float a = math.dot(velocity, velocity);
                float b = math.dot(velocity, segmentDir);
                float c = math.dot(segmentDir, segmentDir);
                float d = math.dot(velocity, w);
                float e = math.dot(segmentDir, w);
                float denom = a * c - b * b;

                if (math.abs(denom) > 1e-8f)
                {
                    float s = math.clamp((a * e - b * d) / denom, 0f, segmentLength);
                    float3 segmentPoint = p1 + segmentDir * s;
                    return SweptSphereSphere(startPos, velocity, radius, new StaticColliderData { Center = segmentPoint, Radius = capsule.Radius }, out hitTime, out hitPoint, out hitNormal);
                }
                return false;
            }

            private void ApplyCollisionResolution(int idx, float mass, StaticColliderData hitMatter, float3 currentWorldPos, float3 collisionPos, float3 contactPoint, float3 normal, float penetration)
            {
                float invMass = 1f / mass;
                float3 predicted = predictedPositions[idx];
                float3 preVel = (predicted - currentWorldPos) / dt;
                float preNormalVel = math.dot(preVel, normal);

                float3 projectedPos = predicted + normal * penetration;
                float3 postVel = (projectedPos - currentWorldPos) / dt;

                // FIX: Apply friction/bounce if moving into the surface OR if currently penetrating
                if (preNormalVel < 0f || penetration > 0f)
                {
                    const float BOUNCE_THRESHOLD = 0.2f;
                    float restA = (preNormalVel < -BOUNCE_THRESHOLD) ? defaultRestitution : 0f;
                    float bounceVel = restA > 0f ? -preNormalVel * restA : 0f;

                    float currentNormalVel = math.dot(postVel, normal);
                    float targetNormalVel = math.max(currentNormalVel, bounceVel);
                    postVel += normal * (targetNormalVel - currentNormalVel);

                    // FIX: Calculate normal impulse correctly based on impact and penetration
                    float jn = preNormalVel < 0f ? -(1f + restA) * preNormalVel / invMass : 0f;
                    float positionalImpulseMag = (penetration / dt) / invMass;
                    float effectiveJn = jn + positionalImpulseMag;

                    float sf = (defaultStaticFriction + hitMatter.StaticFriction) * 0.5f;
                    float kf = (defaultSlidingFriction + hitMatter.SlidingFriction) * 0.5f;

                    float3 velTan = postVel - math.dot(postVel, normal) * normal;
                    float tangSpeed = math.length(velTan);

                    if (tangSpeed > 0.001f)
                    {
                        float3 tangDir = velTan / tangSpeed;
                        float jt = tangSpeed / invMass;
                        float frictionImpulse = jt <= sf * effectiveJn ? jt : math.min(kf * effectiveJn, jt);

                        // FIX: Removed the hardcoded 'alpha' logic that was killing the friction impulse
                        postVel -= tangDir * (frictionImpulse * invMass);
                    }
                }

                predictedPositions[idx] = currentWorldPos + postVel * dt;
            }
        }

            private void ResolveStaticCollisions(NodeManager nodeManager, Transform owner, Matter softBodyMatter, List<float> nodeMasses, float dt, float epsilon, bool visualizeForces)
        {
            RefreshStaticColliders();
            int nodeCount = nodeManager.NodeCount;
            if (nodeCount == 0 || _staticColliders.Length == 0) return;

            NativeArray<float3> currPos = new NativeArray<float3>(nodeCount, Allocator.TempJob);
            NativeArray<float3> predPos = new NativeArray<float3>(nodeCount, Allocator.TempJob);
            NativeArray<float> masses = new NativeArray<float>(nodeCount, Allocator.TempJob);

            for (int i = 0; i < nodeCount; i++)
            {
                currPos[i] = nodeManager.GetPosition(i);
                predPos[i] = nodeManager.PredictedPositions[i];
                masses[i] = nodeMasses[i];
            }

            var job = new StaticCollisionJob
            {
                currentPositions = currPos,
                predictedPositions = predPos,
                nodeMasses = masses,
                staticColliders = _staticColliders.AsArray(),
                staticGrid = _staticGrid,
                radius = nodeManager.GetNodeRadius() + SKIN_WIDTH,
                dt = dt,
                epsilon = epsilon,
                gridCellSize = STATIC_GRID_CELL_SIZE,
                defaultRestitution = softBodyMatter != null ? softBodyMatter.Restitution : 0.02f,
                defaultStaticFriction = softBodyMatter != null ? softBodyMatter.StaticFriction : 0.5f,
                defaultSlidingFriction = softBodyMatter != null ? softBodyMatter.SlidingFriction : 0.4f
            };

            job.Schedule(nodeCount, 64).Complete();

            for (int i = 0; i < nodeCount; i++)
            {
                nodeManager.PredictedPositions[i] = predPos[i];
            }

            currPos.Dispose();
            predPos.Dispose();
            masses.Dispose();
        }

        private void RefreshStaticColliders()
        {
            if (_staticColliders.Length > 0 && Time.time - _lastStaticColliderRefreshTime < STATIC_COLLIDER_REFRESH_INTERVAL) return;
            _lastStaticColliderRefreshTime = Time.time;

            _staticColliders.Clear();
            _staticGrid.Clear();

            float invCell = 1f / STATIC_GRID_CELL_SIZE; // Declare once at the top

            foreach (StaticBody sb in StaticBody.AllStaticBodies)
            {
                if (sb == null || !sb.isActiveAndEnabled) continue;

                Collider[] colliders = sb.GetComponents<Collider>();
                foreach (Collider col in colliders)
                {
                    if (col == null || !col.enabled || !col.gameObject.activeInHierarchy) continue;

                    StaticColliderData data = new StaticColliderData
                    {
                        Center = col.bounds.center,
                        StaticFriction = sb.matter != null ? sb.matter.StaticFriction : 0.5f,
                        SlidingFriction = sb.matter != null ? sb.matter.SlidingFriction : 0.4f,
                        Restitution = sb.matter != null ? sb.matter.Restitution : 0.2f,
                        Layer = col.gameObject.layer
                    };

                    Transform t = col.transform;

                    if (col is SphereCollider sphere)
                    {
                        data.Type = ColliderType.Sphere;
                        data.Radius = sphere.radius * math.max(t.lossyScale.x, math.max(t.lossyScale.y, t.lossyScale.z));
                        data.Center = t.TransformPoint(sphere.center);
                    }
                    else if (col is BoxCollider box)
                    {
                        data.Type = ColliderType.Box;
                        data.Size = (float3)box.size * (float3)t.lossyScale;
                        data.Rotation = t.rotation;
                        data.Center = t.TransformPoint(box.center);
                    }
                    else if (col is MeshCollider meshCol && meshCol.sharedMesh != null)
                    {
                        Mesh mesh = meshCol.sharedMesh;
                        Vector3[] verts = mesh.vertices;
                        int[] tris = mesh.triangles;

                        for (int i = 0; i < tris.Length; i += 3)
                        {
                            StaticColliderData triData = data;
                            triData.Type = ColliderType.Triangle;
                            triData.V0 = t.TransformPoint(verts[tris[i]]);
                            triData.V1 = t.TransformPoint(verts[tris[i + 1]]);
                            triData.V2 = t.TransformPoint(verts[tris[i + 2]]);
                            triData.Center = (triData.V0 + triData.V1 + triData.V2) / 3f;

                            _staticColliders.Add(triData);
                            int triangleIdx = _staticColliders.Length - 1; // Renamed to avoid conflict

                            Bounds triBounds = new Bounds(triData.Center, Vector3.zero);
                            triBounds.Encapsulate(triData.V0);
                            triBounds.Encapsulate(triData.V1);
                            triBounds.Encapsulate(triData.V2);
                            triBounds.Expand(0.05f);

                            int tx0 = Mathf.FloorToInt(triBounds.min.x * invCell);
                            int ty0 = Mathf.FloorToInt(triBounds.min.y * invCell);
                            int tz0 = Mathf.FloorToInt(triBounds.min.z * invCell);
                            int tx1 = Mathf.FloorToInt(triBounds.max.x * invCell);
                            int ty1 = Mathf.FloorToInt(triBounds.max.y * invCell);
                            int tz1 = Mathf.FloorToInt(triBounds.max.z * invCell);

                            for (int cx = tx0; cx <= tx1; cx++)
                                for (int cy = ty0; cy <= ty1; cy++)
                                    for (int cz = tz0; cz <= tz1; cz++)
                                    {
                                        int hash = (cx * 73856093) ^ (cy * 19349663) ^ (cz * 83492791);
                                        _staticGrid.Add(hash, triangleIdx);
                                    }
                        }
                        continue;
                    }
                    else if (col is CapsuleCollider cap)
                    {
                        data.Type = ColliderType.Capsule;
                        data.Radius = cap.radius * math.max(t.lossyScale.x, t.lossyScale.z);
                        data.Height = cap.height * t.lossyScale.y;
                        data.Center = t.TransformPoint(cap.center);
                        data.Axis = cap.direction == 0 ? t.right : (cap.direction == 2 ? t.forward : t.up);
                    }
                    else continue;

                    _staticColliders.Add(data);
                    int idx = _staticColliders.Length - 1;

                    Bounds b = col.bounds;
                    b.Expand(0.05f);

                    int x0 = Mathf.FloorToInt(b.min.x * invCell);
                    int y0 = Mathf.FloorToInt(b.min.y * invCell);
                    int z0 = Mathf.FloorToInt(b.min.z * invCell);
                    int x1 = Mathf.FloorToInt(b.max.x * invCell);
                    int y1 = Mathf.FloorToInt(b.max.y * invCell);
                    int z1 = Mathf.FloorToInt(b.max.z * invCell);

                    for (int cx = x0; cx <= x1; cx++)
                        for (int cy = y0; cy <= y1; cy++)
                            for (int cz = z0; cz <= z1; cz++)
                            {
                                int hash = (cx * 73856093) ^ (cy * 19349663) ^ (cz * 83492791);
                                _staticGrid.Add(hash, idx);
                            }
                }
            }
        }

        #endregion

        #region Utility

        private bool ValidateCollisionParameters(NodeManager nodeManager, Transform owner, List<float> nodeMasses)
        {
            if (nodeManager == null || nodeManager.NodeCount == 0) return false;
            if (nodeMasses == null || nodeMasses.Count != nodeManager.NodeCount) return false;
            return true;
        }

        public void IgnoreBodyCollision(Transform bodyA, Transform bodyB)
        {
            int idA = bodyA.GetInstanceID();
            int idB = bodyB.GetInstanceID();
            _ignoredBodyPairs.Add(idA < idB ? new int2(idA, idB) : new int2(idB, idA));
        }

        public void SetIgnoreCollision(Transform bodyA, Transform bodyB, bool ignore)
        {
            if (ignore) IgnoreBodyCollision(bodyA, bodyB);
            else RemoveIgnoredBodyCollision(bodyA, bodyB);
        }

        public void RemoveIgnoredBodyCollision(Transform bodyA, Transform bodyB)
        {
            int idA = bodyA.GetInstanceID();
            int idB = bodyB.GetInstanceID();
            _ignoredBodyPairs.Remove(idA < idB ? new int2(idA, idB) : new int2(idB, idA));
        }

        public void HardExcludeBody(Transform body)
        {
            if (body != null) _hardExcludedBodyIds.Add(body.GetInstanceID());
        }

        public void RemoveHardExcludedBody(Transform body)
        {
            if (body != null) _hardExcludedBodyIds.Remove(body.GetInstanceID());
        }

        public void ClearIgnoredPairs() => _ignoredBodyPairs.Clear();

        #endregion
    }
}