/* ╔═══════════════════════════════════════════════════════════╗
   ║  DYNAMICENGINE3D - FIXED FRICTION VERSION                 ║
   ║  AI-Assisted Soft-Body Physics for Unity3D                ║
   ║  By: Elitmers                                             ║
   ╚═══════════════════════════════════════════════════════════╝ */
using System.Collections.Generic;
using System.Linq;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace DynamicEngine
{
    #region Collision Strategy Interface
    internal interface ICollisionStrategy
    {
        bool GetSweptCollisionInfo(Collider col, Vector3 startPos, Vector3 endPos, float radius,
                                   out Vector3 contactPoint, out Vector3 normal,
                                   out float penetration, out float hitTime);
    }
    #endregion

    struct ContactCache
    {
        public Vector3 normal;
        public float penetration;
        public int frameLastSeen;
    }
    struct CombinedMatterProps
    {
        public float StaticFriction;
        public float SlidingFriction;
    }

    public class CollisionHandler
    {
        #region Fields

        private readonly Dictionary<int, ContactCache> _contactCache = new Dictionary<int, ContactCache>();
        private int _frameCount = 0;
        private readonly List<ICollisionStrategy> _collisionStrategies;
        private readonly HashSet<(int, int)> _ignoredBodyPairs = new HashSet<(int, int)>();
        private readonly HashSet<int> _hardExcludedBodyIds = new HashSet<int>();
        private LayerMask _collisionLayerMask = ~0;
        private const float SPATIAL_CELL_SIZE = 0.15f;
        private readonly List<int> _faceNodeNearbyKeys = new List<int>();

        private const float FACE_NODE_CELL_SIZE = 0.3f;
        private readonly List<int> _faceNodeCandidates = new List<int>();
        private bool[] _nodeVisitedFlags = System.Array.Empty<bool>();

        private readonly Dictionary<Mesh, MeshData> _meshCache = new Dictionary<Mesh, MeshData>();
        private readonly List<int> _triQueryBuffer = new List<int>(64);
        private readonly List<Collider> _tempColliders = new List<Collider>(8);
        private readonly List<Collider> _validColliders = new List<Collider>(128);
        private readonly RaycastHit[] _raycastHitBuffer = new RaycastHit[16];

        private struct MeshData
        {
            public Vector3[] vertices;
            public int[] triangles;
            public MeshTriGrid grid;
        }

        private sealed class MeshTriGrid
        {
            private readonly Vector3 _origin;
            private readonly float _invCellSize;
            private readonly int _nx, _ny, _nz;
            private readonly int[][] _cells;

            public MeshTriGrid(Vector3[] verts, int[] tris)
            {
                if (verts == null || tris == null || tris.Length < 3)
                { _cells = System.Array.Empty<int[]>(); return; }

                Vector3 mn = verts[0], mx = verts[0];
                for (int i = 1; i < verts.Length; i++)
                {
                    if (verts[i].x < mn.x) mn.x = verts[i].x; else if (verts[i].x > mx.x) mx.x = verts[i].x;
                    if (verts[i].y < mn.y) mn.y = verts[i].y; else if (verts[i].y > mx.y) mx.y = verts[i].y;
                    if (verts[i].z < mn.z) mn.z = verts[i].z; else if (verts[i].z > mx.z) mx.z = verts[i].z;
                }

                Vector3 extent = mx - mn;
                float maxExt = Mathf.Max(extent.x, Mathf.Max(extent.y, extent.z));
                if (maxExt < 1e-5f) { _cells = System.Array.Empty<int[]>(); return; }

                const int MAX_DIM = 32;
                float cellSize = Mathf.Max(maxExt / MAX_DIM, 0.05f);
                _invCellSize = 1f / cellSize;
                _origin = mn - Vector3.one * (cellSize * 0.5f);

                _nx = Mathf.Clamp(Mathf.CeilToInt(extent.x * _invCellSize) + 1, 1, MAX_DIM);
                _ny = Mathf.Clamp(Mathf.CeilToInt(extent.y * _invCellSize) + 1, 1, MAX_DIM);
                _nz = Mathf.Clamp(Mathf.CeilToInt(extent.z * _invCellSize) + 1, 1, MAX_DIM);

                int totalCells = _nx * _ny * _nz;
                _cells = new int[totalCells][];
                int[] counts = new int[totalCells];

                int triCount = tris.Length / 3;
                for (int ti = 0; ti < triCount; ti++)
                {
                    GetTriCellRange(verts, tris, ti, out int x0, out int y0, out int z0,
                                                      out int x1, out int y1, out int z1);
                    for (int cx = x0; cx <= x1; cx++)
                        for (int cy = y0; cy <= y1; cy++)
                            for (int cz = z0; cz <= z1; cz++)
                                counts[CellIdx(cx, cy, cz)]++;
                }

                for (int c = 0; c < totalCells; c++)
                    if (counts[c] > 0) _cells[c] = new int[counts[c]];

                int[] cursors = new int[totalCells];
                for (int ti = 0; ti < triCount; ti++)
                {
                    GetTriCellRange(verts, tris, ti, out int x0, out int y0, out int z0,
                                                      out int x1, out int y1, out int z1);
                    for (int cx = x0; cx <= x1; cx++)
                        for (int cy = y0; cy <= y1; cy++)
                            for (int cz = z0; cz <= z1; cz++)
                            {
                                int idx = CellIdx(cx, cy, cz);
                                _cells[idx][cursors[idx]++] = ti * 3;
                            }
                }
            }

            public void Query(Vector3 bMin, Vector3 bMax, List<int> outList)
            {
                if (_cells == null || _cells.Length == 0) return;

                int x0 = Mathf.Clamp(Mathf.FloorToInt((bMin.x - _origin.x) * _invCellSize), 0, _nx - 1);
                int y0 = Mathf.Clamp(Mathf.FloorToInt((bMin.y - _origin.y) * _invCellSize), 0, _ny - 1);
                int z0 = Mathf.Clamp(Mathf.FloorToInt((bMin.z - _origin.z) * _invCellSize), 0, _nz - 1);
                int x1 = Mathf.Clamp(Mathf.FloorToInt((bMax.x - _origin.x) * _invCellSize), 0, _nx - 1);
                int y1 = Mathf.Clamp(Mathf.FloorToInt((bMax.y - _origin.y) * _invCellSize), 0, _ny - 1);
                int z1 = Mathf.Clamp(Mathf.FloorToInt((bMax.z - _origin.z) * _invCellSize), 0, _nz - 1);

                for (int cx = x0; cx <= x1; cx++)
                    for (int cy = y0; cy <= y1; cy++)
                        for (int cz = z0; cz <= z1; cz++)
                        {
                            int[] cell = _cells[CellIdx(cx, cy, cz)];
                            if (cell == null) continue;
                            foreach (int t in cell) outList.Add(t);
                        }
            }

            private int CellIdx(int cx, int cy, int cz) => cx + _nx * (cy + _ny * cz);

            private void GetTriCellRange(Vector3[] verts, int[] tris, int ti,
                out int x0, out int y0, out int z0,
                out int x1, out int y1, out int z1)
            {
                Vector3 a = verts[tris[ti * 3]];
                Vector3 b = verts[tris[ti * 3 + 1]];
                Vector3 c = verts[tris[ti * 3 + 2]];

                float mnx = Mathf.Min(a.x, Mathf.Min(b.x, c.x)) - _origin.x;
                float mny = Mathf.Min(a.y, Mathf.Min(b.y, c.y)) - _origin.y;
                float mnz = Mathf.Min(a.z, Mathf.Min(b.z, c.z)) - _origin.z;
                float mxx = Mathf.Max(a.x, Mathf.Max(b.x, c.x)) - _origin.x;
                float mxy = Mathf.Max(a.y, Mathf.Max(b.y, c.y)) - _origin.y;
                float mxz = Mathf.Max(a.z, Mathf.Max(b.z, c.z)) - _origin.z;

                x0 = Mathf.Clamp(Mathf.FloorToInt(mnx * _invCellSize), 0, _nx - 1);
                y0 = Mathf.Clamp(Mathf.FloorToInt(mny * _invCellSize), 0, _ny - 1);
                z0 = Mathf.Clamp(Mathf.FloorToInt(mnz * _invCellSize), 0, _nz - 1);
                x1 = Mathf.Clamp(Mathf.FloorToInt(mxx * _invCellSize), 0, _nx - 1);
                y1 = Mathf.Clamp(Mathf.FloorToInt(mxy * _invCellSize), 0, _ny - 1);
                z1 = Mathf.Clamp(Mathf.FloorToInt(mxz * _invCellSize), 0, _nz - 1);
            }
        }

        public readonly List<Vector3> collisionPoints;
        public bool enableDebugLogging = false;
        public float triangleThickness = 0.02f;

        private const float SKIN_WIDTH = 0.005f;

        private Collider[] _staticColliders;
        private float _lastStaticColliderRefreshTime;
        private const float STATIC_COLLIDER_REFRESH_INTERVAL = 1.0f;
        private const float MIN_EFFECTIVE_SUBSTEP = 0.001f;
        private const float MIN_SWEEP_DISTANCE = 0.0001f;
        private readonly List<int> _cacheKeysToRemove = new List<int>();

        #endregion

        #region Properties
        public LayerMask CollisionLayerMask
        {
            get => _collisionLayerMask;
            set => _collisionLayerMask = value;
        }
        #endregion

        #region Constructor
        public CollisionHandler()
        {
            collisionPoints = new List<Vector3>();
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
            foreach (var kvp in _contactCache)
                if (_frameCount - kvp.Value.frameLastSeen > 5)
                    _cacheKeysToRemove.Add(kvp.Key);
            foreach (int key in _cacheKeysToRemove)
                _contactCache.Remove(key);
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

                var pair = ownerAId < ownerBId ? (ownerAId, ownerBId) : (ownerBId, ownerAId);
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

        // Updated method: NodeToNodeCollisionJob (friction now correctly distinguishes StaticFriction for sticking threshold vs SlidingFriction for kinetic sliding in soft-soft collisions)
        [BurstCompile]
        private struct NodeToNodeCollisionJob : IJob
        {
            public NativeArray<float3> predictedPosA;
            public NativeArray<float3> predictedPosB;
            public NativeArray<float3> currentPosA;
            public NativeArray<float3> currentPosB;
            [ReadOnly] public NativeArray<float> massesA;
            [ReadOnly] public NativeArray<float> massesB;
            public NativeQueue<float3> collisionPointsOut;

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

                                            predictedPosA[i] += normal * (penetrationDepth * ratioA + epsilon * 0.5f);
                                            predictedPosB[j] -= normal * (penetrationDepth * ratioB + epsilon * 0.5f);

                                            float3 velA = (predictedPosA[i] - currentPosA[i]) / dt;
                                            float3 velB = (predictedPosB[j] - currentPosB[j]) / dt;
                                            float3 relVel = velA - velB;

                                            float normalVel = math.dot(relVel, normal);
                                            if (normalVel < 0)
                                            {
                                                float invMassA = 1f / massA;
                                                float invMassB = 1f / massB;
                                                float invMassSum = invMassA + invMassB;
                                                float impactMultiplier = 1f + (math.abs(massA - massB) / math.max(massA, massB)) * 0.5f;

                                                float jn = -(1f + restitution) * normalVel / invMassSum * impactMultiplier;
                                                float3 normalImpulse = normal * jn;

                                                // Calculate true effective normal force including positional penetration 
                                                float positionalImpulseMag = (penetrationDepth / dt) / invMassSum;
                                                float effectiveJn = math.abs(jn) + positionalImpulseMag;

                                                float3 tangentVel = relVel - normal * normalVel;
                                                float3 frictionImpulse = float3.zero;

                                                if (math.length(tangentVel) > 0.001f)
                                                {
                                                    float stopImpulseMagnitude = math.length(tangentVel) / invMassSum;
                                                    float maxStaticMagnitude = staticFriction * effectiveJn;

                                                    float frictionMagnitude;
                                                    if (stopImpulseMagnitude <= maxStaticMagnitude)
                                                    {
                                                        frictionMagnitude = stopImpulseMagnitude;
                                                    }
                                                    else
                                                    {
                                                        frictionMagnitude = math.min(kineticFriction * effectiveJn, stopImpulseMagnitude);
                                                    }
                                                    frictionImpulse = -math.normalize(tangentVel) * frictionMagnitude;
                                                }

                                                float3 totalImpulse = normalImpulse + frictionImpulse;

                                                currentPosA[i] -= (totalImpulse * invMassA) * dt;
                                                currentPosB[j] += (totalImpulse * invMassB) * dt;
                                            }

                                            collisionPointsOut.Enqueue(posA - normal * nodeRadiusA);
                                            posA = predictedPosA[i];
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

        #region Narrow Phase Node-to-Node (Jobified)

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

            float restA = matterA != null ? matterA.Restitution : PhysicsConstants.DEFAULT_RESTITUTION;
            float restB = matterB != null ? matterB.Restitution : PhysicsConstants.DEFAULT_RESTITUTION;

            float sfA = matterA != null ? matterA.StaticFriction : PhysicsConstants.DEFAULT_STATIC_FRICTION;
            float kfA = matterA != null ? matterA.SlidingFriction : PhysicsConstants.DEFAULT_DYNAMIC_FRICTION;
            float sfB = matterB != null ? matterB.StaticFriction : PhysicsConstants.DEFAULT_STATIC_FRICTION;
            float kfB = matterB != null ? matterB.SlidingFriction : PhysicsConstants.DEFAULT_DYNAMIC_FRICTION;

            float staticFric = (sfA + sfB) * 0.5f;
            float kineticFric = (kfA + kfB) * 0.5f;

            var job = new NodeToNodeCollisionJob
            {
                predictedPosA = predA,
                predictedPosB = predB,
                currentPosA = currA,
                currentPosB = currB,
                massesA = massA,
                massesB = massB,
                collisionPointsOut = colQueue,
                nodeRadiusA = nmA.GetNodeRadius(),
                nodeRadiusB = nmB.GetNodeRadius(),
                epsilon = epsilon,
                dt = dt,
                restitution = (restA + restB) * 0.5f,
                staticFriction = staticFric,
                kineticFriction = kineticFric,
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

        #region Face-to-Node Collision (CCD - Job Native Collections Refactor)

        private void ResolveFaceToNode(NodeManager nmNode, List<float> massesNode, SoftBody bodyFace,
                                       NodeManager nmFace, List<float> massesFace, Matter matterNode,
                                       Matter matterFace, float dt, float epsilon, bool visualizeForces)
        {
            Truss truss = bodyFace.truss;
            if (truss == null) return;

            List<Face> faces = truss.GetTrussFaces();
            if (faces == null || faces.Count == 0) return;

            int nodeCount = nmNode.PredictedPositions.Count;
            if (nodeCount == 0) return;

            float radiusNode = nmNode.GetNodeRadius() + SKIN_WIDTH;
            float thickness = radiusNode + triangleThickness;

            if (_nodeVisitedFlags.Length < nodeCount)
                _nodeVisitedFlags = new bool[nodeCount];

            NativeParallelMultiHashMap<int, int> fnHashMap = new NativeParallelMultiHashMap<int, int>(nodeCount * 2, Allocator.Temp);

            for (int n = 0; n < nodeCount; n++)
            {
                int3 gridPosS = new int3(math.floor((float3)nmNode.PreviousPositions[n] / FACE_NODE_CELL_SIZE));
                int hashS = (gridPosS.x * 73856093) ^ (gridPosS.y * 19349663) ^ (gridPosS.z * 83492791);
                fnHashMap.Add(hashS, n);

                int3 gridPosE = new int3(math.floor((float3)nmNode.PredictedPositions[n] / FACE_NODE_CELL_SIZE));
                int hashE = (gridPosE.x * 73856093) ^ (gridPosE.y * 19349663) ^ (gridPosE.z * 83492791);
                fnHashMap.Add(hashE, n);
            }

            foreach (Face face in faces)
            {
                if (face.nodeA < 0 || face.nodeA >= nmFace.PredictedPositions.Count ||
                    face.nodeB < 0 || face.nodeB >= nmFace.PredictedPositions.Count ||
                    face.nodeC < 0 || face.nodeC >= nmFace.PredictedPositions.Count) continue;

                Vector3 v0s = nmFace.PreviousPositions[face.nodeA];
                Vector3 v1s = nmFace.PreviousPositions[face.nodeB];
                Vector3 v2s = nmFace.PreviousPositions[face.nodeC];
                Vector3 v0e = nmFace.PredictedPositions[face.nodeA];
                Vector3 v1e = nmFace.PredictedPositions[face.nodeB];
                Vector3 v2e = nmFace.PredictedPositions[face.nodeC];

                float bMinX = Mathf.Min(Mathf.Min(v0s.x, v1s.x), Mathf.Min(v2s.x, Mathf.Min(Mathf.Min(v0e.x, v1e.x), v2e.x))) - thickness;
                float bMinY = Mathf.Min(Mathf.Min(v0s.y, v1s.y), Mathf.Min(v2s.y, Mathf.Min(Mathf.Min(v0e.y, v1e.y), v2e.y))) - thickness;
                float bMinZ = Mathf.Min(Mathf.Min(v0s.z, v1s.z), Mathf.Min(v2s.z, Mathf.Min(Mathf.Min(v0e.z, v1e.z), v2e.z))) - thickness;
                float bMaxX = Mathf.Max(Mathf.Max(v0s.x, v1s.x), Mathf.Max(v2s.x, Mathf.Max(Mathf.Max(v0e.x, v1e.x), v2e.x))) + thickness;
                float bMaxY = Mathf.Max(Mathf.Max(v0s.y, v1s.y), Mathf.Max(v2s.y, Mathf.Max(Mathf.Max(v0e.y, v1e.y), v2e.y))) + thickness;
                float bMaxZ = Mathf.Max(Mathf.Max(v0s.z, v1s.z), Mathf.Max(v2s.z, Mathf.Max(Mathf.Max(v0e.z, v1e.z), v2e.z))) + thickness;

                GetFaceNodeCellKeysAABB(
                    new Vector3(bMinX, bMinY, bMinZ),
                    new Vector3(bMaxX, bMaxY, bMaxZ),
                    _faceNodeNearbyKeys);

                _faceNodeCandidates.Clear();
                foreach (int key in _faceNodeNearbyKeys)
                {
                    if (fnHashMap.TryGetFirstValue(key, out int n, out NativeParallelMultiHashMapIterator<int> it))
                    {
                        do
                        {
                            if (_nodeVisitedFlags[n]) continue;
                            _nodeVisitedFlags[n] = true;
                            _faceNodeCandidates.Add(n);
                        } while (fnHashMap.TryGetNextValue(out n, ref it));
                    }
                }

                foreach (int n in _faceNodeCandidates)
                {
                    _nodeVisitedFlags[n] = false;

                    Vector3 nodeStart = nmNode.PreviousPositions[n];
                    Vector3 nodeEnd = nmNode.PredictedPositions[n];

                    float nsX = nodeStart.x < nodeEnd.x ? nodeStart.x : nodeEnd.x;
                    float nsY = nodeStart.y < nodeEnd.y ? nodeStart.y : nodeEnd.y;
                    float nsZ = nodeStart.z < nodeEnd.z ? nodeStart.z : nodeEnd.z;
                    float neX = nodeStart.x > nodeEnd.x ? nodeStart.x : nodeEnd.x;
                    float neY = nodeStart.y > nodeEnd.y ? nodeStart.y : nodeEnd.y;
                    float neZ = nodeStart.z > nodeEnd.z ? nodeStart.z : nodeEnd.z;

                    if (nsX > bMaxX || neX < bMinX ||
                        nsY > bMaxY || neY < bMinY ||
                        nsZ > bMaxZ || neZ < bMinZ) continue;

                    if (CheckSweptFaceNode(nodeStart, nodeEnd, radiusNode, v0s, v0e, v1s, v1e, v2s, v2e, out float t, out Vector3 hitNormal))
                    {
                        Vector3 pAtT = Vector3.Lerp(nodeStart, nodeEnd, t);
                        Vector3 aAtT = Vector3.Lerp(v0s, v0e, t);
                        Vector3 bAtT = Vector3.Lerp(v1s, v1e, t);
                        Vector3 cAtT = Vector3.Lerp(v2s, v2e, t);

                        if (GetBarycentric(pAtT, aAtT, bAtT, cAtT, out float u, out float v, out float w))
                        {
                            Vector3 triPoint = u * v0e + v * v1e + w * v2e;
                            float dist = Vector3.Dot(nodeEnd - triPoint, hitNormal);
                            float penetration = (radiusNode + epsilon) - dist;

                            if (penetration > 0 || t < 1.0f)
                            {
                                penetration = Mathf.Max(penetration, epsilon);

                                ApplyXPBDFaceNode(nmNode, n, nmFace, face.nodeA, face.nodeB, face.nodeC,
                                    hitNormal, penetration, u, v, w, dt, matterNode, matterFace, massesNode, massesFace, visualizeForces);

                                collisionPoints.Add(pAtT);
                                if (visualizeForces)
                                    Debug.DrawLine(pAtT, pAtT + hitNormal * 0.15f, Color.magenta, dt);
                            }
                        }
                    }
                }
            }

            fnHashMap.Dispose();
        }

        private void GetFaceNodeCellKeysAABB(Vector3 bMin, Vector3 bMax, List<int> outKeys)
        {
            outKeys.Clear();
            int x0 = Mathf.FloorToInt(bMin.x / FACE_NODE_CELL_SIZE);
            int y0 = Mathf.FloorToInt(bMin.y / FACE_NODE_CELL_SIZE);
            int z0 = Mathf.FloorToInt(bMin.z / FACE_NODE_CELL_SIZE);
            int x1 = Mathf.FloorToInt(bMax.x / FACE_NODE_CELL_SIZE);
            int y1 = Mathf.FloorToInt(bMax.y / FACE_NODE_CELL_SIZE);
            int z1 = Mathf.FloorToInt(bMax.z / FACE_NODE_CELL_SIZE);

            for (int x = x0; x <= x1; x++)
                for (int y = y0; y <= y1; y++)
                    for (int z = z0; z <= z1; z++)
                        outKeys.Add((x * 73856093) ^ (y * 19349663) ^ (z * 83492791));
        }

        private bool CheckSweptFaceNode(Vector3 p0, Vector3 p1, float radius, Vector3 a0, Vector3 a1, Vector3 b0, Vector3 b1, Vector3 c0, Vector3 c1, out float t, out Vector3 normal)
        {
            t = 1.0f;
            normal = Vector3.Cross(b0 - a0, c0 - a0).normalized;

            Vector3 tri0 = (a0 + b0 + c0) / 3f;
            Vector3 relMove = (p1 - p0) - ((a1 + b1 + c1) / 3f - tri0);

            float d0 = Vector3.Dot(p0 - tri0, normal);
            float d1 = Vector3.Dot(p0 + relMove - tri0, normal);

            if (Mathf.Sign(d0) != Mathf.Sign(d1))
            {
                t = Mathf.Clamp01(Mathf.Abs(d0) / (Mathf.Abs(d0) + Mathf.Abs(d1)));
                if (d0 < 0) normal = -normal;
                return true;
            }

            if (Mathf.Abs(d0) < radius) { t = 0; return true; }
            return false;
        }

        private void ApplyXPBDFaceNode(NodeManager nmP, int idxP,
                                       NodeManager nmT, int idxA, int idxB, int idxC,
                                       Vector3 normal, float penetration,
                                       float u, float v, float w,
                                       float dt, Matter matP, Matter matT,
                                       List<float> massesP, List<float> massesT, bool visualize)
        {
            float wP = nmP.IsPinned[idxP] ? 0f : 1f / massesP[idxP];
            float wA = nmT.IsPinned[idxA] ? 0f : 1f / massesT[idxA];
            float wB = nmT.IsPinned[idxB] ? 0f : 1f / massesT[idxB];
            float wC = nmT.IsPinned[idxC] ? 0f : 1f / massesT[idxC];

            float wTri = (u * u * wA) + (v * v * wB) + (w * w * wC);
            float wSum = wP + wTri;
            if (wSum <= 1e-9f) return;

            float dLambda = penetration / wSum;
            Vector3 correction = dLambda * normal;

            if (!nmP.IsPinned[idxP]) nmP.PredictedPositions[idxP] += wP * correction;
            if (!nmT.IsPinned[idxA]) nmT.PredictedPositions[idxA] -= wA * u * correction;
            if (!nmT.IsPinned[idxB]) nmT.PredictedPositions[idxB] -= wB * v * correction;
            if (!nmT.IsPinned[idxC]) nmT.PredictedPositions[idxC] -= wC * w * correction;
        }

        private bool GetBarycentric(Vector3 p, Vector3 a, Vector3 b, Vector3 c, out float u, out float v, out float w)
        {
            Vector3 v0 = b - a, v1 = c - a, v2 = p - a;
            float d00 = Vector3.Dot(v0, v0), d01 = Vector3.Dot(v0, v1), d11 = Vector3.Dot(v1, v1);
            float d20 = Vector3.Dot(v2, v0), d21 = Vector3.Dot(v2, v1);
            float denom = d00 * d11 - d01 * d01;
            if (Mathf.Abs(denom) < 1e-9f) { u = v = w = 0; return false; }
            v = (d11 * d20 - d01 * d21) / denom;
            w = (d00 * d21 - d01 * d20) / denom;
            u = 1.0f - v - w;
            return (u >= -0.05f && v >= -0.05f && w >= -0.05f && (u + v + w) <= 1.05f);
        }

        #endregion

        #region Custom Static Collision Detection with CCD


        private void ResolveStaticCollisions(NodeManager nodeManager, Transform owner, Matter softBodyMatter, List<float> nodeMasses, float dt, float epsilon, bool visualizeForces)
        {
            RefreshStaticColliders();

            int nodeCount = nodeManager.NodeCount;

            for (int i = 0; i < nodeCount; i++)
            {
                Vector3 currentPos = nodeManager.GetPosition(i);
                Vector3 predictedPos = nodeManager.PredictedPositions[i];
                float radius = nodeManager.GetNodeRadius() + SKIN_WIDTH;

                bool hit = PerformAnalyticalSweptTest(currentPos, predictedPos, radius, owner, out Vector3 hitPoint, out Vector3 hitNormal, out float hitTime, out Collider hitCollider);

                if (hit && hitCollider.TryGetComponent<StaticBody>(out StaticBody sb))
                {
                    Matter hitMatter = sb.matter != null ? sb.matter : softBodyMatter;
                    hitTime = Mathf.Clamp01(hitTime);
                    Vector3 collisionPos = Vector3.Lerp(currentPos, predictedPos, hitTime);
                    float sweepAlongNormal = Vector3.Dot(currentPos - predictedPos, hitNormal);
                    float penetration = Mathf.Max(radius - Vector3.Dot(predictedPos - hitPoint, hitNormal), 0f);

                    ApplyCollisionResolution(i, nodeManager, nodeMasses[i], softBodyMatter, hitMatter, currentPos, collisionPos, hitPoint, hitNormal, penetration, dt, epsilon, visualizeForces);
                    collisionPoints.Add(hitPoint);
                }
            }
        }

        private void RefreshStaticColliders()
        {
            if (_staticColliders != null && Time.time - _lastStaticColliderRefreshTime < STATIC_COLLIDER_REFRESH_INTERVAL) return;
            _lastStaticColliderRefreshTime = Time.time;

            StaticBody[] staticBodies = UnityEngine.Object.FindObjectsOfType<StaticBody>();
            _validColliders.Clear();

            foreach (StaticBody sb in staticBodies)
            {
                if (sb != null && sb.isActiveAndEnabled)
                {
                    // Gets components without allocating a new array
                    sb.GetComponents(_tempColliders);
                    foreach (Collider col in _tempColliders)
                    {
                        if (col != null && col.enabled && col.gameObject.activeInHierarchy)
                        {
                            _validColliders.Add(col);
                        }
                    }
                }
            }
            _staticColliders = _validColliders.ToArray();
        }

        private bool PerformAnalyticalSweptTest(Vector3 startPos, Vector3 endPos, float radius, Transform owner, out Vector3 hitPoint, out Vector3 hitNormal, out float hitTime, out Collider hitCollider)
        {
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;
            hitTime = float.MaxValue;
            hitCollider = null;

            Vector3 sweepVelocity = endPos - startPos;
            bool foundHit = false;
            float earliestTime = float.MaxValue;
            if (_staticColliders == null) return false;
            foreach (Collider col in _staticColliders)
            {
                if (col == null || col.transform == owner || !col.enabled) continue;

                float t = float.MaxValue;
                Vector3 point = Vector3.zero, normal = Vector3.up;

                if (col is SphereCollider sphere)
                    SweptSphereSphere(startPos, sweepVelocity, radius, sphere, out t, out point, out normal);
                else if (col is BoxCollider box)
                    SweptSphereBox(startPos, sweepVelocity, radius, box, out t, out point, out normal);
                else if (col is TerrainCollider terrain)
                    SweptSphereTerrain(startPos, sweepVelocity, radius, terrain, out t, out point, out normal);
                else if (col is CapsuleCollider capsuleCollider)
                    SweptSphereCapsule(startPos, sweepVelocity, radius, capsuleCollider, out t, out point, out normal);
                else if (col is MeshCollider meshCollider)
                    SweptSphereMesh(startPos, sweepVelocity, radius, meshCollider, out t, out point, out normal);

                if (t < earliestTime && t >= 0f && t <= 1f)
                {
                    earliestTime = t;
                    hitPoint = point;
                    hitNormal = normal;
                    hitCollider = col;
                    foundHit = true;
                }
            }

            if (foundHit) hitTime = earliestTime;
            return foundHit;
        }

        #endregion

        #region Analytical Swept Collision Tests
        private bool SweptSphereSphere(Vector3 startPos, Vector3 velocity, float radiusA, SphereCollider sphereB, out float hitTime, out Vector3 hitPoint, out Vector3 hitNormal)
        {
            hitTime = float.MaxValue;
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;

            Vector3 centerB = sphereB.transform.TransformPoint(sphereB.center);
            float radiusB = sphereB.radius * Mathf.Max(sphereB.transform.lossyScale.x, sphereB.transform.lossyScale.y, sphereB.transform.lossyScale.z);
            float combinedRadius = radiusA + radiusB;

            Vector3 p = startPos - centerB;
            float a = Vector3.Dot(velocity, velocity);
            float b = 2f * Vector3.Dot(p, velocity);
            float c = Vector3.Dot(p, p) - combinedRadius * combinedRadius;

            if (a < 1e-12f) return false;
            float discriminant = b * b - 4f * a * c;
            if (discriminant < 0) return false;

            float t = (-b - Mathf.Sqrt(discriminant)) / (2f * a);
            if (t < 0f || t > 1f) return false;

            hitTime = t;
            hitNormal = ((startPos + velocity * t) - centerB).normalized;
            hitPoint = centerB + hitNormal * radiusB;
            return true;
        }

        private bool SweptSphereBox(Vector3 startPos, Vector3 velocity, float radius, BoxCollider box, out float hitTime, out Vector3 hitPoint, out Vector3 hitNormal)
        {
            hitTime = float.MaxValue;
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;

            Vector3 localStart = box.transform.InverseTransformPoint(startPos);
            Vector3 localVelocity = box.transform.InverseTransformDirection(velocity);
            Vector3 scale = box.transform.lossyScale;
            Vector3 perAxisRadius = new Vector3(radius / scale.x, radius / scale.y, radius / scale.z);

            Vector3 boxMin = box.center - box.size * 0.5f - perAxisRadius;
            Vector3 boxMax = box.center + box.size * 0.5f + perAxisRadius;

            float tMin = 0f, tMax = 1f;
            int hitAxis = -1, hitSign = 0;

            for (int i = 0; i < 3; i++)
            {
                float p = localStart[i], v = localVelocity[i], min = boxMin[i], max = boxMax[i];

                if (Mathf.Abs(v) < 1e-6f)
                {
                    if (p + v < min || p + v > max) return false;
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
            Vector3 localNormal = Vector3.zero;
            if (hitAxis >= 0) localNormal[hitAxis] = hitSign;

            hitNormal = box.transform.TransformDirection(localNormal).normalized;
            hitPoint = startPos + velocity * hitTime - hitNormal * radius;
            return true;
        }

        private bool SweptSphereCapsule(Vector3 startPos, Vector3 velocity, float radius, CapsuleCollider capsule,
                                       out float hitTime, out Vector3 hitPoint, out Vector3 hitNormal)
        {
            hitTime = float.MaxValue;
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;

            Vector3 capsuleCenter = capsule.transform.TransformPoint(capsule.center);
            float capsuleRadius = capsule.radius * Mathf.Max(capsule.transform.lossyScale.x, capsule.transform.lossyScale.z);
            float capsuleHeight = capsule.height * capsule.transform.lossyScale.y;

            Vector3 axis = Vector3.up;
            if (capsule.direction == 0) axis = Vector3.right;
            else if (capsule.direction == 2) axis = Vector3.forward;
            axis = capsule.transform.TransformDirection(axis);

            float halfHeight = Mathf.Max(0, (capsuleHeight * 0.5f) - capsuleRadius);
            Vector3 p1 = capsuleCenter + axis * halfHeight;
            Vector3 p2 = capsuleCenter - axis * halfHeight;

            return SweptSphereLineSegment(startPos, velocity, radius + capsuleRadius, p1, p2,
                                         out hitTime, out hitPoint, out hitNormal);
        }

        private bool SweptSphereLineSegment(Vector3 startPos, Vector3 velocity, float radius,
                                           Vector3 segmentA, Vector3 segmentB,
                                           out float hitTime, out Vector3 hitPoint, out Vector3 hitNormal)
        {
            hitTime = float.MaxValue;
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;

            Vector3 segmentDir = segmentB - segmentA;
            float segmentLength = segmentDir.magnitude;
            if (segmentLength < 0.0001f)
                return SweptSpherePoint(startPos, velocity, radius, segmentA, out hitTime, out hitPoint, out hitNormal);
            segmentDir /= segmentLength;

            Vector3 w = startPos - segmentA;
            float a = Vector3.Dot(velocity, velocity);
            float b = Vector3.Dot(velocity, segmentDir);
            float c = Vector3.Dot(segmentDir, segmentDir);
            float d = Vector3.Dot(velocity, w);
            float e = Vector3.Dot(segmentDir, w);
            float denom = a * c - b * b;

            if (Mathf.Abs(denom) < 1e-8f)
            {
                bool hit1 = SweptSpherePoint(startPos, velocity, radius, segmentA, out float t1, out Vector3 p1, out Vector3 n1);
                bool hit2 = SweptSpherePoint(startPos, velocity, radius, segmentB, out float t2, out Vector3 p2, out Vector3 n2);
                if (hit1 && hit2) { if (t1 < t2) { hitTime = t1; hitPoint = p1; hitNormal = n1; } else { hitTime = t2; hitPoint = p2; hitNormal = n2; } return true; }
                else if (hit1) { hitTime = t1; hitPoint = p1; hitNormal = n1; return true; }
                else if (hit2) { hitTime = t2; hitPoint = p2; hitNormal = n2; return true; }
                return false;
            }

            float t = (b * e - c * d) / denom;
            float s = (a * e - b * d) / denom;
            s = Mathf.Clamp(s, 0f, segmentLength);

            Vector3 segmentPoint = segmentA + segmentDir * s;
            return SweptSpherePoint(startPos, velocity, radius, segmentPoint, out hitTime, out hitPoint, out hitNormal);
        }

        private bool SweptSpherePoint(Vector3 startPos, Vector3 velocity, float radius, Vector3 point,
                                     out float hitTime, out Vector3 hitPoint, out Vector3 hitNormal)
        {
            hitTime = float.MaxValue;
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;

            Vector3 p = startPos - point;
            Vector3 v = velocity;
            float a = Vector3.Dot(v, v);
            float b = 2f * Vector3.Dot(p, v);
            float c = Vector3.Dot(p, p) - radius * radius;

            if (a < 1e-12f) return false;
            float discriminant = b * b - 4f * a * c;
            if (discriminant < 0) return false;

            float sqrtDisc = Mathf.Sqrt(discriminant);
            float t1 = (-b - sqrtDisc) / (2f * a);
            float t2 = (-b + sqrtDisc) / (2f * a);
            float t = t1 >= 0f ? t1 : t2;

            if (t < 0f || t > 1f) return false;

            hitTime = t;
            Vector3 posAtHit = startPos + velocity * t;
            hitNormal = (posAtHit - point).normalized;
            hitPoint = point;
            return true;
        }

        private bool SweptSphereTerrain(Vector3 startPos, Vector3 velocity, float radius, TerrainCollider terrain, out float hitTime, out Vector3 hitPoint, out Vector3 hitNormal)
        {
            hitTime = float.MaxValue;
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;

            TerrainData tData = terrain.terrainData;
            if (tData == null) return false;

            Vector3 tPos = terrain.transform.position;
            float sweepDist = velocity.magnitude;

            // Fixed loop check for mathematical terrain sampling
            int steps = sweepDist < 0.001f ? 1 : Mathf.Clamp(Mathf.CeilToInt(sweepDist / (radius * 0.5f)), 1, 20);

            for (int i = 0; i <= steps; i++)
            {
                float t = (float)i / steps;
                Vector3 testPos = startPos + velocity * t;
                Vector3 localPos = testPos - tPos;

                float normX = localPos.x / tData.size.x;
                float normZ = localPos.z / tData.size.z;

                if (normX < 0 || normX > 1 || normZ < 0 || normZ > 1) continue;

                float terrainHeight = tData.GetInterpolatedHeight(normX, normZ) + tPos.y;
                if ((testPos.y - radius) <= terrainHeight)
                {
                    hitTime = t;
                    hitPoint = new Vector3(testPos.x, terrainHeight, testPos.z);
                    hitNormal = terrain.transform.TransformDirection(tData.GetInterpolatedNormal(normX, normZ)).normalized;
                    return true;
                }
            }
            return false;
        }

        private bool SweptSphereMesh(Vector3 startPos, Vector3 velocity, float radius, MeshCollider meshCol,
                            out float hitTime, out Vector3 hitPoint, out Vector3 hitNormal)
        {
            hitTime = float.MaxValue;
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;

            if (meshCol.sharedMesh == null || !meshCol.sharedMesh.isReadable) return false;

            Mesh mesh = meshCol.sharedMesh;

            if (!_meshCache.TryGetValue(mesh, out MeshData md))
            {
                md = new MeshData
                {
                    vertices = mesh.vertices,
                    triangles = mesh.triangles,
                    grid = new MeshTriGrid(mesh.vertices, mesh.triangles)
                };
                _meshCache[mesh] = md;
            }

            Vector3[] vertices = md.vertices;
            int[] triangles = md.triangles;

            Vector3 localStart = meshCol.transform.InverseTransformPoint(startPos);
            Vector3 localEnd = meshCol.transform.InverseTransformPoint(startPos + velocity);
            Vector3 localVelocity = localEnd - localStart;
            float localRadius = radius / Mathf.Max(meshCol.transform.lossyScale.x,
                                           Mathf.Max(meshCol.transform.lossyScale.y,
                                                     meshCol.transform.lossyScale.z));

            float pad = localRadius;
            Vector3 sweepMin = Vector3.Min(localStart, localEnd) - Vector3.one * pad;
            Vector3 sweepMax = Vector3.Max(localStart, localEnd) + Vector3.one * pad;

            _triQueryBuffer.Clear();
            md.grid.Query(sweepMin, sweepMax, _triQueryBuffer);

            bool foundHit = false;
            float earliestT = float.MaxValue;
            Vector3 bestHitPoint = Vector3.zero;
            Vector3 bestHitNormal = Vector3.up;

            foreach (int i in _triQueryBuffer)
            {
                Vector3 v0 = vertices[triangles[i]];
                Vector3 v1 = vertices[triangles[i + 1]];
                Vector3 v2 = vertices[triangles[i + 2]];

                if (SweptSphereTriangle(localStart, localVelocity, localRadius, v0, v1, v2,
                                       out float t, out Vector3 point, out Vector3 normal))
                {
                    if (t < earliestT && t >= 0f && t <= 1f)
                    {
                        earliestT = t;
                        bestHitPoint = point;
                        bestHitNormal = normal;
                        foundHit = true;
                    }
                }
            }

            if (foundHit)
            {
                hitTime = earliestT;
                hitPoint = meshCol.transform.TransformPoint(bestHitPoint);
                hitNormal = meshCol.transform.TransformDirection(bestHitNormal).normalized;
                return true;
            }
            return false;
        }

        private bool SweptSphereTriangle(Vector3 startPos, Vector3 velocity, float radius, Vector3 v0, Vector3 v1, Vector3 v2, out float hitTime, out Vector3 hitPoint, out Vector3 hitNormal)
        {
            hitTime = 1f;
            hitPoint = Vector3.zero;
            hitNormal = Vector3.up;

            Vector3 edge1 = v1 - v0;
            Vector3 edge2 = v2 - v0;
            Vector3 triNormal = Vector3.Cross(edge1, edge2);
            if (triNormal.sqrMagnitude < 1e-12f) return false;
            triNormal.Normalize();

            Vector3 closestPt = GetClosestPointOnTriangle(startPos, v0, v1, v2);
            float distSq = (startPos - closestPt).sqrMagnitude;

            if (distSq <= radius * radius)
            {
                hitTime = 0f;
                hitPoint = closestPt;
                float dist = Mathf.Sqrt(distSq);
                hitNormal = (dist > 1e-6f) ? (startPos - closestPt).normalized : triNormal;
                return true;
            }

            float velocityMagnitudeSq = velocity.sqrMagnitude;
            if (velocityMagnitudeSq < 1e-12f)
            {
                Vector3 endPos = startPos + velocity;
                Vector3 endClosestPt = GetClosestPointOnTriangle(endPos, v0, v1, v2);
                if ((endPos - endClosestPt).sqrMagnitude <= radius * radius)
                {
                    hitTime = 1f;
                    hitPoint = endClosestPt;
                    hitNormal = triNormal;
                    return true;
                }
                return false;
            }

            float denom = Vector3.Dot(triNormal, -velocity);
            if (Mathf.Abs(denom) < 1e-12f) return false;

            float d = Vector3.Dot(triNormal, startPos - v0) - radius;
            float t = d / denom;

            if (t >= 0f && t <= 1f)
            {
                Vector3 posAtT = startPos + velocity * t;
                Vector3 planePoint = posAtT - triNormal * radius;

                if (IsPointInTriangleBary(planePoint, v0, v1, v2))
                {
                    hitTime = t;
                    hitPoint = planePoint;
                    hitNormal = triNormal;
                    return true;
                }
            }

            return false;
        }
        private Vector3 GetClosestPointOnTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
        {
            Vector3 ab = b - a;
            Vector3 ac = c - a;
            Vector3 ap = p - a;
            float d1 = Vector3.Dot(ab, ap);
            float d2 = Vector3.Dot(ac, ap);
            if (d1 <= 0f && d2 <= 0f) return a;

            Vector3 bp = p - b;
            float d3 = Vector3.Dot(ab, bp);
            float d4 = Vector3.Dot(ac, bp);
            if (d3 >= 0f && d4 <= d3) return b;

            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0f && d1 >= 0f && d3 <= 0f)
            {
                float v = d1 / (d1 - d3);
                return a + v * ab;
            }

            Vector3 cp = p - c;
            float d5 = Vector3.Dot(ab, cp);
            float d6 = Vector3.Dot(ac, cp);
            if (d6 >= 0f && d5 <= d6) return c;

            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 >= 0f && d6 <= 0f)
            {
                float w = d2 / (d2 - d6);
                return a + w * ac;
            }

            float va = d3 * d6 - d5 * d4;
            if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
            {
                float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                return b + w * (c - b);
            }

            float denom = 1f / (va + vb + vc);
            float v_ = vb * denom;
            float w_ = vc * denom;
            return a + ab * v_ + ac * w_;
        }

        private bool IsPointInTriangleBary(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
        {
            Vector3 v0 = b - a;
            Vector3 v1 = c - a;
            Vector3 v2 = p - a;

            float dot00 = Vector3.Dot(v0, v0);
            float dot01 = Vector3.Dot(v0, v1);
            float dot02 = Vector3.Dot(v0, v2);
            float dot11 = Vector3.Dot(v1, v1);
            float dot12 = Vector3.Dot(v1, v2);

            float denom = dot00 * dot11 - dot01 * dot01;
            if (Mathf.Abs(denom) < 0.0001f) return false;

            float invDenom = 1f / denom;
            float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

            return (u >= -0.001f) && (v >= -0.001f) && (u + v <= 1.001f);
        }
        #endregion

        #region Collision Response

        private void ApplyCollisionResolution(int idx, NodeManager nm, float nodeMass, Matter softMatter, Matter hitMatter, Vector3 currentWorldPos, Vector3 collisionPos, Vector3 contactPoint, Vector3 normal, float penetration, float dt, float epsilon, bool visualize)
        {
            float invMass = 1f / nodeMass;
            if (invMass <= 0f) return;

            Vector3 predicted = nm.PredictedPositions[idx];
            Vector3 preVel = (predicted - currentWorldPos) / dt;
            float preNormalVel = Vector3.Dot(preVel, normal);

            // Always resolve positional penetration
            Vector3 projectedPos = predicted + normal * penetration;
            Vector3 postVel = (projectedPos - currentWorldPos) / dt;

            // Dynamic response (restitution + friction) ONLY when approaching
            if (preNormalVel < 0f)
            {
                const float BOUNCE_THRESHOLD = 0.2f;
                float restA = (preNormalVel < -BOUNCE_THRESHOLD) ? (softMatter != null ? softMatter.Restitution : 0.02f) : 0f;

                float bounceVel = 0f;
                if (restA > 0f)
                {
                    bounceVel = -preNormalVel * restA;
                }

                float currentNormalVel = Vector3.Dot(postVel, normal);

                // FIX: Preserve positional correction by taking the max of bounce and current normal velocity
                float targetNormalVel = Mathf.Max(currentNormalVel, bounceVel);
                postVel += normal * (targetNormalVel - currentNormalVel);

                float jn = -(1f + restA) * preNormalVel / invMass;
                float positionalImpulseMag = (penetration / dt) / invMass;
                float effectiveJn = Mathf.Abs(jn) + positionalImpulseMag;

                postVel = ApplyFriction(postVel, normal, effectiveJn, invMass, softMatter, hitMatter, dt);
            }

            nm.PredictedPositions[idx] = currentWorldPos + postVel * dt;
        }

        private Vector3 ApplyFriction(Vector3 velocity, Vector3 normal, float effectiveJn, float invMass,
                       Matter softMatter, Matter hitMatter, float dt)
        {
            float sfA = softMatter != null ? softMatter.StaticFriction : 0.5f;
            float kfA = softMatter != null ? softMatter.SlidingFriction : 0.4f;
            float sfB = hitMatter != null ? hitMatter.StaticFriction : sfA;
            float kfB = hitMatter != null ? hitMatter.SlidingFriction : kfA;

            float sf = (sfA + sfB) * 0.5f;
            float kf = (kfA + kfB) * 0.5f;

            Vector3 velTan = velocity - Vector3.Dot(velocity, normal) * normal;
            float tangSpeed = velTan.magnitude;
            if (tangSpeed < 0.001f) return velocity;

            Vector3 tangDir = velTan / tangSpeed;
            float jt = tangSpeed / invMass;
            float frictionImpulse = jt <= sf * effectiveJn
                ? jt
                : Mathf.Min(kf * effectiveJn, jt);
            float compliance = softMatter != null ? softMatter.FrictionCompliance : 0.00001f;
            float alpha = compliance / (dt * dt);
            frictionImpulse /= (1f + alpha * invMass);

            return velocity - tangDir * (frictionImpulse * invMass);
        }

        #endregion

        #region Utility

        private bool ValidateCollisionParameters(NodeManager nodeManager, Transform owner, List<float> nodeMasses)
        {
            if (nodeManager == null || nodeManager.NodeCount == 0)
            {
                if (enableDebugLogging) Debug.LogWarning($"[CollisionHandler] Invalid nodeManager for {owner?.name}");
                return false;
            }

            if (nodeMasses == null || nodeMasses.Count != nodeManager.NodeCount)
            {
                if (enableDebugLogging) Debug.LogWarning($"[CollisionHandler] Mass count mismatch for {owner?.name}");
                return false;
            }

            return true;
        }

        public void IgnoreBodyCollision(Transform bodyA, Transform bodyB)
        {
            int idA = bodyA.GetInstanceID();
            int idB = bodyB.GetInstanceID();
            var pair = idA < idB ? (idA, idB) : (idB, idA);
            _ignoredBodyPairs.Add(pair);
        }

        public void SetIgnoreCollision(Transform bodyA, Transform bodyB, bool ignore)
        {
            if (ignore)
                IgnoreBodyCollision(bodyA, bodyB);
            else
                RemoveIgnoredBodyCollision(bodyA, bodyB);
        }

        public void RemoveIgnoredBodyCollision(Transform bodyA, Transform bodyB)
        {
            int idA = bodyA.GetInstanceID();
            int idB = bodyB.GetInstanceID();
            var pair = idA < idB ? (idA, idB) : (idB, idA);
            _ignoredBodyPairs.Remove(pair);
        }

        public void HardExcludeBody(Transform body)
        {
            if (body != null)
                _hardExcludedBodyIds.Add(body.GetInstanceID());
        }

        public void RemoveHardExcludedBody(Transform body)
        {
            if (body != null)
                _hardExcludedBodyIds.Remove(body.GetInstanceID());
        }

        public void ClearIgnoredPairs()
        {
            _ignoredBodyPairs.Clear();
        }

        #endregion
    }
}